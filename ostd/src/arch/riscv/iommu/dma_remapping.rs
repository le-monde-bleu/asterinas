// SPDX-License-Identifier: MPL-2.0

//! DMA remapping for the RISC-V IOMMU.
//!
//! Initialization follows Section 6.2 of the RISC-V IOMMU specification. The
//! feature-control register is finalized while the IOMMU and all queues are
//! off, and queue/DDT memory is published only after initialization succeeds.

use core::mem;

use spin::Once;

use super::{
    IommuError,
    ddt::{DdtTable, DeviceContextFormat},
    fault::FaultSetup,
    msi::MsiPageTable,
    queue::{self, Queue, QueueError},
    registers::{self, InterruptGenerationSupport},
    second_stage::Sv39x4PageTable,
};
use crate::{
    info,
    mm::{Daddr, PAGE_SIZE},
    prelude::Paddr,
    sync::{LocalIrqDisabled, SpinLock},
    warn,
};

const REGISTER_TRANSITION_TIMEOUT: usize = 100_000;
const COMMAND_COMPLETION_TIMEOUT: usize = 1_000_000;
const SUPPORTED_MAJOR_VERSION: u8 = 1;

/// Returns `true` if DMA remapping has been initialized and is active.
pub fn has_dma_remapping() -> bool {
    PAGE_TABLE.get().is_some()
}

/// Returns `true` when MSI writes are translated through an MSI page table.
pub fn has_interrupt_remapping() -> bool {
    MSI_PAGE_TABLE.get().is_some()
}

/// Maps a single page from a device address to a physical address.
///
/// # Safety
///
/// The physical address must point to untyped DMA memory that outlives this
/// mapping.
pub unsafe fn map(daddr: Daddr, paddr: Paddr) -> Result<(), IommuError> {
    let Some(table) = PAGE_TABLE.get() else {
        return Err(IommuError::NoIommu);
    };

    table
        .lock()
        .map(daddr, paddr)
        .map_err(IommuError::SecondStage)?;
    invalidate_second_stage_page(daddr, IoFenceOrdering::None)?;
    super::fault::process_faults();
    Ok(())
}

/// Unmaps a single page at the given device address.
pub fn unmap(daddr: Daddr) -> Result<(), IommuError> {
    let Some(table) = PAGE_TABLE.get() else {
        return Err(IommuError::NoIommu);
    };

    table.lock().unmap(daddr).map_err(IommuError::SecondStage)?;
    invalidate_second_stage_page(daddr, IoFenceOrdering::ReadsAndWrites)
}

/// Initializes DMA remapping.
pub fn init() -> Result<(), IommuError> {
    quiesce_hardware()?;
    let (capabilities, use_wired_interrupts) = configure_features()?;
    let physical_address_bits = capabilities.physical_address_size();

    let context_format = if capabilities
        .flags()
        .contains(registers::CapabilityFlags::MSI_FLAT)
    {
        DeviceContextFormat::Extended
    } else {
        DeviceContextFormat::Base
    };

    let command_queue = Queue::new().map_err(IommuError::Allocation)?;
    let fault_setup = FaultSetup::prepare(use_wired_interrupts)?;
    let mut device_directory =
        DdtTable::new(context_format).map_err(IommuError::DeviceDirectory)?;
    let mut page_table =
        Sv39x4PageTable::new(physical_address_bits).map_err(IommuError::SecondStage)?;
    let msi_address = crate::arch::irq::IRQ_CHIP
        .get()
        .and_then(|irq_chip| irq_chip.msi_address())
        .ok_or(IommuError::Unsupported("supervisor IMSIC"))?;
    let msi_page_table = if matches!(context_format, DeviceContextFormat::Extended) {
        Some(MsiPageTable::new(msi_address).map_err(IommuError::Allocation)?)
    } else {
        map_msi_page(&mut page_table, msi_address)?;
        None
    };
    validate_iommu_owned_memory(
        physical_address_bits,
        &command_queue,
        &fault_setup,
        &device_directory,
        &page_table,
        msi_page_table.as_ref(),
    )?;

    // PCI requester IDs are 16-bit BDF values. Populate every possible ID so
    // devices enumerated after IOMMU initialization always find a context.
    for device_id in 0..=u16::MAX {
        device_directory
            .enable_device(device_id.into(), &page_table, msi_page_table.as_ref())
            .map_err(IommuError::DeviceDirectory)?;
    }

    let mut pending = PendingIommuState {
        command_queue: Some(command_queue),
        fault_setup: Some(fault_setup),
        device_directory: Some(device_directory),
        page_table: Some(page_table),
        msi_page_table,
        hardware_active: true,
    };

    {
        let mut iommu_regs = registers::IOMMU_REGS.get().unwrap().lock();
        enable_command_queue(pending.command_queue.as_ref().unwrap(), &mut iommu_regs)?;
        pending
            .fault_setup
            .as_ref()
            .unwrap()
            .enable_queue(&mut iommu_regs)?;
        enable_device_directory(pending.device_directory.as_ref().unwrap(), &mut iommu_regs)?;

        let initial_fence = queue::cmd_iofence_c(true, true);
        submit_commands_locked(
            pending.command_queue.as_mut().unwrap(),
            &mut iommu_regs,
            &[initial_fence],
        )?;
    }

    pending.publish();
    super::fault::process_faults();
    Ok(())
}

fn validate_iommu_owned_memory(
    physical_address_bits: u8,
    command_queue: &Queue<16>,
    fault_setup: &FaultSetup,
    device_directory: &DdtTable,
    page_table: &Sv39x4PageTable,
    msi_page_table: Option<&MsiPageTable>,
) -> Result<(), IommuError> {
    ensure_paddr_supported(command_queue.base_paddr(), physical_address_bits)?;
    ensure_paddr_supported(fault_setup.queue_paddr(), physical_address_bits)?;
    ensure_paddr_supported(device_directory.root_paddr(), physical_address_bits)?;
    ensure_paddr_supported(page_table.root_paddr(), physical_address_bits)?;
    if let Some(msi_page_table) = msi_page_table {
        ensure_paddr_supported(msi_page_table.root_paddr(), physical_address_bits)?;
    }
    Ok(())
}

fn ensure_paddr_supported(paddr: Paddr, physical_address_bits: u8) -> Result<(), IommuError> {
    if physical_address_bits >= usize::BITS as u8 || paddr < (1usize << physical_address_bits) {
        Ok(())
    } else {
        Err(IommuError::Unsupported("IOMMU physical address size"))
    }
}

fn configure_features() -> Result<(registers::Capability, bool), IommuError> {
    let mut iommu_regs = registers::IOMMU_REGS.get().unwrap().lock();
    let capabilities = iommu_regs.capabilities;
    if capabilities.version() >> 4 != SUPPORTED_MAJOR_VERSION {
        return Err(IommuError::Unsupported("IOMMU specification version"));
    }
    if !capabilities
        .flags()
        .contains(registers::CapabilityFlags::SV39X4)
    {
        return Err(IommuError::Unsupported("Sv39x4"));
    }

    let use_wired_interrupts = match capabilities.interrupt_generation_support() {
        InterruptGenerationSupport::Wired | InterruptGenerationSupport::Both => true,
        InterruptGenerationSupport::Msi => {
            warn!(
                "IOMMU-originated MSI configuration is not implemented; fault interrupts are disabled"
            );
            false
        }
        InterruptGenerationSupport::Reserved => {
            return Err(IommuError::Unsupported("interrupt generation method"));
        }
    };

    let mut fctl = iommu_regs.fctl.as_ptr().read();
    fctl &= !(registers::FCTL_BE | registers::FCTL_GXL | registers::FCTL_WSI);
    if use_wired_interrupts {
        fctl |= registers::FCTL_WSI;
    }
    iommu_regs.fctl.as_mut_ptr().write(fctl);

    let fctl = iommu_regs.fctl.as_ptr().read();
    if fctl & registers::FCTL_BE != 0 {
        return Err(IommuError::Unsupported("little-endian IOMMU structures"));
    }
    if fctl & registers::FCTL_GXL != 0 {
        return Err(IommuError::Unsupported("64-bit guest physical addresses"));
    }
    if (fctl & registers::FCTL_WSI != 0) != use_wired_interrupts {
        return Err(IommuError::Unsupported("selected IOMMU interrupt method"));
    }

    Ok((capabilities, use_wired_interrupts))
}

fn enable_command_queue(
    command_queue: &Queue<16>,
    iommu_regs: &mut registers::IommuRegisters,
) -> Result<(), IommuError> {
    let cq_base = registers::QueueBase::new(
        (command_queue.base_paddr() >> 12) as u64,
        command_queue.log2sz_minus_1(),
    );
    iommu_regs.cqb.as_mut_ptr().write(cq_base.value());
    iommu_regs.cqt.as_mut_ptr().write(0);
    iommu_regs.cqcsr.as_mut_ptr().write(registers::CQCSR_CQEN);

    for _ in 0..REGISTER_TRANSITION_TIMEOUT {
        let cqcsr = iommu_regs.cqcsr.as_ptr().read();
        let error_bits = registers::CQCSR_CQMF | registers::CQCSR_CMD_TO | registers::CQCSR_CMD_ILL;
        if cqcsr & error_bits != 0 {
            return Err(IommuError::CommandQueueError(cqcsr));
        }
        if cqcsr & registers::CQCSR_BUSY == 0 && cqcsr & registers::CQCSR_CQON != 0 {
            return Ok(());
        }
        core::hint::spin_loop();
    }

    Err(IommuError::InitializationTimeout("command queue"))
}

fn enable_device_directory(
    device_directory: &DdtTable,
    iommu_regs: &mut registers::IommuRegisters,
) -> Result<(), IommuError> {
    let mode = device_directory.mode();
    let mut ddtp = registers::Ddtp::new();
    ddtp.set_mode(mode as u8);
    ddtp.set_ppn((device_directory.root_paddr() >> 12) as u64);
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    iommu_regs.ddtp.as_mut_ptr().write(ddtp.value());

    for _ in 0..REGISTER_TRANSITION_TIMEOUT {
        let ddtp = iommu_regs.ddtp.as_ptr().read();
        if ddtp & registers::DDTP_BUSY == 0 && ddtp & 0x0f == mode {
            return Ok(());
        }
        core::hint::spin_loop();
    }

    Err(IommuError::InitializationTimeout("device directory table"))
}

fn map_msi_page(page_table: &mut Sv39x4PageTable, msi_address: Paddr) -> Result<(), IommuError> {
    let msi_page = msi_address & !(PAGE_SIZE - 1);
    page_table
        .map(msi_page, msi_page)
        .map_err(IommuError::SecondStage)
}

struct PendingIommuState {
    command_queue: Option<Queue<16>>,
    fault_setup: Option<FaultSetup>,
    device_directory: Option<DdtTable>,
    page_table: Option<Sv39x4PageTable>,
    msi_page_table: Option<MsiPageTable>,
    hardware_active: bool,
}

impl PendingIommuState {
    fn publish(mut self) {
        let has_msi_translation = self.msi_page_table.is_some();
        DDT_TABLE.call_once(|| SpinLock::new(self.device_directory.take().unwrap()));
        PAGE_TABLE.call_once(|| SpinLock::new(self.page_table.take().unwrap()));
        COMMAND_QUEUE.call_once(|| SpinLock::new(self.command_queue.take().unwrap()));
        if let Some(msi_page_table) = self.msi_page_table.take() {
            MSI_PAGE_TABLE.call_once(|| msi_page_table);
        }
        self.fault_setup.take().unwrap().publish();
        self.hardware_active = false;

        if has_msi_translation {
            info!("DMA and MSI remapping enabled (Sv39x4, flat MSI page table)");
        } else {
            info!("DMA remapping enabled (Sv39x4, identity-mapped MSI page)");
        }
    }

    fn leak_resources(&mut self) {
        if let Some(command_queue) = self.command_queue.take() {
            mem::forget(command_queue);
        }
        if let Some(fault_setup) = self.fault_setup.take() {
            mem::forget(fault_setup);
        }
        if let Some(device_directory) = self.device_directory.take() {
            mem::forget(device_directory);
        }
        if let Some(page_table) = self.page_table.take() {
            mem::forget(page_table);
        }
        if let Some(msi_page_table) = self.msi_page_table.take() {
            mem::forget(msi_page_table);
        }
    }
}

impl Drop for PendingIommuState {
    fn drop(&mut self) {
        if !self.hardware_active {
            return;
        }
        if let Err(error) = quiesce_hardware() {
            warn!(
                "Failed to quiesce IOMMU after initialization error: {:?}; leaking hardware-owned memory",
                error
            );
            self.leak_resources();
        }
    }
}

/// Device Directory Table singleton, initialized by [`init`].
static DDT_TABLE: Once<SpinLock<DdtTable, LocalIrqDisabled>> = Once::new();

/// Shared second-stage page table for all devices, initialized by [`init`].
static PAGE_TABLE: Once<SpinLock<Sv39x4PageTable, LocalIrqDisabled>> = Once::new();

/// Command queue singleton for invalidation and fencing commands.
static COMMAND_QUEUE: Once<SpinLock<Queue<16>, LocalIrqDisabled>> = Once::new();

/// Flat MSI page table used by every configured device context.
static MSI_PAGE_TABLE: Once<MsiPageTable> = Once::new();

#[derive(Clone, Copy)]
enum IoFenceOrdering {
    None,
    ReadsAndWrites,
}

fn invalidate_second_stage_page(daddr: Daddr, ordering: IoFenceOrdering) -> Result<(), IommuError> {
    let Some(command_queue) = COMMAND_QUEUE.get() else {
        return Err(IommuError::NoIommu);
    };

    let invalidate = queue::cmd_iotinval_gvma(Some(0), Some(daddr as u64));
    let (read, write) = match ordering {
        IoFenceOrdering::None => (false, false),
        IoFenceOrdering::ReadsAndWrites => (true, true),
    };
    let fence = queue::cmd_iofence_c(read, write);

    let mut command_queue = command_queue.lock();
    let mut iommu_regs = registers::IOMMU_REGS.get().unwrap().lock();
    submit_commands_locked(&mut command_queue, &mut iommu_regs, &[invalidate, fence])
}

fn submit_commands_locked(
    queue: &mut Queue<16>,
    iommu_regs: &mut registers::IommuRegisters,
    commands: &[queue::CmdEntry],
) -> Result<(), IommuError> {
    let index_mask = queue.index_mask();
    let head = iommu_regs.cqh.as_ptr().read() as usize & index_mask;

    for command in commands {
        queue.push(head, command).map_err(|error| match error {
            QueueError::Full => IommuError::CommandQueueFull,
            QueueError::MemoryAccess(error) => IommuError::CommandQueueMemory(error),
        })?;
    }

    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    let expected_head = queue.tail();
    iommu_regs.cqt.as_mut_ptr().write(expected_head as u32);

    for _ in 0..COMMAND_COMPLETION_TIMEOUT {
        let cqcsr = iommu_regs.cqcsr.as_ptr().read();
        let error_bits = registers::CQCSR_CQMF | registers::CQCSR_CMD_TO | registers::CQCSR_CMD_ILL;
        if cqcsr & error_bits != 0 {
            return Err(IommuError::CommandQueueError(cqcsr));
        }

        let current_head = iommu_regs.cqh.as_ptr().read() as usize & index_mask;
        if current_head == expected_head {
            return Ok(());
        }

        core::hint::spin_loop();
    }

    Err(IommuError::CommandQueueTimeout)
}

fn quiesce_hardware() -> Result<(), IommuError> {
    let mut iommu_regs = registers::IOMMU_REGS.get().unwrap().lock();

    wait_ddtp_idle(&mut iommu_regs)?;
    iommu_regs.ddtp.as_mut_ptr().write(registers::DDTP_MODE_OFF);
    for _ in 0..REGISTER_TRANSITION_TIMEOUT {
        let ddtp = iommu_regs.ddtp.as_ptr().read();
        if ddtp & registers::DDTP_BUSY == 0 && ddtp & 0x0f == registers::DDTP_MODE_OFF {
            break;
        }
        core::hint::spin_loop();
    }
    if iommu_regs.ddtp.as_ptr().read() & 0x0f != registers::DDTP_MODE_OFF {
        return Err(IommuError::InitializationTimeout("IOMMU off"));
    }

    disable_queue(
        &mut iommu_regs,
        QueueKind::PageRequest,
        registers::PQCSR_BUSY,
        registers::PQCSR_PQON,
    )?;
    disable_queue(
        &mut iommu_regs,
        QueueKind::Fault,
        registers::FQCSR_BUSY,
        registers::FQCSR_FQON,
    )?;
    disable_queue(
        &mut iommu_regs,
        QueueKind::Command,
        registers::CQCSR_BUSY,
        registers::CQCSR_CQON,
    )
}

fn wait_ddtp_idle(iommu_regs: &mut registers::IommuRegisters) -> Result<(), IommuError> {
    for _ in 0..REGISTER_TRANSITION_TIMEOUT {
        if iommu_regs.ddtp.as_ptr().read() & registers::DDTP_BUSY == 0 {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(IommuError::InitializationTimeout("DDTP busy"))
}

#[derive(Clone, Copy)]
enum QueueKind {
    Command,
    Fault,
    PageRequest,
}

fn disable_queue(
    iommu_regs: &mut registers::IommuRegisters,
    queue: QueueKind,
    busy_bit: u32,
    active_bit: u32,
) -> Result<(), IommuError> {
    let read = |iommu_regs: &registers::IommuRegisters| match queue {
        QueueKind::Command => iommu_regs.cqcsr.as_ptr().read(),
        QueueKind::Fault => iommu_regs.fqcsr.as_ptr().read(),
        QueueKind::PageRequest => iommu_regs.pqcsr.as_ptr().read(),
    };
    let write_zero = |iommu_regs: &mut registers::IommuRegisters| match queue {
        QueueKind::Command => iommu_regs.cqcsr.as_mut_ptr().write(0),
        QueueKind::Fault => iommu_regs.fqcsr.as_mut_ptr().write(0),
        QueueKind::PageRequest => iommu_regs.pqcsr.as_mut_ptr().write(0),
    };

    for _ in 0..REGISTER_TRANSITION_TIMEOUT {
        if read(iommu_regs) & busy_bit == 0 {
            write_zero(iommu_regs);
            break;
        }
        core::hint::spin_loop();
    }
    for _ in 0..REGISTER_TRANSITION_TIMEOUT {
        let status = read(iommu_regs);
        if status & (busy_bit | active_bit) == 0 {
            return Ok(());
        }
        core::hint::spin_loop();
    }

    let name = match queue {
        QueueKind::Command => "command queue off",
        QueueKind::Fault => "fault queue off",
        QueueKind::PageRequest => "page-request queue off",
    };
    Err(IommuError::InitializationTimeout(name))
}
