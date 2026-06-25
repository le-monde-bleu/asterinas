// SPDX-License-Identifier: MPL-2.0

//! Fault queue management for the RISC-V IOMMU.

use spin::Once;

use super::{IommuError, queue::Queue, registers};
use crate::{
    arch::{
        boot::DEVICE_TREE,
        irq::{IRQ_CHIP, InterruptSourceInFdt, MappedIrqLine},
    },
    error,
    irq::IrqLine,
    sync::{LocalIrqDisabled, SpinLock},
    warn,
};

const FAULT_ENTRY_SIZE: usize = 32;
const QUEUE_ENABLE_TIMEOUT: usize = 100_000;
const QUEUE_CSR_WRITE_TIMEOUT: usize = 100_000;
const REQUESTED_FAULT_VECTOR: u64 = 1;

/// Fault-reporting resources that are not published until IOMMU setup succeeds.
pub(super) struct FaultSetup {
    queue: Queue<FAULT_ENTRY_SIZE>,
    interrupt: Option<MappedIrqLine>,
}

impl FaultSetup {
    /// Allocates the fault queue and, when selected, maps its wired interrupt.
    pub(super) fn prepare(use_wired_interrupts: bool) -> Result<Self, IommuError> {
        let queue = Queue::new().map_err(IommuError::Allocation)?;
        let interrupt = use_wired_interrupts.then(map_fault_interrupt).flatten();
        if use_wired_interrupts && interrupt.is_none() {
            warn!(
                "IOMMU fault interrupt is unavailable; fault records are checked after DMA map operations"
            );
        }
        Ok(Self { queue, interrupt })
    }

    /// Returns the physical address of the fault queue memory.
    pub(super) fn queue_paddr(&self) -> crate::prelude::Paddr {
        self.queue.base_paddr()
    }

    /// Programs and enables the fault queue without enabling its interrupt.
    pub(super) fn enable_queue(
        &self,
        iommu_regs: &mut registers::IommuRegisters,
    ) -> Result<(), IommuError> {
        let fq_base = registers::QueueBase::new(
            (self.queue.base_paddr() >> 12) as u64,
            self.queue.log2sz_minus_1(),
        );
        iommu_regs.fqb.as_mut_ptr().write(fq_base.value());
        iommu_regs.fqh.as_mut_ptr().write(0);
        iommu_regs.fqcsr.as_mut_ptr().write(registers::FQCSR_FQEN);

        if wait_fault_queue_enabled(iommu_regs) {
            Ok(())
        } else {
            Err(IommuError::InitializationTimeout("fault queue"))
        }
    }

    /// Publishes the queue and enables interrupts only after all state is live.
    pub(super) fn publish(self) {
        let Self { queue, interrupt } = self;
        let enable_interrupt = interrupt.is_some();
        FAULT_QUEUE.call_once(|| SpinLock::new(queue));
        if let Some(interrupt) = interrupt {
            FAULT_IRQ.call_once(|| interrupt);
        }

        if enable_interrupt {
            let mut iommu_regs = registers::IOMMU_REGS.get().unwrap().lock();
            if !write_fault_queue_csr_when_idle(
                &mut iommu_regs,
                registers::FQCSR_FQEN | registers::FQCSR_FIE,
            ) {
                warn!("IOMMU fault interrupt was mapped but could not be enabled");
            }
        }
    }
}

fn map_fault_interrupt() -> Option<MappedIrqLine> {
    let fault_vector = {
        let mut iommu_regs = registers::IOMMU_REGS.get()?.lock();
        let icvec = iommu_regs.icvec.as_ptr().read();
        iommu_regs.icvec.as_mut_ptr().write(
            (icvec & !registers::ICVEC_FIV_MASK)
                | (REQUESTED_FAULT_VECTOR << registers::ICVEC_FIV_SHIFT),
        );
        (iommu_regs.icvec.as_ptr().read() & registers::ICVEC_FIV_MASK) >> registers::ICVEC_FIV_SHIFT
    } as usize;

    let node = DEVICE_TREE.get()?.all_nodes().find(|node| {
        node.compatible().is_some_and(|compatibles| {
            compatibles
                .all()
                .any(|compatible| compatible == "riscv,iommu")
        })
    })?;
    let interrupt_parent = node.property("interrupt-parent")?.as_usize()? as u32;
    let fault_interrupt = node.interrupts()?.nth(fault_vector)?;
    let irq_line = IrqLine::alloc().ok()?;
    let mut mapped_irq = IRQ_CHIP
        .get()?
        .map_fdt_pin_to(
            InterruptSourceInFdt::new(interrupt_parent, fault_interrupt),
            irq_line,
        )
        .ok()?;
    mapped_irq.on_active(|_| process_faults());
    Some(mapped_irq)
}

enum FaultDrainResult {
    Empty,
    Retry,
    Record(Result<(usize, [u8; FAULT_ENTRY_SIZE]), (usize, crate::Error)>),
}

/// Drains and logs all pending fault records from the fault queue.
pub(super) fn process_faults() {
    let Some(fault_queue) = FAULT_QUEUE.get() else {
        return;
    };

    loop {
        let (queue_error, result) = {
            // Lock order is fault queue then IOMMU registers. Disabling local
            // interrupts prevents the fault IRQ from recursively taking this lock.
            let queue = fault_queue.lock();
            let mut iommu_regs = registers::IOMMU_REGS.get().unwrap().lock();
            let fqcsr = iommu_regs.fqcsr.as_ptr().read();
            let error_bits = registers::FQCSR_FQMF | registers::FQCSR_FQOF;
            let queue_error = (fqcsr & error_bits != 0).then_some(fqcsr);
            if queue_error.is_some() {
                let enabled = fqcsr & (registers::FQCSR_FQEN | registers::FQCSR_FIE);
                if !write_fault_queue_csr_when_idle(&mut iommu_regs, enabled | (fqcsr & error_bits))
                {
                    return;
                }
            }

            let index_mask = queue.index_mask();
            let tail = iommu_regs.fqt.as_ptr().read() as usize & index_mask;
            let head = iommu_regs.fqh.as_ptr().read() as usize & index_mask;
            if tail == head {
                iommu_regs.ipsr.as_mut_ptr().write(registers::IPSR_FIP);
                let tail_after_clear = iommu_regs.fqt.as_ptr().read() as usize & index_mask;
                let head_after_clear = iommu_regs.fqh.as_ptr().read() as usize & index_mask;
                let result = if tail_after_clear == head_after_clear {
                    FaultDrainResult::Empty
                } else {
                    FaultDrainResult::Retry
                };
                (queue_error, result)
            } else {
                let mut entry = [0u8; FAULT_ENTRY_SIZE];
                let read_result = queue
                    .read_entry(head, &mut entry)
                    .map(|()| (head, entry))
                    .map_err(|error| (head, error));
                if read_result.is_ok() {
                    iommu_regs
                        .fqh
                        .as_mut_ptr()
                        .write(((head + 1) & index_mask) as u32);
                }
                (queue_error, FaultDrainResult::Record(read_result))
            }
        };

        if let Some(fqcsr) = queue_error {
            error!("IOMMU fault queue error: fqcsr=0x{:x}", fqcsr);
        }

        match result {
            FaultDrainResult::Empty => return,
            FaultDrainResult::Retry => continue,
            FaultDrainResult::Record(Ok((index, entry))) => log_fault_record(index, entry),
            FaultDrainResult::Record(Err((index, error))) => {
                error!(
                    "IOMMU fault queue read failed at index {}: {:?}",
                    index, error
                );
                return;
            }
        }
    }
}

fn log_fault_record(index: usize, entry: [u8; FAULT_ENTRY_SIZE]) {
    let dw0 = read_u64(&entry, 0);
    let dw1 = read_u64(&entry, 8);
    let iotval = read_u64(&entry, 16);
    let iotval2 = read_u64(&entry, 24);

    // DW0 contains the fault record's CAUSE, TTYP, and DID fields.
    let cause = (dw0 & 0x0fff) as u16;
    let device_id = ((dw0 >> 40) & 0x00ff_ffff) as u32;
    let transaction_type = ((dw0 >> 34) & 0x3f) as u8;

    error!(
        "IOMMU fault[{}]: cause={} ({}), DID=0x{:x}, TTYP=0x{:x}, iotval=0x{:x}, iotval2=0x{:x}, dw1=0x{:x}",
        index,
        cause,
        cause_str(cause),
        device_id,
        transaction_type,
        iotval,
        iotval2,
        dw1,
    );
}

fn read_u64(entry: &[u8; FAULT_ENTRY_SIZE], offset: usize) -> u64 {
    let mut bytes = [0u8; size_of::<u64>()];
    bytes.copy_from_slice(&entry[offset..offset + size_of::<u64>()]);
    u64::from_le_bytes(bytes)
}

/// Decodes a fault `CAUSE` code into a short human-readable string.
fn cause_str(cause: u16) -> &'static str {
    match cause {
        1 => "instruction access fault",
        4 => "read access fault",
        5 => "load access fault",
        7 => "store/AMO access fault",
        12 => "instruction page fault",
        13 => "load page fault",
        15 => "store/AMO page fault",
        20 => "instruction guest-page fault",
        21 => "load guest-page fault",
        23 => "store/AMO guest-page fault",
        256 => "all inbound transactions disallowed",
        257 => "DDT entry load access fault",
        258 => "DDT entry not valid",
        259 => "DDT entry misconfigured",
        260 => "transaction type disallowed",
        261 => "MSI PTE load access fault",
        262 => "MSI PTE not valid",
        263 => "MSI PTE misconfigured",
        264 => "MRIF access fault",
        265 => "PDT entry load access fault",
        266 => "PDT entry not valid",
        267 => "PDT entry misconfigured",
        268 => "DDT data corruption",
        269 => "PDT data corruption",
        270 => "MSI page table data corruption",
        271 => "MRIF data corruption",
        272 => "internal data path error",
        273 => "IOMMU MSI write access fault",
        274 => "page table data corruption",
        _ => "unknown",
    }
}

/// Fault queue singleton, published only after IOMMU initialization succeeds.
static FAULT_QUEUE: Once<SpinLock<Queue<FAULT_ENTRY_SIZE>, LocalIrqDisabled>> = Once::new();

/// Wired fault interrupt mapping kept alive for the lifetime of the IOMMU.
static FAULT_IRQ: Once<MappedIrqLine> = Once::new();

fn write_fault_queue_csr_when_idle(iommu_regs: &mut registers::IommuRegisters, value: u32) -> bool {
    for _ in 0..QUEUE_CSR_WRITE_TIMEOUT {
        if iommu_regs.fqcsr.as_ptr().read() & registers::FQCSR_BUSY == 0 {
            iommu_regs.fqcsr.as_mut_ptr().write(value);
            return true;
        }
        core::hint::spin_loop();
    }

    false
}

fn wait_fault_queue_enabled(iommu_regs: &mut registers::IommuRegisters) -> bool {
    for _ in 0..QUEUE_ENABLE_TIMEOUT {
        let fqcsr = iommu_regs.fqcsr.as_ptr().read();
        let error_bits = registers::FQCSR_FQMF | registers::FQCSR_FQOF;
        if fqcsr & error_bits != 0 {
            let _ = write_fault_queue_csr_when_idle(iommu_regs, fqcsr & error_bits);
            return false;
        }

        if fqcsr & registers::FQCSR_BUSY == 0 && fqcsr & registers::FQCSR_FQON != 0 {
            return true;
        }

        core::hint::spin_loop();
    }

    false
}
