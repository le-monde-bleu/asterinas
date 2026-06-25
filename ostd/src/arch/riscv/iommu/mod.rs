// SPDX-License-Identifier: MPL-2.0

//! The IOMMU support for RISC-V.
//!
//! Implements DMA remapping via a second-stage page table (Sv39x4) shared
//! across all devices, connected through a format-appropriate Device Directory
//! Table. MSI writes are translated through a flat MSI page table when
//! supported by the IOMMU.
//!
//! The public interface consists of [`has_dma_remapping`], [`map`], and [`unmap`],
//! which are called from the generic DMA layer in [`crate::mm::dma`].
//! The second-stage translation uses a dedicated Sv39x4 implementation so its
//! 16-KiB root-table requirement is represented exactly.
//!
//! See the parent module ([`crate::arch::riscv`]) for the initialization path.
//!
//! For more details, see the RISC-V IOMMU specification:
//! <https://docs.riscv.org/reference/iommu/index.html>.

// Set this module's log prefix for `ostd::log`.
macro_rules! __log_prefix {
    () => {
        "iommu: "
    };
}

mod ddt;
mod dma_remapping;
mod fault;
mod msi;
mod queue;
mod registers;
mod second_stage;

pub(crate) use dma_remapping::{has_dma_remapping, has_interrupt_remapping, map, unmap};

use crate::{Error, io::IoMemAllocatorBuilder};

/// Errors reported while initializing or operating the RISC-V IOMMU.
#[derive(Debug)]
pub(crate) enum IommuError {
    /// No IOMMU is available.
    NoIommu,
    /// The device-tree register region cannot safely cover the required MMIO.
    InvalidRegisterRegion,
    /// A required hardware capability or writable control is unavailable.
    Unsupported(&'static str),
    /// An IOMMU-owned memory object could not be allocated or initialized.
    Allocation(Error),
    /// Error encountered while constructing or modifying the second stage.
    SecondStage(second_stage::SecondStageError),
    /// Error encountered while constructing the Device Directory Table.
    DeviceDirectory(ddt::DdtError),
    /// The command queue does not have enough free entries.
    CommandQueueFull,
    /// A command queue entry could not be written to memory.
    CommandQueueMemory(Error),
    /// The command queue reported an execution error.
    CommandQueueError(u32),
    /// The command queue did not complete before the timeout.
    CommandQueueTimeout,
    /// A hardware initialization transition did not complete.
    InitializationTimeout(&'static str),
}

pub(crate) fn init(io_mem_builder: &IoMemAllocatorBuilder) -> Result<(), IommuError> {
    registers::init(io_mem_builder)?;
    dma_remapping::init()
}
