// SPDX-License-Identifier: MPL-2.0

use crate::arch::iommu::has_interrupt_remapping;

pub(crate) struct IrqRemapping {
    _private: (),
}

impl IrqRemapping {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    /// RISC-V flat MSI translation is configured per interrupt file rather
    /// than per interrupt identity, so no per-IRQ table entry is required.
    pub(crate) fn init(&self, _irq_num: u8) {}

    /// Returns the single interrupt-file index when MSI translation is active.
    pub(crate) fn remapping_index(&self) -> Option<u16> {
        has_interrupt_remapping().then_some(0)
    }
}
