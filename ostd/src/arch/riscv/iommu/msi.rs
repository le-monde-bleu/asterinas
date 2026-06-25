// SPDX-License-Identifier: MPL-2.0

//! MSI address translation tables for the RISC-V IOMMU.

use crate::{
    Error,
    mm::{Frame, FrameAllocOptions, HasPaddr, Paddr, VmIo},
};

const MSIPTP_MODE_FLAT: u64 = 1;
const MSI_PTE_MODE_BASIC: u64 = 3;

#[repr(C)]
#[derive(Clone, Copy, Pod)]
struct MsiPte {
    pte: u64,
    mrif_info: u64,
}

/// A flat MSI page table containing one physical interrupt file.
pub(super) struct MsiPageTable {
    frame: Frame<()>,
    address_pattern: u64,
}

impl MsiPageTable {
    /// Creates a table that translates one guest interrupt-file page to the
    /// physical IMSIC interrupt file at `message_address`.
    pub(super) fn new(message_address: Paddr) -> Result<Self, Error> {
        if !message_address.is_multiple_of(4096) {
            return Err(Error::InvalidArgs);
        }

        let frame = FrameAllocOptions::new().zeroed(true).alloc_frame()?;
        frame.write_val(0, &MsiPte::new_basic(message_address))?;

        Ok(Self {
            frame,
            address_pattern: (message_address >> 12) as u64,
        })
    }

    pub(super) fn root_paddr(&self) -> Paddr {
        self.frame.paddr()
    }

    pub(super) fn msiptp(&self) -> u64 {
        MSIPTP_MODE_FLAT << 60 | (self.frame.paddr() >> 12) as u64
    }

    pub(super) fn address_mask(&self) -> u64 {
        0
    }

    pub(super) fn address_pattern(&self) -> u64 {
        self.address_pattern
    }
}

impl MsiPte {
    fn new_basic(message_address: Paddr) -> Self {
        let ppn = (message_address >> 12) as u64;
        Self {
            pte: ppn << 10 | MSI_PTE_MODE_BASIC << 1 | 1,
            mrif_info: 0,
        }
    }
}

#[cfg(ktest)]
mod tests {
    use super::*;
    use crate::prelude::*;

    #[ktest]
    fn iommu_basic_msi_pte_encoding() {
        let pte = MsiPte::new_basic(0x2800_0000);
        assert_eq!(pte.pte, 0x0a00_0007);
        assert_eq!(pte.mrif_info, 0);
    }
}
