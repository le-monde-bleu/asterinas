// SPDX-License-Identifier: MPL-2.0

//! Device Directory Table (DDT) for the RISC-V IOMMU.

use alloc::collections::BTreeMap;
use core::sync::atomic::{Ordering, fence};

use super::{msi::MsiPageTable, second_stage::Sv39x4PageTable};
use crate::{
    Error,
    mm::{Frame, FrameAllocOptions, HasPaddr, Paddr, VmIo},
};

// A non-leaf DDT entry.
#[repr(C)]
#[derive(Clone, Copy, Pod)]
struct DdtEntry(u64);

impl DdtEntry {
    const PPN_SHIFT: u32 = 10;
    const PPN_MASK: u64 = 0xFFFFFFFFFFF; // 44-bit PPN

    fn is_valid(&self) -> bool {
        (self.0 & 0x1) != 0
    }

    fn paddr(&self) -> Paddr {
        (((self.0 >> Self::PPN_SHIFT) & Self::PPN_MASK) as usize) << 12
    }

    fn new(paddr: Paddr) -> Self {
        let ppn = (paddr >> 12) as u64;
        Self((ppn << Self::PPN_SHIFT) | 0x1)
    }
}

/// The in-memory Device Context format selected by `capabilities.MSI_FLAT`.
#[derive(Clone, Copy)]
pub(super) enum DeviceContextFormat {
    Base,
    Extended,
}

impl DeviceContextFormat {
    fn size(self) -> usize {
        match self {
            Self::Base => 32,
            Self::Extended => 64,
        }
    }

    fn ddi0_bits(self) -> u32 {
        match self {
            Self::Base => 7,
            Self::Extended => 6,
        }
    }

    fn indices(self, device_id: u32) -> (usize, usize, usize) {
        debug_assert!(device_id < 1 << 24);
        let ddi0_bits = self.ddi0_bits();
        let ddi0 = (device_id as usize) & ((1 << ddi0_bits) - 1);
        let ddi1 = ((device_id as usize) >> ddi0_bits) & 0x1ff;
        let ddi2 = ((device_id as usize) >> (ddi0_bits + 9)) & 0x1ff;
        (ddi0, ddi1, ddi2)
    }
}

// The `MODE` field value for Sv39x4 in `iohgatp` when `fctl.GXL=0`.
const IOHGATP_MODE_SV39X4: u64 = 8;

/// Errors that can occur during DDT manipulation.
#[derive(Debug)]
pub(crate) enum DdtError {
    /// A DDT frame could not be allocated.
    Allocation(Error),
    /// A DDT entry could not be read or written.
    MemoryAccess(Error),
    /// A valid DDT pointer does not refer to an owned child table.
    Corrupted,
}

/// Device Directory Table for translating `device_id` to a Device Context.
pub(super) struct DdtTable {
    root_frame: Frame<()>,
    middle_frames: BTreeMap<Paddr, Frame<()>>,
    leaf_frames: BTreeMap<Paddr, Frame<()>>,
    context_format: DeviceContextFormat,
}

impl DdtTable {
    pub(super) fn new(context_format: DeviceContextFormat) -> Result<Self, DdtError> {
        Ok(Self {
            root_frame: FrameAllocOptions::new()
                .zeroed(true)
                .alloc_frame()
                .map_err(DdtError::Allocation)?,
            middle_frames: BTreeMap::new(),
            leaf_frames: BTreeMap::new(),
            context_format,
        })
    }

    /// Returns the root table physical address.
    pub(super) fn root_paddr(&self) -> Paddr {
        self.root_frame.paddr()
    }

    /// Returns the DDTP mode required by this Device Context format.
    pub(super) fn mode(&self) -> u64 {
        match self.context_format {
            DeviceContextFormat::Base => super::registers::DDTP_MODE_2LVL,
            DeviceContextFormat::Extended => super::registers::DDTP_MODE_3LVL,
        }
    }

    fn child_paddr(parent: &Frame<()>, index: usize) -> Result<Option<Paddr>, DdtError> {
        let offset = index * size_of::<DdtEntry>();
        let entry = parent
            .read_val::<DdtEntry>(offset)
            .map_err(DdtError::MemoryAccess)?;
        Ok(entry.is_valid().then(|| entry.paddr()))
    }

    fn install_child(parent: &Frame<()>, index: usize, paddr: Paddr) -> Result<(), DdtError> {
        let offset = index * size_of::<DdtEntry>();
        parent
            .write_val(offset, &DdtEntry::new(paddr))
            .map_err(DdtError::MemoryAccess)
    }

    fn get_or_create_leaf(&mut self, ddi2: usize, ddi1: usize) -> Result<&Frame<()>, DdtError> {
        let leaf_parent = match self.context_format {
            DeviceContextFormat::Base => &self.root_frame,
            DeviceContextFormat::Extended => {
                let middle_paddr = match Self::child_paddr(&self.root_frame, ddi2)? {
                    Some(paddr) => paddr,
                    None => {
                        let frame = FrameAllocOptions::new()
                            .zeroed(true)
                            .alloc_frame()
                            .map_err(DdtError::Allocation)?;
                        let paddr = frame.paddr();
                        fence(Ordering::Release);
                        Self::install_child(&self.root_frame, ddi2, paddr)?;
                        self.middle_frames.insert(paddr, frame);
                        paddr
                    }
                };
                self.middle_frames
                    .get(&middle_paddr)
                    .ok_or(DdtError::Corrupted)?
            }
        };

        let leaf_paddr = match Self::child_paddr(leaf_parent, ddi1)? {
            Some(paddr) => paddr,
            None => {
                let frame = FrameAllocOptions::new()
                    .zeroed(true)
                    .alloc_frame()
                    .map_err(DdtError::Allocation)?;
                let paddr = frame.paddr();
                fence(Ordering::Release);
                Self::install_child(leaf_parent, ddi1, paddr)?;
                self.leaf_frames.insert(paddr, frame);
                paddr
            }
        };
        self.leaf_frames.get(&leaf_paddr).ok_or(DdtError::Corrupted)
    }

    /// Writes a Device Context for `device_id` pointing at the given page table.
    pub(super) fn enable_device(
        &mut self,
        device_id: u32,
        page_table: &Sv39x4PageTable,
        msi_page_table: Option<&MsiPageTable>,
    ) -> Result<(), DdtError> {
        let context_format = self.context_format;
        let (ddi0, ddi1, ddi2) = context_format.indices(device_id);
        let context_size = context_format.size();

        let leaf = self.get_or_create_leaf(ddi2, ddi1)?;
        let dc_offset = ddi0 * context_size;

        // GSCID is left at 0 because per-VM invalidation is not yet
        // implemented; all VMs share one invalidation domain.
        let root_paddr = page_table.root_paddr();
        let ppn = (root_paddr >> 12) as u64;
        let iohgatp: u64 = (IOHGATP_MODE_SV39X4 << 60) | ppn;
        leaf.write_val(dc_offset + 8, &iohgatp)
            .map_err(DdtError::MemoryAccess)?;

        if let Some(msi_page_table) = msi_page_table {
            debug_assert!(matches!(context_format, DeviceContextFormat::Extended));
            leaf.write_val(dc_offset + 32, &msi_page_table.msiptp())
                .map_err(DdtError::MemoryAccess)?;
            leaf.write_val(dc_offset + 40, &msi_page_table.address_mask())
                .map_err(DdtError::MemoryAccess)?;
            leaf.write_val(dc_offset + 48, &msi_page_table.address_pattern())
                .map_err(DdtError::MemoryAccess)?;
        }

        // Publish the context only after every field is initialized.
        fence(Ordering::Release);
        let tc: u64 = 0x1;
        leaf.write_val(dc_offset, &tc)
            .map_err(DdtError::MemoryAccess)?;

        Ok(())
    }
}

#[cfg(ktest)]
mod tests {
    use super::*;
    use crate::prelude::*;

    #[ktest]
    fn iommu_device_id_indices_cover_the_supported_width() {
        assert_eq!(DeviceContextFormat::Base.indices(0xffff), (0x7f, 0x1ff, 0));
        assert_eq!(
            DeviceContextFormat::Extended.indices(0x7fff),
            (0x3f, 0x1ff, 0)
        );
        assert_eq!(DeviceContextFormat::Extended.indices(0x8000), (0, 0, 1));
        assert_eq!(
            DeviceContextFormat::Extended.indices(0xffff),
            (0x3f, 0x1ff, 1)
        );
        assert_eq!(
            DeviceContextFormat::Extended.indices(0xff_ffff),
            (0x3f, 0x1ff, 0x1ff)
        );
    }

    #[ktest]
    fn iommu_base_device_context_points_to_second_stage_root() {
        let context_format = DeviceContextFormat::Base;
        let mut ddt = DdtTable::new(context_format).unwrap();
        let page_table = Sv39x4PageTable::new(56).unwrap();
        let device_id = 0x1234;

        ddt.enable_device(device_id, &page_table, None).unwrap();

        assert_eq!(ddt.mode(), super::super::registers::DDTP_MODE_2LVL);

        let (ddi0, ddi1, ddi2) = context_format.indices(device_id);
        let leaf = ddt.get_or_create_leaf(ddi2, ddi1).unwrap();
        let dc_offset = ddi0 * context_format.size();
        let tc = leaf.read_val::<u64>(dc_offset).unwrap();
        let iohgatp = leaf.read_val::<u64>(dc_offset + 8).unwrap();
        let expected_iohgatp = (IOHGATP_MODE_SV39X4 << 60) | (page_table.root_paddr() >> 12) as u64;
        assert_eq!(tc, 1);
        assert_eq!(iohgatp, expected_iohgatp);
    }

    #[ktest]
    fn iommu_extended_device_context_contains_msi_translation() {
        let context_format = DeviceContextFormat::Extended;
        let mut ddt = DdtTable::new(context_format).unwrap();
        let page_table = Sv39x4PageTable::new(56).unwrap();
        let msi_page_table = MsiPageTable::new(0x2800_0000).unwrap();
        let device_id = 0x8000;

        ddt.enable_device(device_id, &page_table, Some(&msi_page_table))
            .unwrap();

        assert_eq!(ddt.mode(), super::super::registers::DDTP_MODE_3LVL);

        let (ddi0, ddi1, ddi2) = context_format.indices(device_id);
        let leaf = ddt.get_or_create_leaf(ddi2, ddi1).unwrap();
        let dc_offset = ddi0 * context_format.size();
        assert_eq!(leaf.read_val::<u64>(dc_offset).unwrap(), 1);
        assert_eq!(
            leaf.read_val::<u64>(dc_offset + 32).unwrap(),
            msi_page_table.msiptp()
        );
        assert_eq!(
            leaf.read_val::<u64>(dc_offset + 40).unwrap(),
            msi_page_table.address_mask()
        );
        assert_eq!(
            leaf.read_val::<u64>(dc_offset + 48).unwrap(),
            msi_page_table.address_pattern()
        );
    }
}
