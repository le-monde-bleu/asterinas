// SPDX-License-Identifier: MPL-2.0

//! Sv39x4 second-stage page tables for the RISC-V IOMMU.
//!
//! Unlike an Sv39 CPU page table, an Sv39x4 page table has a 16-KiB,
//! 16-KiB-aligned root containing 2048 entries. Lower-level tables remain
//! 4-KiB pages with 512 entries. A dedicated implementation is used here
//! because the generic CPU page-table implementation assumes that every
//! level has the same page size.

use alloc::collections::BTreeMap;
use core::sync::atomic::{Ordering, fence};

use crate::{
    Error,
    mm::{Frame, FrameAllocOptions, HasPaddr, PAGE_SIZE, Paddr, Segment, VmIo},
};

const ROOT_TABLE_PAGES: usize = 4;
const ROOT_TABLE_SIZE: usize = ROOT_TABLE_PAGES * PAGE_SIZE;
const ROOT_ALLOCATION_PAGES: usize = ROOT_TABLE_PAGES * 2 - 1;
const ROOT_ENTRY_COUNT: usize = 2048;
const CHILD_ENTRY_COUNT: usize = 512;
const ADDRESS_WIDTH: usize = 41;
const MAX_PHYSICAL_ADDRESS_WIDTH: u8 = 56;

bitflags::bitflags! {
    #[repr(C)]
    #[derive(Pod)]
    struct PteFlags: u64 {
        const VALID = 1 << 0;
        const READ = 1 << 1;
        const WRITE = 1 << 2;
        const USER = 1 << 4;
        const ACCESSED = 1 << 6;
        const DIRTY = 1 << 7;
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod)]
struct PageTableEntry(u64);

impl PageTableEntry {
    const PPN_MASK: u64 = 0x003f_ffff_ffff_fc00;

    fn is_valid(self) -> bool {
        self.0 & PteFlags::VALID.bits() != 0
    }

    fn paddr(self) -> Paddr {
        ((self.0 & Self::PPN_MASK) >> 10 << 12) as Paddr
    }

    fn table(paddr: Paddr) -> Self {
        Self(((paddr as u64 >> 12) << 10) | PteFlags::VALID.bits())
    }

    fn page(paddr: Paddr) -> Self {
        let flags = PteFlags::VALID
            | PteFlags::READ
            | PteFlags::WRITE
            | PteFlags::USER
            | PteFlags::ACCESSED
            | PteFlags::DIRTY;
        Self(((paddr as u64 >> 12) << 10) | flags.bits())
    }
}

/// Errors encountered while constructing or modifying an Sv39x4 page table.
#[derive(Debug)]
pub(crate) enum SecondStageError {
    /// A page-table frame could not be allocated.
    Allocation(Error),
    /// A page-table entry could not be read or written.
    MemoryAccess(Error),
    /// A device or physical address is invalid for a base-page mapping.
    InvalidAddress,
    /// The IOMMU does not support the requested physical address width.
    UnsupportedPhysicalAddressWidth,
    /// The device address is already mapped.
    AlreadyMapped,
    /// The device address is not mapped.
    NotMapped,
    /// A valid page-table pointer does not refer to an owned child table.
    Corrupted,
}

/// A complete Sv39x4 page table with a 16-KiB root.
pub(super) struct Sv39x4PageTable {
    root_segment: Segment<()>,
    middle_frames: BTreeMap<Paddr, Frame<()>>,
    leaf_frames: BTreeMap<Paddr, Frame<()>>,
    physical_address_bits: u8,
}

impl Sv39x4PageTable {
    /// Allocates an empty Sv39x4 page table.
    pub(super) fn new(physical_address_bits: u8) -> Result<Self, SecondStageError> {
        if !(12..=MAX_PHYSICAL_ADDRESS_WIDTH).contains(&physical_address_bits) {
            return Err(SecondStageError::UnsupportedPhysicalAddressWidth);
        }

        let allocation = FrameAllocOptions::new()
            .zeroed(true)
            .alloc_segment(ROOT_ALLOCATION_PAGES)
            .map_err(SecondStageError::Allocation)?;
        let alignment_offset =
            (ROOT_TABLE_SIZE - allocation.paddr() % ROOT_TABLE_SIZE) % ROOT_TABLE_SIZE;
        let root_segment =
            allocation.slice(&(alignment_offset..alignment_offset + ROOT_TABLE_SIZE));
        drop(allocation);

        debug_assert_eq!(root_segment.paddr() % ROOT_TABLE_SIZE, 0);
        Ok(Self {
            root_segment,
            middle_frames: BTreeMap::new(),
            leaf_frames: BTreeMap::new(),
            physical_address_bits,
        })
    }

    /// Returns the 16-KiB-aligned root table physical address.
    pub(super) fn root_paddr(&self) -> Paddr {
        self.root_segment.paddr()
    }

    /// Maps one 4-KiB device page to one physical page.
    pub(super) fn map(&mut self, daddr: usize, paddr: Paddr) -> Result<(), SecondStageError> {
        let [root_index, middle_index, leaf_index] =
            page_indices(daddr, paddr, self.physical_address_bits)?;

        let middle_paddr =
            get_or_create_child(&self.root_segment, root_index, &mut self.middle_frames)?;
        let middle_frame = self
            .middle_frames
            .get(&middle_paddr)
            .ok_or(SecondStageError::Corrupted)?;
        let leaf_paddr = get_or_create_child(middle_frame, middle_index, &mut self.leaf_frames)?;
        let leaf_frame = self
            .leaf_frames
            .get(&leaf_paddr)
            .ok_or(SecondStageError::Corrupted)?;

        if read_entry(leaf_frame, leaf_index)?.is_valid() {
            return Err(SecondStageError::AlreadyMapped);
        }
        write_entry(leaf_frame, leaf_index, PageTableEntry::page(paddr))
    }

    /// Removes one 4-KiB device-page mapping.
    pub(super) fn unmap(&mut self, daddr: usize) -> Result<(), SecondStageError> {
        let [root_index, middle_index, leaf_index] =
            page_indices(daddr, 0, self.physical_address_bits)?;
        let root_entry = read_entry(&self.root_segment, root_index)?;
        if !root_entry.is_valid() {
            return Err(SecondStageError::NotMapped);
        }
        let middle_frame = self
            .middle_frames
            .get(&root_entry.paddr())
            .ok_or(SecondStageError::NotMapped)?;
        let middle_entry = read_entry(middle_frame, middle_index)?;
        if !middle_entry.is_valid() {
            return Err(SecondStageError::NotMapped);
        }
        let leaf_frame = self
            .leaf_frames
            .get(&middle_entry.paddr())
            .ok_or(SecondStageError::NotMapped)?;
        if !read_entry(leaf_frame, leaf_index)?.is_valid() {
            return Err(SecondStageError::NotMapped);
        }
        write_entry(leaf_frame, leaf_index, PageTableEntry::default())
    }
}

fn page_indices(
    daddr: usize,
    paddr: Paddr,
    physical_address_bits: u8,
) -> Result<[usize; 3], SecondStageError> {
    if !daddr.is_multiple_of(PAGE_SIZE)
        || !paddr.is_multiple_of(PAGE_SIZE)
        || daddr >= 1usize << ADDRESS_WIDTH
        || !paddr_fits(paddr, physical_address_bits)
    {
        return Err(SecondStageError::InvalidAddress);
    }

    Ok([
        (daddr >> 30) & (ROOT_ENTRY_COUNT - 1),
        (daddr >> 21) & (CHILD_ENTRY_COUNT - 1),
        (daddr >> 12) & (CHILD_ENTRY_COUNT - 1),
    ])
}

fn paddr_fits(paddr: Paddr, physical_address_bits: u8) -> bool {
    paddr < (1usize << physical_address_bits)
}

fn get_or_create_child(
    parent: &impl VmIo,
    index: usize,
    children: &mut BTreeMap<Paddr, Frame<()>>,
) -> Result<Paddr, SecondStageError> {
    let entry = read_entry(parent, index)?;
    if entry.is_valid() {
        return Ok(entry.paddr());
    }

    let frame = FrameAllocOptions::new()
        .zeroed(true)
        .alloc_frame()
        .map_err(SecondStageError::Allocation)?;
    let paddr = frame.paddr();
    fence(Ordering::Release);
    write_entry(parent, index, PageTableEntry::table(paddr))?;
    children.insert(paddr, frame);
    Ok(paddr)
}

fn read_entry(parent: &impl VmIo, index: usize) -> Result<PageTableEntry, SecondStageError> {
    parent
        .read_val(index * size_of::<PageTableEntry>())
        .map_err(SecondStageError::MemoryAccess)
}

fn write_entry(
    parent: &impl VmIo,
    index: usize,
    entry: PageTableEntry,
) -> Result<(), SecondStageError> {
    parent
        .write_val(index * size_of::<PageTableEntry>(), &entry)
        .map_err(SecondStageError::MemoryAccess)
}

#[cfg(ktest)]
mod tests {
    use super::*;
    use crate::prelude::*;

    #[ktest]
    fn iommu_g_stage_leaf_pte_sets_mandatory_bits() {
        let entry = PageTableEntry::page(0x1234_5000);
        let required = PteFlags::VALID
            | PteFlags::READ
            | PteFlags::WRITE
            | PteFlags::USER
            | PteFlags::ACCESSED
            | PteFlags::DIRTY;

        assert_eq!(entry.paddr(), 0x1234_5000);
        assert_eq!(entry.0 & required.bits(), required.bits());
    }

    #[ktest]
    fn iommu_nonleaf_pte_contains_only_pointer_state() {
        let entry = PageTableEntry::table(0x2345_6000);
        assert_eq!(entry.paddr(), 0x2345_6000);
        assert_eq!(entry.0 & 0x3ff, PteFlags::VALID.bits());
    }

    #[ktest]
    fn iommu_sv39x4_indices_cover_41_bits() {
        let highest_page = (1usize << ADDRESS_WIDTH) - PAGE_SIZE;
        assert_eq!(
            page_indices(highest_page, 0, MAX_PHYSICAL_ADDRESS_WIDTH).unwrap(),
            [2047, 511, 511]
        );
        assert!(page_indices(1usize << ADDRESS_WIDTH, 0, MAX_PHYSICAL_ADDRESS_WIDTH).is_err());
    }

    #[ktest]
    fn iommu_rejects_physical_addresses_beyond_capability() {
        assert!(page_indices(0, 1usize << 40, 40).is_err());
        assert!(page_indices(0, (1usize << 40) - PAGE_SIZE, 40).is_ok());
    }

    #[ktest]
    fn iommu_sv39x4_root_is_16kib_aligned() {
        let page_table = Sv39x4PageTable::new(MAX_PHYSICAL_ADDRESS_WIDTH).unwrap();
        assert_eq!(page_table.root_paddr() % ROOT_TABLE_SIZE, 0);
    }

    #[ktest]
    fn iommu_sv39x4_map_and_unmap_base_page() {
        let mut page_table = Sv39x4PageTable::new(MAX_PHYSICAL_ADDRESS_WIDTH).unwrap();
        let daddr = 0x1234_5000;
        let paddr = 0x2345_6000;
        let [root_index, middle_index, leaf_index] =
            page_indices(daddr, paddr, MAX_PHYSICAL_ADDRESS_WIDTH).unwrap();

        page_table.map(daddr, paddr).unwrap();
        assert!(matches!(
            page_table.map(daddr, paddr),
            Err(SecondStageError::AlreadyMapped)
        ));

        let root_entry = read_entry(&page_table.root_segment, root_index).unwrap();
        let middle_frame = page_table.middle_frames.get(&root_entry.paddr()).unwrap();
        let middle_entry = read_entry(middle_frame, middle_index).unwrap();
        let leaf_paddr = middle_entry.paddr();
        let leaf_frame = page_table.leaf_frames.get(&leaf_paddr).unwrap();
        let leaf_entry = read_entry(leaf_frame, leaf_index).unwrap();
        assert_eq!(leaf_entry.paddr(), paddr);
        assert!(leaf_entry.is_valid());

        page_table.unmap(daddr).unwrap();
        let leaf_frame = page_table.leaf_frames.get(&leaf_paddr).unwrap();
        assert!(!read_entry(leaf_frame, leaf_index).unwrap().is_valid());
        assert!(matches!(
            page_table.unmap(daddr),
            Err(SecondStageError::NotMapped)
        ));
    }
}
