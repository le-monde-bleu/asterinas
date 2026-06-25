// SPDX-License-Identifier: MPL-2.0

//! In-memory queue infrastructure for the RISC-V IOMMU.

use crate::{
    Error,
    mm::{FrameAllocOptions, HasPaddr, PAGE_SIZE, Paddr, Segment, VmIo},
};

/// A circular buffer of fixed-size entries stored in a dedicated page.
pub(super) struct Queue<const ENTRY_SIZE: usize> {
    segment: Segment<()>,
    capacity: usize,
    tail: usize,
}

impl<const ENTRY_SIZE: usize> Queue<ENTRY_SIZE> {
    /// Creates a queue backed by a single page.
    pub(super) fn new() -> Result<Self, Error> {
        let segment = FrameAllocOptions::new().zeroed(true).alloc_segment(1)?;
        let capacity = PAGE_SIZE / ENTRY_SIZE;
        // The IOMMU requires the queue to have at least 2 entries and the
        // size must be a power of two so log2sz_minus_1() is well-defined.
        debug_assert!(capacity >= 2 && capacity.is_power_of_two());
        Ok(Self {
            segment,
            capacity,
            tail: 0,
        })
    }

    /// Appends an entry and returns the new tail index.
    pub(super) fn push(
        &mut self,
        head: usize,
        entry: &[u8; ENTRY_SIZE],
    ) -> Result<usize, QueueError> {
        let next_tail = (self.tail + 1) % self.capacity;
        if next_tail == head % self.capacity {
            return Err(QueueError::Full);
        }

        self.segment
            .write_val(self.tail * ENTRY_SIZE, entry)
            .map_err(QueueError::MemoryAccess)?;
        self.tail = next_tail;
        Ok(self.tail)
    }

    /// Returns the current tail index (the number of entries pushed modulo
    /// the queue capacity). The IOMMU compares this against its internal
    /// head index and wraps independently, so the ring wraparound is
    /// expected.
    pub(super) fn tail(&self) -> usize {
        self.tail
    }

    /// Returns the physical address of the queue memory (for `cqb`/`fqb`/`pqb`).
    pub(super) fn base_paddr(&self) -> Paddr {
        self.segment.paddr()
    }

    /// Returns `log2(queue size) - 1`, the format required by the queue base
    /// register's LOG2SZ field. The caller must guarantee `capacity` is a
    /// power of two and >= 2.
    pub(super) fn log2sz_minus_1(&self) -> u8 {
        (self.capacity.ilog2() - 1) as u8
    }

    /// Returns the mask used by the queue head and tail registers.
    pub(super) fn index_mask(&self) -> usize {
        self.capacity - 1
    }

    /// Reads an entry at the given index into `out`. Used by the fault
    /// handler to inspect queued fault records.
    pub(super) fn read_entry(&self, index: usize, out: &mut [u8; ENTRY_SIZE]) -> Result<(), Error> {
        self.segment.read_bytes(index * ENTRY_SIZE, out)
    }
}

/// A command queue entry.
pub(super) type CmdEntry = [u8; 16];

/// A fault queue entry.
pub(super) type FaultEntry = [u8; 32];

/// An error encountered when adding an entry to a queue.
#[derive(Debug)]
pub(super) enum QueueError {
    /// The queue has no free entry.
    Full,
    /// The queue entry could not be written to memory.
    MemoryAccess(Error),
}

const OPCODE_IOTINVAL: u64 = 1;
const OPCODE_IOFENCE: u64 = 2;
const OPCODE_IODIR: u64 = 3;
const FUNC_SHIFT: u32 = 7;

fn command_entry(dword0: u64, dword1: u64) -> CmdEntry {
    let mut entry = [0u8; 16];
    entry[..8].copy_from_slice(&dword0.to_le_bytes());
    entry[8..].copy_from_slice(&dword1.to_le_bytes());
    entry
}

/// Constructs an `IOFENCE.C` command descriptor.
pub(super) fn cmd_iofence_c(pr: bool, pw: bool) -> CmdEntry {
    let pr_bit = if pr { 1 << 12 } else { 0 };
    let pw_bit = if pw { 1 << 13 } else { 0 };
    command_entry(OPCODE_IOFENCE | pr_bit | pw_bit, 0)
}

/// Constructs an `IODIR.INVAL_DDT` command descriptor.
#[cfg_attr(not(ktest), expect(dead_code))]
pub(super) fn cmd_iodir_inval_ddt(device_id: Option<u32>) -> CmdEntry {
    let operands = device_id.map_or(0, |device_id| {
        debug_assert!(device_id < 1 << 24);
        (1 << 33) | ((device_id as u64) << 40)
    });
    command_entry(OPCODE_IODIR | operands, 0)
}

/// Constructs an `IOTINVAL.VMA` command descriptor.
#[expect(dead_code)]
pub(super) fn cmd_iotinval_vma(
    gscid: Option<u16>,
    pscid: Option<u32>,
    addr: Option<u64>,
) -> CmdEntry {
    let mut dword0 = OPCODE_IOTINVAL;
    if let Some(addr) = addr {
        dword0 |= 1 << 10;
        debug_assert_eq!(addr & 0xfff, 0);
    }
    if let Some(pscid) = pscid {
        debug_assert!(pscid < 1 << 20);
        dword0 |= 1 << 32;
        dword0 |= (pscid as u64) << 12;
    }
    if let Some(gscid) = gscid {
        dword0 |= 1 << 33;
        dword0 |= (gscid as u64) << 44;
    }

    command_entry(dword0, addr.map_or(0, |addr| addr >> 2))
}

/// Constructs an `IOTINVAL.GVMA` command descriptor.
pub(super) fn cmd_iotinval_gvma(gscid: Option<u16>, addr: Option<u64>) -> CmdEntry {
    let mut dword0 = OPCODE_IOTINVAL | (1 << FUNC_SHIFT);
    if let Some(addr) = addr {
        dword0 |= 1 << 10;
        debug_assert_eq!(addr & 0xfff, 0);
    }
    if let Some(gscid) = gscid {
        dword0 |= 1 << 33;
        dword0 |= (gscid as u64) << 44;
    }

    command_entry(dword0, addr.map_or(0, |addr| addr >> 2))
}

#[cfg(ktest)]
mod tests {
    use super::*;
    use crate::prelude::*;

    fn words(entry: CmdEntry) -> (u64, u64) {
        (
            u64::from_le_bytes(entry[..8].try_into().unwrap()),
            u64::from_le_bytes(entry[8..].try_into().unwrap()),
        )
    }

    #[ktest]
    fn iommu_command_encodings_follow_the_spec() {
        assert_eq!(words(cmd_iofence_c(true, true)), (2 | 1 << 12 | 1 << 13, 0));
        assert_eq!(
            words(cmd_iodir_inval_ddt(Some(0x12_3456))),
            (3 | 1 << 33 | 0x12_3456_u64 << 40, 0)
        );
        assert_eq!(
            words(cmd_iotinval_gvma(Some(0x1234), Some(0x1234_5000))),
            (
                1 | 1 << 7 | 1 << 10 | 1 << 33 | 0x1234_u64 << 44,
                0x1234_5000 >> 2
            )
        );
    }
}
