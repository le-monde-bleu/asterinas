// SPDX-License-Identifier: MPL-2.0

use core::ops::Range;

use aster_rights::{Dup, Read, Rights, TRightSet, TRights, Write};
use aster_rights_proc::require;

use super::{VmPerms, Vmar, VmarMapOptions, VmarQueryGuard, VmarRightsOp, Vmar_};
use crate::{
    prelude::*, thread::exception::PageFaultInfo, vm::page_fault_handler::PageFaultHandler,
};

impl<R: TRights> Vmar<TRightSet<R>> {
    /// Creates a root VMAR.
    ///
    /// # Access rights
    ///
    /// A root VMAR is initially given full access rights.
    pub fn new_root() -> Self {
        let inner = Vmar_::new_root();
        let rights = R::new();
        Self(inner, TRightSet(rights))
    }

    /// Creates a mapping into the VMAR through a set of VMAR mapping options.
    ///
    /// # Example
    ///
    /// ```
    /// use aster_nix::prelude::*;
    /// use aster_nix::vm::{PAGE_SIZE, Vmar, VmoOptions};
    ///
    /// let vmar = Vmar::<RightsWrapper<Full>>::new().unwrap();
    /// let vmo = VmoOptions::new(10 * PAGE_SIZE).alloc().unwrap();
    /// let target_vaddr = 0x1234000;
    /// let real_vaddr = vmar
    ///     // Create a 4 * PAGE_SIZE bytes, read-only mapping
    ///     .new_map(PAGE_SIZE * 4, VmPerms::READ)
    ///     // Provide an optional offset for the mapping inside the VMAR
    ///     .offset(target_vaddr)
    ///     // Specify an optional binding VMO.
    ///     .vmo(vmo)
    ///     // Provide an optional offset to indicate the corresponding offset
    ///     // in the VMO for the mapping
    ///     .vmo_offset(2 * PAGE_SIZE)
    ///     .build()
    ///     .unwrap();
    /// assert!(real_vaddr == target_vaddr);
    /// ```
    ///
    /// For more details on the available options, see `VmarMapOptions`.
    ///
    /// # Access rights
    ///
    /// This method requires the following access rights:
    /// 1. The VMAR contains the rights corresponding to the memory permissions of
    /// the mapping. For example, if `perms` contains `VmPerms::WRITE`,
    /// then the VMAR must have the Write right.
    /// 2. Similarly, the VMO contains the rights corresponding to the memory
    /// permissions of the mapping.
    ///
    /// Memory permissions may be changed through the `protect` method,
    /// which ensures that any updated memory permissions do not go beyond
    /// the access rights of the underlying VMOs.
    #[require(R > Dup)]
    pub fn new_map(
        &self,
        size: usize,
        perms: VmPerms,
    ) -> Result<VmarMapOptions<TRightSet<R>, Rights>> {
        Ok(VmarMapOptions::new(self, size, perms))
    }

    /// Change the permissions of the memory mappings in the specified range.
    ///
    /// The range's start and end addresses must be page-aligned.
    /// Also, the range must be completely mapped.
    ///
    /// # Access rights
    ///
    /// The VMAR must have the rights corresponding to the specified memory
    /// permissions.
    pub fn protect(&self, perms: VmPerms, range: Range<usize>) -> Result<()> {
        self.check_rights(perms.into())?;
        self.0.protect(perms, range)
    }

    /// Finds all the mapped regions that intersect with the specified range.
    pub fn query(&self, range: Range<usize>) -> VmarQueryGuard<'_> {
        self.0.query(range)
    }

    /// Clears all mappings.
    ///
    /// After being cleared, this vmar will become an empty vmar
    pub fn clear(&self) -> Result<()> {
        self.0.clear_root_vmar()
    }

    /// Destroys all mappings that fall within the specified
    /// range in bytes.
    ///
    /// The range's start and end addresses must be page-aligned.
    ///
    /// Mappings may fall partially within the range; only the overlapped
    /// portions of the mappings are unmapped.
    #[require(R > Write)]
    pub fn remove_mapping(&self, range: Range<usize>) -> Result<()> {
        self.0.remove_mapping(range)
    }

    /// Duplicates the capability.
    ///
    /// # Access rights
    ///
    /// The method requires the Dup right.
    #[require(R > Dup)]
    pub fn dup(&self) -> Result<Self> {
        Ok(Vmar(self.0.clone(), self.1))
    }

    /// Creates a new root VMAR whose content is inherited from another
    /// using copy-on-write (COW) technique.
    ///
    /// # Access rights
    ///
    /// The method requires the Read right.
    #[require(R > Read)]
    pub fn fork_from(vmar: &Self) -> Result<Self> {
        let vmar_ = vmar.0.new_fork_root()?;
        Ok(Vmar(vmar_, TRightSet(R::new())))
    }

    /// Stricts the access rights.
    #[require(R > R1)]
    pub fn restrict<R1: TRights>(self) -> Vmar<R1> {
        Vmar(self.0, R1::new())
    }

    fn check_rights(&self, rights: Rights) -> Result<()> {
        if self.rights().contains(rights) {
            Ok(())
        } else {
            return_errno_with_message!(Errno::EACCES, "check rights failed");
        }
    }
}

impl<R: TRights> PageFaultHandler for Vmar<TRightSet<R>> {
    fn handle_page_fault(&self, page_fault_info: &PageFaultInfo) -> Result<()> {
        self.check_rights(page_fault_info.required_perms.into())?;
        self.0.handle_page_fault(page_fault_info)
    }
}

impl<R: TRights> VmarRightsOp for Vmar<TRightSet<R>> {
    fn rights(&self) -> Rights {
        Rights::from_bits(R::BITS).unwrap()
    }
}
