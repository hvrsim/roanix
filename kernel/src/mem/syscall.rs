//! Virtual-memory syscall implementations.

use crate::{
    fs::OpenFlags,
    mem::{
        self, PAGE_SIZE, USER_ADDRESS_MIN, VirtAddr, VmAdvice, VmBacking, VmInheritance, VmMapping,
        VmPlacement, VmProtection,
    },
    proc::Descriptor,
    syscall::{Errno, current_process, map_fs_error, map_memory_error},
};

const MADV_NORMAL: u64 = 0;
const MADV_RANDOM: u64 = 1;
const MADV_SEQUENTIAL: u64 = 2;
const MADV_WILLNEED: u64 = 3;
const MADV_DONTNEED: u64 = 4;

const PROT_READ: u64 = 0x01;
const PROT_WRITE: u64 = 0x02;
const PROT_EXEC: u64 = 0x04;

const MAP_SHARED: u64 = 0x01;
const MAP_PRIVATE: u64 = 0x02;
const MAP_FIXED: u64 = 0x10;
const MAP_ANONYMOUS: u64 = 0x20;
const MAP_DENYWRITE: u64 = 0x800;
const MAP_EXECUTABLE: u64 = 0x1000;
const MAP_NORESERVE: u64 = 0x4000;
const MAP_STACK: u64 = 0x20000;

crate::syscall_handler! {
    syscall_memory_map(
        _frame,
        hint: u64 = 0,
        size: u64 = 1,
        protection: u64 = 2,
        flags: u64 = 3,
        fd: u64 = 4,
        offset: u64 = 5,
    ) {
        if size == 0 || protection & !(PROT_READ | PROT_WRITE | PROT_EXEC) != 0 {
            return Err(Errno::Invalid);
        }
        // Simultaneously writable and executable mappings are refused so that
        // no user page can be rewritten and then run.
        if protection & PROT_WRITE != 0 && protection & PROT_EXEC != 0 {
            return Err(Errno::Access);
        }
        let allowed_flags = MAP_SHARED
            | MAP_PRIVATE
            | MAP_FIXED
            | MAP_ANONYMOUS
            | MAP_DENYWRITE
            | MAP_EXECUTABLE
            | MAP_NORESERVE
            | MAP_STACK;
        if flags & !allowed_flags != 0
            || (flags & MAP_SHARED != 0) == (flags & MAP_PRIVATE != 0)
        {
            return Err(Errno::Invalid);
        }

        let process = current_process()?;
        let space = process.address_space();
        let protection = vm_protection(protection);
        let inheritance = if flags & MAP_SHARED != 0 {
            VmInheritance::Share
        } else {
            VmInheritance::Copy
        };
        // Validate and retain file-backed state before MAP_FIXED tears down an
        // existing mapping. A bad descriptor or unsupported vnode must leave
        // the old range intact when mmap fails.
        let (backing, maximum_protection) = if flags & MAP_ANONYMOUS != 0 {
            if offset != 0 {
                return Err(Errno::Invalid);
            }
            (VmBacking::Anonymous, all_user_protections())
        } else {
            let offset = i64::try_from(offset).map_err(|_| Errno::Invalid)?;
            if offset % PAGE_SIZE as i64 != 0 {
                return Err(Errno::Invalid);
            }
            let fd = i32::try_from(fd).map_err(|_| Errno::BadFileDescriptor)?;
            let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
            let Descriptor::File(file) = descriptor else {
                return Err(Errno::BadFileDescriptor);
            };
            if !file.flags().contains(OpenFlags::READ) {
                return Err(Errno::Access);
            }
            if protection.contains(VmProtection::WRITE)
                && flags & MAP_SHARED != 0
                && !file.flags().contains(OpenFlags::WRITE)
            {
                return Err(Errno::Access);
            }
            let vnode = file.vnode();
            let file_size = vnode.getattr().map_err(map_fs_error)?.size;
            let mapped_size = file_size
                .checked_add(PAGE_SIZE - 1)
                .map(|size| size & !(PAGE_SIZE - 1))
                .ok_or(Errno::Overflow)?;
            if (offset as u64)
                .checked_add(size)
                .is_none_or(|end| end > mapped_size)
            {
                // SIGBUS-on-access is not implemented yet, so reject ranges
                // that could otherwise fabricate zero pages past EOF.
                return Err(Errno::Invalid);
            }
            let mut maximum = VmProtection::READ | VmProtection::EXECUTE;
            if flags & MAP_PRIVATE != 0 || file.flags().contains(OpenFlags::WRITE) {
                maximum |= VmProtection::WRITE;
            }
            (
                VmBacking::Object {
                    object: vnode.memory_object().map_err(map_fs_error)?,
                    offset: offset as u64,
                    private: flags & MAP_PRIVATE != 0,
                },
                maximum,
            )
        };
        let placement = if flags & MAP_FIXED != 0 {
            if hint < USER_ADDRESS_MIN || !hint.is_multiple_of(PAGE_SIZE) {
                return Err(Errno::Invalid);
            }
            VmPlacement::Fixed(VirtAddr::new(hint))
        } else if hint == 0 {
            VmPlacement::Any
        } else {
            VmPlacement::Hint(VirtAddr::new(hint))
        };
        let replace = matches!(placement, VmPlacement::Fixed(_));
        let mapping = VmMapping {
            placement,
            length: size,
            protection,
            maximum_protection,
            inheritance,
            backing,
        };
        let start = if replace {
            space.replace(mapping)
        } else {
            space.map(mapping)
        };
        Ok(start.map_err(map_memory_error)?.as_u64())
    }
}

crate::syscall_handler! {
    syscall_memory_unmap(_frame, address: u64 = 0, size: u64 = 1) {
        if size == 0 || !address.is_multiple_of(PAGE_SIZE) {
            return Err(Errno::Invalid);
        }
        // Unmapping a range that holds no mapping succeeds, matching the
        // behaviour every portable allocator depends on.
        match current_process()?
            .address_space()
            .unmap(VirtAddr::new(address), size)
        {
            Ok(()) | Err(mem::Error::NotMapped) => Ok(0),
            Err(error) => Err(map_memory_error(error)),
        }
    }
}

crate::syscall_handler! {
    syscall_memory_protect(_frame, address: u64 = 0, size: u64 = 1, protection: u64 = 2) {
        if size == 0
            || !address.is_multiple_of(PAGE_SIZE)
            || protection & !(PROT_READ | PROT_WRITE | PROT_EXEC) != 0
        {
            return Err(Errno::Invalid);
        }
        let protection = vm_protection(protection);
        if protection.contains(VmProtection::WRITE | VmProtection::EXECUTE) {
            return Err(Errno::Access);
        }
        current_process()?
            .address_space()
            .protect(VirtAddr::new(address), size, protection)
            .map_err(map_memory_error)?;
        Ok(0)
    }
}

crate::syscall_handler! {
    syscall_memory_advise(_frame, address: u64 = 0, size: u64 = 1, advice: u64 = 2) {
        if size == 0 || !address.is_multiple_of(PAGE_SIZE) {
            return Err(Errno::Invalid);
        }
        let advice = match advice {
            MADV_NORMAL => VmAdvice::Normal,
            MADV_RANDOM => VmAdvice::Random,
            MADV_SEQUENTIAL => VmAdvice::Sequential,
            // These commands require prefault/discard semantics; accepting
            // them as inert hints would mislead callers about data lifetime.
            MADV_WILLNEED | MADV_DONTNEED => return Err(Errno::NotSupported),
            _ => return Err(Errno::Invalid),
        };
        current_process()?
            .address_space()
            .advise(VirtAddr::new(address), size, advice)
            .map_err(map_memory_error)?;
        Ok(0)
    }
}

fn vm_protection(bits: u64) -> VmProtection {
    let mut protection = VmProtection::empty();
    if bits & PROT_READ != 0 {
        protection |= VmProtection::READ;
    }
    if bits & PROT_WRITE != 0 {
        protection |= VmProtection::WRITE;
    }
    if bits & PROT_EXEC != 0 {
        protection |= VmProtection::EXECUTE;
    }
    protection
}

fn all_user_protections() -> VmProtection {
    VmProtection::READ | VmProtection::WRITE | VmProtection::EXECUTE
}
