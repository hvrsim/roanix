//! Virtual-memory syscall implementations.

use crate::{
    fs::OpenFlags,
    mem::{
        self, PAGE_SIZE, USER_ADDRESS_MIN, VirtAddr, VmAdvice, VmInheritance, VmPlacement,
        VmProtection,
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
        let placement = if flags & MAP_FIXED != 0 {
            if hint < USER_ADDRESS_MIN || !hint.is_multiple_of(PAGE_SIZE) {
                return Err(Errno::Invalid);
            }
            // A fixed mapping replaces whatever occupies the range, so the
            // range is cleared before it is reserved again.
            match space.unmap(VirtAddr::new(hint), size) {
                Ok(()) | Err(mem::Error::NotMapped) => {}
                Err(error) => return Err(map_memory_error(error)),
            }
            VmPlacement::Fixed(VirtAddr::new(hint))
        } else if hint == 0 {
            VmPlacement::Any
        } else {
            VmPlacement::Hint(VirtAddr::new(hint))
        };
        let protection = vm_protection(protection);
        let inheritance = if flags & MAP_SHARED != 0 {
            VmInheritance::Share
        } else {
            VmInheritance::Copy
        };

        let start = if flags & MAP_ANONYMOUS != 0 {
            space
                .map_anonymous(placement, size, protection, protection, inheritance)
                .map_err(map_memory_error)?
        } else {
            let offset = offset as i64;
            if offset < 0 || !(offset as u64).is_multiple_of(PAGE_SIZE) {
                return Err(Errno::Invalid);
            }
            let fd = fd as i32;
            let descriptor = process.descriptor(fd).ok_or(Errno::BadFileDescriptor)?;
            let Descriptor::File(file) = descriptor else {
                return Err(Errno::BadFileDescriptor);
            };
            if protection.contains(VmProtection::WRITE)
                && flags & MAP_SHARED != 0
                && !file.flags().contains(OpenFlags::WRITE)
            {
                return Err(Errno::Access);
            }
            let object = file.vnode().memory_object().map_err(map_fs_error)?;
            space
                .map_object(
                    placement,
                    size,
                    object,
                    offset as u64,
                    protection,
                    protection,
                    inheritance,
                    flags & MAP_PRIVATE != 0,
                )
                .map_err(map_memory_error)?
        };
        Ok(start.as_u64())
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
            MADV_WILLNEED => VmAdvice::WillNeed,
            MADV_DONTNEED => VmAdvice::DontNeed,
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
