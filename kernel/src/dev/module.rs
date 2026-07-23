//! ELF shared-object loading for in-tree kernel drivers.

use alloc::{
    alloc::{alloc_zeroed, dealloc},
    vec,
    vec::Vec,
};
use core::{
    alloc::Layout,
    mem,
    ptr::{self, NonNull},
};

use xmas_elf::{
    ElfFile,
    header::{Class, Data, Machine, Type as ElfType},
    program::{Flags, ProgramHeader64, Type as ProgramType},
};

use crate::{
    arch,
    fs::{self, OpenFlags},
    mem::{PAGE_SIZE, VirtAddr, VmFlags, align_down},
};

use super::{
    Error, Result,
    abi::DriverModule,
};

const MAX_MODULE_FILE_SIZE: usize = 16 * 1024 * 1024;
const MAX_MODULE_IMAGE_SIZE: u64 = 64 * 1024 * 1024;
const ELF64_RELA_SIZE: u64 = 24;

const DT_NULL: i64 = 0;
const DT_NEEDED: i64 = 1;
const DT_RELA: i64 = 7;
const DT_RELASZ: i64 = 8;
const DT_RELAENT: i64 = 9;
const DT_REL: i64 = 17;
const DT_RELSZ: i64 = 18;
const DT_TEXTREL: i64 = 22;
const DT_JMPREL: i64 = 23;
const DT_PLTRELSZ: i64 = 2;
const DT_RELR: i64 = 36;
const DT_RELRSZ: i64 = 35;

#[cfg(target_arch = "x86_64")]
const RELATIVE_RELOCATION: u32 = 8;
#[cfg(target_arch = "riscv64")]
const RELATIVE_RELOCATION: u32 = 3;

#[derive(Copy, Clone)]
struct Segment {
    virtual_address: u64,
    file_offset: u64,
    file_size: u64,
    memory_size: u64,
    flags: Flags,
}

/// Executable mappings owned by one loaded driver.
pub(super) struct ModuleImage {
    allocation: NonNull<u8>,
    layout: Layout,
    minimum_address: u64,
    image_size: u64,
}

// SAFETY: module pages are immutable after relocation except for explicitly
// writable data segments, whose synchronization is the driver's responsibility.
unsafe impl Send for ModuleImage {}
// SAFETY: executable and read-only pages are immutable after publication, and
// writable module state follows the same driver synchronization contract.
unsafe impl Sync for ModuleImage {}

impl ModuleImage {
    fn load_bias(&self) -> Result<u64> {
        (self.allocation.as_ptr() as u64)
            .checked_sub(self.minimum_address)
            .ok_or(Error::InvalidArgument)
    }

    fn address(&self, virtual_address: u64, size: u64) -> Result<usize> {
        let offset = virtual_address
            .checked_sub(self.minimum_address)
            .ok_or(Error::InvalidArgument)?;
        offset
            .checked_add(size)
            .filter(|end| *end <= self.image_size)
            .ok_or(Error::InvalidArgument)?;
        let offset = usize::try_from(offset).map_err(|_| Error::InvalidArgument)?;
        Ok(self.allocation.as_ptr() as usize + offset)
    }

    fn contains_pointer<T>(&self, pointer: *const T) -> bool {
        if pointer.is_null() || !(pointer as usize).is_multiple_of(mem::align_of::<T>()) {
            return false;
        }
        let start = self.allocation.as_ptr() as usize;
        let Some(end) = start.checked_add(self.layout.size()) else {
            return false;
        };
        let address = pointer as usize;
        address >= start
            && address
                .checked_add(mem::size_of::<T>())
                .is_some_and(|value| value <= end)
    }
}

impl Drop for ModuleImage {
    fn drop(&mut self) {
        // SAFETY: this allocation was created with the same layout and remains
        // exclusively owned by the image until drop.
        unsafe { dealloc(self.allocation.as_ptr(), self.layout) };
    }
}

pub(super) fn load(path: &str) -> Result<(ModuleImage, *const DriverModule)> {
    let bytes = read_file(path)?;
    let elf = ElfFile::new(&bytes).map_err(|_| Error::InvalidArgument)?;
    validate_header(&elf)?;
    let (segments, dynamic) = collect_segments(&elf, bytes.len())?;
    let mut image = allocate_image(&segments)?;
    copy_segments(&mut image, &bytes, &segments)?;
    apply_relocations(&image, dynamic)?;
    finalize_permissions(&image, &segments)?;

    let entry_address = image
        .load_bias()?
        .checked_add(elf.header.pt2.entry_point())
        .ok_or(Error::InvalidArgument)?;
    if !segments.iter().any(|segment| {
        segment.flags.is_execute()
            && elf.header.pt2.entry_point() >= segment.virtual_address
            && elf.header.pt2.entry_point()
                < segment.virtual_address.saturating_add(segment.memory_size)
    }) {
        return Err(Error::InvalidArgument);
    }

    crate::mem::synchronize_kernel_mappings();
    arch::sync_instruction_cache();

    type ModuleEntry = unsafe extern "C" fn() -> *const DriverModule;
    // SAFETY: the validated ELF entry lies in an executable module segment and
    // the in-tree module contract fixes this exact function signature.
    let entry: ModuleEntry = unsafe { mem::transmute(entry_address as usize) };
    // SAFETY: upheld by the module entry contract above.
    let descriptor = unsafe { entry() };
    if !image.contains_pointer(descriptor) {
        return Err(Error::InvalidArgument);
    }
    Ok((image, descriptor))
}

fn read_file(path: &str) -> Result<Vec<u8>> {
    let file = fs::open(path, OpenFlags::READ, 0).map_err(map_fs_error)?;
    let size = usize::try_from(file.getattr().map_err(map_fs_error)?.size)
        .map_err(|_| Error::InvalidArgument)?;
    if size == 0 || size > MAX_MODULE_FILE_SIZE {
        return Err(Error::InvalidArgument);
    }
    let mut bytes = vec![0u8; size];
    let mut offset = 0usize;
    while offset < bytes.len() {
        let read = file
            .read_at(offset as u64, &mut bytes[offset..])
            .map_err(map_fs_error)?;
        if read == 0 {
            return Err(Error::InvalidArgument);
        }
        offset += read;
    }
    Ok(bytes)
}

fn validate_header(elf: &ElfFile<'_>) -> Result<()> {
    if elf.header.pt1.class() != Class::SixtyFour
        || elf.header.pt1.data() != Data::LittleEndian
        || elf.header.pt2.type_().as_type() != ElfType::SharedObject
        || elf.header.pt2.machine().as_machine() != expected_machine()
        || elf.header.pt2.ph_entry_size() as usize != mem::size_of::<ProgramHeader64>()
        || elf.header.pt2.entry_point() == 0
    {
        return Err(Error::Unsupported);
    }
    Ok(())
}

fn collect_segments(elf: &ElfFile<'_>, file_size: usize) -> Result<(Vec<Segment>, Option<(u64, u64)>)> {
    let mut segments = Vec::new();
    let mut dynamic = None;
    for index in 0..elf.header.pt2.ph_count() {
        let header = elf
            .program_header(index)
            .map_err(|_| Error::InvalidArgument)?;
        let file_end = header
            .offset()
            .checked_add(header.file_size())
            .filter(|end| *end <= file_size as u64)
            .ok_or(Error::InvalidArgument)?;
        let alignment = header.align();
        if alignment > 1
            && (!alignment.is_power_of_two()
                || header.virtual_addr() % alignment != header.offset() % alignment)
        {
            return Err(Error::InvalidArgument);
        }

        match header.get_type().map_err(|_| Error::InvalidArgument)? {
            ProgramType::Load => {
                if header.file_size() > header.mem_size() {
                    return Err(Error::InvalidArgument);
                }
                let _ = file_end;
                segments.push(Segment {
                    virtual_address: header.virtual_addr(),
                    file_offset: header.offset(),
                    file_size: header.file_size(),
                    memory_size: header.mem_size(),
                    flags: header.flags(),
                });
            }
            ProgramType::Dynamic => {
                if dynamic.replace((header.virtual_addr(), header.mem_size())).is_some() {
                    return Err(Error::InvalidArgument);
                }
            }
            ProgramType::Interp | ProgramType::ShLib => return Err(Error::Unsupported),
            _ => {}
        }
    }
    if segments.is_empty() {
        return Err(Error::InvalidArgument);
    }
    Ok((segments, dynamic))
}

fn allocate_image(segments: &[Segment]) -> Result<ModuleImage> {
    let minimum_address = segments
        .iter()
        .map(|segment| align_down(segment.virtual_address, PAGE_SIZE))
        .min()
        .ok_or(Error::InvalidArgument)?;
    let mut maximum_address = 0u64;
    for segment in segments {
        let end = segment
            .virtual_address
            .checked_add(segment.memory_size)
            .ok_or(Error::InvalidArgument)?;
        maximum_address = maximum_address.max(checked_page_align_up(end)?);
    }
    let image_size = maximum_address
        .checked_sub(minimum_address)
        .filter(|size| *size != 0 && *size <= MAX_MODULE_IMAGE_SIZE)
        .ok_or(Error::InvalidArgument)?;
    let layout = Layout::from_size_align(
        usize::try_from(image_size).map_err(|_| Error::InvalidArgument)?,
        PAGE_SIZE as usize,
    )
    .map_err(|_| Error::InvalidArgument)?;
    // SAFETY: the validated non-zero layout requests page-aligned module pages.
    let allocation = NonNull::new(unsafe { alloc_zeroed(layout) }).ok_or(Error::OutOfMemory)?;
    Ok(ModuleImage {
        allocation,
        layout,
        minimum_address,
        image_size,
    })
}

fn copy_segments(image: &mut ModuleImage, bytes: &[u8], segments: &[Segment]) -> Result<()> {
    for segment in segments {
        if segment.memory_size == 0 || segment.file_size == 0 {
            continue;
        }
        let source_start =
            usize::try_from(segment.file_offset).map_err(|_| Error::InvalidArgument)?;
        let source_end = usize::try_from(
            segment
                .file_offset
                .checked_add(segment.file_size)
                .ok_or(Error::InvalidArgument)?,
        )
        .map_err(|_| Error::InvalidArgument)?;
        let destination = image.address(segment.virtual_address, segment.file_size)?;
        // SAFETY: both ranges were bounds-checked, do not overlap, and the
        // module allocation remains writable throughout relocation.
        unsafe {
            ptr::copy_nonoverlapping(
                bytes[source_start..source_end].as_ptr(),
                destination as *mut u8,
                source_end - source_start,
            );
        }
    }
    Ok(())
}

fn apply_relocations(image: &ModuleImage, dynamic: Option<(u64, u64)>) -> Result<()> {
    let Some((dynamic_address, dynamic_size)) = dynamic else {
        return Ok(());
    };
    if !dynamic_size.is_multiple_of(16) {
        return Err(Error::InvalidArgument);
    }
    let dynamic = image.address(dynamic_address, dynamic_size)?;
    let mut rela_address = None;
    let mut rela_size = 0u64;
    let mut rela_entry_size = ELF64_RELA_SIZE;

    for offset in (0..dynamic_size).step_by(16) {
        let entry = dynamic
            .checked_add(usize::try_from(offset).map_err(|_| Error::InvalidArgument)?)
            .ok_or(Error::InvalidArgument)?;
        // SAFETY: the dynamic table range and fixed entry width were validated.
        let tag = unsafe { ptr::read_unaligned(entry as *const i64) };
        // SAFETY: the value immediately follows the validated tag.
        let value = unsafe { ptr::read_unaligned((entry + 8) as *const u64) };
        match tag {
            DT_NULL => break,
            DT_RELA => rela_address = Some(value),
            DT_RELASZ => rela_size = value,
            DT_RELAENT => rela_entry_size = value,
            DT_NEEDED | DT_REL | DT_RELSZ | DT_TEXTREL | DT_JMPREL | DT_PLTRELSZ | DT_RELR
            | DT_RELRSZ => {
                if value != 0 || matches!(tag, DT_NEEDED | DT_TEXTREL) {
                    return Err(Error::Unsupported);
                }
            }
            _ => {}
        }
    }

    if rela_size == 0 {
        return Ok(());
    }
    if rela_entry_size != ELF64_RELA_SIZE || !rela_size.is_multiple_of(ELF64_RELA_SIZE) {
        return Err(Error::Unsupported);
    }
    let rela_address = rela_address.ok_or(Error::InvalidArgument)?;
    let table = image.address(rela_address, rela_size)?;
    for offset in (0..rela_size).step_by(ELF64_RELA_SIZE as usize) {
        let entry = table
            .checked_add(usize::try_from(offset).map_err(|_| Error::InvalidArgument)?)
            .ok_or(Error::InvalidArgument)?;
        // SAFETY: the complete RELA table was validated inside the image.
        let relocation_offset = unsafe { ptr::read_unaligned(entry as *const u64) };
        // SAFETY: fixed-width field inside the validated RELA entry.
        let info = unsafe { ptr::read_unaligned((entry + 8) as *const u64) };
        // SAFETY: fixed-width field inside the validated RELA entry.
        let addend = unsafe { ptr::read_unaligned((entry + 16) as *const i64) };
        let relocation_type = info as u32;
        let symbol = info >> 32;
        if relocation_type == 0 {
            continue;
        }
        if relocation_type != RELATIVE_RELOCATION || symbol != 0 {
            return Err(Error::Unsupported);
        }
        let target = image.address(relocation_offset, mem::size_of::<u64>() as u64)?;
        let value = add_signed(image.load_bias()?, addend).ok_or(Error::InvalidArgument)?;
        // SAFETY: the relocation target is an eight-byte writable range within
        // the private module image and unaligned stores are explicitly used.
        unsafe { ptr::write_unaligned(target as *mut u64, value) };
    }
    Ok(())
}

fn finalize_permissions(image: &ModuleImage, segments: &[Segment]) -> Result<()> {
    let page_count = image.layout.size() / PAGE_SIZE as usize;
    let mut permissions = vec![VmFlags::READ; page_count];
    for segment in segments {
        if segment.memory_size == 0 {
            continue;
        }
        let start = align_down(segment.virtual_address, PAGE_SIZE);
        let end = checked_page_align_up(
            segment
                .virtual_address
                .checked_add(segment.memory_size)
                .ok_or(Error::InvalidArgument)?,
        )?;
        let mut flags = VmFlags::READ;
        if segment.flags.is_write() {
            flags |= VmFlags::WRITE;
        }
        if segment.flags.is_execute() {
            flags |= VmFlags::EXECUTE;
        }
        if flags.contains(VmFlags::WRITE | VmFlags::EXECUTE) {
            return Err(Error::Unsupported);
        }
        let first =
            usize::try_from((start - image.minimum_address) / PAGE_SIZE).map_err(|_| Error::InvalidArgument)?;
        let last =
            usize::try_from((end - image.minimum_address) / PAGE_SIZE).map_err(|_| Error::InvalidArgument)?;
        for permission in permissions.get_mut(first..last).ok_or(Error::InvalidArgument)? {
            *permission |= flags;
            if permission.contains(VmFlags::WRITE | VmFlags::EXECUTE) {
                return Err(Error::Unsupported);
            }
        }
    }

    let root = arch::paging::active_root();
    for (index, flags) in permissions.into_iter().enumerate() {
        let address = (image.allocation.as_ptr() as usize)
            .checked_add(index * PAGE_SIZE as usize)
            .ok_or(Error::InvalidArgument)?;
        let virtual_address = VirtAddr::new(address as u64);
        // SAFETY: the module allocation owns every mapped heap page in this
        // range and remains unpublished while permissions are finalized.
        let physical = unsafe { arch::paging::translate(root, virtual_address) }
            .ok_or(Error::InvalidArgument)?
            .align_down();
        // SAFETY: this remaps the same owned physical page at the same virtual
        // address, changing only its final W^X permissions.
        unsafe { arch::paging::remap_page(root, virtual_address, physical, flags) }
            .map_err(|_| Error::OutOfMemory)?;
    }
    Ok(())
}

fn add_signed(base: u64, addend: i64) -> Option<u64> {
    if addend >= 0 {
        base.checked_add(addend as u64)
    } else {
        base.checked_sub(addend.unsigned_abs())
    }
}

fn checked_page_align_up(value: u64) -> Result<u64> {
    value
        .checked_add(PAGE_SIZE - 1)
        .map(|value| value & !(PAGE_SIZE - 1))
        .ok_or(Error::InvalidArgument)
}

fn map_fs_error(error: fs::Error) -> Error {
    match error {
        fs::Error::NotFound => Error::NotFound,
        fs::Error::OutOfMemory => Error::OutOfMemory,
        fs::Error::PermissionDenied => Error::PermissionDenied,
        _ => Error::Filesystem,
    }
}

#[cfg(target_arch = "x86_64")]
fn expected_machine() -> Machine {
    Machine::X86_64
}

#[cfg(target_arch = "riscv64")]
fn expected_machine() -> Machine {
    Machine::RISC_V
}
