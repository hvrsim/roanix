//! Module image loading.
//!
//! A module is an ELF shared object with a single entry point. The loader
//! accepts only self-contained images: no external symbol references, no text
//! relocations, and no dependence on a runtime linker. That restriction is what
//! removes the need for a driver support library - everything a module needs
//! either comes from the service table or is inlined by its own compiler.
//!
//! Segments are placed in kernel memory, position-independent relocations are
//! applied, and permissions are tightened so that no page is both writable and
//! executable before the entry point runs.

use alloc::{
    alloc::{alloc_zeroed, dealloc},
    boxed::Box,
    string::String,
    sync::Arc,
    vec,
    vec::Vec,
};
use core::{
    alloc::Layout,
    mem,
    ptr::{self, NonNull},
};

use log::{debug, error};

use xmas_elf::{
    ElfFile,
    header::{Class, Data, Machine, Type as ElfType},
    program::{Flags, ProgramHeader64, Type as ProgramType},
};

use crate::{
    arch,
    fs::{self, OpenFlags, VnodeKind},
    mem::{PAGE_SIZE, VirtAddr, VmFlags, align_down},
};

use super::{
    super::{
        core::module::{self, Module, ModuleBacking, ModuleDefinition},
        error::{Error, Result},
    },
    api::API,
    types::{ABI_MAJOR, ABI_MINOR, ModuleDef, ModuleEntryFn, borrow_opt_str, borrow_str},
};

/// File name suffix identifying a module image.
pub const MODULE_SUFFIX: &[u8] = b".ko";

const MAX_FILE_SIZE: usize = 16 * 1024 * 1024;
const MAX_IMAGE_SIZE: u64 = 64 * 1024 * 1024;
const ELF64_RELA_SIZE: u64 = 24;

const DT_NULL: i64 = 0;
const DT_NEEDED: i64 = 1;
const DT_PLTRELSZ: i64 = 2;
const DT_RELA: i64 = 7;
const DT_RELASZ: i64 = 8;
const DT_RELAENT: i64 = 9;
const DT_REL: i64 = 17;
const DT_RELSZ: i64 = 18;
const DT_TEXTREL: i64 = 22;
const DT_JMPREL: i64 = 23;
const DT_RELRSZ: i64 = 35;
const DT_RELR: i64 = 36;

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

/// Resident pages holding one module's code and data.
pub struct Image {
    allocation: NonNull<u8>,
    layout: Layout,
    minimum_address: u64,
    image_size: u64,
}

// SAFETY: module pages are immutable after relocation except for writable data
// segments, whose synchronization is the module's own responsibility.
unsafe impl Send for Image {}
// SAFETY: executable and read-only pages never change after publication.
unsafe impl Sync for Image {}

impl ModuleBacking for Image {}

impl Image {
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

    fn contains<T>(&self, pointer: *const T) -> bool {
        if pointer.is_null() || !(pointer as usize).is_multiple_of(mem::align_of::<T>()) {
            return false;
        }
        self.contains_range(pointer as usize, mem::size_of::<T>())
    }

    fn contains_range(&self, address: usize, length: usize) -> bool {
        let start = self.allocation.as_ptr() as usize;
        let Some(end) = start.checked_add(self.layout.size()) else {
            return false;
        };
        address >= start && address.checked_add(length).is_some_and(|value| value <= end)
    }

    /// Returns whether a NUL-terminated string lies entirely inside the image.
    fn contains_string(&self, pointer: *const core::ffi::c_char) -> bool {
        if pointer.is_null() {
            return true;
        }
        let start = self.allocation.as_ptr() as usize;
        let end = start + self.layout.size();
        let mut address = pointer as usize;
        if address < start || address >= end {
            return false;
        }
        while address < end {
            // SAFETY: `address` was just bounds-checked against the image.
            if unsafe { *(address as *const u8) } == 0 {
                return true;
            }
            address += 1;
        }
        false
    }
}

impl Drop for Image {
    fn drop(&mut self) {
        // SAFETY: the allocation was created with this layout and is owned
        // exclusively by the image until it is dropped.
        unsafe { dealloc(self.allocation.as_ptr(), self.layout) };
    }
}

/// Loads and starts every module image in `path`, in filename order.
///
/// A module that fails to load is reported and skipped: one broken driver must
/// not stop the rest of the system from coming up.
pub fn load_directory(path: &[u8]) -> Result<usize> {
    let directory = fs::open(
        path,
        OpenFlags::READ | OpenFlags::DIRECTORY | OpenFlags::NOFOLLOW,
        0,
    )?;
    let mut names = Vec::new();
    loop {
        let entries = directory.readdir(64)?;
        if entries.is_empty() {
            break;
        }
        for entry in entries {
            if entry.kind != VnodeKind::Regular || !entry.name.ends_with(MODULE_SUFFIX) {
                continue;
            }
            let name = core::str::from_utf8(&entry.name).map_err(|_| Error::InvalidArgument)?;
            names.push(String::from(name));
        }
    }
    names.sort_unstable();

    let mut directory = path;
    while directory.len() > 1 && directory.last() == Some(&b'/') {
        directory = &directory[..directory.len() - 1];
    }
    let mut loaded = 0usize;
    for name in &names {
        let mut full = directory.to_vec();
        full.push(b'/');
        full.extend_from_slice(name.as_bytes());
        match load_file(&full) {
            Ok(module) => {
                debug!("loaded module {}", module.name());
                loaded += 1;
            }
            Err(failure) => error!("failed to load module {name}: {failure:?}"),
        }
    }
    super::super::core::probe::retrigger();
    Ok(loaded)
}

/// Loads and starts one module image.
pub fn load_file(path: &[u8]) -> Result<Arc<Module>> {
    let definition = prepare(path)?;
    // SAFETY: `prepare` validated that both callbacks lie inside the image, and
    // the image is owned by the module for as long as it stays loaded.
    unsafe { module::load(definition) }
}

fn prepare(path: &[u8]) -> Result<ModuleDefinition> {
    let bytes = read_file(path)?;
    let elf = ElfFile::new(&bytes).map_err(|_| Error::InvalidArgument)?;
    validate_header(&elf)?;
    let (segments, dynamic) = collect_segments(&elf, bytes.len())?;
    let mut image = allocate_image(&segments)?;
    copy_segments(&mut image, &bytes, &segments)?;
    apply_relocations(&image, dynamic)?;
    finalize_permissions(&image, &segments)?;

    let entry_point = elf.header.pt2.entry_point();
    if !segments.iter().any(|segment| {
        segment.flags.is_execute()
            && entry_point >= segment.virtual_address
            && entry_point < segment.virtual_address.saturating_add(segment.memory_size)
    }) {
        return Err(Error::InvalidArgument);
    }
    let entry_address = image
        .load_bias()?
        .checked_add(entry_point)
        .ok_or(Error::InvalidArgument)?;

    crate::mem::synchronize_kernel_mappings();
    arch::sync_instruction_cache();

    // SAFETY: the entry address lies inside an executable segment and the
    // module contract fixes this exact signature.
    let entry: ModuleEntryFn = unsafe { mem::transmute(entry_address as usize) };
    // SAFETY: upheld by the module entry contract above. The service table is
    // static and immutable.
    let descriptor = unsafe { entry(&raw const API) };
    if !image.contains(descriptor) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the descriptor pointer was just verified to lie inside the image.
    let definition = validate_descriptor(&image, unsafe { &*descriptor })?;
    Ok(ModuleDefinition {
        backing: Some(Box::new(image)),
        ..definition
    })
}

fn validate_descriptor(image: &Image, descriptor: &ModuleDef) -> Result<ModuleDefinition> {
    if (descriptor.size as usize) < mem::size_of::<ModuleDef>() {
        return Err(Error::InvalidArgument);
    }
    if descriptor.abi_major != ABI_MAJOR
        || descriptor.abi_minor > ABI_MINOR
        || descriptor.flags != 0
    {
        return Err(Error::Unsupported);
    }
    if !image.contains_string(descriptor.name) || !image.contains_string(descriptor.description) {
        return Err(Error::InvalidArgument);
    }
    let init = descriptor.init.ok_or(Error::InvalidArgument)?;
    if !image.contains_range(init as usize, 1) {
        return Err(Error::InvalidArgument);
    }
    if let Some(exit) = descriptor.exit
        && !image.contains_range(exit as usize, 1)
    {
        return Err(Error::InvalidArgument);
    }

    // SAFETY: both strings were verified to be NUL-terminated inside the image.
    let name = unsafe { borrow_str(descriptor.name) }?;
    // SAFETY: as above.
    let description = unsafe { borrow_opt_str(descriptor.description) }?;
    Ok(ModuleDefinition {
        name: String::from(name).into_boxed_str(),
        description: description.map(|text| String::from(text).into_boxed_str()),
        init,
        exit: descriptor.exit,
        backing: None,
    })
}

fn read_file(path: &[u8]) -> Result<Vec<u8>> {
    let file = fs::open(path, OpenFlags::READ, 0)?;
    let size = usize::try_from(file.getattr()?.size).map_err(|_| Error::InvalidArgument)?;
    if size == 0 || size > MAX_FILE_SIZE {
        return Err(Error::InvalidArgument);
    }
    let mut bytes = vec![0u8; size];
    let mut offset = 0usize;
    while offset < bytes.len() {
        let read = file.read_at(
            offset as u64,
            &mut crate::mem::IoSink::kernel(&mut bytes[offset..]),
        )?;
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

fn collect_segments(
    elf: &ElfFile<'_>,
    file_size: usize,
) -> Result<(Vec<Segment>, Option<(u64, u64)>)> {
    let mut segments = Vec::new();
    let mut dynamic = None;
    for index in 0..elf.header.pt2.ph_count() {
        let header = elf
            .program_header(index)
            .map_err(|_| Error::InvalidArgument)?;
        header
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
                segments.push(Segment {
                    virtual_address: header.virtual_addr(),
                    file_offset: header.offset(),
                    file_size: header.file_size(),
                    memory_size: header.mem_size(),
                    flags: header.flags(),
                });
            }
            ProgramType::Dynamic => {
                if dynamic
                    .replace((header.virtual_addr(), header.mem_size()))
                    .is_some()
                {
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

fn allocate_image(segments: &[Segment]) -> Result<Image> {
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
        maximum_address = maximum_address.max(page_align_up(end)?);
    }
    let image_size = maximum_address
        .checked_sub(minimum_address)
        .filter(|size| *size != 0 && *size <= MAX_IMAGE_SIZE)
        .ok_or(Error::InvalidArgument)?;
    let layout = Layout::from_size_align(
        usize::try_from(image_size).map_err(|_| Error::InvalidArgument)?,
        PAGE_SIZE as usize,
    )
    .map_err(|_| Error::InvalidArgument)?;
    // SAFETY: the layout has a non-zero size and page alignment.
    let allocation = NonNull::new(unsafe { alloc_zeroed(layout) }).ok_or(Error::OutOfMemory)?;
    Ok(Image {
        allocation,
        layout,
        minimum_address,
        image_size,
    })
}

fn copy_segments(image: &mut Image, bytes: &[u8], segments: &[Segment]) -> Result<()> {
    for segment in segments {
        if segment.memory_size == 0 || segment.file_size == 0 {
            continue;
        }
        let start = usize::try_from(segment.file_offset).map_err(|_| Error::InvalidArgument)?;
        let end = usize::try_from(
            segment
                .file_offset
                .checked_add(segment.file_size)
                .ok_or(Error::InvalidArgument)?,
        )
        .map_err(|_| Error::InvalidArgument)?;
        let destination = image.address(segment.virtual_address, segment.file_size)?;
        // SAFETY: both ranges were bounds-checked, they cannot overlap because
        // the image is a fresh allocation, and it is still writable.
        unsafe {
            ptr::copy_nonoverlapping(
                bytes[start..end].as_ptr(),
                destination as *mut u8,
                end - start,
            );
        }
    }
    Ok(())
}

fn apply_relocations(image: &Image, dynamic: Option<(u64, u64)>) -> Result<()> {
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
        // SAFETY: the dynamic table range and its fixed entry width were
        // validated against the image.
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
        // SAFETY: the whole relocation table was validated inside the image.
        let relocation_offset = unsafe { ptr::read_unaligned(entry as *const u64) };
        // SAFETY: fixed-width field inside the validated entry.
        let info = unsafe { ptr::read_unaligned((entry + 8) as *const u64) };
        // SAFETY: fixed-width field inside the validated entry.
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
        // SAFETY: the target is an eight-byte writable range inside the private
        // image, and the store is explicitly unaligned.
        unsafe { ptr::write_unaligned(target as *mut u64, value) };
    }
    Ok(())
}

fn finalize_permissions(image: &Image, segments: &[Segment]) -> Result<()> {
    let page_count = image.layout.size() / PAGE_SIZE as usize;
    let mut permissions = vec![VmFlags::READ; page_count];
    for segment in segments {
        if segment.memory_size == 0 {
            continue;
        }
        let start = align_down(segment.virtual_address, PAGE_SIZE);
        let end = page_align_up(
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
        let first = usize::try_from((start - image.minimum_address) / PAGE_SIZE)
            .map_err(|_| Error::InvalidArgument)?;
        let last = usize::try_from((end - image.minimum_address) / PAGE_SIZE)
            .map_err(|_| Error::InvalidArgument)?;
        for permission in permissions
            .get_mut(first..last)
            .ok_or(Error::InvalidArgument)?
        {
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
        // SAFETY: the image owns every page in this range and is not yet
        // published, so no other mapping observes the change.
        let physical = unsafe { arch::paging::translate(root, virtual_address) }
            .ok_or(Error::InvalidArgument)?
            .align_down();
        // SAFETY: this remaps the same owned physical page at the same address,
        // changing only its final permissions.
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

fn page_align_up(value: u64) -> Result<u64> {
    value
        .checked_add(PAGE_SIZE - 1)
        .map(|value| value & !(PAGE_SIZE - 1))
        .ok_or(Error::InvalidArgument)
}

#[cfg(target_arch = "x86_64")]
fn expected_machine() -> Machine {
    Machine::X86_64
}

#[cfg(target_arch = "riscv64")]
fn expected_machine() -> Machine {
    Machine::RISC_V
}
