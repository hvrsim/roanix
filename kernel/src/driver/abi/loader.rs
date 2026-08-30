//! Module image loading.
//!
//! A module is an ELF shared object with a single entry point. The loader
//! accepts only self-contained images: no runtime-library dependencies, no text
//! relocations, and no general symbol lookup. The only external references it
//! resolves are versioned names in the kernel C ABI export list.
//!
//! Segments are placed in kernel memory, position-independent and import
//! relocations are applied, and permissions are tightened so that no page is
//! both writable and executable before the entry point runs.

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
    slice,
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
    types::{ABI_MAJOR, ABI_MINOR, ModuleDef, ModuleEntryFn, borrow_opt_str, borrow_str},
};

/// File name suffix identifying a module image.
pub const MODULE_SUFFIX: &[u8] = b".ko";

const MAX_FILE_SIZE: usize = 16 * 1024 * 1024;
const MAX_IMAGE_SIZE: u64 = 64 * 1024 * 1024;
const ELF64_DYN_SIZE: u64 = 16;
const ELF64_RELA_SIZE: u64 = 24;
const ELF64_SYM_SIZE: u64 = 24;

const DT_NULL: i64 = 0;
const DT_NEEDED: i64 = 1;
const DT_PLTRELSZ: i64 = 2;
const DT_HASH: i64 = 4;
const DT_STRTAB: i64 = 5;
const DT_SYMTAB: i64 = 6;
const DT_RELA: i64 = 7;
const DT_RELASZ: i64 = 8;
const DT_RELAENT: i64 = 9;
const DT_STRSZ: i64 = 10;
const DT_SYMENT: i64 = 11;
const DT_REL: i64 = 17;
const DT_RELSZ: i64 = 18;
const DT_PLTREL: i64 = 20;
const DT_TEXTREL: i64 = 22;
const DT_JMPREL: i64 = 23;
const DT_RELRSZ: i64 = 35;
const DT_RELR: i64 = 36;

#[cfg(target_arch = "x86_64")]
const RELATIVE_RELOCATION: u32 = 8;
#[cfg(target_arch = "riscv64")]
const RELATIVE_RELOCATION: u32 = 3;

#[cfg(target_arch = "x86_64")]
const ABSOLUTE_RELOCATION: u32 = 1;
#[cfg(target_arch = "x86_64")]
const GLOBAL_DATA_RELOCATION: u32 = 6;
#[cfg(target_arch = "x86_64")]
const JUMP_SLOT_RELOCATION: u32 = 7;
#[cfg(target_arch = "riscv64")]
const ABSOLUTE_RELOCATION: u32 = 2;
#[cfg(target_arch = "riscv64")]
const JUMP_SLOT_RELOCATION: u32 = 5;

#[derive(Copy, Clone)]
struct Segment {
    virtual_address: u64,
    file_offset: u64,
    file_size: u64,
    memory_size: u64,
    flags: Flags,
}

#[derive(Default)]
struct DynamicInfo {
    rela_address: Option<u64>,
    rela_size: u64,
    rela_entry_size: u64,
    plt_rela_address: Option<u64>,
    plt_rela_size: u64,
    plt_rela_type: Option<u64>,
    symbol_table: Option<u64>,
    symbol_entry_size: u64,
    string_table: Option<u64>,
    string_table_size: u64,
    hash_table: Option<u64>,
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
        address >= start
            && address
                .checked_add(length)
                .is_some_and(|value| value <= end)
    }

    fn contains_executable(&self, segments: &[Segment], address: usize) -> bool {
        segments.iter().any(|segment| {
            if !segment.flags.is_execute() || segment.memory_size == 0 {
                return false;
            }
            let Ok(start) = self.address(segment.virtual_address, segment.memory_size) else {
                return false;
            };
            let Ok(length) = usize::try_from(segment.memory_size) else {
                return false;
            };
            start
                .checked_add(length)
                .is_some_and(|end| address >= start && address < end)
        })
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
            Err(Error::AlreadyExists) => {
                debug!("module {name} was loaded during early boot; skipping packaged copy");
            }
            Err(failure) => error!("failed to load module {name}: {failure:?}"),
        }
    }
    super::super::core::probe::retrigger();
    Ok(loaded)
}

/// Loads and starts one module image.
pub fn load_file(path: &[u8]) -> Result<Arc<Module>> {
    let bytes = read_file(path)?;
    load_bytes(&bytes)
}

/// Loads and starts one module image already resident in memory.
///
/// The image is copied into resident module memory before this function
/// returns, so the caller may release its input storage after the load.
pub fn load_bytes(bytes: &[u8]) -> Result<Arc<Module>> {
    let definition = prepare_bytes(bytes)?;
    // SAFETY: `prepare_bytes` validated that both callbacks lie inside the
    // image, and the image is owned by the module for as long as it stays
    // loaded.
    unsafe { module::load(definition) }
}

fn prepare_bytes(bytes: &[u8]) -> Result<ModuleDefinition> {
    if bytes.is_empty() || bytes.len() > MAX_FILE_SIZE {
        return Err(Error::InvalidArgument);
    }
    let elf = ElfFile::new(bytes).map_err(|_| Error::InvalidArgument)?;
    validate_header(&elf)?;
    let (segments, dynamic) = collect_segments(&elf, bytes.len())?;
    let mut image = allocate_image(&segments)?;
    copy_segments(&mut image, bytes, &segments)?;
    apply_relocations(&image, &segments, dynamic)?;
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
    // SAFETY: upheld by the module entry contract above. The reserved argument
    // remains null so legacy table-based modules reject themselves safely.
    let descriptor = unsafe { entry(ptr::null()) };
    if !image.contains(descriptor) {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the descriptor pointer was just verified to lie inside the image.
    let definition = validate_descriptor(&image, &segments, unsafe { &*descriptor })?;
    Ok(ModuleDefinition {
        backing: Some(Box::new(image)),
        ..definition
    })
}

fn validate_descriptor(
    image: &Image,
    segments: &[Segment],
    descriptor: &ModuleDef,
) -> Result<ModuleDefinition> {
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
    if !image.contains_executable(segments, init as usize) {
        return Err(Error::InvalidArgument);
    }
    if let Some(exit) = descriptor.exit
        && !image.contains_executable(segments, exit as usize)
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

type DynamicRange = Option<(u64, u64)>;

fn collect_segments(elf: &ElfFile<'_>, file_size: usize) -> Result<(Vec<Segment>, DynamicRange)> {
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

fn apply_relocations(
    image: &Image,
    segments: &[Segment],
    dynamic: Option<(u64, u64)>,
) -> Result<()> {
    let Some((dynamic_address, dynamic_size)) = dynamic else {
        return Ok(());
    };
    if !dynamic_size.is_multiple_of(ELF64_DYN_SIZE) {
        return Err(Error::InvalidArgument);
    }
    let dynamic = image.address(dynamic_address, dynamic_size)?;
    let mut info = DynamicInfo {
        rela_entry_size: ELF64_RELA_SIZE,
        symbol_entry_size: ELF64_SYM_SIZE,
        ..DynamicInfo::default()
    };

    for offset in (0..dynamic_size).step_by(ELF64_DYN_SIZE as usize) {
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
            DT_RELA => info.rela_address = Some(value),
            DT_RELASZ => info.rela_size = value,
            DT_RELAENT => info.rela_entry_size = value,
            DT_JMPREL => info.plt_rela_address = Some(value),
            DT_PLTRELSZ => info.plt_rela_size = value,
            DT_PLTREL => info.plt_rela_type = Some(value),
            DT_SYMTAB => info.symbol_table = Some(value),
            DT_SYMENT => info.symbol_entry_size = value,
            DT_STRTAB => info.string_table = Some(value),
            DT_STRSZ => info.string_table_size = value,
            DT_HASH => info.hash_table = Some(value),
            DT_NEEDED | DT_REL | DT_RELSZ | DT_TEXTREL | DT_RELR | DT_RELRSZ => {
                return Err(Error::Unsupported);
            }
            _ => {}
        }
    }

    apply_rela_table(image, segments, &info, info.rela_address, info.rela_size)?;
    if info.plt_rela_size != 0 {
        if info.plt_rela_type != Some(DT_RELA as u64) {
            return Err(Error::Unsupported);
        }
        apply_rela_table(
            image,
            segments,
            &info,
            info.plt_rela_address,
            info.plt_rela_size,
        )?;
    } else if info.plt_rela_address.is_some() || info.plt_rela_type.is_some() {
        return Err(Error::InvalidArgument);
    }

    Ok(())
}

fn apply_rela_table(
    image: &Image,
    segments: &[Segment],
    dynamic: &DynamicInfo,
    address: Option<u64>,
    size: u64,
) -> Result<()> {
    if size == 0 {
        return Ok(());
    }
    if dynamic.rela_entry_size != ELF64_RELA_SIZE || !size.is_multiple_of(ELF64_RELA_SIZE) {
        return Err(Error::Unsupported);
    }
    let address = address.ok_or(Error::InvalidArgument)?;
    let table = image.address(address, size)?;
    for offset in (0..size).step_by(ELF64_RELA_SIZE as usize) {
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
        if !relocation_target_is_writable(segments, relocation_offset) {
            return Err(Error::Unsupported);
        }
        let target = image.address(relocation_offset, mem::size_of::<u64>() as u64)?;
        let value = if relocation_type == RELATIVE_RELOCATION && symbol == 0 {
            add_signed(image.load_bias()?, addend).ok_or(Error::InvalidArgument)?
        } else if is_import_relocation(relocation_type) && symbol != 0 {
            let export = resolve_import_symbol(image, dynamic, symbol)?;
            add_signed(export as u64, addend).ok_or(Error::InvalidArgument)?
        } else {
            return Err(Error::Unsupported);
        };
        // SAFETY: the target is an eight-byte writable range inside the private
        // image, and the store is explicitly unaligned.
        unsafe { ptr::write_unaligned(target as *mut u64, value) };
    }
    Ok(())
}

fn relocation_target_is_writable(segments: &[Segment], address: u64) -> bool {
    let Some(end) = address.checked_add(mem::size_of::<u64>() as u64) else {
        return false;
    };
    segments.iter().any(|segment| {
        if !segment.flags.is_write() {
            return false;
        }
        let Some(segment_end) = segment.virtual_address.checked_add(segment.memory_size) else {
            return false;
        };
        address >= segment.virtual_address && end <= segment_end
    })
}

fn is_import_relocation(relocation: u32) -> bool {
    relocation == ABSOLUTE_RELOCATION || relocation == JUMP_SLOT_RELOCATION || {
        #[cfg(target_arch = "x86_64")]
        {
            relocation == GLOBAL_DATA_RELOCATION
        }
        #[cfg(target_arch = "riscv64")]
        {
            false
        }
    }
}

fn resolve_import_symbol(image: &Image, dynamic: &DynamicInfo, index: u64) -> Result<usize> {
    if dynamic.symbol_entry_size != ELF64_SYM_SIZE {
        return Err(Error::Unsupported);
    }
    let symbol_table = dynamic.symbol_table.ok_or(Error::Unsupported)?;
    let hash_table = dynamic.hash_table.ok_or(Error::Unsupported)?;
    let hash = image.address(hash_table, 8)?;
    // SAFETY: the two fields lie inside the validated module image.
    let symbol_count = unsafe { ptr::read_unaligned((hash + 4) as *const u32) } as u64;
    if index >= symbol_count {
        return Err(Error::InvalidArgument);
    }
    let symbol_offset = index
        .checked_mul(ELF64_SYM_SIZE)
        .and_then(|offset| symbol_table.checked_add(offset))
        .ok_or(Error::InvalidArgument)?;
    let symbol = image.address(symbol_offset, ELF64_SYM_SIZE)?;
    // SAFETY: the fixed-width ELF symbol record lies inside the image.
    let name_offset = unsafe { ptr::read_unaligned(symbol as *const u32) } as u64;
    // SAFETY: `st_info` is the one-byte field at offset four in Elf64_Sym.
    let info = unsafe { ptr::read_unaligned((symbol + 4) as *const u8) };
    // SAFETY: `st_shndx` is the two-byte field at offset six in Elf64_Sym.
    let section = unsafe { ptr::read_unaligned((symbol + 6) as *const u16) };
    let binding = info >> 4;
    // Rust's linker emits `STT_NOTYPE` for an `extern "C"` function
    // declaration, so the versioned export name—not the optional ELF type—is
    // authoritative here.
    if section != 0 || !matches!(binding, 1 | 2) {
        return Err(Error::Unsupported);
    }
    let string_table = dynamic.string_table.ok_or(Error::Unsupported)?;
    if name_offset >= dynamic.string_table_size {
        return Err(Error::InvalidArgument);
    }
    let remaining = dynamic
        .string_table_size
        .checked_sub(name_offset)
        .ok_or(Error::InvalidArgument)?;
    let string_address = string_table
        .checked_add(name_offset)
        .ok_or(Error::InvalidArgument)?;
    let string = image.address(string_address, remaining)?;
    let length = usize::try_from(remaining).map_err(|_| Error::InvalidArgument)?;
    // SAFETY: `string` spans the checked string-table tail in the module image.
    let bytes = unsafe { slice::from_raw_parts(string as *const u8, length) };
    let name_end = bytes
        .iter()
        .position(|byte| *byte == 0)
        .ok_or(Error::InvalidArgument)?;
    super::api::resolve_import(&bytes[..name_end]).ok_or(Error::Unsupported)
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
