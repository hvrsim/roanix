//! Process ELF loading and initial userspace stack construction.

use alloc::{string::String, sync::Arc, vec, vec::Vec};
use core::{mem, str};

use xmas_elf::{
    ElfFile,
    header::{Class, Data, Machine, Type as ElfType},
    program::{Flags, ProgramHeader64, Type as ProgramType},
};

use crate::{
    fs::{self, OpenFlags},
    mem::{
        ObjectKind, PAGE_SIZE, USER_ADDRESS_MAX, VirtAddr, VmInheritance, VmObject, VmProtection,
        VmSpace, align_down,
    },
};

use super::{Error, Result};

const INTERPRETER_BASE: u64 = 0x4000_0000;
const PIE_BASE: u64 = 0x2000_0000;
const STACK_SIZE: u64 = 1024 * 1024;
const STACK_TOP: u64 = USER_ADDRESS_MAX - PAGE_SIZE;

const AT_NULL: u64 = 0;
const AT_PHDR: u64 = 3;
const AT_PHENT: u64 = 4;
const AT_PHNUM: u64 = 5;
const AT_PAGESZ: u64 = 6;
const AT_BASE: u64 = 7;
const AT_ENTRY: u64 = 9;
const AT_UID: u64 = 11;
const AT_EUID: u64 = 12;
const AT_GID: u64 = 13;
const AT_EGID: u64 = 14;
const AT_SECURE: u64 = 23;
const AT_RANDOM: u64 = 25;
const AT_EXECFN: u64 = 31;

pub(super) struct LoadedProgram {
    pub address_space: Arc<VmSpace>,
    pub entry: u64,
    pub stack: u64,
}

struct Segment {
    virtual_address: u64,
    file_offset: u64,
    file_size: u64,
    memory_size: u64,
    flags: Flags,
}

struct ElfSpec {
    image_type: ElfType,
    entry: u64,
    program_header_address: Option<u64>,
    program_header_offset: u64,
    program_header_size: u16,
    program_header_count: u16,
    interpreter: Option<String>,
    segments: Vec<Segment>,
}

struct LoadedElf {
    entry: u64,
    bias: u64,
    program_headers: u64,
    program_header_size: u16,
    program_header_count: u16,
}

pub(super) fn load<A: AsRef<[u8]>, E: AsRef<[u8]>>(
    path: &[u8],
    arguments: &[A],
    environment: &[E],
) -> Result<LoadedProgram> {
    let address_space = VmSpace::new()?;
    let main_bytes = read_file(path)?;
    let main_spec = parse_elf(&main_bytes)?;
    let main_base = if main_spec.image_type == ElfType::SharedObject {
        PIE_BASE
    } else {
        0
    };
    let main = map_elf(&address_space, &main_bytes, &main_spec, main_base)?;

    let (entry, interpreter_base) = if let Some(interpreter) = &main_spec.interpreter {
        let interpreter_bytes = read_file(interpreter.as_bytes())?;
        let interpreter_spec = parse_elf(&interpreter_bytes)?;
        if interpreter_spec.image_type != ElfType::SharedObject {
            return Err(Error::UnsupportedElf);
        }
        let interpreter = map_elf(
            &address_space,
            &interpreter_bytes,
            &interpreter_spec,
            INTERPRETER_BASE,
        )?;
        (interpreter.entry, interpreter.bias)
    } else {
        (main.entry, 0)
    };

    let argument_refs: Vec<_> = arguments.iter().map(|value| value.as_ref()).collect();
    let environment_refs: Vec<_> = environment.iter().map(|value| value.as_ref()).collect();
    let stack = build_stack(
        &address_space,
        path,
        &argument_refs,
        &environment_refs,
        &main,
        interpreter_base,
    )?;
    Ok(LoadedProgram {
        address_space,
        entry,
        stack,
    })
}

fn read_file(path: &[u8]) -> Result<Vec<u8>> {
    let file = fs::open(path, OpenFlags::READ, 0)?;
    let size =
        usize::try_from(file.getattr()?.size).map_err(|_| Error::InvalidElf("file too large"))?;
    let mut bytes = vec![0u8; size];
    let mut offset = 0usize;
    while offset < bytes.len() {
        let read = file.read_at(
            offset as u64,
            &mut crate::mem::IoSink::kernel(&mut bytes[offset..]),
        )?;
        if read == 0 {
            return Err(Error::InvalidElf("unexpected end of file"));
        }
        offset += read;
    }
    Ok(bytes)
}

fn parse_elf(bytes: &[u8]) -> Result<ElfSpec> {
    let elf = ElfFile::new(bytes).map_err(Error::InvalidElf)?;
    if elf.header.pt1.class() != Class::SixtyFour
        || elf.header.pt1.data() != Data::LittleEndian
        || elf.header.pt2.ph_entry_size() as usize != mem::size_of::<ProgramHeader64>()
    {
        return Err(Error::UnsupportedElf);
    }
    if elf.header.pt2.machine().as_machine() != expected_machine() {
        return Err(Error::UnsupportedElf);
    }

    let image_type = elf.header.pt2.type_().as_type();
    if image_type != ElfType::Executable && image_type != ElfType::SharedObject {
        return Err(Error::UnsupportedElf);
    }

    let program_table_size = u64::from(elf.header.pt2.ph_entry_size())
        .checked_mul(u64::from(elf.header.pt2.ph_count()))
        .ok_or(Error::InvalidElf("program header table overflow"))?;
    let program_table_end = elf
        .header
        .pt2
        .ph_offset()
        .checked_add(program_table_size)
        .ok_or(Error::InvalidElf("program header table overflow"))?;
    if program_table_end > bytes.len() as u64 {
        return Err(Error::InvalidElf("program header table outside file"));
    }

    let mut segments = Vec::new();
    let mut interpreter = None;
    let mut program_header_address = None;
    for index in 0..elf.header.pt2.ph_count() {
        let header = elf.program_header(index).map_err(Error::InvalidElf)?;
        let file_end = header
            .offset()
            .checked_add(header.file_size())
            .ok_or(Error::InvalidElf("segment file range overflow"))?;
        if file_end > bytes.len() as u64 {
            return Err(Error::InvalidElf("segment data outside file"));
        }
        let alignment = header.align();
        if alignment > 1
            && (!alignment.is_power_of_two()
                || header.virtual_addr() % alignment != header.offset() % alignment)
        {
            return Err(Error::InvalidElf("invalid segment alignment"));
        }

        match header.get_type().map_err(Error::InvalidElf)? {
            ProgramType::Load => {
                if header.file_size() > header.mem_size() {
                    return Err(Error::InvalidElf(
                        "load segment file size exceeds memory size",
                    ));
                }
                segments.push(Segment {
                    virtual_address: header.virtual_addr(),
                    file_offset: header.offset(),
                    file_size: header.file_size(),
                    memory_size: header.mem_size(),
                    flags: header.flags(),
                });
            }
            ProgramType::Interp => {
                if interpreter.is_some() {
                    return Err(Error::InvalidElf("multiple interpreters"));
                }
                let start = usize::try_from(header.offset())
                    .map_err(|_| Error::InvalidElf("interpreter offset overflow"))?;
                let end = usize::try_from(file_end)
                    .map_err(|_| Error::InvalidElf("interpreter range overflow"))?;
                let data = &bytes[start..end];
                let nul = data
                    .iter()
                    .position(|byte| *byte == 0)
                    .ok_or(Error::InvalidElf("unterminated interpreter path"))?;
                if data[nul + 1..].iter().any(|byte| *byte != 0) {
                    return Err(Error::InvalidElf("invalid interpreter path"));
                }
                interpreter = Some(
                    str::from_utf8(&data[..nul])
                        .map_err(|_| Error::InvalidElf("non-UTF-8 interpreter path"))?
                        .into(),
                );
            }
            ProgramType::Phdr => {
                program_header_address = Some(header.virtual_addr());
            }
            ProgramType::ShLib => return Err(Error::UnsupportedElf),
            _ => {}
        }
    }
    if segments.is_empty() {
        return Err(Error::InvalidElf("no loadable segments"));
    }

    Ok(ElfSpec {
        image_type,
        entry: elf.header.pt2.entry_point(),
        program_header_address,
        program_header_offset: elf.header.pt2.ph_offset(),
        program_header_size: elf.header.pt2.ph_entry_size(),
        program_header_count: elf.header.pt2.ph_count(),
        interpreter,
        segments,
    })
}

fn map_elf(
    space: &Arc<VmSpace>,
    bytes: &[u8],
    spec: &ElfSpec,
    requested_base: u64,
) -> Result<LoadedElf> {
    let minimum = spec
        .segments
        .iter()
        .map(|segment| align_down(segment.virtual_address, PAGE_SIZE))
        .min()
        .ok_or(Error::InvalidElf("no loadable segments"))?;
    let bias = match spec.image_type {
        ElfType::Executable => 0,
        ElfType::SharedObject => requested_base
            .checked_sub(minimum)
            .ok_or(Error::InvalidElf("invalid load base"))?,
        _ => return Err(Error::UnsupportedElf),
    };

    for segment in &spec.segments {
        if segment.memory_size == 0 {
            continue;
        }
        if segment.virtual_address % PAGE_SIZE != segment.file_offset % PAGE_SIZE {
            return Err(Error::InvalidElf("load segment page offsets differ"));
        }
        let virtual_address = segment
            .virtual_address
            .checked_add(bias)
            .ok_or(Error::InvalidElf("segment address overflow"))?;
        let start = align_down(virtual_address, PAGE_SIZE);
        let page_offset = virtual_address - start;
        let length = checked_align_up(
            page_offset
                .checked_add(segment.memory_size)
                .ok_or(Error::InvalidElf("segment length overflow"))?,
        )?;
        let protection = protection_from_flags(segment.flags);
        let object = VmObject::new(ObjectKind::Anonymous);
        let file_start = usize::try_from(segment.file_offset)
            .map_err(|_| Error::InvalidElf("segment offset overflow"))?;
        let file_end = usize::try_from(
            segment
                .file_offset
                .checked_add(segment.file_size)
                .ok_or(Error::InvalidElf("segment size overflow"))?,
        )
        .map_err(|_| Error::InvalidElf("segment range overflow"))?;
        let payload = &bytes[file_start..file_end];
        if object.write_at(page_offset, payload)? != payload.len() {
            return Err(Error::Memory(crate::mem::Error::OutOfMemory));
        }
        space.map_object(
            VirtAddr::new(start),
            length,
            object,
            0,
            protection,
            protection,
            VmInheritance::Copy,
            true,
        )?;
    }

    let program_headers = if let Some(address) = spec.program_header_address {
        address
            .checked_add(bias)
            .ok_or(Error::InvalidElf("program header address overflow"))?
    } else {
        derive_program_header_address(spec, bias)?
    };
    Ok(LoadedElf {
        entry: spec
            .entry
            .checked_add(bias)
            .ok_or(Error::InvalidElf("entry address overflow"))?,
        bias,
        program_headers,
        program_header_size: spec.program_header_size,
        program_header_count: spec.program_header_count,
    })
}

fn derive_program_header_address(spec: &ElfSpec, bias: u64) -> Result<u64> {
    for segment in &spec.segments {
        let segment_end = segment
            .file_offset
            .checked_add(segment.file_size)
            .ok_or(Error::InvalidElf("segment range overflow"))?;
        if spec.program_header_offset >= segment.file_offset
            && spec.program_header_offset < segment_end
        {
            return bias
                .checked_add(segment.virtual_address)
                .and_then(|address| {
                    address.checked_add(spec.program_header_offset - segment.file_offset)
                })
                .ok_or(Error::InvalidElf("program header address overflow"));
        }
    }
    Err(Error::InvalidElf("program headers are not loadable"))
}

fn build_stack(
    space: &Arc<VmSpace>,
    executable: &[u8],
    arguments: &[&[u8]],
    environment: &[&[u8]],
    main: &LoadedElf,
    interpreter_base: u64,
) -> Result<u64> {
    if arguments.is_empty() {
        return Err(Error::InvalidArgument);
    }

    let stack_base = STACK_TOP
        .checked_sub(STACK_SIZE)
        .ok_or(Error::InvalidArgument)?;
    let mut bytes = vec![0u8; STACK_SIZE as usize];
    let mut cursor = bytes.len();

    let mut argument_pointers = Vec::with_capacity(arguments.len());
    for argument in arguments {
        argument_pointers.push(push_string(&mut bytes, &mut cursor, stack_base, argument)?);
    }
    let mut environment_pointers = Vec::with_capacity(environment.len());
    for variable in environment {
        environment_pointers.push(push_string(&mut bytes, &mut cursor, stack_base, variable)?);
    }
    let executable_pointer = push_string(&mut bytes, &mut cursor, stack_base, executable)?;

    let mut random = [0u8; 16];
    crate::sys::random::fill_bytes(&mut random);
    let random_pointer = push_bytes(&mut bytes, &mut cursor, stack_base, &random, 16)?;

    let mut words = Vec::new();
    words.push(arguments.len() as u64);
    words.extend(argument_pointers);
    words.push(0);
    words.extend(environment_pointers);
    words.push(0);
    words.extend_from_slice(&[
        AT_PHDR,
        main.program_headers,
        AT_PHENT,
        u64::from(main.program_header_size),
        AT_PHNUM,
        u64::from(main.program_header_count),
        AT_PAGESZ,
        PAGE_SIZE,
        AT_BASE,
        interpreter_base,
        AT_ENTRY,
        main.entry,
        AT_UID,
        0,
        AT_EUID,
        0,
        AT_GID,
        0,
        AT_EGID,
        0,
        AT_SECURE,
        0,
        AT_RANDOM,
        random_pointer,
        AT_EXECFN,
        executable_pointer,
        AT_NULL,
        0,
    ]);

    let words_size = words
        .len()
        .checked_mul(mem::size_of::<u64>())
        .ok_or(Error::InvalidArgument)?;
    cursor = cursor
        .checked_sub(words_size)
        .ok_or(Error::InvalidArgument)?
        & !0xF;
    for (index, word) in words.into_iter().enumerate() {
        let start = cursor + index * mem::size_of::<u64>();
        bytes[start..start + mem::size_of::<u64>()].copy_from_slice(&word.to_ne_bytes());
    }

    let object = VmObject::new(ObjectKind::Anonymous);
    if object.write_at(cursor as u64, &bytes[cursor..])? != bytes.len() - cursor {
        return Err(Error::Memory(crate::mem::Error::OutOfMemory));
    }
    let protection = VmProtection::READ | VmProtection::WRITE;
    space.map_object(
        VirtAddr::new(stack_base),
        STACK_SIZE,
        object,
        0,
        protection,
        protection,
        VmInheritance::Copy,
        true,
    )?;
    Ok(stack_base + cursor as u64)
}

fn push_string(
    stack: &mut [u8],
    cursor: &mut usize,
    stack_base: u64,
    value: impl AsRef<[u8]>,
) -> Result<u64> {
    let value = value.as_ref();
    let size = value.len().checked_add(1).ok_or(Error::InvalidArgument)?;
    *cursor = cursor.checked_sub(size).ok_or(Error::InvalidArgument)?;
    stack[*cursor..*cursor + value.len()].copy_from_slice(value);
    stack[*cursor + value.len()] = 0;
    Ok(stack_base + *cursor as u64)
}

fn push_bytes(
    stack: &mut [u8],
    cursor: &mut usize,
    stack_base: u64,
    value: &[u8],
    alignment: usize,
) -> Result<u64> {
    *cursor = cursor
        .checked_sub(value.len())
        .ok_or(Error::InvalidArgument)?
        & !(alignment - 1);
    stack[*cursor..*cursor + value.len()].copy_from_slice(value);
    Ok(stack_base + *cursor as u64)
}

fn protection_from_flags(flags: Flags) -> VmProtection {
    let mut protection = VmProtection::empty();
    if flags.is_read() {
        protection |= VmProtection::READ;
    }
    if flags.is_write() {
        protection |= VmProtection::WRITE;
    }
    if flags.is_execute() {
        protection |= VmProtection::EXECUTE;
    }
    protection
}

fn checked_align_up(value: u64) -> Result<u64> {
    value
        .checked_add(PAGE_SIZE - 1)
        .map(|value| value & !(PAGE_SIZE - 1))
        .ok_or(Error::InvalidElf("alignment overflow"))
}

#[cfg(target_arch = "x86_64")]
fn expected_machine() -> Machine {
    Machine::X86_64
}

#[cfg(target_arch = "riscv64")]
fn expected_machine() -> Machine {
    Machine::RISC_V
}
