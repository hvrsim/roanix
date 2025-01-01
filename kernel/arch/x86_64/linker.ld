OUTPUT_FORMAT(elf64-x86-64)

ENTRY(rmain)

PHDRS
{
    limine_requests PT_LOAD;
    text     PT_LOAD;
    rodata   PT_LOAD;
    data     PT_LOAD;
}

SECTIONS
{
    /* load the kernel in at the top 2GiB of the address space, according to limine spec */
    . = 0xffffffff80000000;

    /* create a section for limine reqs, and a phdr to go with it */
    .limine_requests : {
        KEEP(*(.limine_requests_start_marker))
        KEEP(*(.limine_requests))
        KEEP(*(.limine_requests_end_marker))
    } :limine_requests

    /* move to the next memory page for .text */
    . = ALIGN(CONSTANT(MAXPAGESIZE));

    .text : {
        *(.text .text.*)
    } :text

    /* move to the next memory page for .rodata */
    . = ALIGN(CONSTANT(MAXPAGESIZE));

    .rodata : {
        *(.rodata .rodata.*)
    } :rodata

    /* init array section for C++ global constructors */
    .init_array : {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP (*(.init_array .ctors))
        PROVIDE_HIDDEN (__init_array_end = .);
    } :rodata

    /* move to the next memory page for .data */
    . = ALIGN(CONSTANT(MAXPAGESIZE));

    .data : {
        *(.data .data.*)
    } :data

    /*
     * NOTE: .bss needs to be the last thing mapped
     * to :data, otherwise lots of unnecessary zeros
     * will be written to the binary.
     */
    .bss : {
        *(.bss .bss.*)
        *(COMMON)
    } :data

    /*
     * discard .note.* and .eh_frame since they may
     * cause issues on some hosts.
     */
    /DISCARD/ : {
        *(.eh_frame*)
        *(.note .note.*)
    }
}