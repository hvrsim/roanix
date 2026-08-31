#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define PAGE_BYTES 4096u
#define BLOCK_BYTES 64u
#define DEFAULT_MIB 48u

static uint8_t pattern(size_t page, size_t block)
{
    return (uint8_t)(page * 17u + block * 29u + (page >> 8));
}

static size_t parse_mib(int argc, char **argv)
{
    if (argc > 2) {
        fprintf(stderr, "usage: %s [MiB]\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    if (argc == 1)
        return DEFAULT_MIB;

    char *end = NULL;
    errno = 0;
    uintmax_t value = strtoumax(argv[1], &end, 10);
    if (errno != 0 || end == argv[1] || *end != '\0' || value == 0 ||
            value > SIZE_MAX / (1024u * 1024u)) {
        fprintf(stderr, "%s: invalid size '%s'\n", argv[0], argv[1]);
        exit(EXIT_FAILURE);
    }
    return (size_t)value;
}

int main(int argc, char **argv)
{
    size_t mib = parse_mib(argc, argv);
    size_t length = mib * 1024u * 1024u;
    size_t pages = length / PAGE_BYTES;
    long system_page = sysconf(_SC_PAGESIZE);
    if (system_page != (long)PAGE_BYTES) {
        fprintf(stderr, "%s: expected %u-byte pages, got %ld\n",
                argv[0], PAGE_BYTES, system_page);
        return EXIT_FAILURE;
    }

    printf("vm-swap-test: mapping %zu MiB\n", mib);
    fflush(stdout);
    uint8_t *region = mmap(NULL, length, PROT_READ | PROT_WRITE,
            MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (region == MAP_FAILED) {
        perror("vm-swap-test: mmap");
        return EXIT_FAILURE;
    }

    printf("vm-swap-test: writing %zu MiB (%zu pages)\n", mib, pages);
    fflush(stdout);
    for (size_t page = 0; page < pages; page++) {
        uint8_t *bytes = region + page * PAGE_BYTES;
        for (size_t block = 0; block < PAGE_BYTES / BLOCK_BYTES; block++)
            memset(bytes + block * BLOCK_BYTES, pattern(page, block), BLOCK_BYTES);
        if ((page + 1) % 4096u == 0) {
            printf("vm-swap-test: wrote %zu/%zu pages\n", page + 1, pages);
            fflush(stdout);
        }
    }

    printf("vm-swap-test: verifying in reverse order\n");
    fflush(stdout);
    for (size_t page = pages; page-- > 0;) {
        const uint8_t *bytes = region + page * PAGE_BYTES;
        for (size_t offset = 0; offset < PAGE_BYTES; offset++) {
            uint8_t expected = pattern(page, offset / BLOCK_BYTES);
            if (bytes[offset] != expected) {
                fprintf(stderr,
                        "vm-swap-test: corruption at page %zu offset %zu: "
                        "got 0x%02x, expected 0x%02x\n",
                        page, offset, bytes[offset], expected);
                return EXIT_FAILURE;
            }
        }
        if ((pages - page) % 4096u == 0) {
            printf("vm-swap-test: verified %zu/%zu pages\n", pages - page, pages);
            fflush(stdout);
        }
    }

    if (munmap(region, length) < 0) {
        perror("vm-swap-test: munmap");
        return EXIT_FAILURE;
    }
    printf("vm-swap-test: PASS (%zu pages preserved)\n", pages);
    return EXIT_SUCCESS;
}
