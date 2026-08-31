#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

static size_t page_size;
static size_t length;
static unsigned checks;

static void fail(const char *message)
{
    perror(message);
    exit(1);
}

static void check(int condition, const char *message)
{
    ++checks;
    if (!condition) {
        fprintf(stderr, "fs-cache-test: FAIL: %s\n", message);
        exit(1);
    }
}

static void fill_pattern(uint8_t *bytes, size_t size, uint8_t seed)
{
    for (size_t index = 0; index < size; ++index)
        bytes[index] = (uint8_t)(seed + index * 17u + index / 251u);
}

static void write_all(int fd, const void *buffer, size_t size)
{
    const uint8_t *bytes = buffer;
    size_t done = 0;
    while (done < size) {
        ssize_t result = write(fd, bytes + done, size - done);
        if (result <= 0)
            fail("fs-cache-test: write");
        done += (size_t)result;
    }
}

static void read_all(int fd, void *buffer, size_t size)
{
    uint8_t *bytes = buffer;
    size_t done = 0;
    while (done < size) {
        ssize_t result = read(fd, bytes + done, size - done);
        if (result <= 0)
            fail("fs-cache-test: read");
        done += (size_t)result;
    }
}

static void seek_to(int fd, off_t offset)
{
    if (lseek(fd, offset, SEEK_SET) != offset)
        fail("fs-cache-test: lseek");
}

static void test_sparse_file(const char *path)
{
    int fd = open(path, O_CREAT | O_TRUNC | O_RDWR, 0600);
    if (fd < 0)
        fail("fs-cache-test: open sparse");

    off_t marker_offset = (off_t)(page_size * 5 + 37);
    uint8_t marker = 0xa7;
    seek_to(fd, marker_offset);
    write_all(fd, &marker, 1);

    uint8_t *bytes = malloc((size_t)marker_offset + 1);
    if (bytes == NULL)
        fail("fs-cache-test: malloc sparse");
    memset(bytes, 0xff, (size_t)marker_offset + 1);
    seek_to(fd, 0);
    read_all(fd, bytes, (size_t)marker_offset + 1);
    for (off_t index = 0; index < marker_offset; ++index)
        check(bytes[index] == 0, "sparse holes read as zero");
    check(bytes[marker_offset] == marker, "sparse extent retains data");

    free(bytes);
    close(fd);
}

static void test_tail_sanitization(const char *path)
{
    int fd = open(path, O_CREAT | O_TRUNC | O_RDWR, 0600);
    if (fd < 0)
        fail("fs-cache-test: open tail");
    uint8_t zero = 0;
    write_all(fd, &zero, 1);

    uint8_t *mapping = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, 0);
    if (mapping == MAP_FAILED)
        fail("fs-cache-test: mmap tail");
    mapping[37] = 0xcc; /* Legal rounded-page access, but still beyond EOF. */

    uint8_t marker = 0x5a;
    seek_to(fd, 100);
    write_all(fd, &marker, 1);
    uint8_t bytes[101];
    memset(bytes, 0xff, sizeof(bytes));
    seek_to(fd, 0);
    read_all(fd, bytes, sizeof(bytes));
    for (size_t index = 0; index < 100; ++index)
        check(bytes[index] == 0,
              "sparse growth clears mapped writes beyond the old EOF");
    check(bytes[100] == marker, "sparse growth retains the new extent");

    check(munmap(mapping, page_size) == 0, "tail mapping can be unmapped");
    close(fd);
}

static void test_unified_cache(const char *path)
{
    uint8_t *expected = malloc(length);
    uint8_t *actual_storage = malloc(length + 19);
    if (expected == NULL || actual_storage == NULL)
        fail("fs-cache-test: malloc cache buffers");
    uint8_t *actual = actual_storage + 11; /* Exercise unaligned user windows. */
    fill_pattern(expected, length, 0x23);

    int fd = open(path, O_CREAT | O_TRUNC | O_RDWR, 0600);
    if (fd < 0)
        fail("fs-cache-test: open cache");
    write_all(fd, expected, length);
    seek_to(fd, 0);
    read_all(fd, actual, length);
    check(memcmp(actual, expected, length) == 0,
          "multi-page provider I/O preserves every window");

    uint8_t *mapping = mmap(NULL, length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, 0);
    if (mapping == MAP_FAILED)
        fail("fs-cache-test: mmap shared");
    check(memcmp(mapping, expected, length) == 0,
          "mmap observes data written through file I/O");

    const size_t fd_offset = page_size + 29;
    uint8_t fd_value = 0xd4;
    seek_to(fd, (off_t)fd_offset);
    write_all(fd, &fd_value, 1);
    expected[fd_offset] = fd_value;
    check(mapping[fd_offset] == fd_value,
          "shared mapping observes descriptor writes");

    const size_t map_offset = page_size * 2 + 71;
    mapping[map_offset] = 0x6b;
    expected[map_offset] = 0x6b;
    seek_to(fd, (off_t)map_offset);
    uint8_t observed = 0;
    read_all(fd, &observed, 1);
    check(observed == expected[map_offset],
          "descriptor reads observe shared-mapping writes");

    pid_t child = fork();
    if (child < 0)
        fail("fs-cache-test: fork");
    if (child == 0) {
        mapping[page_size * 3 + 9] = 0x91;
        _exit(0);
    }
    int status = 0;
    if (waitpid(child, &status, 0) != child)
        fail("fs-cache-test: waitpid");
    check(WIFEXITED(status) && WEXITSTATUS(status) == 0,
          "shared-cache child exits normally");
    check(mapping[page_size * 3 + 9] == 0x91,
          "MAP_SHARED remains shared across fork");

    int replacement = open(path, O_TRUNC | O_RDWR, 0600);
    if (replacement < 0)
        fail("fs-cache-test: reopen truncate");
    fill_pattern(expected, length, 0x77);
    write_all(replacement, expected, length);
    close(replacement);
    check(memcmp(mapping, expected, length) == 0,
          "truncate invalidates old PTEs and preserves cache identity");

    check(link(path, "/tmp/fs-cache-test-link") == 0,
          "hard link creation succeeds");
    check(rename("/tmp/fs-cache-test-link", "/tmp/fs-cache-test-renamed") == 0,
          "rename succeeds");
    check(unlink(path) == 0, "unlink of mapped vnode succeeds");
    int alias = open("/tmp/fs-cache-test-renamed", O_RDONLY);
    if (alias < 0)
        fail("fs-cache-test: open renamed link");
    seek_to(alias, 0);
    read_all(alias, actual, length);
    check(memcmp(actual, expected, length) == 0,
          "renamed link retains the unified cache");
    close(alias);
    check(unlink("/tmp/fs-cache-test-renamed") == 0,
          "final namespace link can be removed");
    check(mapping[page_size + 3] == expected[page_size + 3],
          "unlinked mapping retains cached pages");

    check(munmap(mapping, length) == 0, "shared mapping can be unmapped");
    close(fd);
    free(actual_storage);
    free(expected);
}

int main(void)
{
    long value = sysconf(_SC_PAGESIZE);
    if (value <= 0)
        fail("fs-cache-test: sysconf");
    page_size = (size_t)value;
    length = page_size * 4;

    char cache_path[80];
    char sparse_path[80];
    snprintf(cache_path, sizeof(cache_path), "/tmp/fs-cache-test-%ld", (long)getpid());
    snprintf(sparse_path, sizeof(sparse_path), "/tmp/fs-sparse-test-%ld", (long)getpid());
    unlink("/tmp/fs-cache-test-link");
    unlink("/tmp/fs-cache-test-renamed");

    test_sparse_file(sparse_path);
    test_tail_sanitization(sparse_path);
    test_unified_cache(cache_path);
    unlink(sparse_path);

    printf("fs-cache-test: PASS (%u checks)\n", checks);
    return 0;
}
