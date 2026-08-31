#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

static size_t page_size;
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
        fprintf(stderr, "vm-test: FAIL: %s\n", message);
        exit(1);
    }
}

static void *map_anon(size_t length, int protection)
{
    void *mapping = mmap(NULL, length, protection,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (mapping == MAP_FAILED)
        fail("vm-test: mmap anonymous");
    return mapping;
}

static void test_protection_and_fixed(void)
{
    uint8_t *mapping = map_anon(page_size * 2, PROT_NONE);
    check(mprotect(mapping, page_size * 2, PROT_READ | PROT_WRITE) == 0,
          "anonymous protection can be broadened");
    mapping[0] = 0x41;
    mapping[page_size] = 0x52;

    void *replacement = mmap(mapping + page_size, page_size,
                             PROT_READ | PROT_WRITE,
                             MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    check(replacement == mapping + page_size, "MAP_FIXED returns requested base");
    check(mapping[0] == 0x41, "MAP_FIXED preserves bytes outside its range");
    check(mapping[page_size] == 0, "MAP_FIXED installs fresh zero-fill memory");
    check(munmap(mapping, page_size * 2) == 0, "fixed mapping can be unmapped");
}

static void test_cow_after_mprotect(void)
{
    uint8_t *mapping = map_anon(page_size, PROT_READ | PROT_WRITE);
    mapping[0] = 0x19;

    pid_t child = fork();
    if (child < 0)
        fail("vm-test: fork");
    if (child == 0) {
        volatile uint8_t value = mapping[0];
        (void)value; /* Install the shared read-only COW translation. */
        if (mprotect(mapping, page_size, PROT_READ | PROT_WRITE) != 0)
            _exit(2);
        mapping[0] = 0x73;
        _exit(mapping[0] == 0x73 ? 0 : 3);
    }

    int status;
    if (waitpid(child, &status, 0) != child)
        fail("vm-test: waitpid");
    check(WIFEXITED(status) && WEXITSTATUS(status) == 0,
          "child can write after mprotect");
    check(mapping[0] == 0x19, "mprotect write upgrade preserves fork COW");
    check(munmap(mapping, page_size) == 0, "COW mapping can be unmapped");
}

static void test_range_validation(void)
{
    uint8_t *mapping = map_anon(page_size * 3, PROT_READ | PROT_WRITE);
    check(munmap(mapping + page_size, page_size) == 0, "middle page unmaps");
    errno = 0;
    check(madvise(mapping, page_size * 3, MADV_NORMAL) == -1,
          "madvise rejects a range containing a hole");
    check(madvise(mapping, page_size, MADV_RANDOM) == 0,
          "madvise accepts a completely mapped range");
    errno = 0;
    check(madvise(mapping, page_size, MADV_DONTNEED) == -1
              && errno == ENOTSUP,
          "unsupported destructive advice fails explicitly");
    check(munmap(mapping, page_size * 3) == 0,
          "munmap tolerates holes and removes intersections");
    check(munmap(mapping, page_size * 3) == 0,
          "munmap of an unmapped range succeeds");
}

static void test_rejected_requests(void)
{
    errno = 0;
    void *mapping = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, (off_t)page_size);
    check(mapping == MAP_FAILED && errno == EINVAL,
          "anonymous mmap rejects a nonzero offset");

    errno = 0;
    mapping = mmap(NULL, page_size, PROT_WRITE | PROT_EXEC,
                   MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    check(mapping == MAP_FAILED && errno == EACCES, "mmap enforces W^X");

    mapping = map_anon(page_size, PROT_READ | PROT_WRITE);
    errno = 0;
    check(mprotect(mapping, page_size, PROT_WRITE | PROT_EXEC) == -1
              && errno == EACCES,
          "mprotect enforces W^X");
    check(munmap(mapping, page_size) == 0, "W^X test mapping can be unmapped");
}

static void test_file_access(void)
{
    char path[64];
    snprintf(path, sizeof(path), "/tmp/vm-test-%ld", (long)getpid());
    int fd = open(path, O_CREAT | O_TRUNC | O_WRONLY, 0600);
    if (fd < 0)
        fail("vm-test: open temporary file");

    uint8_t *bytes = calloc(1, page_size);
    if (bytes == NULL)
        fail("vm-test: calloc");
    if (write(fd, bytes, page_size) != (ssize_t)page_size)
        fail("vm-test: write temporary file");
    free(bytes);

    errno = 0;
    void *mapping = mmap(NULL, page_size, PROT_READ, MAP_PRIVATE, fd, 0);
    check(mapping == MAP_FAILED && errno == EACCES,
          "file mmap requires a read-open descriptor");
    close(fd);
    unlink(path);
}

int main(void)
{
    long value = sysconf(_SC_PAGESIZE);
    if (value <= 0)
        fail("vm-test: sysconf");
    page_size = (size_t)value;

    test_protection_and_fixed();
    test_cow_after_mprotect();
    test_range_validation();
    test_rejected_requests();
    test_file_access();

    printf("vm-test: PASS (%u checks)\n", checks);
    return 0;
}
