/* Run the same work with -smp 1 and -smp 22, in both dev and release kernels. */
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define MAX_WORKERS 64
#define VM_ROUNDS 16
#define VM_PAGES 65
#define LOCK_ROUNDS 4000

static pthread_barrier_t start;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned counter;
static size_t page_size;
static const char *mode;

static void check(int ok, const char *what) {
    if (!ok) {
        fprintf(stderr, "smp-test: FAIL: %s (errno=%d)\n", what, errno);
        exit(1);
    }
}

static uint64_t now_ns(void) {
    struct timespec t;
    check(clock_gettime(CLOCK_MONOTONIC, &t) == 0, "clock_gettime");
    return (uint64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}

static void *worker(void *arg) {
    unsigned id = (uintptr_t)arg;
    int result = pthread_barrier_wait(&start);
    check(result == 0 || result == PTHREAD_BARRIER_SERIAL_THREAD, "barrier");
    if (strcmp(mode, "lock") == 0) {
        for (unsigned i = 0; i < LOCK_ROUNDS; ++i) {
            check(pthread_mutex_lock(&lock) == 0, "lock");
            ++counter;
            check(pthread_mutex_unlock(&lock) == 0, "unlock");
        }
    } else if (strcmp(mode, "cpu") == 0) {
        uint64_t value = id + 1;
        for (unsigned i = 0; i < 20000000; ++i) {
            value ^= value << 13;
            value ^= value >> 7;
            value ^= value << 17;
        }
        return (void *)(uintptr_t)value;
    } else {
        size_t length = VM_PAGES * page_size;
        for (unsigned round = 0; round < VM_ROUNDS; ++round) {
            volatile unsigned char *p = mmap(NULL, length, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
            check(p != MAP_FAILED, "mmap");
            unsigned char tag = (id + round) % 254 + 1;
            for (size_t j = 0; j < VM_PAGES; ++j)
                p[j * page_size] = tag;
            check(mprotect((void *)p, length, PROT_READ) == 0, "mprotect");
            for (size_t j = 0; j < VM_PAGES; ++j)
                check(p[j * page_size] == tag, "read after protection change");
            check(mmap((void *)p, length, PROT_READ | PROT_WRITE,
                        MAP_FIXED | MAP_PRIVATE | MAP_ANONYMOUS, -1, 0) == (void *)p,
                    "replace mapping");
            for (size_t j = 0; j < VM_PAGES; ++j) {
                check(p[j * page_size] == 0, "replacement has no stale translation");
                p[j * page_size] = tag;
            }
            check(munmap((void *)p, length) == 0, "munmap");
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
    unsigned workers = argc > 1 ? strtoul(argv[1], NULL, 10) : 22;
    mode = argc > 2 ? argv[2] : "vm";
    check(workers > 0 && workers <= MAX_WORKERS, "workers must be 1..64");
    check(strcmp(mode, "vm") == 0 || strcmp(mode, "lock") == 0 ||
            strcmp(mode, "cpu") == 0, "mode must be vm, lock, or cpu");
    long size = sysconf(_SC_PAGESIZE);
    check(size > 0, "page size");
    page_size = size;
    pthread_t threads[MAX_WORKERS];
    check(pthread_barrier_init(&start, NULL, workers + 1) == 0, "barrier init");
    for (unsigned i = 0; i < workers; ++i)
        check(pthread_create(&threads[i], NULL, worker, (void *)(uintptr_t)i) == 0,
                "pthread_create");
    uint64_t begin = now_ns();
    int result = pthread_barrier_wait(&start);
    check(result == 0 || result == PTHREAD_BARRIER_SERIAL_THREAD, "start barrier");
    uint64_t checksum = 0;
    for (unsigned i = 0; i < workers; ++i) {
        void *value;
        check(pthread_join(threads[i], &value) == 0, "pthread_join");
        checksum ^= (uintptr_t)value;
    }
    if (strcmp(mode, "lock") == 0)
        check(counter == workers * LOCK_ROUNDS, "mutex counter");
    check(pthread_barrier_destroy(&start) == 0, "barrier destroy");
    printf("smp-test: PASS mode=%s workers=%u elapsed_ms=%llu checksum=%llu\n",
            mode, workers, (unsigned long long)((now_ns() - begin) / 1000000),
            (unsigned long long)checksum);
    return 0;
}
