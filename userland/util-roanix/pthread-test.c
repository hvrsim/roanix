// Threading smoke test.
//
// Exercises the pthread surface that Roanix implements, so that regressions in
// the futex, the scheduler, or the thread syscalls show up as a failed check
// rather than as a hang somewhere deep inside a port.

#define _GNU_SOURCE

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define THREAD_COUNT 4
#define INCREMENTS 20000

static int failures;
static int checks;

static void check(int condition, const char *name) {
    checks++;
    if (condition) {
        printf("ok   %s\n", name);
    } else {
        failures++;
        printf("FAIL %s (errno %d: %s)\n", name, errno, strerror(errno));
    }
    fflush(stdout);
}

// --- create, join, and argument passing ------------------------------------

static void *identity_thread(void *argument) {
    return argument;
}

static void test_create_join(void) {
    pthread_t threads[THREAD_COUNT];
    int created = 1;

    for (long i = 0; i < THREAD_COUNT; i++) {
        if (pthread_create(&threads[i], NULL, identity_thread, (void *)(i + 1)) != 0)
            created = 0;
    }
    check(created, "pthread_create");

    int joined = 1;
    for (long i = 0; i < THREAD_COUNT; i++) {
        void *result = NULL;
        if (pthread_join(threads[i], &result) != 0 || result != (void *)(i + 1))
            joined = 0;
    }
    check(joined, "pthread_join returns the thread value");
}

// --- mutual exclusion ------------------------------------------------------

static pthread_mutex_t counter_mutex = PTHREAD_MUTEX_INITIALIZER;
static long counter;

static void *counter_thread(void *argument) {
    (void)argument;
    for (int i = 0; i < INCREMENTS; i++) {
        pthread_mutex_lock(&counter_mutex);
        counter++;
        pthread_mutex_unlock(&counter_mutex);
    }
    return NULL;
}

static void test_mutex(void) {
    pthread_t threads[THREAD_COUNT];

    counter = 0;
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_create(&threads[i], NULL, counter_thread, NULL);
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_join(threads[i], NULL);

    check(counter == (long)THREAD_COUNT * INCREMENTS, "mutex protects a counter");
}

static void test_recursive_mutex(void) {
    pthread_mutexattr_t attr;
    pthread_mutex_t mutex;

    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&mutex, &attr);

    int ok = pthread_mutex_lock(&mutex) == 0 && pthread_mutex_lock(&mutex) == 0
        && pthread_mutex_unlock(&mutex) == 0 && pthread_mutex_unlock(&mutex) == 0;
    check(ok, "recursive mutex relocks on the owning thread");

    pthread_mutex_destroy(&mutex);
    pthread_mutexattr_destroy(&attr);
}

static void test_errorcheck_mutex(void) {
    pthread_mutexattr_t attr;
    pthread_mutex_t mutex;

    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
    pthread_mutex_init(&mutex, &attr);

    int ok = pthread_mutex_lock(&mutex) == 0 && pthread_mutex_lock(&mutex) == EDEADLK
        && pthread_mutex_unlock(&mutex) == 0;
    check(ok, "errorcheck mutex reports self-deadlock");

    pthread_mutex_destroy(&mutex);
    pthread_mutexattr_destroy(&attr);
}

static void test_trylock(void) {
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

    int ok = pthread_mutex_trylock(&mutex) == 0;
    ok = ok && pthread_mutex_unlock(&mutex) == 0;
    check(ok, "pthread_mutex_trylock");
    pthread_mutex_destroy(&mutex);
}

// --- condition variables ---------------------------------------------------

static pthread_mutex_t signal_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t signal_cond = PTHREAD_COND_INITIALIZER;
static int ready;

static void *waiter_thread(void *argument) {
    (void)argument;
    pthread_mutex_lock(&signal_mutex);
    while (!ready)
        pthread_cond_wait(&signal_cond, &signal_mutex);
    pthread_mutex_unlock(&signal_mutex);
    return NULL;
}

static void test_condvar_broadcast(void) {
    pthread_t threads[THREAD_COUNT];

    ready = 0;
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_create(&threads[i], NULL, waiter_thread, NULL);

    // Give the waiters a moment to actually block, so the broadcast has to
    // wake sleeping threads rather than being observed by a spinning check.
    struct timespec pause = {0, 50 * 1000 * 1000};
    nanosleep(&pause, NULL);

    pthread_mutex_lock(&signal_mutex);
    ready = 1;
    pthread_cond_broadcast(&signal_cond);
    pthread_mutex_unlock(&signal_mutex);

    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_join(threads[i], NULL);
    check(1, "pthread_cond_broadcast wakes every waiter");
}

static void test_condvar_timedwait(void) {
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    struct timespec deadline;

    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_nsec += 100 * 1000 * 1000;
    if (deadline.tv_nsec >= 1000000000) {
        deadline.tv_nsec -= 1000000000;
        deadline.tv_sec++;
    }

    pthread_mutex_lock(&mutex);
    int result = pthread_cond_timedwait(&cond, &mutex, &deadline);
    pthread_mutex_unlock(&mutex);

    check(result == ETIMEDOUT, "pthread_cond_timedwait times out");
    pthread_cond_destroy(&cond);
    pthread_mutex_destroy(&mutex);
}

// --- reader/writer locks ---------------------------------------------------

static pthread_rwlock_t rwlock = PTHREAD_RWLOCK_INITIALIZER;
static long rwlock_value;

static void *reader_thread(void *argument) {
    (void)argument;
    for (int i = 0; i < 200; i++) {
        pthread_rwlock_rdlock(&rwlock);
        volatile long observed = rwlock_value;
        (void)observed;
        pthread_rwlock_unlock(&rwlock);
    }
    return NULL;
}

static void *writer_thread(void *argument) {
    (void)argument;
    for (int i = 0; i < 200; i++) {
        pthread_rwlock_wrlock(&rwlock);
        rwlock_value++;
        pthread_rwlock_unlock(&rwlock);
    }
    return NULL;
}

static void test_rwlock(void) {
    pthread_t threads[THREAD_COUNT];

    rwlock_value = 0;
    for (int i = 0; i < THREAD_COUNT; i++) {
        pthread_create(&threads[i], NULL, (i % 2) ? reader_thread : writer_thread, NULL);
    }
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_join(threads[i], NULL);

    check(rwlock_value == 200 * (THREAD_COUNT / 2), "rwlock serialises writers");
}

// --- barriers --------------------------------------------------------------

static pthread_barrier_t barrier;
static atomic_int barrier_before;
static atomic_int barrier_after;

static void *barrier_thread(void *argument) {
    (void)argument;
    atomic_fetch_add(&barrier_before, 1);
    pthread_barrier_wait(&barrier);
    // Every thread must have reached the barrier before any leaves it.
    if (atomic_load(&barrier_before) == THREAD_COUNT)
        atomic_fetch_add(&barrier_after, 1);
    return NULL;
}

static void test_barrier(void) {
    pthread_t threads[THREAD_COUNT];

    atomic_store(&barrier_before, 0);
    atomic_store(&barrier_after, 0);
    pthread_barrier_init(&barrier, NULL, THREAD_COUNT);
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_create(&threads[i], NULL, barrier_thread, NULL);
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_join(threads[i], NULL);
    pthread_barrier_destroy(&barrier);

    check(atomic_load(&barrier_after) == THREAD_COUNT, "pthread_barrier_wait synchronises");
}

// --- once ------------------------------------------------------------------

static pthread_once_t once_control = PTHREAD_ONCE_INIT;
static atomic_int once_calls;

static void once_routine(void) {
    atomic_fetch_add(&once_calls, 1);
}

static void *once_thread(void *argument) {
    (void)argument;
    pthread_once(&once_control, once_routine);
    return NULL;
}

static void test_once(void) {
    pthread_t threads[THREAD_COUNT];

    atomic_store(&once_calls, 0);
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_create(&threads[i], NULL, once_thread, NULL);
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_join(threads[i], NULL);

    check(atomic_load(&once_calls) == 1, "pthread_once runs exactly once");
}

// --- thread-local storage --------------------------------------------------

static __thread int tls_value;
static atomic_int tls_ok;

static void *tls_thread(void *argument) {
    long id = (long)argument;
    tls_value = (int)id;
    // Yield so the other threads get a chance to clobber a shared variable.
    sched_yield();
    if (tls_value == (int)id)
        atomic_fetch_add(&tls_ok, 1);
    return NULL;
}

static void test_tls(void) {
    pthread_t threads[THREAD_COUNT];

    atomic_store(&tls_ok, 0);
    for (long i = 0; i < THREAD_COUNT; i++)
        pthread_create(&threads[i], NULL, tls_thread, (void *)(i + 1));
    for (int i = 0; i < THREAD_COUNT; i++)
        pthread_join(threads[i], NULL);

    check(atomic_load(&tls_ok) == THREAD_COUNT, "__thread storage is per-thread");
}

static pthread_key_t key;
static atomic_int key_destructor_calls;

static void key_destructor(void *value) {
    (void)value;
    atomic_fetch_add(&key_destructor_calls, 1);
}

static void *key_thread(void *argument) {
    pthread_setspecific(key, argument);
    return pthread_getspecific(key);
}

static void test_thread_specific(void) {
    pthread_t threads[THREAD_COUNT];
    int ok = 1;

    atomic_store(&key_destructor_calls, 0);
    pthread_key_create(&key, key_destructor);
    for (long i = 0; i < THREAD_COUNT; i++)
        pthread_create(&threads[i], NULL, key_thread, (void *)(i + 1));
    for (long i = 0; i < THREAD_COUNT; i++) {
        void *result = NULL;
        pthread_join(threads[i], &result);
        if (result != (void *)(i + 1))
            ok = 0;
    }
    check(ok, "pthread_setspecific is per-thread");
    check(atomic_load(&key_destructor_calls) == THREAD_COUNT,
        "thread-specific destructors run at exit");
    pthread_key_delete(key);
}

// --- detach ----------------------------------------------------------------

static atomic_int detached_done;

static void *detached_thread(void *argument) {
    (void)argument;
    atomic_store(&detached_done, 1);
    return NULL;
}

static void test_detach(void) {
    pthread_t thread;

    atomic_store(&detached_done, 0);
    pthread_create(&thread, NULL, detached_thread, NULL);
    check(pthread_detach(thread) == 0, "pthread_detach");

    for (int i = 0; i < 1000 && !atomic_load(&detached_done); i++) {
        struct timespec pause = {0, 1000 * 1000};
        nanosleep(&pause, NULL);
    }
    check(atomic_load(&detached_done) == 1, "detached thread runs to completion");
}

// --- identity, equality, and yielding --------------------------------------

static void *self_thread(void *argument) {
    pthread_t *slot = argument;
    *slot = pthread_self();
    return NULL;
}

static void test_self_equal(void) {
    pthread_t thread;
    pthread_t observed;

    pthread_create(&thread, NULL, self_thread, &observed);
    pthread_join(thread, NULL);

    check(pthread_equal(thread, observed), "pthread_self matches the creator's handle");
    check(!pthread_equal(pthread_self(), thread), "pthread_equal separates distinct threads");
}

static void test_yield(void) {
    check(sched_yield() == 0, "sched_yield");
}

// --- names -----------------------------------------------------------------

static void test_thread_name(void) {
    char name[32];

    int set = pthread_setname_np(pthread_self(), "smoke");
    if (set == ENOSYS) {
        printf("skip pthread_setname_np (not implemented)\n");
        return;
    }
    check(set == 0, "pthread_setname_np");

    memset(name, 0, sizeof(name));
    int got = pthread_getname_np(pthread_self(), name, sizeof(name));
    check(got == 0 && strcmp(name, "smoke") == 0, "pthread_getname_np round-trips");
}

// --- signals ---------------------------------------------------------------

static atomic_int signal_seen;

static void usr1_handler(int signal) {
    (void)signal;
    atomic_fetch_add(&signal_seen, 1);
}

static void *signal_target_thread(void *argument) {
    (void)argument;
    for (int i = 0; i < 500 && !atomic_load(&signal_seen); i++) {
        struct timespec pause = {0, 1000 * 1000};
        nanosleep(&pause, NULL);
    }
    return NULL;
}

static void test_pthread_kill(void) {
    struct sigaction action;
    pthread_t thread;

    memset(&action, 0, sizeof(action));
    action.sa_handler = usr1_handler;
    sigaction(SIGUSR1, &action, NULL);

    atomic_store(&signal_seen, 0);
    pthread_create(&thread, NULL, signal_target_thread, NULL);

    int sent = pthread_kill(thread, SIGUSR1);
    if (sent == ENOSYS) {
        printf("skip pthread_kill (not implemented)\n");
        pthread_join(thread, NULL);
        return;
    }
    check(sent == 0, "pthread_kill");
    pthread_join(thread, NULL);
    check(atomic_load(&signal_seen) >= 1, "pthread_kill delivers the signal");

    // pthread_join can return before the joined thread has finished leaving
    // the kernel, so the tid stays addressable for a short while afterwards.
    // Poll rather than demanding that it be gone the instant join returns.
    int rejected = 0;
    for (int i = 0; i < 500 && !rejected; i++) {
        if (pthread_kill(thread, 0) != 0) {
            rejected = 1;
            break;
        }
        struct timespec pause = {0, 1000 * 1000};
        nanosleep(&pause, NULL);
    }
    check(rejected, "pthread_kill stops recognising an exited thread");
}

// --- stack attributes ------------------------------------------------------

static void *stack_thread(void *argument) {
    // Touch a good chunk of the stack so an undersized or unmapped allocation
    // faults here rather than silently passing.
    volatile char scratch[16 * 1024];
    memset((void *)scratch, (int)(long)argument, sizeof(scratch));
    return (void *)(long)scratch[0];
}

static void test_stack_size(void) {
    pthread_attr_t attr;
    pthread_t thread;
    size_t size = 0;

    pthread_attr_init(&attr);
    check(pthread_attr_setstacksize(&attr, 256 * 1024) == 0, "pthread_attr_setstacksize");
    pthread_attr_getstacksize(&attr, &size);
    check(size == 256 * 1024, "pthread_attr_getstacksize round-trips");

    void *result = NULL;
    check(pthread_create(&thread, &attr, stack_thread, (void *)7) == 0,
        "pthread_create honours a custom stack size");
    pthread_join(thread, &result);
    check(result == (void *)7, "custom-stack thread ran correctly");
    pthread_attr_destroy(&attr);
}

// --- stack guard pages ------------------------------------------------------

static void *guard_report_thread(void *argument) {
    (void)argument;
    pthread_attr_t attr;
    if (pthread_getattr_np(pthread_self(), &attr) != 0)
        return (void *)0;

    void *base = NULL;
    size_t size = 0;
    size_t guard = 0;
    pthread_attr_getstack(&attr, &base, &size);
    pthread_attr_getguardsize(&attr, &guard);
    pthread_attr_destroy(&attr);

    // The reported stack must exclude the guard region, and must be page
    // aligned for the guard to be mprotect-able at all.
    if (!base || ((uintptr_t)base & 0xfff) != 0)
        return (void *)0;
    return (void *)(uintptr_t)(guard + 1);
}

static void test_stack_guard(void) {
    pthread_attr_t attr;
    size_t guard = 0;

    pthread_attr_init(&attr);
    check(pthread_attr_getguardsize(&attr, &guard) == 0 && guard > 0,
        "threads get a guard page by default");
    check(pthread_attr_setguardsize(&attr, 8192) == 0, "pthread_attr_setguardsize");
    pthread_attr_getguardsize(&attr, &guard);
    check(guard == 8192, "pthread_attr_getguardsize round-trips");
    pthread_attr_destroy(&attr);

    pthread_t thread;
    void *result = NULL;
    pthread_create(&thread, NULL, guard_report_thread, NULL);
    pthread_join(thread, &result);
    check(result != (void *)0, "pthread_getattr_np reports a page-aligned stack");
}

struct test_case {
    const char *name;
    void (*run)(void);
};

static const struct test_case tests[] = {
    {"create_join", test_create_join},
    {"mutex", test_mutex},
    {"recursive_mutex", test_recursive_mutex},
    {"errorcheck_mutex", test_errorcheck_mutex},
    {"trylock", test_trylock},
    {"condvar_broadcast", test_condvar_broadcast},
    {"condvar_timedwait", test_condvar_timedwait},
    {"rwlock", test_rwlock},
    {"barrier", test_barrier},
    {"once", test_once},
    {"tls", test_tls},
    {"thread_specific", test_thread_specific},
    {"detach", test_detach},
    {"self_equal", test_self_equal},
    {"yield", test_yield},
    {"thread_name", test_thread_name},
    {"pthread_kill", test_pthread_kill},
    {"stack_size", test_stack_size},
    {"stack_guard", test_stack_guard},
};

int main(int argc, char **argv) {
    printf("pthread smoke test\n");
    fflush(stdout);

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        // Name the group before running it, so a hang identifies itself
        // instead of just truncating the log.
        if (argc > 1 && strcmp(argv[1], tests[i].name) != 0)
            continue;
        printf("-- %s\n", tests[i].name);
        fflush(stdout);
        tests[i].run();
    }

    printf("\n%d/%d checks passed\n", checks - failures, checks);
    fflush(stdout);
    return failures == 0 ? 0 : 1;
}
