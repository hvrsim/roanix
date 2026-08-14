/*
 * Roanix driver framework: synchronization and byte-order helpers.
 *
 * Locks live entirely in the driver's own address space. Taking an uncontended
 * spin lock never enters the kernel.
 */
#ifndef ROANIX_SYNC_H
#define ROANIX_SYNC_H

#include <roanix/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Hints to the CPU that this thread is spinning. */
static inline void rdf_cpu_relax(void)
{
#if defined(__x86_64__)
    __asm__ volatile("pause" ::: "memory");
#elif defined(__riscv)
    __asm__ volatile("nop" ::: "memory");
#endif
}

/* Disables local interrupt delivery and returns the previous state. */
static inline uintptr_t rdf_irq_save(void)
{
#if defined(__x86_64__)
    uintptr_t flags;
    __asm__ volatile("pushfq; popq %0; cli" : "=r"(flags) : : "memory");
    return flags;
#elif defined(__riscv)
    uintptr_t previous;
    uintptr_t mask = 1u << 1;
    __asm__ volatile("csrrc %0, sstatus, %1" : "=r"(previous) : "r"(mask) : "memory");
    return previous;
#else
#error "Unsupported driver architecture"
#endif
}

/* Restores the interrupt state returned by rdf_irq_save(). */
static inline void rdf_irq_restore(uintptr_t state)
{
#if defined(__x86_64__)
    if ((state & (UINT64_C(1) << 9)) != 0)
        __asm__ volatile("sti" ::: "memory");
#elif defined(__riscv)
    if ((state & (UINT64_C(1) << 1)) != 0) {
        uintptr_t mask = 1u << 1;
        __asm__ volatile("csrs sstatus, %0" : : "r"(mask) : "memory");
    }
#endif
}

/*
 * A ticket spin lock.
 *
 * Tickets give first-come-first-served ordering, which keeps one CPU from
 * starving another on a lock a device shares with its interrupt handler.
 */
struct rdf_spinlock {
    uint32_t next;
    uint32_t serving;
};

/* Static initializer for a spin lock. */
#define RDF_SPINLOCK_INIT {0, 0}

/* Initializes a spin lock at run time. */
static inline void rdf_spin_init(struct rdf_spinlock *lock)
{
    __atomic_store_n(&lock->next, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&lock->serving, 0u, __ATOMIC_RELAXED);
}

/* Acquires a spin lock without changing interrupt state. */
static inline void rdf_spin_lock(struct rdf_spinlock *lock)
{
    uint32_t ticket = __atomic_fetch_add(&lock->next, 1u, __ATOMIC_RELAXED);
    while (__atomic_load_n(&lock->serving, __ATOMIC_ACQUIRE) != ticket)
        rdf_cpu_relax();
}

/* Releases a spin lock. */
static inline void rdf_spin_unlock(struct rdf_spinlock *lock)
{
    uint32_t next = __atomic_load_n(&lock->serving, __ATOMIC_RELAXED) + 1u;
    __atomic_store_n(&lock->serving, next, __ATOMIC_RELEASE);
}

/*
 * Acquires a lock with local interrupts disabled.
 *
 * Use this for any lock an interrupt handler also takes.
 */
static inline uintptr_t rdf_spin_lock_irqsave(struct rdf_spinlock *lock)
{
    uintptr_t state = rdf_irq_save();
    rdf_spin_lock(lock);
    return state;
}

/* Releases a lock and restores the saved interrupt state. */
static inline void rdf_spin_unlock_irqrestore(struct rdf_spinlock *lock, uintptr_t state)
{
    rdf_spin_unlock(lock);
    rdf_irq_restore(state);
}

/* --- Byte order --------------------------------------------------------- */

/* Reads a little-endian 16-bit value from a byte buffer. */
static inline uint16_t rdf_le16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | (uint16_t)bytes[1] << 8);
}

/* Reads a little-endian 32-bit value from a byte buffer. */
static inline uint32_t rdf_le32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 |
           (uint32_t)bytes[3] << 24;
}

/* Reads a little-endian 64-bit value from a byte buffer. */
static inline uint64_t rdf_le64(const uint8_t *bytes)
{
    return (uint64_t)rdf_le32(bytes) | (uint64_t)rdf_le32(bytes + 4) << 32;
}

/* Reads a big-endian 16-bit value from a byte buffer. */
static inline uint16_t rdf_be16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[1] | (uint16_t)bytes[0] << 8);
}

/* Reads a big-endian 32-bit value from a byte buffer. */
static inline uint32_t rdf_be32(const uint8_t *bytes)
{
    return (uint32_t)bytes[3] | (uint32_t)bytes[2] << 8 | (uint32_t)bytes[1] << 16 |
           (uint32_t)bytes[0] << 24;
}

/* Reads a big-endian 64-bit value from a byte buffer. */
static inline uint64_t rdf_be64(const uint8_t *bytes)
{
    return (uint64_t)rdf_be32(bytes + 4) | (uint64_t)rdf_be32(bytes) << 32;
}

/* Returns the eight-bit sum of a byte range, as firmware tables use. */
static inline uint8_t rdf_checksum(const uint8_t *bytes, size_t length)
{
    uint8_t sum = 0;
    for (size_t index = 0; index < length; ++index)
        sum = (uint8_t)(sum + bytes[index]);
    return sum;
}

#ifdef __cplusplus
}
#endif

#endif
