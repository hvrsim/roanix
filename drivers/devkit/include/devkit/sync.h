#ifndef DEVKIT_SYNC_H
#define DEVKIT_SYNC_H

#include <devkit/base.h>

#include <stdatomic.h>

struct dk_spinlock {
    atomic_flag held;
};

#define DK_SPINLOCK_INIT \
    { ATOMIC_FLAG_INIT }

static inline void dk_cpu_relax(void)
{
#if defined(__x86_64__)
    __asm__ volatile("pause");
#elif defined(__riscv)
    __asm__ volatile("nop");
#endif
}

static inline uintptr_t dk_irq_save(void)
{
#if defined(__x86_64__)
    uintptr_t flags;
    __asm__ volatile("pushfq; popq %0; cli" : "=r"(flags) :: "memory");
    return flags;
#elif defined(__riscv)
    uintptr_t previous;
    uintptr_t mask = 1u << 1;
    __asm__ volatile(
        "csrrc %0, sstatus, %1"
        : "=r"(previous)
        : "r"(mask)
        : "memory");
    return previous;
#else
#error "Unsupported DevKit driver architecture"
#endif
}

static inline void dk_irq_restore(uintptr_t state)
{
#if defined(__x86_64__)
    if ((state & (UINT64_C(1) << 9)) != 0)
        __asm__ volatile("sti" ::: "memory");
#elif defined(__riscv)
    if ((state & (UINT64_C(1) << 1)) != 0) {
        uintptr_t mask = 1u << 1;
        __asm__ volatile("csrs sstatus, %0" :: "r"(mask) : "memory");
    }
#endif
}

static inline uintptr_t dk_spin_lock_irqsave(struct dk_spinlock *lock)
{
    uintptr_t state = dk_irq_save();
    while (atomic_flag_test_and_set_explicit(&lock->held, memory_order_acquire))
        dk_cpu_relax();
    return state;
}

static inline void dk_spin_unlock_irqrestore(
    struct dk_spinlock *lock,
    uintptr_t state)
{
    atomic_flag_clear_explicit(&lock->held, memory_order_release);
    dk_irq_restore(state);
}

static inline uint16_t dk_read_le16(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | (uint16_t)bytes[1] << 8;
}

static inline uint32_t dk_read_le32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 |
           (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}

static inline uint64_t dk_read_le64(const uint8_t *bytes)
{
    return (uint64_t)dk_read_le32(bytes) |
           (uint64_t)dk_read_le32(bytes + 4) << 32;
}

static inline uint32_t dk_read_be32(const uint8_t *bytes)
{
    return (uint32_t)bytes[3] | (uint32_t)bytes[2] << 8 |
           (uint32_t)bytes[1] << 16 | (uint32_t)bytes[0] << 24;
}

static inline uint64_t dk_read_be64(const uint8_t *bytes)
{
    return (uint64_t)dk_read_be32(bytes + 4) |
           (uint64_t)dk_read_be32(bytes) << 32;
}

static inline uint8_t dk_checksum(const uint8_t *bytes, size_t length)
{
    uint8_t sum = 0;
    for (size_t index = 0; index < length; ++index)
        sum = (uint8_t)(sum + bytes[index]);
    return sum;
}

#if defined(__x86_64__)
static inline uint8_t dk_in8(uint16_t port)
{
    uint8_t value;
    __asm__ volatile("inb %1, %0" : "=a"(value) : "Nd"(port));
    return value;
}

static inline void dk_out8(uint16_t port, uint8_t value)
{
    __asm__ volatile("outb %0, %1" :: "a"(value), "Nd"(port));
}
#endif

#endif
