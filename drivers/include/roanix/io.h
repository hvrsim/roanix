/*
 * Roanix driver framework: register and port access.
 *
 * Every accessor here is inline. Once a window is mapped, touching a device
 * register costs a single load or store plus whatever barrier the access
 * ordering requires - no framework call is involved.
 */
#ifndef ROANIX_IO_H
#define ROANIX_IO_H

#include <roanix/api.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Barriers ----------------------------------------------------------- */

/* Orders all preceding memory accesses before all following ones. */
static inline void rdf_mb(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

/* Orders preceding reads before following reads. */
static inline void rdf_rmb(void)
{
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
}

/* Orders preceding writes before following writes. */
static inline void rdf_wmb(void)
{
    __atomic_thread_fence(__ATOMIC_RELEASE);
}

/* --- Register access ---------------------------------------------------- */

/* Returns the address of an offset inside a mapped window. */
static inline volatile void *rdf_mmio_at(const struct rdf_mmio *window, size_t offset)
{
    return (volatile uint8_t *)window->base + offset;
}

/* Reads one byte from a mapped window. */
static inline uint8_t rdf_read8(const struct rdf_mmio *window, size_t offset)
{
    return *(const volatile uint8_t *)rdf_mmio_at(window, offset);
}

/* Reads two bytes from a mapped window. */
static inline uint16_t rdf_read16(const struct rdf_mmio *window, size_t offset)
{
    return *(const volatile uint16_t *)rdf_mmio_at(window, offset);
}

/* Reads four bytes from a mapped window. */
static inline uint32_t rdf_read32(const struct rdf_mmio *window, size_t offset)
{
    return *(const volatile uint32_t *)rdf_mmio_at(window, offset);
}

/* Reads eight bytes from a mapped window. */
static inline uint64_t rdf_read64(const struct rdf_mmio *window, size_t offset)
{
    return *(const volatile uint64_t *)rdf_mmio_at(window, offset);
}

/* Writes one byte to a mapped window. */
static inline void rdf_write8(const struct rdf_mmio *window, size_t offset, uint8_t value)
{
    *(volatile uint8_t *)rdf_mmio_at(window, offset) = value;
}

/* Writes two bytes to a mapped window. */
static inline void rdf_write16(const struct rdf_mmio *window, size_t offset, uint16_t value)
{
    *(volatile uint16_t *)rdf_mmio_at(window, offset) = value;
}

/* Writes four bytes to a mapped window. */
static inline void rdf_write32(const struct rdf_mmio *window, size_t offset, uint32_t value)
{
    *(volatile uint32_t *)rdf_mmio_at(window, offset) = value;
}

/* Writes eight bytes to a mapped window. */
static inline void rdf_write64(const struct rdf_mmio *window, size_t offset, uint64_t value)
{
    *(volatile uint64_t *)rdf_mmio_at(window, offset) = value;
}

/*
 * Spins until a masked register field reaches `want`, or the timeout expires.
 *
 * Returns RDF_OK on success and RDF_ETIMEDOUT otherwise. This is the shape
 * every controller reset and doorbell handshake needs. It lives in
 * <roanix/driver.h> because it needs the kernel's clock.
 */

/* --- Port access -------------------------------------------------------- */

#if defined(__x86_64__)
#define RDF_HAVE_PORT_IO 1

/* Reads one byte from an I/O port. */
static inline uint8_t rdf_in8(uint16_t port)
{
    uint8_t value;
    __asm__ volatile("inb %1, %0" : "=a"(value) : "Nd"(port) : "memory");
    return value;
}

/* Reads two bytes from an I/O port. */
static inline uint16_t rdf_in16(uint16_t port)
{
    uint16_t value;
    __asm__ volatile("inw %1, %0" : "=a"(value) : "Nd"(port) : "memory");
    return value;
}

/* Reads four bytes from an I/O port. */
static inline uint32_t rdf_in32(uint16_t port)
{
    uint32_t value;
    __asm__ volatile("inl %1, %0" : "=a"(value) : "Nd"(port) : "memory");
    return value;
}

/* Writes one byte to an I/O port. */
static inline void rdf_out8(uint16_t port, uint8_t value)
{
    __asm__ volatile("outb %0, %1" : : "a"(value), "Nd"(port) : "memory");
}

/* Writes two bytes to an I/O port. */
static inline void rdf_out16(uint16_t port, uint16_t value)
{
    __asm__ volatile("outw %0, %1" : : "a"(value), "Nd"(port) : "memory");
}

/* Writes four bytes to an I/O port. */
static inline void rdf_out32(uint16_t port, uint32_t value)
{
    __asm__ volatile("outl %0, %1" : : "a"(value), "Nd"(port) : "memory");
}
#else
#define RDF_HAVE_PORT_IO 0
#endif

#ifdef __cplusplus
}
#endif

#endif
