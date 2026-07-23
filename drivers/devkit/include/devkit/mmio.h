#ifndef DEVKIT_MMIO_H
#define DEVKIT_MMIO_H

#include <devkit/resource.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint64_t dk_mmio_mapping_t;

struct dk_mmio_mapping {
    uint32_t size;
    uint32_t reserved;
    dk_mmio_mapping_t handle;
    uintptr_t address;
    size_t length;
};

int32_t dk_mmio_map(
    uint64_t physical,
    size_t size,
    uintptr_t *out_address);
int32_t dk_mmio_map_resource(
    const struct dk_memory_resource *resource,
    uint64_t offset,
    size_t size,
    struct dk_mmio_mapping *out_mapping);
int32_t dk_mmio_unmap(dk_mmio_mapping_t mapping);
int32_t dk_firmware_physical_to_virtual(
    uint64_t physical,
    uintptr_t *out_address);

static inline uint8_t dk_mmio_read8(uintptr_t address)
{
    return *(const volatile uint8_t *)address;
}

static inline uint16_t dk_mmio_read16(uintptr_t address)
{
    return *(const volatile uint16_t *)address;
}

static inline uint32_t dk_mmio_read32(uintptr_t address)
{
    return *(const volatile uint32_t *)address;
}

static inline uint64_t dk_mmio_read64(uintptr_t address)
{
    return *(const volatile uint64_t *)address;
}

static inline void dk_mmio_write8(uintptr_t address, uint8_t value)
{
    *(volatile uint8_t *)address = value;
}

static inline void dk_mmio_write16(uintptr_t address, uint16_t value)
{
    *(volatile uint16_t *)address = value;
}

static inline void dk_mmio_write32(uintptr_t address, uint32_t value)
{
    *(volatile uint32_t *)address = value;
}

static inline void dk_mmio_write64(uintptr_t address, uint64_t value)
{
    *(volatile uint64_t *)address = value;
}

static inline void dk_mmio_fence(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

#ifdef __cplusplus
}
#endif

#endif
