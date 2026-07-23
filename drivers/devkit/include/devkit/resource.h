#ifndef DEVKIT_RESOURCE_H
#define DEVKIT_RESOURCE_H

#include <devkit/base.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DK_RESOURCE_CACHEABLE (UINT64_C(1) << 0)
#define DK_RESOURCE_MMIO (UINT64_C(1) << 1)
#define DK_RESOURCE_MAY_BLOCK (UINT64_C(1) << 2)
#define DK_RESOURCE_IRQ_SAFE (UINT64_C(1) << 3)
#define DK_RESOURCE_ORDERLY (UINT64_C(1) << 4)
#define DK_RESOURCE_CONCURRENT (UINT64_C(1) << 5)
#define DK_RESOURCE_REENTRANT (UINT64_C(1) << 6)

#define DK_FIRMWARE_NAMESPACE UINT64_C(0x524f414e49584657)
#define DK_RESOURCE_FIRMWARE_ACPI_RSDP \
    ((struct dk_resource_key){DK_FIRMWARE_NAMESPACE, UINT64_C(1)})
#define DK_RESOURCE_FIRMWARE_DTB \
    ((struct dk_resource_key){DK_FIRMWARE_NAMESPACE, UINT64_C(2)})

#define DK_RESOURCE_DEVICE_FRONTEND                                      \
    ((struct dk_resource_key){UINT64_C(0x524f414e49584445), UINT64_C(1)})
#define DK_RESOURCE_CONSOLE_SERVICE                                      \
    ((struct dk_resource_key){UINT64_C(0x524f414e4958434f), UINT64_C(2)})

#define DK_PROPERTY_NAMESPACE UINT64_C(0x524f414e49585052)
#define DK_PROPERTY_SUBSYSTEM \
    ((struct dk_resource_key){DK_PROPERTY_NAMESPACE, UINT64_C(1)})
#define DK_PROPERTY_DEVICE_TYPE \
    ((struct dk_resource_key){DK_PROPERTY_NAMESPACE, UINT64_C(2)})
#define DK_PROPERTY_MODALIAS \
    ((struct dk_resource_key){DK_PROPERTY_NAMESPACE, UINT64_C(3)})
#define DK_PROPERTY_COMPATIBLE \
    ((struct dk_resource_key){DK_PROPERTY_NAMESPACE, UINT64_C(4)})
#define DK_PROPERTY_CLASS \
    ((struct dk_resource_key){DK_PROPERTY_NAMESPACE, UINT64_C(5)})

struct dk_data_resource {
    uint32_t size;
    uint64_t flags;
    dk_resource_lease_t lease;
    struct dk_slice data;
};

struct dk_protocol_resource {
    uint32_t size;
    uint32_t revision;
    uint64_t flags;
    dk_resource_lease_t lease;
    uintptr_t context;
    const void *operations;
    size_t operations_size;
};

struct dk_memory_resource {
    uint32_t size;
    uint64_t flags;
    dk_resource_lease_t lease;
    uint64_t length;
};

int32_t dk_root_bus(dk_bus_t *out_bus);
int32_t dk_bus_create(
    dk_node_t parent,
    struct dk_slice name,
    dk_bus_t *out_bus);
int32_t dk_device_create(
    dk_node_t parent,
    struct dk_slice name,
    dk_device_t *out_device);
int32_t dk_node_remove(dk_node_t node);
int32_t dk_node_set_property(
    dk_node_t node,
    struct dk_resource_key key,
    struct dk_slice value);
int32_t dk_node_read_property(
    dk_node_t node,
    struct dk_resource_key key,
    uint8_t *output,
    size_t output_len,
    size_t *written);

int32_t dk_resource_publish_data(
    dk_node_t node,
    struct dk_resource_key key,
    uint64_t flags,
    struct dk_slice data,
    dk_resource_t *out_resource);
int32_t dk_resource_publish_memory(
    dk_node_t node,
    struct dk_resource_key key,
    uint64_t flags,
    uint64_t physical,
    uint64_t length,
    dk_resource_t *out_resource);
int32_t dk_resource_publish_protocol(
    dk_node_t node,
    struct dk_resource_key key,
    uint64_t flags,
    uint32_t revision,
    uintptr_t context,
    const void *operations,
    size_t operations_size,
    dk_resource_t *out_resource);
int32_t dk_resource_remove(
    dk_node_t node,
    struct dk_resource_key key);
int32_t dk_resource_acquire_data(
    dk_node_t node,
    struct dk_resource_key key,
    struct dk_data_resource *out_resource);
int32_t dk_resource_acquire_memory(
    dk_node_t node,
    struct dk_resource_key key,
    struct dk_memory_resource *out_resource);
int32_t dk_resource_acquire_protocol(
    dk_node_t node,
    struct dk_resource_key key,
    uint32_t minimum_revision,
    struct dk_protocol_resource *out_resource);
int32_t dk_resource_release(dk_resource_lease_t lease);

#ifdef __cplusplus
}
#endif

#endif
