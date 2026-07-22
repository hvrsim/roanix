#ifndef ROANIX_DRIVER_H
#define ROANIX_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROANIX_DRIVER_ABI_V1 1u
#define ROANIX_INTERRUPT_ABI_V1 1u

#define ROANIX_OK 0
#define ROANIX_EINVAL (-1)
#define ROANIX_ENOENT (-2)
#define ROANIX_EEXIST (-3)
#define ROANIX_EKIND (-4)
#define ROANIX_EPERM (-5)
#define ROANIX_EBUSY (-6)
#define ROANIX_EABI (-7)
#define ROANIX_ENOTSUP (-8)
#define ROANIX_ENOSPC (-9)
#define ROANIX_EIO (-10)
#define ROANIX_ENOMEM (-11)

#define ROANIX_DEV_CHARACTER 1u
#define ROANIX_DEV_BLOCK 2u

#define ROANIX_LOG_ERROR 1u
#define ROANIX_LOG_WARN 2u
#define ROANIX_LOG_INFO 3u
#define ROANIX_LOG_DEBUG 4u
#define ROANIX_LOG_TRACE 5u

#define ROANIX_RESOURCE_CACHEABLE (UINT64_C(1) << 0)
#define ROANIX_RESOURCE_MMIO (UINT64_C(1) << 1)
#define ROANIX_RESOURCE_MAY_BLOCK (UINT64_C(1) << 2)
/* Reserved until resource lookup is interrupt-safe; publication returns ENOTSUP. */
#define ROANIX_RESOURCE_IRQ_SAFE (UINT64_C(1) << 3)

#define ROANIX_INTERRUPT_CONTROLLER_ROOT (UINT64_C(1) << 0)

#define ROANIX_INTERRUPT_EDGE (UINT64_C(1) << 0)
#define ROANIX_INTERRUPT_LEVEL (UINT64_C(1) << 1)
#define ROANIX_INTERRUPT_ACTIVE_HIGH (UINT64_C(1) << 2)
#define ROANIX_INTERRUPT_ACTIVE_LOW (UINT64_C(1) << 3)
#define ROANIX_INTERRUPT_START_MASKED (UINT64_C(1) << 4)

#define ROANIX_INTERRUPT_RESCHEDULE (UINT32_C(1) << 0)

#define ROANIX_INTERRUPT_DOMAIN_NAMESPACE UINT64_C(0x524F414E49584952)
#define ROANIX_INTERRUPT_DOMAIN_RESOURCE UINT64_C(1)

struct roanix_driver_host_v1;

struct roanix_slice {
    const uint8_t *data;
    size_t len;
};

struct roanix_resource_key {
    uint64_t namespace_id;
    uint64_t resource_id;
};

struct roanix_interrupt_route_v1 {
    uint32_t size;
    uint32_t abi_version;
    uint64_t interrupt;
    uint32_t vector;
    uint32_t target_cpu;
    uint64_t target_platform_id;
    uint64_t flags;
    struct roanix_slice specifier;
};

typedef int32_t (*roanix_interrupt_connect_fn)(
    uintptr_t context,
    const struct roanix_interrupt_route_v1 *route,
    uint64_t *out_cookie);
typedef int32_t (*roanix_interrupt_disconnect_fn)(
    uintptr_t context,
    uint64_t cookie);
typedef int32_t (*roanix_interrupt_line_fn)(
    uintptr_t context,
    uint64_t cookie);
typedef int32_t (*roanix_interrupt_set_affinity_fn)(
    uintptr_t context,
    uint64_t cookie,
    const struct roanix_interrupt_route_v1 *route);
/* IRQ-safe: must not block, allocate, or call thread-context host services. */
typedef int32_t (*roanix_interrupt_claim_fn)(
    uintptr_t context,
    uint32_t cpu,
    uint64_t platform_id,
    uint64_t *out_interrupt,
    uint64_t *out_cookie);
/* IRQ-safe: completes one value returned by the matching claim callback. */
typedef void (*roanix_interrupt_complete_fn)(
    uintptr_t context,
    uint32_t cpu,
    uint64_t platform_id,
    uint64_t interrupt,
    uint64_t cookie);
/* IRQ-safe: return ROANIX_INTERRUPT_RESCHEDULE when trap return must schedule. */
typedef uint32_t (*roanix_interrupt_handler_fn)(
    uintptr_t context,
    uint64_t interrupt);

struct roanix_interrupt_controller_v1 {
    uint32_t size;
    uint32_t abi_version;
    uint64_t flags;
    uintptr_t context;
    roanix_interrupt_connect_fn connect;
    roanix_interrupt_disconnect_fn disconnect;
    roanix_interrupt_line_fn mask;
    roanix_interrupt_line_fn unmask;
    roanix_interrupt_set_affinity_fn set_affinity;
    roanix_interrupt_claim_fn claim;
    roanix_interrupt_complete_fn complete;
};

typedef int32_t (*roanix_resource_fn)(
    uintptr_t context,
    const uint8_t *input,
    size_t input_len,
    uint8_t *output,
    size_t output_len,
    size_t *written);

typedef int32_t (*roanix_driver_init_fn)(
    const struct roanix_driver_host_v1 *host,
    uint64_t driver,
    uintptr_t context);

typedef void (*roanix_driver_fini_fn)(uint64_t driver, uintptr_t context);

struct roanix_driver_module_v1 {
    uint32_t size;
    uint32_t abi_version;
    struct roanix_slice name;
    uintptr_t context;
    roanix_driver_init_fn init;
    roanix_driver_fini_fn fini;
};

typedef int32_t (*roanix_device_open_fn)(uintptr_t context, uint32_t flags);
typedef void (*roanix_device_close_fn)(uintptr_t context, uint32_t flags);
typedef int64_t (*roanix_device_read_fn)(
    uintptr_t context,
    uint64_t offset,
    uint8_t *data,
    size_t len);
typedef int64_t (*roanix_device_write_fn)(
    uintptr_t context,
    uint64_t offset,
    const uint8_t *data,
    size_t len);
typedef uint64_t (*roanix_device_size_fn)(uintptr_t context);
typedef int32_t (*roanix_device_sync_fn)(uintptr_t context);
typedef int64_t (*roanix_device_ioctl_fn)(
    uintptr_t context,
    uintptr_t process_id,
    int32_t process_group,
    int32_t session_id,
    uint8_t is_session_leader,
    uint64_t request,
    uint64_t value,
    uint8_t *argument,
    size_t argument_len);

struct roanix_device_ops_v1 {
    uint32_t size;
    uint32_t abi_version;
    uintptr_t context;
    roanix_device_open_fn open;
    roanix_device_close_fn close;
    roanix_device_read_fn read;
    roanix_device_write_fn write;
    roanix_device_size_fn size_bytes;
    roanix_device_sync_fn sync;
    roanix_device_ioctl_fn ioctl;
};

struct roanix_driver_host_v1 {
    uint32_t size;
    uint32_t abi_version;
    int32_t (*root_bus)(uint64_t *out_bus);
    int32_t (*register_bus)(
        uint64_t driver,
        uint64_t parent,
        struct roanix_slice name,
        uint64_t *out_bus);
    int32_t (*register_device)(
        uint64_t driver,
        uint64_t parent,
        struct roanix_slice name,
        uint64_t *out_device);
    int32_t (*remove_node)(uint64_t driver, uint64_t node);
    int32_t (*publish_data_resource)(
        uint64_t driver,
        uint64_t bus,
        struct roanix_resource_key key,
        uint64_t flags,
        struct roanix_slice data,
        uint64_t *out_resource);
    int32_t (*publish_method_resource)(
        uint64_t driver,
        uint64_t bus,
        struct roanix_resource_key key,
        uint64_t flags,
        uintptr_t context,
        roanix_resource_fn callback,
        uint64_t *out_resource);
    int32_t (*read_resource)(
        uint64_t node,
        struct roanix_resource_key key,
        uint8_t *output,
        size_t output_len,
        size_t *written);
    int32_t (*invoke_resource)(
        uint64_t node,
        struct roanix_resource_key key,
        /* Input and output ranges must not overlap. */
        struct roanix_slice input,
        uint8_t *output,
        size_t output_len,
        size_t *written);
    int32_t (*devfs_root)(uint64_t *out_node);
    int32_t (*devfs_create_dir)(
        uint64_t driver,
        uint64_t parent,
        struct roanix_slice name,
        uint16_t mode,
        uint64_t *out_node);
    int32_t (*devfs_create_device)(
        uint64_t driver,
        uint64_t parent,
        struct roanix_slice name,
        uint32_t kind,
        uint16_t mode,
        uint64_t device,
        const struct roanix_device_ops_v1 *operations,
        uint64_t *out_node);
    int32_t (*devfs_remove_node)(uint64_t driver, uint64_t node);
    uint8_t *(*allocate)(size_t size, size_t align);
    uint8_t *(*allocate_zeroed)(size_t size, size_t align);
    int32_t (*deallocate)(uint8_t *data, size_t size, size_t align);
    int32_t (*log)(uint32_t level, struct roanix_slice message);
    int32_t (*register_interrupt_controller)(
        uint64_t driver,
        uint64_t bus,
        const struct roanix_interrupt_controller_v1 *controller,
        uint64_t *out_controller);
    int32_t (*unregister_interrupt_controller)(
        uint64_t driver,
        uint64_t controller);
    int32_t (*request_interrupt)(
        uint64_t driver,
        uint64_t node,
        struct roanix_slice specifier,
        uint64_t flags,
        uint32_t target_cpu,
        roanix_interrupt_handler_fn handler,
        uintptr_t context,
        uint64_t *out_interrupt);
    int32_t (*release_interrupt)(uint64_t driver, uint64_t interrupt);
    int32_t (*mask_interrupt)(uint64_t driver, uint64_t interrupt);
    int32_t (*unmask_interrupt)(uint64_t driver, uint64_t interrupt);
    int32_t (*set_interrupt_affinity)(
        uint64_t driver,
        uint64_t interrupt,
        uint32_t target_cpu);
    /*
     * Establishes a persistent uncached/device mapping in the kernel direct
     * map. The returned address remains valid after driver unload.
     */
    int32_t (*map_mmio)(
        uint64_t driver,
        uint64_t physical,
        size_t size,
        uintptr_t *out_address);
};

#define ROANIX_HOST_HAS(host, field)                                         \
    ((host) != NULL &&                                                       \
     (host)->size >= offsetof(struct roanix_driver_host_v1, field) +         \
                         sizeof((host)->field))

int32_t roanix_driver_load_v1(
    const struct roanix_driver_module_v1 *module,
    uint64_t *out_driver);
int32_t roanix_driver_unload_v1(uint64_t driver);

#if defined(__GNUC__) || defined(__clang__)
#define ROANIX_DRIVER_EXPORT(symbol)                                          \
    static const struct roanix_driver_module_v1 *const                        \
        roanix_driver_export_##symbol                                         \
        __attribute__((used, section(".roanix_drivers"), aligned(sizeof(void *)))) = \
            &(symbol)
#else
#error "ROANIX_DRIVER_EXPORT requires a compiler with section attributes"
#endif

#ifdef __cplusplus
#define ROANIX_SLICE_LITERAL(text) \
    roanix_slice{(const uint8_t *)(text), sizeof(text) - 1u}
#else
#define ROANIX_SLICE_LITERAL(text) \
    ((struct roanix_slice){(const uint8_t *)(text), sizeof(text) - 1u})
#endif

#ifdef __cplusplus
}
#endif

#endif
