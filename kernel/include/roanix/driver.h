#ifndef ROANIX_DRIVER_H
#define ROANIX_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROANIX_DRIVER_ABI_V1 1u

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

struct roanix_driver_host_v1;

struct roanix_slice {
    const uint8_t *data;
    size_t len;
};

struct roanix_resource_key {
    uint64_t namespace_id;
    uint64_t resource_id;
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
};

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
