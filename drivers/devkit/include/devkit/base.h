#ifndef DEVKIT_BASE_H
#define DEVKIT_BASE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DK_OK 0
#define DK_EINVAL (-1)
#define DK_ENOENT (-2)
#define DK_EEXIST (-3)
#define DK_EKIND (-4)
#define DK_EPERM (-5)
#define DK_EBUSY (-6)
#define DK_ENOTSUP (-7)
#define DK_ENOSPC (-8)
#define DK_EIO (-9)
#define DK_ENOMEM (-10)
#define DK_EAGAIN (-11)
#define DK_EINTR (-12)
#define DK_ENOTTY (-13)
#define DK_ESPIPE (-14)
#define DK_EDEADLK (-15)
#define DK_EDEFER (-16)

#define DK_ABI_MAJOR 1u
#define DK_ABI_MINOR 0u

#define DK_LOG_ERROR 1u
#define DK_LOG_WARN 2u
#define DK_LOG_INFO 3u
#define DK_LOG_DEBUG 4u
#define DK_LOG_TRACE 5u

typedef uint64_t dk_driver_t;
typedef uint64_t dk_node_t;
typedef uint64_t dk_bus_t;
typedef uint64_t dk_device_t;
typedef uint64_t dk_resource_t;
typedef uint64_t dk_resource_lease_t;
typedef uint64_t dk_devnode_t;
typedef uintptr_t dk_event_t;

struct dk_slice {
    const uint8_t *data;
    size_t len;
};

struct dk_resource_key {
    uint64_t namespace_id;
    uint64_t resource_id;
};

struct dk_abi_service_header {
    uint32_t size;
    uint32_t revision;
};

struct dk_abi_bootstrap {
    uint32_t size;
    uint16_t abi_major;
    uint16_t abi_minor;
    int32_t (*get_service)(
        uint64_t service,
        uint32_t minimum_revision,
        const struct dk_abi_service_header **out_service);
};

typedef int32_t (*dk_abi_init_fn)(
    const struct dk_abi_bootstrap *bootstrap,
    dk_driver_t driver,
    uintptr_t context);
typedef void (*dk_abi_fini_fn)(dk_driver_t driver, uintptr_t context);

struct dk_abi_driver_module {
    uint32_t size;
    uint16_t abi_major;
    uint16_t abi_minor;
    uint32_t flags;
    struct dk_slice name;
    uintptr_t context;
    dk_abi_init_fn init;
    dk_abi_fini_fn fini;
};

typedef int32_t (*dk_driver_start_fn)(dk_driver_t driver, uintptr_t context);
typedef void (*dk_driver_stop_fn)(dk_driver_t driver, uintptr_t context);

struct dk_driver_class;

struct dk_driver_definition {
    struct dk_slice name;
    uintptr_t context;
    dk_driver_start_fn start;
    dk_driver_stop_fn stop;
    const struct dk_driver_class *classes;
    size_t class_count;
};

int32_t dk_runtime_start(
    const struct dk_abi_bootstrap *bootstrap,
    dk_driver_t driver,
    const struct dk_driver_definition *definition);
void dk_runtime_stop(
    dk_driver_t driver,
    const struct dk_driver_definition *definition);

dk_driver_t dk_driver_current(void);

void *dk_allocate(size_t size, size_t align);
void *dk_allocate_zeroed(size_t size, size_t align);
int32_t dk_deallocate(void *data, size_t size, size_t align);

int32_t dk_log(uint32_t level, struct dk_slice message);
int32_t dk_log_literal(
    uint32_t level,
    const char *message,
    size_t length);

int32_t dk_event_create(dk_event_t *out_event);
int32_t dk_event_destroy(dk_event_t event);
int32_t dk_event_wait(dk_event_t event);
int32_t dk_event_reset(dk_event_t event);
int64_t dk_event_signal(dk_event_t event);

void dk_sleep_ns(uint64_t nanoseconds);
void dk_random_fill(uint8_t *output, size_t len);
void dk_random_mix(const uint8_t *input, size_t len);

uint64_t dk_kmsg_start(void);
uint64_t dk_kmsg_end(void);
int64_t dk_kmsg_read(
    uint64_t offset,
    uint8_t *output,
    size_t len,
    uint8_t nonblocking);
int32_t dk_kmsg_append(const uint8_t *input, size_t len);
void dk_kmsg_disable_console_output(void);
void dk_kmsg_enable_console_output(void);

void *memcpy(void *restrict destination, const void *restrict source, size_t length);
void *memmove(void *destination, const void *source, size_t length);
void *memset(void *destination, int value, size_t length);
int memcmp(const void *left, const void *right, size_t length);
size_t strlen(const char *string);

#define DK_SLICE_LITERAL(text) \
    ((struct dk_slice){(const uint8_t *)(text), sizeof(text) - 1u})

#define DK_LOG_LITERAL(level, text) \
    dk_log_literal((level), (text), sizeof(text) - 1u)

#if defined(__GNUC__) || defined(__clang__)
#define DK_DRIVER_EXPORT(symbol)                                               \
    static int32_t dk_export_init(                                             \
        const struct dk_abi_bootstrap *bootstrap,                              \
        dk_driver_t driver,                                                    \
        uintptr_t context)                                                     \
    {                                                                          \
        return dk_runtime_start(                                               \
            bootstrap,                                                         \
            driver,                                                            \
            (const struct dk_driver_definition *)context);                     \
    }                                                                          \
    static void dk_export_fini(dk_driver_t driver, uintptr_t context)          \
    {                                                                          \
        dk_runtime_stop(                                                       \
            driver,                                                            \
            (const struct dk_driver_definition *)context);                     \
    }                                                                          \
    __attribute__((used, visibility("default")))                              \
    const struct dk_abi_driver_module *roanix_driver_entry(void)               \
    {                                                                          \
        static struct dk_abi_driver_module module;                             \
        module = (struct dk_abi_driver_module){                                \
            .size = sizeof(module),                                            \
            .abi_major = DK_ABI_MAJOR,                                         \
            .abi_minor = DK_ABI_MINOR,                                         \
            .flags = 0,                                                        \
            .name = (symbol).name,                                             \
            .context = (uintptr_t)&(symbol),                                   \
            .init = dk_export_init,                                            \
            .fini = dk_export_fini,                                            \
        };                                                                     \
        return &module;                                                        \
    }
#else
#error "DevKit driver export requires GCC or Clang"
#endif

#ifdef __cplusplus
}
#endif

#endif
