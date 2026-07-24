#ifndef DEVKIT_DEVICE_H
#define DEVKIT_DEVICE_H

#include <devkit/resource.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DK_DEVICE_CHARACTER 1u
#define DK_DEVICE_BLOCK 2u

#define DK_OPEN_READ (UINT32_C(1) << 0)
#define DK_OPEN_WRITE (UINT32_C(1) << 1)
#define DK_OPEN_APPEND (UINT32_C(1) << 5)
#define DK_OPEN_NONBLOCK (UINT32_C(1) << 8)

#define DK_POLL_IN UINT16_C(0x0001)
#define DK_POLL_PRI UINT16_C(0x0002)
#define DK_POLL_OUT UINT16_C(0x0004)
#define DK_POLL_ERR UINT16_C(0x0008)
#define DK_POLL_HUP UINT16_C(0x0010)
#define DK_POLL_NVAL UINT16_C(0x0020)
#define DK_POLL_RDNORM UINT16_C(0x0040)
#define DK_POLL_RDBAND UINT16_C(0x0080)
#define DK_POLL_WRNORM UINT16_C(0x0100)
#define DK_POLL_WRBAND UINT16_C(0x0200)

#define DK_CONSOLE_RESET_ON_LAST_CLOSE (UINT64_C(1) << 0)

typedef int32_t (*dk_device_open_fn)(
    uintptr_t context,
    uint32_t flags,
    uintptr_t *out_file_context);
typedef void (*dk_device_close_fn)(
    uintptr_t context,
    uintptr_t file_context,
    uint32_t flags);
typedef int64_t (*dk_device_initial_offset_fn)(
    uintptr_t context,
    uintptr_t file_context,
    uint32_t flags);
typedef int64_t (*dk_device_read_fn)(
    uintptr_t context,
    uintptr_t file_context,
    uint64_t offset,
    uint8_t *data,
    size_t len,
    uint32_t flags);
typedef int64_t (*dk_device_write_fn)(
    uintptr_t context,
    uintptr_t file_context,
    uint64_t offset,
    const uint8_t *data,
    size_t len,
    uint32_t flags);
typedef uint64_t (*dk_device_size_fn)(uintptr_t context);
typedef int32_t (*dk_device_sync_fn)(uintptr_t context);
typedef int64_t (*dk_device_poll_fn)(
    uintptr_t context,
    uintptr_t file_context,
    uint64_t offset,
    uint16_t events,
    uint32_t flags);
typedef dk_event_t (*dk_device_event_fn)(
    uintptr_t context,
    uintptr_t file_context);
typedef int64_t (*dk_device_ioctl_fn)(
    uintptr_t context,
    uintptr_t file_context,
    uintptr_t process_id,
    int32_t process_group,
    int32_t session_id,
    uint8_t is_session_leader,
    uint64_t request,
    uint64_t value,
    uint8_t *argument,
    size_t argument_len);

struct dk_device_ops {
    uint32_t size;
    uintptr_t context;
    dk_device_open_fn open;
    dk_device_close_fn close;
    dk_device_initial_offset_fn initial_offset;
    dk_device_read_fn read;
    dk_device_write_fn write;
    dk_device_size_fn size_bytes;
    dk_device_sync_fn sync;
    dk_device_poll_fn poll;
    dk_device_ioctl_fn ioctl;
    dk_device_event_fn readable_event;
    dk_device_event_fn writable_event;
    dk_device_event_fn hangup_event;
};

struct dk_serial_settings {
    uint32_t baud;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t odd_parity;
};

typedef int32_t (*dk_console_open_fn)(uintptr_t context);
typedef void (*dk_console_close_fn)(uintptr_t context);
typedef int32_t (*dk_console_try_read_fn)(
    uintptr_t context,
    uint8_t *out_byte);
typedef int64_t (*dk_console_read_fn)(
    uintptr_t context,
    uint8_t *output,
    size_t length);
typedef int32_t (*dk_console_write_fn)(
    uintptr_t context,
    const uint8_t *data,
    size_t len,
    uint8_t nonblocking);
typedef int32_t (*dk_console_configure_fn)(
    uintptr_t context,
    const struct dk_serial_settings *settings);
typedef int32_t (*dk_console_simple_fn)(uintptr_t context);
typedef int32_t (*dk_console_break_fn)(
    uintptr_t context,
    uint64_t duration_ms);
typedef int32_t (*dk_console_state_fn)(uintptr_t context);
typedef int64_t (*dk_console_queued_fn)(uintptr_t context);
typedef void (*dk_console_destroy_fn)(uintptr_t context);

struct dk_console_ops {
    uint32_t size;
    uint64_t flags;
    uintptr_t context;
    dk_console_open_fn open;
    dk_console_close_fn close;
    dk_console_try_read_fn try_read;
    dk_console_write_fn write;
    dk_console_configure_fn configure;
    dk_console_simple_fn flush;
    dk_console_break_fn send_break;
    dk_console_state_fn writable;
    dk_console_state_fn hung_up;
    dk_console_destroy_fn destroy;
    dk_console_read_fn read;
    dk_console_simple_fn flush_input;
    dk_console_simple_fn flush_output;
    dk_console_queued_fn queued_output;
    dk_event_t readable_event;
    dk_event_t writable_event;
    dk_event_t hangup_event;
};

int32_t dk_devfs_root(dk_devnode_t *out_node);
int32_t dk_devfs_create_dir(
    dk_devnode_t parent,
    struct dk_slice name,
    uint16_t mode,
    dk_devnode_t *out_node);
int32_t dk_devfs_create_device(
    dk_devnode_t parent,
    struct dk_slice name,
    uint32_t kind,
    uint16_t mode,
    dk_device_t device,
    const struct dk_device_ops *operations,
    dk_devnode_t *out_node);
int32_t dk_devfs_remove_node(dk_devnode_t node);

int32_t dk_console_bus(dk_bus_t *out_bus);
int32_t dk_console_create_tty(
    dk_device_t device,
    dk_devnode_t parent,
    struct dk_slice name,
    uint16_t mode,
    struct dk_slice path,
    uint32_t baud,
    const struct dk_console_ops *operations,
    dk_devnode_t *out_node);

#ifdef __cplusplus
}
#endif

#endif
