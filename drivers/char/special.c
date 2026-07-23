#include <devkit/devkit.h>

enum special_kind {
    SPECIAL_KMSG,
    SPECIAL_NULL,
    SPECIAL_ZERO,
    SPECIAL_RANDOM,
};

static uint8_t console_output_disabled;

static int32_t special_open(
    uintptr_t context,
    uint32_t flags,
    uintptr_t *out_file_context)
{
    (void)flags;
    if (out_file_context == NULL)
        return DK_EINVAL;
    *out_file_context = context == SPECIAL_KMSG
        ? (uintptr_t)dk_kmsg_end()
        : 0;
    return DK_OK;
}

static int64_t special_initial_offset(
    uintptr_t context,
    uintptr_t file_context,
    uint32_t flags)
{
    (void)file_context;
    (void)flags;
    if (context != SPECIAL_KMSG)
        return 0;
    uint64_t offset = dk_kmsg_start();
    return offset <= INT64_MAX ? (int64_t)offset : DK_EIO;
}

static int64_t special_read(
    uintptr_t context,
    uintptr_t file_context,
    uint64_t offset,
    uint8_t *data,
    size_t len,
    uint32_t flags)
{
    (void)flags;
    switch ((enum special_kind)context) {
    case SPECIAL_KMSG: {
        uint64_t snapshot_end = (uint64_t)file_context;
        if (offset >= snapshot_end)
            return 0;
        uint64_t remaining = snapshot_end - offset;
        if ((uint64_t)len > remaining)
            len = (size_t)remaining;
        int64_t result = dk_kmsg_read(offset, data, len, 1);
        return result == DK_EAGAIN ? 0 : result;
    }
    case SPECIAL_NULL:
        return 0;
    case SPECIAL_ZERO:
        memset(data, 0, len);
        return (int64_t)len;
    case SPECIAL_RANDOM:
        dk_random_fill(data, len);
        return (int64_t)len;
    }
    return DK_EINVAL;
}

static int64_t special_write(
    uintptr_t context,
    uintptr_t file_context,
    uint64_t offset,
    const uint8_t *data,
    size_t len,
    uint32_t flags)
{
    (void)file_context;
    (void)offset;
    (void)flags;
    switch ((enum special_kind)context) {
    case SPECIAL_KMSG: {
        int32_t status = dk_kmsg_append(data, len);
        return status == DK_OK ? (int64_t)len : status;
    }
    case SPECIAL_RANDOM:
        dk_random_mix(data, len);
        return (int64_t)len;
    case SPECIAL_NULL:
    case SPECIAL_ZERO:
        return (int64_t)len;
    }
    return DK_EINVAL;
}

static uint64_t special_size(uintptr_t context)
{
    return context == SPECIAL_KMSG ? dk_kmsg_end() : 0;
}

static int64_t special_poll(
    uintptr_t context,
    uintptr_t file_context,
    uint64_t offset,
    uint16_t events,
    uint32_t flags)
{
    (void)flags;
    uint16_t ready = events & (DK_POLL_OUT | DK_POLL_WRNORM);

    if (context == SPECIAL_KMSG) {
        uint64_t snapshot_end = (uint64_t)file_context;
        if (offset < dk_kmsg_start())
            ready |= DK_POLL_ERR;
        else if (offset < snapshot_end)
            ready |= events & (DK_POLL_IN | DK_POLL_RDNORM);
        else
            ready |= DK_POLL_HUP;
        return ready;
    }
    return events & (DK_POLL_IN | DK_POLL_RDNORM |
                     DK_POLL_OUT | DK_POLL_WRNORM);
}

static const struct dk_device_ops kmsg_operations = {
    .size = sizeof(kmsg_operations),
    .context = SPECIAL_KMSG,
    .open = special_open,
    .close = NULL,
    .initial_offset = special_initial_offset,
    .read = special_read,
    .write = special_write,
    .size_bytes = special_size,
    .sync = NULL,
    .poll = special_poll,
    .ioctl = NULL,
};

static const struct dk_device_ops null_operations = {
    .size = sizeof(null_operations),
    .context = SPECIAL_NULL,
    .open = NULL,
    .close = NULL,
    .initial_offset = NULL,
    .read = special_read,
    .write = special_write,
    .size_bytes = NULL,
    .sync = NULL,
    .poll = special_poll,
    .ioctl = NULL,
};

static const struct dk_device_ops zero_operations = {
    .size = sizeof(zero_operations),
    .context = SPECIAL_ZERO,
    .open = NULL,
    .close = NULL,
    .initial_offset = NULL,
    .read = special_read,
    .write = special_write,
    .size_bytes = NULL,
    .sync = NULL,
    .poll = special_poll,
    .ioctl = NULL,
};

static const struct dk_device_ops random_operations = {
    .size = sizeof(random_operations),
    .context = SPECIAL_RANDOM,
    .open = NULL,
    .close = NULL,
    .initial_offset = NULL,
    .read = special_read,
    .write = special_write,
    .size_bytes = NULL,
    .sync = NULL,
    .poll = special_poll,
    .ioctl = NULL,
};

static int32_t publish_device(
    dk_bus_t bus,
    dk_devnode_t devfs,
    const char *name,
    size_t name_len,
    uint16_t mode,
    const struct dk_device_ops *operations)
{
    struct dk_slice slice = {
        .data = (const uint8_t *)name,
        .len = name_len,
    };
    dk_device_t device = 0;
    dk_devnode_t node = 0;
    int32_t status = dk_device_create(bus, slice, &device);
    if (status != DK_OK)
        return status;
    status = dk_node_set_property(
        device,
        DK_PROPERTY_SUBSYSTEM,
        DK_SLICE_LITERAL("mem"));
    if (status != DK_OK)
        return status;
    return dk_devfs_create_device(
        devfs,
        slice,
        DK_DEVICE_CHARACTER,
        mode,
        device,
        operations,
        &node);
}

static int32_t special_start(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;

    dk_bus_t root = 0;
    dk_bus_t bus = 0;
    dk_devnode_t devfs = 0;
    int32_t status = dk_root_bus(&root);
    if (status != DK_OK)
        return status;
    status = dk_bus_create(root, DK_SLICE_LITERAL("special"), &bus);
    if (status != DK_OK)
        return status;
    status = dk_devfs_root(&devfs);
    if (status != DK_OK)
        return status;

    status = publish_device(
        bus,
        devfs,
        "kmsg",
        4,
        0600,
        &kmsg_operations);
    if (status != DK_OK)
        return status;

    status = publish_device(
        bus,
        devfs,
        "null",
        4,
        0666,
        &null_operations);
    if (status != DK_OK)
        return status;
    status = publish_device(
        bus,
        devfs,
        "zero",
        4,
        0666,
        &zero_operations);
    if (status != DK_OK)
        return status;
    status = publish_device(
        bus,
        devfs,
        "random",
        6,
        0666,
        &random_operations);
    if (status != DK_OK)
        return status;
    status = publish_device(
        bus,
        devfs,
        "urandom",
        7,
        0666,
        &random_operations);
    if (status != DK_OK)
        return status;
    dk_kmsg_disable_console_output();
    console_output_disabled = 1;
    return DK_OK;
}

static void special_stop(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;
    if (console_output_disabled != 0) {
        dk_kmsg_enable_console_output();
        console_output_disabled = 0;
    }
}

static const struct dk_driver_definition driver = {
    .name = DK_SLICE_LITERAL("special"),
    .context = 0,
    .start = special_start,
    .stop = special_stop,
};

DK_DRIVER_EXPORT(driver)
