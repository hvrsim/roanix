#include <roanix/driver.h>

enum special_kind {
    SPECIAL_KMSG,
    SPECIAL_NULL,
    SPECIAL_ZERO,
    SPECIAL_RANDOM,
};

static uint8_t console_output_disabled;

static int32_t special_open(
    void *context,
    uint32_t flags,
    uintptr_t *out_file_context)
{
    (void)flags;
    if (out_file_context == NULL)
        return RDF_EINVAL;
    *out_file_context = (uintptr_t)context == SPECIAL_KMSG
        ? (uintptr_t)rdf_kmsg_end()
        : 0;
    return RDF_OK;
}

static int64_t special_initial_offset(
    void *context,
    uintptr_t file_context,
    uint32_t flags)
{
    (void)file_context;
    (void)flags;
    if ((uintptr_t)context != SPECIAL_KMSG)
        return 0;
    uint64_t offset = rdf_kmsg_start();
    return offset <= INT64_MAX ? (int64_t)offset : RDF_EIO;
}

static int64_t special_read(
    void *context,
    uintptr_t file_context,
    uint64_t offset,
    uint8_t *data,
    size_t len,
    uint32_t flags)
{
    (void)flags;
    switch ((enum special_kind)(uintptr_t)context) {
    case SPECIAL_KMSG: {
        uint64_t snapshot_end = (uint64_t)file_context;
        if (offset >= snapshot_end)
            return 0;
        uint64_t remaining = snapshot_end - offset;
        if ((uint64_t)len > remaining)
            len = (size_t)remaining;
        int64_t result = rdf_kmsg_read(offset, data, len, 1);
        return result == RDF_EAGAIN ? 0 : result;
    }
    case SPECIAL_NULL:
        return 0;
    case SPECIAL_ZERO:
        memset(data, 0, len);
        return (int64_t)len;
    case SPECIAL_RANDOM:
        rdf_random_fill(data, len);
        return (int64_t)len;
    }
    return RDF_EINVAL;
}

static int64_t special_write(
    void *context,
    uintptr_t file_context,
    uint64_t offset,
    const uint8_t *data,
    size_t len,
    uint32_t flags)
{
    (void)file_context;
    (void)offset;
    (void)flags;
    switch ((enum special_kind)(uintptr_t)context) {
    case SPECIAL_KMSG: {
        int32_t status = rdf_kmsg_append(data, len);
        return status == RDF_OK ? (int64_t)len : status;
    }
    case SPECIAL_RANDOM:
        rdf_random_mix(data, len);
        return (int64_t)len;
    case SPECIAL_NULL:
    case SPECIAL_ZERO:
        return (int64_t)len;
    }
    return RDF_EINVAL;
}

static uint64_t special_size(void *context)
{
    return (uintptr_t)context == SPECIAL_KMSG ? rdf_kmsg_end() : 0;
}

static int64_t special_poll(
    void *context,
    uintptr_t file_context,
    uint64_t offset,
    uint16_t events,
    uint32_t flags)
{
    (void)flags;
    uint16_t ready = events & (RDF_POLL_OUT | RDF_POLL_WRNORM);

    if ((uintptr_t)context == SPECIAL_KMSG) {
        uint64_t snapshot_end = (uint64_t)file_context;
        if (offset < rdf_kmsg_start())
            ready |= RDF_POLL_ERR;
        else if (offset < snapshot_end)
            ready |= events & (RDF_POLL_IN | RDF_POLL_RDNORM);
        else
            ready |= RDF_POLL_HUP;
        return ready;
    }
    return events & (RDF_POLL_IN | RDF_POLL_RDNORM |
                     RDF_POLL_OUT | RDF_POLL_WRNORM);
}

static const struct rdf_node_ops kmsg_operations = {
    .size = sizeof(kmsg_operations),
    .context = (void *)(uintptr_t)SPECIAL_KMSG,
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

static const struct rdf_node_ops null_operations = {
    .size = sizeof(null_operations),
    .context = (void *)(uintptr_t)SPECIAL_NULL,
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

static const struct rdf_node_ops zero_operations = {
    .size = sizeof(zero_operations),
    .context = (void *)(uintptr_t)SPECIAL_ZERO,
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

static const struct rdf_node_ops random_operations = {
    .size = sizeof(random_operations),
    .context = (void *)(uintptr_t)SPECIAL_RANDOM,
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
    rdf_devnode_t devfs,
    const char *name,
    uint16_t mode,
    const struct rdf_node_ops *operations)
{
    rdf_devnode_t node = 0;
    return rdf_devfs_create(NULL, devfs, name, RDF_NODE_CHARACTER, mode, operations, &node);
}

static int32_t special_init(struct rdf_module *self)
{
    (void)self;

    rdf_devnode_t devfs = 0;
    int32_t status = rdf_devfs_root(&devfs);
    if (status != RDF_OK)
        return status;

    status = publish_device(devfs, "kmsg", 0600, &kmsg_operations);
    if (status != RDF_OK)
        return status;

    status = publish_device(devfs, "null", 0666, &null_operations);
    if (status != RDF_OK)
        return status;
    status = publish_device(devfs, "zero", 0666, &zero_operations);
    if (status != RDF_OK)
        return status;
    status = publish_device(devfs, "random", 0666, &random_operations);
    if (status != RDF_OK)
        return status;
    status = publish_device(devfs, "urandom", 0666, &random_operations);
    if (status != RDF_OK)
        return status;
    rdf_kmsg_mute();
    console_output_disabled = 1;
    return RDF_OK;
}

static void special_exit(struct rdf_module *self)
{
    (void)self;
    if (console_output_disabled != 0) {
        rdf_kmsg_unmute();
        console_output_disabled = 0;
    }
}

RDF_MODULE("special", "Null, zero, random, and kmsg pseudo-devices", special_init, special_exit);
