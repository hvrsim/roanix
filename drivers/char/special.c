#include <roanix/driver.h>

/*
 * Null, zero, and random pseudo-devices.
 *
 * The kernel log devices used to live here too. They now belong to the kernel
 * itself: the log has to be readable even when loading driver modules is the
 * thing that failed.
 */

enum special_kind {
    SPECIAL_NULL,
    SPECIAL_ZERO,
    SPECIAL_RANDOM,
};

static int64_t special_read(
    void *context,
    uintptr_t file_context,
    uint64_t offset,
    uint8_t *data,
    size_t len,
    uint32_t flags)
{
    (void)file_context;
    (void)offset;
    (void)flags;
    switch ((enum special_kind)(uintptr_t)context) {
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
    case SPECIAL_RANDOM:
        rdf_random_mix(data, len);
        return (int64_t)len;
    case SPECIAL_NULL:
    case SPECIAL_ZERO:
        return (int64_t)len;
    }
    return RDF_EINVAL;
}

static int64_t special_poll(
    void *context,
    uintptr_t file_context,
    uint64_t offset,
    uint16_t events,
    uint32_t flags)
{
    (void)context;
    (void)file_context;
    (void)offset;
    (void)flags;
    return events & (RDF_POLL_IN | RDF_POLL_RDNORM |
                     RDF_POLL_OUT | RDF_POLL_WRNORM);
}

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

    status = publish_device(devfs, "null", 0666, &null_operations);
    if (status != RDF_OK)
        return status;
    status = publish_device(devfs, "zero", 0666, &zero_operations);
    if (status != RDF_OK)
        return status;
    status = publish_device(devfs, "random", 0666, &random_operations);
    if (status != RDF_OK)
        return status;
    return publish_device(devfs, "urandom", 0666, &random_operations);
}

static void special_exit(struct rdf_module *self)
{
    (void)self;
}

RDF_MODULE("special", "Null, zero, and random pseudo-devices", special_init, special_exit);
