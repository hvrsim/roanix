#include <devkit/devkit.h>

#define FDT_MAGIC UINT32_C(0xd00dfeed)
#define FDT_HEADER_SIZE 40u

static int32_t dtb_start(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;

    dk_bus_t root = 0;
    int32_t status = dk_root_bus(&root);
    if (status != DK_OK)
        return status;

    struct dk_data_resource blob = {0};
    status = dk_resource_acquire_data(
        root,
        DK_RESOURCE_FIRMWARE_DTB,
        &blob);
    if (status != DK_OK)
        return status;

    size_t length = 0;
    if (blob.data.len < FDT_HEADER_SIZE ||
        dk_read_be32(blob.data.data) != FDT_MAGIC) {
        status = DK_EINVAL;
        goto out;
    }
    length = dk_read_be32(blob.data.data + 4);
    if (length < FDT_HEADER_SIZE || length > blob.data.len) {
        status = DK_EINVAL;
        goto out;
    }

    dk_bus_t bus = 0;
    status = dk_bus_create(root, DK_SLICE_LITERAL("dtb"), &bus);
    if (status != DK_OK)
        goto out;
    status = dk_node_set_property(
        bus,
        DK_PROPERTY_SUBSYSTEM,
        DK_SLICE_LITERAL("platform"));
    if (status != DK_OK)
        goto out;

    dk_resource_t resource = 0;
    status = dk_resource_publish_data(
        bus,
        DK_RESOURCE_FIRMWARE_DTB,
        DK_RESOURCE_CACHEABLE,
        (struct dk_slice){
            .data = blob.data.data,
            .len = length,
        },
        &resource);

out:
    {
        int32_t released = dk_resource_release(blob.lease);
        if (status == DK_OK)
            status = released;
    }
    if (status != DK_OK)
        return status;
    return DK_LOG_LITERAL(
        DK_LOG_INFO,
        "dtb: published boot device tree");
}

static const struct dk_driver_definition driver = {
    .name = DK_SLICE_LITERAL("dtb"),
    .context = 0,
    .start = dtb_start,
    .stop = NULL,
};

DK_DRIVER_EXPORT(driver)
