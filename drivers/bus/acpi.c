#include <devkit/devkit.h>

#define RSDP_V1_SIZE 20u
#define RSDP_V2_MIN_SIZE 36u
#define RSDP_MAX_SIZE 4096u

static int32_t acpi_start(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;

    dk_bus_t root = 0;
    int32_t status = dk_root_bus(&root);
    if (status != DK_OK)
        return status;

    struct dk_data_resource rsdp = {0};
    status = dk_resource_acquire_data(
        root,
        DK_RESOURCE_FIRMWARE_ACPI_RSDP,
        &rsdp);
    if (status == DK_ENOENT)
        return DK_OK;
    if (status != DK_OK)
        return status;

    size_t length = RSDP_V1_SIZE;
    if (rsdp.data.len < RSDP_V1_SIZE ||
        memcmp(rsdp.data.data, "RSD PTR ", 8) != 0 ||
        dk_checksum(rsdp.data.data, RSDP_V1_SIZE) != 0) {
        status = DK_EINVAL;
        goto out;
    }
    if (rsdp.data.data[15] >= 2) {
        if (rsdp.data.len < RSDP_V2_MIN_SIZE) {
            status = DK_EINVAL;
            goto out;
        }
        length = dk_read_le32(rsdp.data.data + 20);
        if (length < RSDP_V2_MIN_SIZE ||
            length > RSDP_MAX_SIZE ||
            length > rsdp.data.len ||
            dk_checksum(rsdp.data.data, length) != 0) {
            status = DK_EINVAL;
            goto out;
        }
    }

    dk_bus_t bus = 0;
    status = dk_bus_create(root, DK_SLICE_LITERAL("acpi"), &bus);
    if (status != DK_OK)
        goto out;
    status = dk_node_set_property(
        bus,
        DK_PROPERTY_SUBSYSTEM,
        DK_SLICE_LITERAL("acpi"));
    if (status != DK_OK)
        goto out;

    dk_resource_t resource = 0;
    status = dk_resource_publish_data(
        bus,
        DK_RESOURCE_FIRMWARE_ACPI_RSDP,
        DK_RESOURCE_CACHEABLE,
        (struct dk_slice){
            .data = rsdp.data.data,
            .len = length,
        },
        &resource);

out:
    {
        int32_t released = dk_resource_release(rsdp.lease);
        if (status == DK_OK)
            status = released;
    }
    if (status != DK_OK)
        return status;
    return DK_LOG_LITERAL(DK_LOG_INFO, "acpi: published boot RSDP");
}

static const struct dk_driver_definition driver = {
    .name = DK_SLICE_LITERAL("acpi"),
    .context = 0,
    .start = acpi_start,
    .stop = NULL,
};

DK_DRIVER_EXPORT(driver)
