/*
 * Device-tree platform enumerator.
 *
 * Walks the flattened device tree the bootloader supplied and turns every
 * enabled node that has a register window into a platform device carrying its
 * `compatible` list, address ranges, and interrupt specifiers.
 *
 * Doing this once, here, is the point: drivers state which hardware they
 * support with a match table and never parse firmware themselves.
 */
#include <roanix/driver.h>

#define FDT_MAGIC UINT32_C(0xd00dfeed)
#define FDT_HEADER_SIZE 40u

#define FDT_BEGIN_NODE UINT32_C(1)
#define FDT_END_NODE UINT32_C(2)
#define FDT_PROP UINT32_C(3)
#define FDT_NOP UINT32_C(4)
#define FDT_END UINT32_C(9)

#define MAX_DEPTH 32u
#define MAX_DEVICES 64u
#define MAX_STRINGS 16u
#define MAX_CELLS 32u
#define MAX_BLOB (2u * 1024u * 1024u)

struct level {
    uint32_t address_cells;
    uint32_t size_cells;
};

struct node {
    const char *name;
    const uint8_t *reg;
    size_t reg_length;
    const uint8_t *compatible;
    size_t compatible_length;
    const uint8_t *interrupts;
    size_t interrupts_length;
    uint32_t reg_shift;
    uint32_t reg_width;
    uint32_t clock;
    uint32_t interrupt_cells;
    uint8_t enabled;
    uint8_t has_reg;
};

static uint8_t blob[MAX_BLOB];
static size_t blob_length;
static const struct rdf_bus *platform_bus;
static struct rdf_device *devices[MAX_DEVICES];
static size_t device_count;

static size_t align4(size_t value)
{
    return (value + 3u) & ~(size_t)3u;
}

static const char *string_at(size_t strings_offset, size_t strings_size, uint32_t offset)
{
    if (offset >= strings_size)
        return NULL;
    const char *base = (const char *)(blob + strings_offset);
    for (size_t index = offset; index < strings_size; ++index) {
        if (base[index] == '\0')
            return base + offset;
    }
    return NULL;
}

static int property_is(const char *name, const char *expected)
{
    return name != NULL && strcmp(name, expected) == 0;
}

static uint32_t cell_or(const uint8_t *value, size_t length, uint32_t fallback)
{
    return length >= 4u ? rdf_be32(value) : fallback;
}

static int status_enabled(const uint8_t *value, size_t length)
{
    size_t text = 0;
    while (text < length && value[text] != 0)
        ++text;
    return rdf_str_equals(value, text, "ok") || rdf_str_equals(value, text, "okay");
}

/* Reads one address or size from a `reg` entry with the parent's cell counts. */
static int read_cells(const uint8_t *bytes, size_t length, uint32_t cells, uint64_t *value)
{
    if (cells == 0 || cells > 2 || length < (size_t)cells * 4u)
        return 0;
    uint64_t result = 0;
    for (uint32_t index = 0; index < cells; ++index)
        result = (result << 32) | rdf_be32(bytes + index * 4u);
    *value = result;
    return 1;
}

/* Splits a NUL-separated string list into individual pointers. */
static size_t split_strings(const uint8_t *bytes, size_t length, const char **out, size_t limit)
{
    size_t count = 0;
    size_t offset = 0;
    while (offset < length && count < limit) {
        size_t end = offset;
        while (end < length && bytes[end] != 0)
            ++end;
        if (end == offset)
            break;
        out[count++] = (const char *)(bytes + offset);
        offset = end + 1u;
    }
    return count;
}

static int32_t publish(const struct node *node, const struct level *parent)
{
    const char *compatible[MAX_STRINGS];
    uint32_t cells[MAX_CELLS];
    uint64_t regions[MAX_CELLS];
    struct rdf_device_builder *builder = NULL;
    struct rdf_device *device = NULL;
    int32_t status;

    if (node->enabled == 0 || node->has_reg == 0 || node->compatible == NULL)
        return RDF_OK;
    if (device_count >= MAX_DEVICES)
        return RDF_ENOSPC;

    size_t compatible_count =
        split_strings(node->compatible, node->compatible_length, compatible, MAX_STRINGS);
    if (compatible_count == 0)
        return RDF_OK;

    uint64_t address = 0;
    uint64_t size = 0;
    size_t address_bytes = (size_t)parent->address_cells * 4u;
    if (!read_cells(node->reg, node->reg_length, parent->address_cells, &address) ||
        !read_cells(node->reg + address_bytes, node->reg_length - address_bytes,
                    parent->size_cells, &size) ||
        size == 0)
        return RDF_OK;

    status = rdf_device_new(node->name, &builder);
    if (status != RDF_OK)
        return status;

    status = rdf_device_set_bus(builder, platform_bus);
    if (status == RDF_OK)
        status = rdf_device_add_strings(builder, RDF_PROP_COMPATIBLE, compatible,
                                        compatible_count);
    if (status == RDF_OK)
        status = rdf_device_add_resource(builder, RDF_RES_MEM, 0, address, size, "regs");
    if (status == RDF_OK && node->reg_shift != 0)
        status = rdf_device_add_u32(builder, "reg-shift", node->reg_shift);
    if (status == RDF_OK && node->reg_width != 0)
        status = rdf_device_add_u32(builder, "reg-io-width", node->reg_width);
    if (status == RDF_OK && node->clock != 0)
        status = rdf_device_add_u32(builder, "clock-frequency", node->clock);

    if (status == RDF_OK && node->interrupts != NULL && node->interrupts_length >= 4u) {
        uint32_t stride = node->interrupt_cells != 0 ? node->interrupt_cells : 1u;
        size_t available = node->interrupts_length / 4u;
        size_t count = available < MAX_CELLS ? available : MAX_CELLS;
        for (size_t index = 0; index < count; ++index)
            cells[index] = rdf_be32(node->interrupts + index * 4u);
        for (size_t index = 0; index + stride <= count; index += stride) {
            status = rdf_device_add_irq(builder, NULL, cells + index, stride);
            if (status != RDF_OK)
                break;
        }
        if (status == RDF_OK) {
            for (size_t index = 0; index < count; ++index)
                regions[index] = cells[index];
            status = rdf_device_add_u32_list(builder, "interrupts", regions, count);
        }
    }

    if (status != RDF_OK) {
        rdf_device_discard(builder);
        return status;
    }

    status = rdf_device_add(builder, &device);
    if (status != RDF_OK)
        return status;
    devices[device_count++] = device;
    return RDF_OK;
}

static int32_t walk(size_t structure_offset, size_t structure_size, size_t strings_offset,
                    size_t strings_size)
{
    struct level levels[MAX_DEPTH];
    struct node current;
    size_t offset = structure_offset;
    size_t end = structure_offset + structure_size;
    size_t depth = 0;

    levels[0].address_cells = 2;
    levels[0].size_cells = 1;
    memset(&current, 0, sizeof(current));

    while (offset + 4u <= end) {
        uint32_t token = rdf_be32(blob + offset);
        offset += 4u;

        if (token == FDT_END)
            break;
        if (token == FDT_NOP)
            continue;

        if (token == FDT_BEGIN_NODE) {
            size_t name_start = offset;
            while (offset < end && blob[offset] != 0)
                ++offset;
            if (offset >= end)
                return RDF_EINVAL;
            offset = align4(offset + 1u);

            if (depth + 1u >= MAX_DEPTH)
                return RDF_EINVAL;
            ++depth;
            levels[depth].address_cells = levels[depth - 1].address_cells;
            levels[depth].size_cells = levels[depth - 1].size_cells;

            memset(&current, 0, sizeof(current));
            current.name = (const char *)(blob + name_start);
            current.enabled = 1;
            if (current.name[0] == '\0')
                current.name = "root";
            continue;
        }

        if (token == FDT_END_NODE) {
            if (depth == 0)
                return RDF_EINVAL;
            int32_t status = publish(&current, &levels[depth - 1]);
            if (status != RDF_OK)
                return status;
            memset(&current, 0, sizeof(current));
            --depth;
            continue;
        }

        if (token != FDT_PROP)
            return RDF_EINVAL;
        if (offset + 8u > end)
            return RDF_EINVAL;

        size_t length = rdf_be32(blob + offset);
        uint32_t name_offset = rdf_be32(blob + offset + 4u);
        offset += 8u;
        if (offset + length > end)
            return RDF_EINVAL;

        const uint8_t *value = blob + offset;
        const char *name = string_at(strings_offset, strings_size, name_offset);
        offset = align4(offset + length);
        if (depth == 0)
            continue;

        if (property_is(name, "#address-cells"))
            levels[depth].address_cells = cell_or(value, length, 2);
        else if (property_is(name, "#size-cells"))
            levels[depth].size_cells = cell_or(value, length, 1);
        else if (property_is(name, "#interrupt-cells"))
            current.interrupt_cells = cell_or(value, length, 1);
        else if (property_is(name, "reg")) {
            current.reg = value;
            current.reg_length = length;
            current.has_reg = 1;
        } else if (property_is(name, "compatible")) {
            current.compatible = value;
            current.compatible_length = length;
        } else if (property_is(name, "interrupts")) {
            current.interrupts = value;
            current.interrupts_length = length;
        } else if (property_is(name, "reg-shift"))
            current.reg_shift = cell_or(value, length, 0);
        else if (property_is(name, "reg-io-width"))
            current.reg_width = cell_or(value, length, 1);
        else if (property_is(name, "clock-frequency"))
            current.clock = cell_or(value, length, 0);
        else if (property_is(name, "status"))
            current.enabled = (uint8_t)status_enabled(value, length);
    }
    return RDF_OK;
}

static int32_t fdt_init(struct rdf_module *self)
{
    int32_t status;
    (void)self;

    status = rdf_firmware_devicetree(blob, sizeof(blob), &blob_length);
    if (status == RDF_ENOENT) {
        RDF_DEBUG("no device tree on this platform");
        return RDF_OK;
    }
    if (status != RDF_OK)
        return status;

    if (blob_length < FDT_HEADER_SIZE || rdf_be32(blob) != FDT_MAGIC) {
        RDF_ERROR("the device tree failed validation");
        return RDF_EINVAL;
    }

    size_t total = rdf_be32(blob + 4);
    size_t structure_offset = rdf_be32(blob + 8);
    size_t strings_offset = rdf_be32(blob + 12);
    size_t strings_size = rdf_be32(blob + 32);
    size_t structure_size = rdf_be32(blob + 36);
    if (total < FDT_HEADER_SIZE || total > blob_length ||
        structure_offset + structure_size > total || strings_offset + strings_size > total)
        return RDF_EINVAL;

    status = rdf_bus_find(RDF_BUS_PLATFORM, &platform_bus);
    if (status != RDF_OK)
        return status;

    status = walk(structure_offset, structure_size, strings_offset, strings_size);
    if (status != RDF_OK)
        return status;

    RDF_INFO("enumerated %zu platform devices", device_count);
    return RDF_OK;
}

static void fdt_exit(struct rdf_module *self)
{
    (void)self;
    while (device_count > 0)
        rdf_device_remove(devices[--device_count]);
}

RDF_MODULE("fdt", "Flattened device-tree platform enumerator", fdt_init, fdt_exit)
