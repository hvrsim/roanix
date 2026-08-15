/*
 * ACPI platform enumerator.
 *
 * Validates the root pointer the bootloader handed the kernel, walks the
 * description tables, and turns what it finds into platform devices. Nothing
 * here touches hardware: the I/O APIC driver and the serial driver bind to the
 * devices this module creates, which is what keeps firmware parsing in one
 * place instead of duplicated in every driver that needs an address.
 */
#include <roanix/driver.h>

#define RSDP_V1_SIZE 20u
#define RSDP_V2_MIN_SIZE 36u
#define RSDP_MAX_SIZE 4096u

#define SDT_HEADER_SIZE 36u
#define SDT_MAX_LENGTH (4u * 1024u * 1024u)

#define MADT_FIXED_SIZE 44u
#define MADT_IOAPIC 1u
#define MADT_INTERRUPT_OVERRIDE 2u

#define MAX_IOAPICS 8u
#define MAX_OVERRIDES 16u
#define MAX_LEGACY_UARTS 4u

/* Legacy PC serial ports, which firmware does not always describe. */
static const uint16_t legacy_ports[MAX_LEGACY_UARTS] = {0x3f8, 0x2f8, 0x3e8, 0x2e8};
static const uint32_t legacy_irqs[MAX_LEGACY_UARTS] = {4, 3, 4, 3};

struct interrupt_override {
    uint32_t gsi;
    uint16_t flags;
    uint8_t present;
};

static struct interrupt_override overrides[MAX_OVERRIDES];
static const struct rdf_bus *platform_bus;
static struct rdf_device *devices[MAX_IOAPICS + MAX_LEGACY_UARTS];
static size_t device_count;

static const uint8_t *map_table(uint64_t physical, size_t *out_length)
{
    void *address = NULL;
    if (rdf_mmio_direct(physical, &address) != RDF_OK)
        return NULL;

    const uint8_t *header = (const uint8_t *)address;
    uint32_t length = rdf_le32(header + 4);
    if (length < SDT_HEADER_SIZE || length > SDT_MAX_LENGTH)
        return NULL;
    if (rdf_checksum(header, length) != 0)
        return NULL;
    *out_length = length;
    return header;
}

/* Records an interrupt source override so ISA lines resolve to the right GSI. */
static void record_override(const uint8_t *entry, size_t length)
{
    if (length < 10u)
        return;
    uint8_t source = entry[3];
    if (source >= MAX_OVERRIDES)
        return;
    overrides[source].gsi = rdf_le32(entry + 4);
    overrides[source].flags = rdf_le16(entry + 8);
    overrides[source].present = 1;
}

/* Translates an ISA interrupt to a global system interrupt and its trigger. */
static void resolve_isa(uint32_t isa, uint32_t *out_gsi, uint32_t *out_flags)
{
    uint32_t flags = RDF_IRQ_EDGE | RDF_IRQ_ACTIVE_HIGH;
    uint32_t gsi = isa;

    if (isa < MAX_OVERRIDES && overrides[isa].present != 0) {
        uint16_t polarity = (uint16_t)(overrides[isa].flags & 3u);
        uint16_t trigger = (uint16_t)((overrides[isa].flags >> 2) & 3u);
        gsi = overrides[isa].gsi;
        flags = polarity == 3u ? RDF_IRQ_ACTIVE_LOW : RDF_IRQ_ACTIVE_HIGH;
        flags |= trigger == 3u ? RDF_IRQ_LEVEL : RDF_IRQ_EDGE;
    }

    *out_gsi = gsi;
    *out_flags = flags;
}

static int32_t remember(struct rdf_device *device)
{
    if (device_count >= RDF_COUNT(devices))
        return RDF_ENOSPC;
    devices[device_count++] = device;
    return RDF_OK;
}

static int32_t create_ioapic(uint32_t id, uint64_t address, uint32_t gsi_base)
{
    static const char *const compatible[] = {"intel,ioapic"};
    char name[32];
    struct rdf_device_builder *builder = NULL;
    struct rdf_device *device = NULL;
    int32_t status;

    rdf_snprintf(name, sizeof(name), "ioapic@%x", (uint32_t)address);
    status = rdf_device_new(name, &builder);
    if (status != RDF_OK)
        return status;

    status = rdf_device_set_bus(builder, platform_bus);
    if (status == RDF_OK)
        status = rdf_device_add_strings(builder, RDF_PROP_COMPATIBLE, compatible, 1);
    if (status == RDF_OK)
        status = rdf_device_add_u32(builder, "acpi.id", id);
    if (status == RDF_OK)
        status = rdf_device_add_u32(builder, "gsi-base", gsi_base);
    if (status == RDF_OK)
        status = rdf_device_add_resource(builder, RDF_RES_MEM, 0, address, 0x20, "regs");
    if (status != RDF_OK) {
        rdf_device_discard(builder);
        return status;
    }

    status = rdf_device_add(builder, &device);
    if (status != RDF_OK)
        return status;
    return remember(device);
}

static int32_t create_serial(size_t index)
{
    static const char *const compatible[] = {"pnp,16550a", "ns16550a"};
    char name[32];
    struct rdf_device_builder *builder = NULL;
    struct rdf_device *device = NULL;
    uint32_t gsi = 0;
    uint32_t flags = 0;
    uint32_t cells[2];
    int32_t status;

    resolve_isa(legacy_irqs[index], &gsi, &flags);
    cells[0] = gsi;
    cells[1] = flags;

    rdf_snprintf(name, sizeof(name), "serial@%x", legacy_ports[index]);
    status = rdf_device_new(name, &builder);
    if (status != RDF_OK)
        return status;

    status = rdf_device_set_bus(builder, platform_bus);
    if (status == RDF_OK)
        status = rdf_device_add_strings(builder, RDF_PROP_COMPATIBLE, compatible,
                                        RDF_COUNT(compatible));
    if (status == RDF_OK)
        status = rdf_device_add_u32(builder, "index", (uint32_t)index);
    if (status == RDF_OK)
        status = rdf_device_add_u32(builder, "clock-frequency", 1843200u);
    if (status == RDF_OK)
        status = rdf_device_add_resource(builder, RDF_RES_IO, 0, legacy_ports[index], 8, "regs");
    if (status == RDF_OK)
        status = rdf_device_add_irq(builder, NULL, cells, RDF_COUNT(cells));
    if (status != RDF_OK) {
        rdf_device_discard(builder);
        return status;
    }

    status = rdf_device_add(builder, &device);
    if (status != RDF_OK)
        return status;
    return remember(device);
}

static int32_t parse_madt(const uint8_t *madt, size_t length)
{
    size_t offset = MADT_FIXED_SIZE;
    size_t created = 0;

    /* Overrides must be known before any device that references an ISA line. */
    while (offset + 2u <= length) {
        size_t entry_length = madt[offset + 1];
        if (entry_length < 2u || offset + entry_length > length)
            break;
        if (madt[offset] == MADT_INTERRUPT_OVERRIDE)
            record_override(madt + offset, entry_length);
        offset += entry_length;
    }

    offset = MADT_FIXED_SIZE;
    while (offset + 2u <= length) {
        size_t entry_length = madt[offset + 1];
        if (entry_length < 2u || offset + entry_length > length)
            break;
        const uint8_t *entry = madt + offset;
        if (entry[0] == MADT_IOAPIC && entry_length >= 12u && created < MAX_IOAPICS) {
            uint64_t address = rdf_le32(entry + 4);
            uint32_t gsi_base = rdf_le32(entry + 8);
            if (address != 0) {
                int32_t status = create_ioapic(entry[2], address, gsi_base);
                if (status != RDF_OK)
                    return status;
                ++created;
            }
        }
        offset += entry_length;
    }

    if (created == 0)
        RDF_WARN("no I/O APIC described by the MADT");
    return RDF_OK;
}

/* Walks the root table, handling both the 32-bit RSDT and the 64-bit XSDT. */
static int32_t walk_tables(uint64_t root, int wide)
{
    size_t length = 0;
    const uint8_t *table = map_table(root, &length);
    if (table == NULL)
        return RDF_EINVAL;

    size_t stride = wide ? 8u : 4u;
    size_t count = (length - SDT_HEADER_SIZE) / stride;
    for (size_t index = 0; index < count; ++index) {
        const uint8_t *slot = table + SDT_HEADER_SIZE + index * stride;
        uint64_t address = wide ? rdf_le64(slot) : rdf_le32(slot);
        if (address == 0)
            continue;

        size_t entry_length = 0;
        const uint8_t *entry = map_table(address, &entry_length);
        if (entry == NULL)
            continue;
        if (memcmp(entry, "APIC", 4) == 0 && entry_length >= MADT_FIXED_SIZE) {
            int32_t status = parse_madt(entry, entry_length);
            if (status != RDF_OK)
                return status;
        }
    }
    return RDF_OK;
}

static int32_t acpi_init(struct rdf_module *self)
{
    static uint8_t rsdp[RSDP_MAX_SIZE];
    size_t length = 0;
    int32_t status;

    (void)self;

    status = rdf_firmware_acpi(rsdp, sizeof(rsdp), &length);
    if (status == RDF_ENOENT) {
        RDF_DEBUG("no ACPI root pointer on this platform");
        return RDF_OK;
    }
    if (status != RDF_OK)
        return status;

    if (length < RSDP_V1_SIZE || memcmp(rsdp, "RSD PTR ", 8) != 0 ||
        rdf_checksum(rsdp, RSDP_V1_SIZE) != 0) {
        RDF_ERROR("the ACPI root pointer failed validation");
        return RDF_EINVAL;
    }

    status = rdf_bus_find(RDF_BUS_PLATFORM, &platform_bus);
    if (status != RDF_OK)
        return status;

    uint64_t root;
    int wide = 0;
    if (rsdp[15] >= 2 && length >= RSDP_V2_MIN_SIZE) {
        uint32_t extended_length = rdf_le32(rsdp + 20);
        if (extended_length < RSDP_V2_MIN_SIZE || extended_length > length ||
            rdf_checksum(rsdp, extended_length) != 0) {
            RDF_ERROR("the extended ACPI root pointer failed validation");
            return RDF_EINVAL;
        }
        root = rdf_le64(rsdp + 24);
        wide = 1;
    } else {
        root = rdf_le32(rsdp + 16);
    }
    if (root == 0)
        return RDF_EINVAL;

    status = walk_tables(root, wide);
    if (status != RDF_OK)
        return status;

    for (size_t index = 0; index < MAX_LEGACY_UARTS; ++index) {
        status = create_serial(index);
        if (status != RDF_OK)
            return status;
    }

    RDF_INFO("enumerated %zu platform devices", device_count);
    return RDF_OK;
}

static void acpi_exit(struct rdf_module *self)
{
    (void)self;
    while (device_count > 0)
        rdf_device_remove(devices[--device_count]);
}

RDF_MODULE("acpi", "ACPI platform enumerator", acpi_init, acpi_exit)
