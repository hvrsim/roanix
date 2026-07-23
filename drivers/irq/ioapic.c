#include <devkit/devkit.h>

#define SDT_HEADER_SIZE 36u
#define MADT_FIXED_SIZE 44u
#define MAX_ACPI_TABLE_SIZE (1024u * 1024u)
#define MAX_IOAPICS 16u
#define MAX_REDIRECTIONS 120u

#define MADT_IOAPIC 1u
#define MADT_INTERRUPT_OVERRIDE 2u

#define IOAPIC_REGISTER_SELECT 0x00u
#define IOAPIC_REGISTER_WINDOW 0x10u
#define IOAPIC_VERSION 0x01u
#define IOAPIC_REDIRECTION_BASE 0x10u

#define REDIRECTION_POLARITY_LOW (UINT32_C(1) << 13)
#define REDIRECTION_TRIGGER_LEVEL (UINT32_C(1) << 15)
#define REDIRECTION_MASKED (UINT32_C(1) << 16)

#define SPECIFIER_ISA_IRQ 1u
#define SPECIFIER_GSI 2u

struct interrupt_override {
    uint32_t gsi;
    uint16_t flags;
    uint8_t present;
};

struct ioapic {
    struct dk_spinlock lock;
    uintptr_t base;
    uint32_t gsi_base;
    uint32_t redirection_count;
    uint64_t active[2];
};

struct ioapic_controller {
    struct ioapic ioapics[MAX_IOAPICS];
    size_t ioapic_count;
    struct interrupt_override overrides[16];
};

static struct ioapic_controller controller;

static uint32_t ioapic_read_locked(const struct ioapic *ioapic, uint8_t reg)
{
    dk_mmio_write32(ioapic->base + IOAPIC_REGISTER_SELECT, reg);
    return dk_mmio_read32(ioapic->base + IOAPIC_REGISTER_WINDOW);
}

static void ioapic_write_locked(
    const struct ioapic *ioapic,
    uint8_t reg,
    uint32_t value)
{
    dk_mmio_write32(ioapic->base + IOAPIC_REGISTER_SELECT, reg);
    dk_mmio_write32(ioapic->base + IOAPIC_REGISTER_WINDOW, value);
}

static void write_redirection_locked(
    const struct ioapic *ioapic,
    uint32_t index,
    uint32_t low,
    uint32_t high)
{
    uint8_t reg = (uint8_t)(IOAPIC_REDIRECTION_BASE + index * 2u);
    ioapic_write_locked(ioapic, (uint8_t)(reg + 1u), high);
    ioapic_write_locked(ioapic, reg, low);
}

static struct ioapic *find_ioapic(uint32_t gsi)
{
    for (size_t index = 0; index < controller.ioapic_count; ++index) {
        struct ioapic *ioapic = &controller.ioapics[index];
        if (gsi >= ioapic->gsi_base &&
            gsi - ioapic->gsi_base < ioapic->redirection_count)
            return ioapic;
    }
    return NULL;
}

static int route_active(const struct ioapic *ioapic, uint32_t index)
{
    return (ioapic->active[index / 64u] &
            (UINT64_C(1) << (index % 64u))) != 0;
}

static void set_route_active(
    struct ioapic *ioapic,
    uint32_t index,
    int active)
{
    uint64_t mask = UINT64_C(1) << (index % 64u);
    if (active)
        ioapic->active[index / 64u] |= mask;
    else
        ioapic->active[index / 64u] &= ~mask;
}

static int route_modes(
    uint64_t flags,
    int default_low,
    int default_level,
    int *polarity_low,
    int *level)
{
    if ((flags & DK_INTERRUPT_ACTIVE_LOW) != 0)
        *polarity_low = 1;
    else if ((flags & DK_INTERRUPT_ACTIVE_HIGH) != 0)
        *polarity_low = 0;
    else
        *polarity_low = default_low;

    if ((flags & DK_INTERRUPT_LEVEL) != 0)
        *level = 1;
    else if ((flags & DK_INTERRUPT_EDGE) != 0)
        *level = 0;
    else
        *level = default_level;
    return 1;
}

static int resolve_source(
    uint32_t kind,
    uint32_t source,
    uint64_t flags,
    uint32_t *gsi,
    int *polarity_low,
    int *level)
{
    if (kind == SPECIFIER_ISA_IRQ) {
        if (source >= 16)
            return 0;
        const struct interrupt_override *override = &controller.overrides[source];
        if (override->present != 0) {
            uint16_t polarity = override->flags & 3u;
            uint16_t trigger = (override->flags >> 2) & 3u;
            if (polarity != 0 && polarity != 1 && polarity != 3)
                return 0;
            if (trigger != 0 && trigger != 1 && trigger != 3)
                return 0;
            *gsi = override->gsi;
            *polarity_low = polarity == 3;
            *level = trigger == 3;
            return 1;
        }
        *gsi = source;
        return route_modes(flags, 0, 0, polarity_low, level);
    }
    if (kind == SPECIFIER_GSI) {
        *gsi = source;
        return route_modes(flags, 0, 0, polarity_low, level);
    }
    return 0;
}

static int32_t connect_interrupt(
    uintptr_t context,
    const struct dk_interrupt_route *route,
    uint64_t *out_cookie)
{
    (void)context;
    if (route == NULL ||
        out_cookie == NULL ||
        route->size < sizeof(*route) ||
        route->vector > UINT8_MAX ||
        route->specifier.data == NULL ||
        route->specifier.len != 8)
        return DK_EINVAL;

    uint32_t kind = dk_read_le32(route->specifier.data);
    uint32_t source = dk_read_le32(route->specifier.data + 4);
    uint32_t gsi = 0;
    int polarity_low = 0;
    int level = 0;
    if (!resolve_source(
            kind,
            source,
            route->flags,
            &gsi,
            &polarity_low,
            &level))
        return DK_EINVAL;
    if (route->target_platform_id > UINT8_MAX)
        return DK_ENOTSUP;

    struct ioapic *ioapic = find_ioapic(gsi);
    if (ioapic == NULL)
        return DK_ENOENT;
    uint32_t index = gsi - ioapic->gsi_base;
    uintptr_t irq_state = dk_spin_lock_irqsave(&ioapic->lock);
    if (route_active(ioapic, index)) {
        dk_spin_unlock_irqrestore(&ioapic->lock, irq_state);
        return DK_EEXIST;
    }

    uint32_t low = route->vector | REDIRECTION_MASKED;
    if (polarity_low)
        low |= REDIRECTION_POLARITY_LOW;
    if (level)
        low |= REDIRECTION_TRIGGER_LEVEL;
    uint32_t high = (uint32_t)route->target_platform_id << 24;
    write_redirection_locked(ioapic, index, low, high);
    set_route_active(ioapic, index, 1);
    dk_spin_unlock_irqrestore(&ioapic->lock, irq_state);
    *out_cookie = gsi;
    return DK_OK;
}

static int32_t update_route(uint64_t cookie, int operation, uint8_t destination)
{
    if (cookie > UINT32_MAX)
        return DK_EINVAL;
    uint32_t gsi = (uint32_t)cookie;
    struct ioapic *ioapic = find_ioapic(gsi);
    if (ioapic == NULL)
        return DK_ENOENT;
    uint32_t index = gsi - ioapic->gsi_base;
    uintptr_t irq_state = dk_spin_lock_irqsave(&ioapic->lock);
    if (!route_active(ioapic, index)) {
        dk_spin_unlock_irqrestore(&ioapic->lock, irq_state);
        return DK_ENOENT;
    }
    uint8_t reg = (uint8_t)(IOAPIC_REDIRECTION_BASE + index * 2u);
    if (operation == 0) {
        uint32_t low = ioapic_read_locked(ioapic, reg);
        ioapic_write_locked(ioapic, reg, low | REDIRECTION_MASKED);
    } else if (operation == 1) {
        uint32_t low = ioapic_read_locked(ioapic, reg);
        ioapic_write_locked(ioapic, reg, low & ~REDIRECTION_MASKED);
    } else {
        ioapic_write_locked(
            ioapic,
            (uint8_t)(reg + 1u),
            (uint32_t)destination << 24);
    }
    dk_spin_unlock_irqrestore(&ioapic->lock, irq_state);
    return DK_OK;
}

static int32_t disconnect_interrupt(uintptr_t context, uint64_t cookie)
{
    (void)context;
    int32_t status = update_route(cookie, 0, 0);
    if (status != DK_OK)
        return status;
    uint32_t gsi = (uint32_t)cookie;
    struct ioapic *ioapic = find_ioapic(gsi);
    uint32_t index = gsi - ioapic->gsi_base;
    uintptr_t irq_state = dk_spin_lock_irqsave(&ioapic->lock);
    set_route_active(ioapic, index, 0);
    dk_spin_unlock_irqrestore(&ioapic->lock, irq_state);
    return DK_OK;
}

static int32_t mask_interrupt(uintptr_t context, uint64_t cookie)
{
    (void)context;
    return update_route(cookie, 0, 0);
}

static int32_t unmask_interrupt(uintptr_t context, uint64_t cookie)
{
    (void)context;
    return update_route(cookie, 1, 0);
}

static int32_t set_affinity(
    uintptr_t context,
    uint64_t cookie,
    const struct dk_interrupt_route *route)
{
    (void)context;
    if (route == NULL ||
        route->size < sizeof(*route) ||
        route->target_platform_id > UINT8_MAX)
        return DK_EINVAL;
    return update_route(cookie, 2, (uint8_t)route->target_platform_id);
}

static const uint8_t *table_at(uint64_t physical, size_t *out_length)
{
    uintptr_t address = 0;
    if (physical == 0 ||
        dk_firmware_physical_to_virtual(physical, &address) != DK_OK)
        return NULL;
    const uint8_t *header = (const uint8_t *)address;
    size_t length = dk_read_le32(header + 4);
    if (length < SDT_HEADER_SIZE || length > MAX_ACPI_TABLE_SIZE)
        return NULL;
    if (dk_checksum(header, length) != 0)
        return NULL;
    *out_length = length;
    return header;
}

static const uint8_t *find_table(
    uint64_t root_physical,
    size_t entry_size,
    const char root_signature[4],
    const char wanted_signature[4],
    size_t *out_length)
{
    size_t root_length = 0;
    const uint8_t *root = table_at(root_physical, &root_length);
    if (root == NULL ||
        memcmp(root, root_signature, 4) != 0 ||
        (root_length - SDT_HEADER_SIZE) % entry_size != 0)
        return NULL;

    for (size_t offset = SDT_HEADER_SIZE;
         offset < root_length;
         offset += entry_size) {
        uint64_t physical = entry_size == 8
            ? dk_read_le64(root + offset)
            : dk_read_le32(root + offset);
        size_t table_length = 0;
        const uint8_t *table = table_at(physical, &table_length);
        if (table != NULL && memcmp(table, wanted_signature, 4) == 0) {
            *out_length = table_length;
            return table;
        }
    }
    return NULL;
}

static int parse_madt(const uint8_t *madt, size_t length)
{
    if (length < MADT_FIXED_SIZE || memcmp(madt, "APIC", 4) != 0)
        return 0;

    size_t offset = MADT_FIXED_SIZE;
    while (offset < length) {
        if (length - offset < 2)
            return 0;
        uint8_t type = madt[offset];
        size_t entry_length = madt[offset + 1];
        if (entry_length < 2 || entry_length > length - offset)
            return 0;
        const uint8_t *entry = madt + offset;
        if (type == MADT_IOAPIC && entry_length >= 12) {
            if (controller.ioapic_count == MAX_IOAPICS)
                return 0;
            uint64_t physical = dk_read_le32(entry + 4);
            if (physical == 0 || (physical & UINT64_C(0xfff)) != 0)
                return 0;
            struct ioapic *ioapic =
                &controller.ioapics[controller.ioapic_count];
            uintptr_t mapped = 0;
            if (dk_mmio_map(physical, 4096, &mapped) != DK_OK)
                return 0;
            atomic_flag_clear_explicit(
                &ioapic->lock.held,
                memory_order_relaxed);
            ioapic->base = mapped;
            ioapic->gsi_base = dk_read_le32(entry + 8);
            uintptr_t irq_state = dk_spin_lock_irqsave(&ioapic->lock);
            uint32_t version = ioapic_read_locked(ioapic, IOAPIC_VERSION);
            dk_spin_unlock_irqrestore(&ioapic->lock, irq_state);
            ioapic->redirection_count = ((version >> 16) & UINT32_C(0xff)) + 1u;
            if (ioapic->redirection_count > MAX_REDIRECTIONS)
                return 0;
            ++controller.ioapic_count;
        } else if (type == MADT_INTERRUPT_OVERRIDE &&
                   entry_length >= 10 &&
                   entry[2] == 0) {
            size_t source = entry[3];
            if (source < 16) {
                if (controller.overrides[source].present != 0)
                    return 0;
                controller.overrides[source] = (struct interrupt_override){
                    .gsi = dk_read_le32(entry + 4),
                    .flags = dk_read_le16(entry + 8),
                    .present = 1,
                };
            }
        }
        offset += entry_length;
    }
    if (controller.ioapic_count == 0)
        return 0;

    for (size_t left = 0; left < controller.ioapic_count; ++left) {
        uint64_t left_start = controller.ioapics[left].gsi_base;
        uint64_t left_end =
            left_start + controller.ioapics[left].redirection_count;
        for (size_t right = left + 1u;
             right < controller.ioapic_count;
             ++right) {
            uint64_t right_start = controller.ioapics[right].gsi_base;
            uint64_t right_end =
                right_start + controller.ioapics[right].redirection_count;
            if (left_start < right_end && right_start < left_end)
                return 0;
        }
    }
    return 1;
}

static int32_t ioapic_start(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;
    controller.ioapic_count = 0;
    memset(controller.overrides, 0, sizeof(controller.overrides));

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
    if (rsdp.data.len < 20 ||
        memcmp(rsdp.data.data, "RSD PTR ", 8) != 0 ||
        dk_checksum(rsdp.data.data, 20) != 0) {
        status = DK_EINVAL;
        goto out;
    }

    uint64_t rsdt = dk_read_le32(rsdp.data.data + 16);
    uint64_t xsdt = rsdp.data.data[15] >= 2 && rsdp.data.len >= 36
        ? dk_read_le64(rsdp.data.data + 24)
        : 0;
    size_t madt_length = 0;
    const uint8_t *madt = NULL;
    if (xsdt != 0)
        madt = find_table(xsdt, 8, "XSDT", "APIC", &madt_length);
    if (madt == NULL)
        madt = find_table(rsdt, 4, "RSDT", "APIC", &madt_length);
    if (madt == NULL || !parse_madt(madt, madt_length)) {
        (void)DK_LOG_LITERAL(
            DK_LOG_WARN,
            "ioapic: no usable controller discovered");
        status = DK_OK;
        goto out;
    }

    dk_irq_controller_t controller_id = 0;
    struct dk_interrupt_controller operations = {
        .size = sizeof(operations),
        .flags = 0,
        .context = (uintptr_t)&controller,
        .connect = connect_interrupt,
        .disconnect = disconnect_interrupt,
        .mask = mask_interrupt,
        .unmask = unmask_interrupt,
        .set_affinity = set_affinity,
        .claim = NULL,
        .complete = NULL,
    };
    status = dk_irq_controller_register(
        root,
        &operations,
        &controller_id);
    if (status == DK_OK)
        status = DK_LOG_LITERAL(
            DK_LOG_INFO,
            "ioapic: registered interrupt domain");

out:
    {
        int32_t released = dk_resource_release(rsdp.lease);
        if (status == DK_OK)
            status = released;
    }
    return status;
}

static const struct dk_driver_definition driver = {
    .name = DK_SLICE_LITERAL("ioapic"),
    .context = 0,
    .start = ioapic_start,
    .stop = NULL,
};

DK_DRIVER_EXPORT(driver)
