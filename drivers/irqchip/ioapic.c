/*
 * I/O APIC interrupt controller.
 *
 * Binds to every I/O APIC the platform enumerator describes and presents them
 * as one interrupt domain covering the global system interrupt space, which is
 * how firmware and device drivers refer to lines. A global system interrupt is
 * mapped to a processor vector on first use and programmed into the owning
 * chip's redirection table.
 */
#include <roanix/driver.h>

#define REGISTER_SELECT 0x00u
#define REGISTER_WINDOW 0x10u

#define REG_ID 0x00u
#define REG_VERSION 0x01u
#define REDIRECTION_BASE 0x10u

#define REDIRECTION_MASKED (UINT32_C(1) << 16)
#define REDIRECTION_TRIGGER_LEVEL (UINT32_C(1) << 15)
#define REDIRECTION_POLARITY_LOW (UINT32_C(1) << 13)

#define MAX_CHIPS 8u
#define MAX_GSI 256u
#define MAX_REDIRECTIONS 120u

struct chip {
    struct rdf_mmio window;
    struct rdf_spinlock lock;
    uint32_t gsi_base;
    uint32_t redirections;
    uint8_t present;
};

struct route {
    uint32_t vector;
    uint8_t active;
};

static struct chip chips[MAX_CHIPS];
static size_t chip_count;
static struct route routes[MAX_GSI];
static const struct rdf_irq_domain *domain;
static struct rdf_spinlock domain_lock = RDF_SPINLOCK_INIT;

static uint32_t chip_read(const struct chip *chip, uint8_t reg)
{
    rdf_write32(&chip->window, REGISTER_SELECT, reg);
    return rdf_read32(&chip->window, REGISTER_WINDOW);
}

static void chip_write(const struct chip *chip, uint8_t reg, uint32_t value)
{
    rdf_write32(&chip->window, REGISTER_SELECT, reg);
    rdf_write32(&chip->window, REGISTER_WINDOW, value);
}

static struct chip *chip_for(uint64_t hwirq)
{
    for (size_t index = 0; index < chip_count; ++index) {
        struct chip *chip = &chips[index];
        if (chip->present == 0)
            continue;
        if (hwirq >= chip->gsi_base && hwirq - chip->gsi_base < chip->redirections)
            return chip;
    }
    return NULL;
}

/* Writes a redirection entry high half first, so the entry is never half-live. */
static void write_redirection(const struct chip *chip, uint32_t index, uint32_t low,
                              uint32_t high)
{
    uint8_t reg = (uint8_t)(REDIRECTION_BASE + index * 2u);
    chip_write(chip, (uint8_t)(reg + 1u), high);
    chip_write(chip, reg, low);
}

static int32_t domain_translate(void *context, const uint32_t *cells, size_t count,
                                uint64_t *out_hwirq, uint32_t *out_flags)
{
    (void)context;
    if (count == 0)
        return RDF_EINVAL;
    *out_hwirq = cells[0];
    *out_flags = count > 1 ? cells[1] : (RDF_IRQ_EDGE | RDF_IRQ_ACTIVE_HIGH);
    return RDF_OK;
}

static int32_t domain_setup(void *context, uint64_t hwirq, uint32_t virq, uint32_t flags)
{
    (void)context;
    struct chip *chip = chip_for(hwirq);
    if (chip == NULL || hwirq >= MAX_GSI)
        return RDF_ENOENT;

    uint32_t vector = 0;
    int32_t status = rdf_irq_alloc_vector(virq, &vector);
    if (status != RDF_OK)
        return status;

    uint32_t low = vector & 0xffu;
    low |= REDIRECTION_MASKED;
    if ((flags & RDF_IRQ_ACTIVE_LOW) != 0)
        low |= REDIRECTION_POLARITY_LOW;
    if ((flags & RDF_IRQ_LEVEL) != 0)
        low |= REDIRECTION_TRIGGER_LEVEL;

    uint64_t destination = 0;
    if (rdf_cpu_platform_id(0, &destination) != RDF_OK || destination > 0xffu)
        destination = 0;

    uintptr_t state = rdf_spin_lock_irqsave(&chip->lock);
    write_redirection(chip, (uint32_t)(hwirq - chip->gsi_base), low,
                      (uint32_t)destination << 24);
    rdf_spin_unlock_irqrestore(&chip->lock, state);

    routes[hwirq].vector = vector;
    routes[hwirq].active = 1;
    return RDF_OK;
}

static void domain_teardown(void *context, uint64_t hwirq, uint32_t virq)
{
    (void)context;
    (void)virq;
    struct chip *chip = chip_for(hwirq);
    if (chip == NULL || hwirq >= MAX_GSI || routes[hwirq].active == 0)
        return;

    uint32_t index = (uint32_t)(hwirq - chip->gsi_base);
    uint8_t reg = (uint8_t)(REDIRECTION_BASE + index * 2u);
    uintptr_t state = rdf_spin_lock_irqsave(&chip->lock);
    chip_write(chip, reg, chip_read(chip, reg) | REDIRECTION_MASKED);
    rdf_spin_unlock_irqrestore(&chip->lock, state);

    rdf_irq_free_vector(routes[hwirq].vector);
    routes[hwirq].active = 0;
}

static void set_mask(uint64_t hwirq, int masked)
{
    struct chip *chip = chip_for(hwirq);
    if (chip == NULL || hwirq >= MAX_GSI || routes[hwirq].active == 0)
        return;

    uint8_t reg = (uint8_t)(REDIRECTION_BASE + (hwirq - chip->gsi_base) * 2u);
    uintptr_t state = rdf_spin_lock_irqsave(&chip->lock);
    uint32_t low = chip_read(chip, reg);
    chip_write(chip, reg, masked ? (low | REDIRECTION_MASKED) : (low & ~REDIRECTION_MASKED));
    rdf_spin_unlock_irqrestore(&chip->lock, state);
}

static void domain_mask(void *context, uint64_t hwirq)
{
    (void)context;
    set_mask(hwirq, 1);
}

static void domain_unmask(void *context, uint64_t hwirq)
{
    (void)context;
    set_mask(hwirq, 0);
}

static int32_t domain_set_affinity(void *context, uint64_t hwirq, uint32_t cpu)
{
    (void)context;
    struct chip *chip = chip_for(hwirq);
    if (chip == NULL || hwirq >= MAX_GSI || routes[hwirq].active == 0)
        return RDF_ENOENT;

    uint64_t destination = 0;
    int32_t status = rdf_cpu_platform_id(cpu, &destination);
    if (status != RDF_OK)
        return status;
    if (destination > 0xffu)
        return RDF_ENOTSUP;

    uint8_t reg = (uint8_t)(REDIRECTION_BASE + (hwirq - chip->gsi_base) * 2u);
    uintptr_t state = rdf_spin_lock_irqsave(&chip->lock);
    chip_write(chip, (uint8_t)(reg + 1u), (uint32_t)destination << 24);
    rdf_spin_unlock_irqrestore(&chip->lock, state);
    return RDF_OK;
}

static const struct rdf_irq_domain_def domain_def = {
    .size = sizeof(struct rdf_irq_domain_def),
    .translate = domain_translate,
    .setup = domain_setup,
    .teardown = domain_teardown,
    .mask = domain_mask,
    .unmask = domain_unmask,
    .set_affinity = domain_set_affinity,
};

static int32_t ensure_domain(void)
{
    if (domain != NULL)
        return RDF_OK;
    return rdf_irq_domain_register("ioapic", RDF_IRQ_DOMAIN_DEFAULT, MAX_GSI, &domain_def,
                                   &domain);
}

static int32_t ioapic_probe(void *context, struct rdf_device *device, uintptr_t match_data)
{
    uint64_t address = 0;
    uint64_t size = 0;
    uint64_t gsi_base = 0;
    struct chip *chip = NULL;
    int32_t status;

    (void)context;
    (void)match_data;

    status = rdf_device_resource(device, RDF_RES_MEM, 0, &address, &size, NULL);
    if (status != RDF_OK)
        return status;
    status = rdf_device_u64(device, "gsi-base", &gsi_base);
    if (status != RDF_OK)
        return status;
    if (gsi_base >= MAX_GSI)
        return RDF_EINVAL;

    uintptr_t state = rdf_spin_lock_irqsave(&domain_lock);
    if (chip_count < MAX_CHIPS)
        chip = &chips[chip_count++];
    rdf_spin_unlock_irqrestore(&domain_lock, state);
    if (chip == NULL)
        return RDF_ENOSPC;

    rdf_spin_init(&chip->lock);
    status = rdf_mmio_map(address, (size_t)size, RDF_MMIO_DEVICE, &chip->window);
    if (status != RDF_OK)
        return status;

    uint32_t version = chip_read(chip, REG_VERSION);
    uint32_t redirections = ((version >> 16) & 0xffu) + 1u;
    if (redirections > MAX_REDIRECTIONS || gsi_base + redirections > MAX_GSI) {
        rdf_mmio_unmap(&chip->window);
        return RDF_EINVAL;
    }
    chip->gsi_base = (uint32_t)gsi_base;
    chip->redirections = redirections;

    /* Leave every line masked until a driver asks for it. */
    for (uint32_t index = 0; index < redirections; ++index)
        write_redirection(chip, index, REDIRECTION_MASKED, 0);
    chip->present = 1;

    status = ensure_domain();
    if (status != RDF_OK) {
        chip->present = 0;
        rdf_mmio_unmap(&chip->window);
        return status;
    }

    rdf_device_set_data(device, chip);
    RDF_INFO("%s: %u lines from global interrupt %u", rdf_device_name(device), redirections,
             chip->gsi_base);
    return RDF_OK;
}

static void ioapic_remove(void *context, struct rdf_device *device)
{
    struct chip *chip = (struct chip *)rdf_device_data(device);
    (void)context;
    if (chip == NULL)
        return;

    for (uint32_t index = 0; index < chip->redirections; ++index)
        write_redirection(chip, index, REDIRECTION_MASKED, 0);
    chip->present = 0;
    rdf_mmio_unmap(&chip->window);
    rdf_device_set_data(device, NULL);
}

static const struct rdf_match matches[] = {
    RDF_MATCH_COMPATIBLE("intel,ioapic"),
};

static struct rdf_driver_def driver = {
    .size = sizeof(struct rdf_driver_def),
    .name = "ioapic",
    .matches = matches,
    .match_count = RDF_COUNT(matches),
    .probe = ioapic_probe,
    .remove = ioapic_remove,
};

static const struct rdf_driver *registration;

static int32_t ioapic_init(struct rdf_module *self)
{
    int32_t status;
    (void)self;

    status = rdf_bus_find(RDF_BUS_PLATFORM, &driver.bus);
    if (status != RDF_OK)
        return status;
    return rdf_driver_register(&driver, &registration);
}

static void ioapic_exit(struct rdf_module *self)
{
    (void)self;
    if (registration != NULL)
        rdf_driver_unregister(registration);
    if (domain != NULL)
        rdf_irq_domain_unregister(domain);
}

RDF_MODULE("ioapic", "I/O APIC interrupt controller", ioapic_init, ioapic_exit)
