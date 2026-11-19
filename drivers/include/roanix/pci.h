/* PCI MSI service ABI, revision 1. Calls may sleep and may run concurrently. */
#ifndef ROANIX_PCI_H
#define ROANIX_PCI_H

#include <roanix/api.h>

/* Operation table published by the PCI message interrupt provider. */
struct rdf_pci_msi_ops {
    /* Extent of this table in bytes. */
    uint32_t size;

    /* Allocates masked routes into the caller's array of requested entries. */
    int32_t (*enable)(void *context, const struct rdf_device *device,
                      uint32_t requested, uint32_t *virqs, uint32_t *actual);

    /* Enables delivery after the consumer has installed its IRQ handlers. */
    int32_t (*unmask)(void *context, const struct rdf_device *device);

    /* Stops message delivery and retires the mapping after handler release. */
    void (*disable)(void *context, const struct rdf_device *device);
};

/* Optional v1 extension for consumers that specify CPU destinations. */
struct rdf_pci_msi_affinity_ops {
    /* The original operation table. Its size includes this extension. */
    struct rdf_pci_msi_ops base;

    /* The CPU and output IRQ arrays each hold requested entries and are disjoint.
     * The provider may return fewer routes in the supplied order. */
    int32_t (*enable_with_affinity)(void *context, const struct rdf_device *device,
                                   uint32_t requested, const uint32_t *cpus,
                                   uint32_t *virqs, uint32_t *actual);
};

#endif
