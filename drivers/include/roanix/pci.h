/* PCI MSI service ABI, revision 1. Calls may sleep and may run concurrently. */
#ifndef ROANIX_PCI_H
#define ROANIX_PCI_H
#include <roanix/api.h>

struct rdf_pci_msi_ops {
    uint32_t size;
    int32_t (*enable)(void *context, const struct rdf_device *device,
                      uint32_t requested, uint32_t *virqs, uint32_t *actual);
    int32_t (*unmask)(void *context, const struct rdf_device *device);
    void (*disable)(void *context, const struct rdf_device *device);
};
#endif
