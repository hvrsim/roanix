#ifndef DEVKIT_BINDING_H
#define DEVKIT_BINDING_H

#include <devkit/resource.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DK_NODE_ANY 0u
#define DK_NODE_BUS 1u
#define DK_NODE_DEVICE 2u

typedef uint64_t dk_driver_class_t;

struct dk_match_property {
    struct dk_resource_key key;
    struct dk_slice value;
};

typedef int32_t (*dk_driver_bind_fn)(
    uintptr_t context,
    dk_node_t provider,
    uintptr_t *out_instance_context);
typedef void (*dk_driver_unbind_fn)(
    uintptr_t context,
    dk_node_t provider,
    uintptr_t instance_context);

struct dk_driver_class {
    uint32_t size;
    int32_t priority;
    uint32_t node_kind;
    uint32_t flags;
    struct dk_slice name;
    uintptr_t context;
    const struct dk_match_property *properties;
    size_t property_count;
    const struct dk_resource_key *resources;
    size_t resource_count;
    dk_driver_bind_fn bind;
    dk_driver_unbind_fn unbind;
};

int32_t dk_driver_class_register(
    const struct dk_driver_class *definition,
    dk_driver_class_t *out_class);
int32_t dk_driver_class_unregister(dk_driver_class_t class_id);

#ifdef __cplusplus
}
#endif

#endif
