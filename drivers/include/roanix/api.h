/*
 * Roanix driver framework: the kernel service table.
 *
 * The layout below must match `struct Api` in the kernel exactly. It is
 * size-prefixed and append-only, so a module built against an earlier revision
 * keeps working against a newer kernel.
 *
 * A module never calls into this table directly. <roanix/driver.h> wraps every
 * entry in a static inline function, so a call costs one load of the module's
 * table pointer plus an indirect branch.
 */
#ifndef ROANIX_API_H
#define ROANIX_API_H

#include <roanix/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Records exchanged with the kernel ---------------------------------- */

/* A mapped register window. */
struct rdf_mmio {
    /* First mapped byte. */
    volatile void *base;
    /* Number of mapped bytes. */
    size_t length;
    /* Receipt consumed by rdf_mmio_unmap(). */
    void *token;
};

/* A coherent buffer shared with a device. */
struct rdf_dma {
    /* CPU address of the buffer. */
    void *cpu;
    /* Address the device must be programmed with. */
    uint64_t device;
    /* Number of usable bytes. */
    size_t length;
    /* Receipt consumed by rdf_dma_free(). */
    void *token;
};

/* One entry of a driver's match table. */
struct rdf_match {
    /* Entry kind, one of RDF_MATCH_*. */
    uint32_t kind;
    /* Entry flags. */
    uint32_t flags;
    /* String operand, or property name for property and identifier entries. */
    const char *key;
    /* Expected string value for property comparisons. */
    const char *value;
    /* Expected primary identifier. */
    uint64_t id0;
    /* Bits of the primary identifier that participate. */
    uint64_t mask0;
    /* Expected secondary identifier. */
    uint64_t id1;
    /* Bits of the secondary identifier that participate. */
    uint64_t mask1;
    /* Cookie handed to the probe callback when this entry wins. */
    uintptr_t data;
    /* Extra score contributed by this entry. */
    int32_t score;
};

/* A driver's registration description. */
struct rdf_driver_def {
    uint32_t size;
    const char *name;
    const struct rdf_bus *bus;
    int32_t priority;
    const struct rdf_match *matches;
    size_t match_count;
    int32_t (*probe)(void *context, struct rdf_device *device, uintptr_t match_data);
    void (*remove)(void *context, struct rdf_device *device);
    void (*shutdown)(void *context, struct rdf_device *device);
    void *context;
};

/* Bus callbacks. */
struct rdf_bus_def {
    uint32_t size;
    int32_t (*match)(void *context, struct rdf_device *device, struct rdf_driver *driver);
    int32_t (*prepare)(void *context, struct rdf_device *device);
    void (*cleanup)(void *context, struct rdf_device *device);
    void (*shutdown)(void *context, struct rdf_device *device);
    void *context;
};

/* Class callbacks. */
struct rdf_class_def {
    uint32_t size;
    int32_t (*attach)(void *context, struct rdf_class_device *member);
    void (*detach)(void *context, struct rdf_class_device *member);
    void *context;
};

/* Interrupt controller callbacks. */
struct rdf_irq_domain_def {
    uint32_t size;
    int32_t (*translate)(void *context, const uint32_t *cells, size_t count,
                         uint64_t *out_hwirq, uint32_t *out_flags);
    int32_t (*setup)(void *context, uint64_t hwirq, uint32_t virq, uint32_t flags);
    void (*teardown)(void *context, uint64_t hwirq, uint32_t virq);
    void (*mask)(void *context, uint64_t hwirq);
    void (*unmask)(void *context, uint64_t hwirq);
    void (*eoi)(void *context, uint64_t hwirq);
    int32_t (*set_affinity)(void *context, uint64_t hwirq, uint32_t cpu);
    int32_t (*claim)(void *context, uint32_t cpu, uint64_t platform_id, uint64_t *out_hwirq);
    void (*complete)(void *context, uint32_t cpu, uint64_t platform_id, uint64_t hwirq);
    int32_t (*compose_message)(void *context, uint64_t hwirq, uint64_t *out_address,
                               uint32_t *out_data);
    void *context;
};

/* Caller identity supplied to a device-node control operation. */
struct rdf_ioctl_identity {
    uint64_t process;
    int32_t group;
    int32_t session;
    uint8_t session_leader;
};

/* Operations a driver implements for a device node. */
struct rdf_node_ops {
    uint32_t size;
    void *context;
    int32_t (*open)(void *context, uint32_t flags, uintptr_t *out_file);
    void (*close)(void *context, uintptr_t file, uint32_t flags);
    int64_t (*initial_offset)(void *context, uintptr_t file, uint32_t flags);
    int64_t (*read)(void *context, uintptr_t file, uint64_t offset, uint8_t *data, size_t len,
                    uint32_t flags);
    int64_t (*write)(void *context, uintptr_t file, uint64_t offset, const uint8_t *data,
                     size_t len, uint32_t flags);
    uint64_t (*size_bytes)(void *context);
    int32_t (*sync)(void *context);
    int64_t (*poll)(void *context, uintptr_t file, uint64_t offset, uint16_t events,
                    uint32_t flags);
    int64_t (*ioctl)(void *context, uintptr_t file, const struct rdf_ioctl_identity *identity,
                     uint64_t request, uint64_t value, uint8_t *argument, size_t argument_len);
    uintptr_t (*readable_event)(void *context, uintptr_t file);
    uintptr_t (*writable_event)(void *context, uintptr_t file);
    uintptr_t (*hangup_event)(void *context, uintptr_t file);
};

/* Serial framing parameters handed to a console backend. */
struct rdf_serial_framing {
    uint32_t baud;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t odd_parity;
};

/* Byte-level operations a console backend implements. */
struct rdf_console_ops {
    uint32_t size;
    uint64_t flags;
    void *context;
    int32_t (*open)(void *context);
    void (*close)(void *context);
    int32_t (*try_read)(void *context, uint8_t *out_byte);
    int64_t (*read)(void *context, uint8_t *output, size_t length);
    int32_t (*write)(void *context, const uint8_t *data, size_t len, uint8_t nonblocking);
    int32_t (*configure)(void *context, const struct rdf_serial_framing *framing);
    int32_t (*flush)(void *context);
    int32_t (*flush_input)(void *context);
    int32_t (*flush_output)(void *context);
    int32_t (*send_break)(void *context, uint64_t duration_ms);
    int32_t (*writable)(void *context);
    int32_t (*hung_up)(void *context);
    int64_t (*queued_output)(void *context);
    void (*destroy)(void *context);
    rdf_event_t readable_event;
    rdf_event_t writable_event;
    rdf_event_t hangup_event;
};

/* DMA translation a bus imposes on its devices. */
struct rdf_dma_ops {
    uint32_t size;
    uint64_t (*to_device)(void *context, uint64_t physical);
    uint64_t (*from_device)(void *context, uint64_t bus);
    void *context;
};

/* Top-half interrupt handler. */
typedef uint32_t (*rdf_irq_handler_fn)(void *context, uint32_t virq);
/* Threaded bottom-half interrupt handler. */
typedef void (*rdf_irq_thread_fn)(void *context, uint32_t virq);
/* Deferred work or timer callback. */
typedef void (*rdf_work_fn)(void *context, uint64_t argument);

/* --- The service table -------------------------------------------------- */

struct rdf_api {
    uint32_t size;
    uint32_t revision;
    uint16_t abi_major;
    uint16_t abi_minor;
    uint32_t reserved;

    void (*log)(uint32_t level, const char *module, const char *message);
    void *(*alloc)(size_t size, size_t align);
    void *(*alloc_zeroed)(size_t size, size_t align);
    void (*free)(void *pointer, size_t size, size_t align);

    const char *(*module_name)(const struct rdf_module *module);
    int32_t (*module_find)(const char *name, const struct rdf_module **out);

    int32_t (*device_new)(const struct rdf_module *module, const char *name,
                          struct rdf_device_builder **out);
    int32_t (*device_set_parent)(struct rdf_device_builder *builder,
                                 const struct rdf_device *parent);
    int32_t (*device_set_bus)(struct rdf_device_builder *builder, const struct rdf_bus *bus);
    int32_t (*device_set_fwnode)(struct rdf_device_builder *builder, uint32_t kind,
                                 uint64_t token, const char *provider, const char *path);
    int32_t (*device_set_dma_mask)(struct rdf_device_builder *builder, uint64_t mask);
    int32_t (*device_add_int)(struct rdf_device_builder *builder, const char *name,
                              uint64_t value, uint32_t width);
    int32_t (*device_add_string)(struct rdf_device_builder *builder, const char *name,
                                 const char *value);
    int32_t (*device_add_strings)(struct rdf_device_builder *builder, const char *name,
                                  const char *const *values, size_t count);
    int32_t (*device_add_cells)(struct rdf_device_builder *builder, const char *name,
                                const uint64_t *values, size_t count, uint32_t width);
    int32_t (*device_add_bytes)(struct rdf_device_builder *builder, const char *name,
                                const uint8_t *values, size_t count);
    int32_t (*device_add_resource)(struct rdf_device_builder *builder, uint32_t kind,
                                   uint32_t flags, uint64_t start, uint64_t size,
                                   const char *name);
    int32_t (*device_add_irq)(struct rdf_device_builder *builder,
                              const struct rdf_irq_domain *domain, const uint32_t *cells,
                              size_t count);
    int32_t (*device_add)(struct rdf_device_builder *builder, struct rdf_device **out);
    void (*device_discard)(struct rdf_device_builder *builder);
    int32_t (*device_remove)(const struct rdf_device *device);

    int32_t (*device_root)(struct rdf_device **out);
    const char *(*device_name)(const struct rdf_device *device);
    struct rdf_device *(*device_parent)(const struct rdf_device *device);
    size_t (*device_child_count)(const struct rdf_device *device);
    struct rdf_device *(*device_child)(const struct rdf_device *device, size_t index);
    void *(*device_data)(const struct rdf_device *device);
    void (*device_set_data)(const struct rdf_device *device, void *value);
    int32_t (*device_int)(const struct rdf_device *device, const char *name, uint64_t *out);
    int32_t (*device_cell)(const struct rdf_device *device, const char *name, size_t index,
                           uint64_t *out);
    int32_t (*device_string)(const struct rdf_device *device, const char *name, size_t index,
                             uint8_t *buffer, size_t capacity, size_t *written);
    int32_t (*device_bytes)(const struct rdf_device *device, const char *name, uint8_t *buffer,
                            size_t capacity, size_t *written);
    size_t (*device_property_len)(const struct rdf_device *device, const char *name);
    int32_t (*device_resource)(const struct rdf_device *device, uint32_t kind, size_t index,
                               uint64_t *start, uint64_t *size, uint32_t *flags);
    int32_t (*device_fwnode)(const struct rdf_device *device, uint32_t *kind, uint64_t *token);

    int32_t (*bus_register)(const struct rdf_module *module, const char *name,
                            const struct rdf_bus_def *definition, const struct rdf_bus **out);
    int32_t (*bus_unregister)(const struct rdf_bus *bus);
    int32_t (*bus_find)(const char *name, const struct rdf_bus **out);
    int32_t (*bus_set_dma_ops)(const struct rdf_bus *bus, const struct rdf_dma_ops *ops);
    int32_t (*driver_register)(const struct rdf_module *module,
                               const struct rdf_driver_def *definition,
                               const struct rdf_driver **out);
    int32_t (*driver_unregister)(const struct rdf_driver *driver);
    int32_t (*class_register)(const struct rdf_module *module, const char *name,
                              const struct rdf_class_def *definition,
                              const struct rdf_class **out);
    int32_t (*class_unregister)(const struct rdf_class *class_object);
    int32_t (*class_find)(const char *name, const struct rdf_class **out);
    int32_t (*class_add)(const struct rdf_module *module, const struct rdf_class *class_object,
                         const struct rdf_device *device, const char *name, const void *ops,
                         size_t ops_size, void *context, const struct rdf_class_device **out);
    void (*class_remove)(const struct rdf_class_device *member);
    int32_t (*class_member_ops)(const struct rdf_class_device *member, const void **ops,
                                void **context);
    const char *(*class_member_name)(const struct rdf_class_device *member);
    struct rdf_device *(*class_member_device)(const struct rdf_class_device *member);
    uintptr_t (*class_member_data)(const struct rdf_class_device *member);
    void (*class_member_set_data)(const struct rdf_class_device *member, uintptr_t value);
    size_t (*class_member_count)(const struct rdf_class *class_object);
    struct rdf_class_device *(*class_member)(const struct rdf_class *class_object, size_t index);

    int32_t (*iface_publish)(const struct rdf_module *module, const struct rdf_device *device,
                             const char *name, uint32_t version, uint32_t flags, const void *ops,
                             size_t ops_size, void *context, const struct rdf_iface **out);
    int32_t (*iface_withdraw)(const struct rdf_iface *iface);
    int32_t (*iface_bind)(const struct rdf_module *module, const struct rdf_device *device,
                          uint32_t scope, const char *name, uint32_t min_version,
                          struct rdf_iface_binding **binding, void **ops, void **context);
    void (*iface_unbind)(struct rdf_iface_binding *binding);
    int32_t (*iface_available)(const char *name, uint32_t min_version);
    size_t (*iface_count)(const char *name, uint32_t min_version);
    struct rdf_iface *(*iface_provider)(const char *name, uint32_t min_version, size_t index);
    void (*probe_retrigger)(void);

    int32_t (*irq_domain_register)(const struct rdf_module *module, const char *name,
                                   uint32_t flags, uint32_t hwirq_count,
                                   const struct rdf_irq_domain_def *definition,
                                   const struct rdf_irq_domain **out);
    int32_t (*irq_domain_unregister)(const struct rdf_irq_domain *domain);
    int32_t (*irq_map)(const struct rdf_irq_domain *domain, uint64_t hwirq, uint32_t flags,
                       uint32_t *out_virq);
    int32_t (*irq_of_device)(const struct rdf_device *device, size_t index, uint32_t *out_virq);
    int32_t (*irq_request)(const struct rdf_module *module, const struct rdf_device *device,
                           uint32_t virq, const char *name, uint32_t flags,
                           rdf_irq_handler_fn handler, rdf_irq_thread_fn thread, void *context,
                           struct rdf_irq **out);
    int32_t (*irq_release)(struct rdf_irq *irq);
    int32_t (*irq_mask)(uint32_t virq);
    int32_t (*irq_unmask)(uint32_t virq);
    int32_t (*irq_set_affinity)(uint32_t virq, uint32_t cpu);
    int32_t (*irq_alloc_vector)(uint32_t virq, uint32_t *out_vector);
    int32_t (*irq_free_vector)(uint32_t vector);
    int32_t (*irq_compose_message)(const struct rdf_irq_domain *domain, uint64_t hwirq,
                                   uint64_t *out_address, uint32_t *out_data);

    int32_t (*mmio_map)(const struct rdf_module *module, uint64_t physical, size_t length,
                        uint32_t flags, struct rdf_mmio *out);
    int32_t (*mmio_unmap)(struct rdf_mmio *window);
    int32_t (*mmio_direct)(uint64_t physical, void **out);
    uint32_t (*port_read8)(uint16_t port);
    uint32_t (*port_read16)(uint16_t port);
    uint32_t (*port_read32)(uint16_t port);
    void (*port_write8)(uint16_t port, uint8_t value);
    void (*port_write16)(uint16_t port, uint16_t value);
    void (*port_write32)(uint16_t port, uint32_t value);
    int32_t (*dma_alloc)(const struct rdf_module *module, const struct rdf_device *device,
                         size_t size, size_t align, uint32_t flags, struct rdf_dma *out);
    int32_t (*dma_free)(struct rdf_dma *buffer);
    int32_t (*dma_map)(const struct rdf_device *device, void *address, size_t size,
                       uint32_t direction, uint64_t *out);
    void (*dma_unmap)(const struct rdf_device *device, uint64_t address, size_t size,
                      uint32_t direction);
    void (*dma_sync)(void *address, size_t size, uint32_t direction);

    int32_t (*work_create)(const struct rdf_module *module, const char *name,
                           struct rdf_work **out);
    int32_t (*work_queue)(struct rdf_work *queue, rdf_work_fn callback, void *context,
                          uint64_t argument);
    int32_t (*work_flush)(struct rdf_work *queue);
    int32_t (*work_destroy)(struct rdf_work *queue);
    int32_t (*timer_create)(const struct rdf_module *module, rdf_work_fn callback, void *context,
                            uint64_t argument, struct rdf_timer **out);
    int32_t (*timer_arm)(struct rdf_timer *timer, uint64_t delay_ns, uint64_t period_ns);
    int32_t (*timer_cancel)(struct rdf_timer *timer);
    int32_t (*timer_destroy)(struct rdf_timer *timer);
    int32_t (*event_create)(const struct rdf_module *module, rdf_event_t *out);
    int32_t (*event_destroy)(rdf_event_t event);
    int32_t (*event_wait)(rdf_event_t event);
    int32_t (*event_signal)(rdf_event_t event);
    int32_t (*event_reset)(rdf_event_t event);

    uint64_t (*time_monotonic)(void);
    void (*time_delay)(uint64_t nanoseconds);
    void (*time_sleep)(uint64_t nanoseconds);
    void (*random_fill)(uint8_t *buffer, size_t length);
    void (*random_mix)(const uint8_t *buffer, size_t length);
    uint32_t (*cpu_count)(void);
    uint32_t (*cpu_current)(void);
    int32_t (*cpu_platform_id)(uint32_t cpu, uint64_t *out);
    int32_t (*in_interrupt)(void);

    int32_t (*firmware_acpi)(uint8_t *buffer, size_t capacity, size_t *written);
    int32_t (*firmware_devicetree)(uint8_t *buffer, size_t capacity, size_t *written);

    int32_t (*devfs_root)(rdf_devnode_t *out);
    int32_t (*devfs_mkdir)(const struct rdf_module *module, rdf_devnode_t parent,
                           const char *name, uint16_t mode, rdf_devnode_t *out);
    int32_t (*devfs_create)(const struct rdf_module *module, const struct rdf_device *device,
                            rdf_devnode_t parent, const char *name, uint32_t kind, uint16_t mode,
                            const struct rdf_node_ops *ops, rdf_devnode_t *out);
    int32_t (*devfs_remove)(const struct rdf_module *module, rdf_devnode_t node);
    int32_t (*devfs_lookup)(const char *path, rdf_devnode_t *out);
    int32_t (*tty_register)(const struct rdf_module *module, const struct rdf_device *device,
                            rdf_devnode_t parent, const char *name, uint16_t mode, uint32_t baud,
                            const struct rdf_console_ops *ops, struct rdf_tty **out);
    int32_t (*tty_unregister)(struct rdf_tty *tty);

    uint32_t (*klog_level)(void);
    uint32_t (*klog_set_level)(uint32_t level);
    uint32_t (*klog_console_level)(void);
    uint32_t (*klog_set_console_level)(uint32_t level);
};

/* Module entry point resolved as the image's ELF entry. */
struct rdf_module_def {
    uint32_t size;
    uint16_t abi_major;
    uint16_t abi_minor;
    uint32_t flags;
    const char *name;
    const char *description;
    int32_t (*init)(struct rdf_module *self);
    void (*exit)(struct rdf_module *self);
};

#ifdef __cplusplus
}
#endif

#endif
