/*
 * Roanix driver framework: versioned kernel imports.
 *
 * <roanix/driver.h> wraps these C ABI functions with the public driver API.
 * A module imports only the functions it actually uses.
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

/* Terminal job-control state reported by a device node. */
struct rdf_terminal_state {
    int32_t session;
    int32_t foreground_group;
    uint8_t stop_background_output;
    uint8_t reserved[3];
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
    /*
     * Appended in ABI minor 1. The kernel accepts ABI-minor-0 112-byte
     * tables, where this callback is absent.
     */
    int32_t (*terminal_state)(void *context, struct rdf_terminal_state *out);
};

/* Exact logical-block operations implemented by a hardware driver. */
struct rdf_block_ops {
    uint32_t size;
    uint32_t flags;
    void *context;
    uint32_t logical_block_size;
    uint32_t max_transfer_blocks;
    uint64_t block_count;
    int32_t (*read)(void *context, uint64_t lba, uint32_t blocks, uint8_t *output);
    int32_t (*write)(void *context, uint64_t lba, uint32_t blocks, const uint8_t *input);
    int32_t (*flush)(void *context);
    int32_t (*discard)(void *context, uint64_t lba, uint32_t blocks);
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
/* Joinable worker callback for deferred driver work. */
typedef void (*rdf_worker_fn)(void *context);

/* --- Versioned kernel imports ------------------------------------------ */

extern void rdf_api_v2_log(uint32_t level, const char *module, const char *message);
extern void *rdf_api_v2_alloc(size_t size, size_t align);
extern void *rdf_api_v2_alloc_zeroed(size_t size, size_t align);
extern void rdf_api_v2_free(void *pointer, size_t size, size_t align);

extern const char *rdf_api_v2_module_name(const struct rdf_module *module);
extern int32_t rdf_api_v2_module_find(const char *name, const struct rdf_module **out);

extern int32_t rdf_api_v2_device_new(const struct rdf_module *module, const char *name,
                      struct rdf_device_builder **out);
extern int32_t rdf_api_v2_device_set_parent(struct rdf_device_builder *builder,
                             const struct rdf_device *parent);
extern int32_t rdf_api_v2_device_set_bus(struct rdf_device_builder *builder, const struct rdf_bus *bus);
extern int32_t rdf_api_v2_device_set_fwnode(struct rdf_device_builder *builder, uint32_t kind,
                             uint64_t token, const char *provider, const char *path);
extern int32_t rdf_api_v2_device_set_dma_mask(struct rdf_device_builder *builder, uint64_t mask);
extern int32_t rdf_api_v2_device_add_int(struct rdf_device_builder *builder, const char *name,
                          uint64_t value, uint32_t width);
extern int32_t rdf_api_v2_device_add_string(struct rdf_device_builder *builder, const char *name,
                             const char *value);
extern int32_t rdf_api_v2_device_add_strings(struct rdf_device_builder *builder, const char *name,
                              const char *const *values, size_t count);
extern int32_t rdf_api_v2_device_add_cells(struct rdf_device_builder *builder, const char *name,
                            const uint64_t *values, size_t count, uint32_t width);
extern int32_t rdf_api_v2_device_add_bytes(struct rdf_device_builder *builder, const char *name,
                            const uint8_t *values, size_t count);
extern int32_t rdf_api_v2_device_add_resource(struct rdf_device_builder *builder, uint32_t kind,
                               uint32_t flags, uint64_t start, uint64_t size,
                               const char *name);
extern int32_t rdf_api_v2_device_add_irq(struct rdf_device_builder *builder,
                          const struct rdf_irq_domain *domain, const uint32_t *cells,
                          size_t count);
extern int32_t rdf_api_v2_device_add(struct rdf_device_builder *builder, struct rdf_device **out);
extern void rdf_api_v2_device_discard(struct rdf_device_builder *builder);
extern int32_t rdf_api_v2_device_remove(const struct rdf_device *device);

extern int32_t rdf_api_v2_device_root(struct rdf_device **out);
extern const char *rdf_api_v2_device_name(const struct rdf_device *device);
extern struct rdf_device *rdf_api_v2_device_parent(const struct rdf_device *device);
extern size_t rdf_api_v2_device_child_count(const struct rdf_device *device);
extern struct rdf_device *rdf_api_v2_device_child(const struct rdf_device *device, size_t index);
extern void *rdf_api_v2_device_data(const struct rdf_device *device);
extern void rdf_api_v2_device_set_data(const struct rdf_device *device, void *value);
extern int32_t rdf_api_v2_device_int(const struct rdf_device *device, const char *name, uint64_t *out);
extern int32_t rdf_api_v2_device_cell(const struct rdf_device *device, const char *name, size_t index,
                       uint64_t *out);
extern int32_t rdf_api_v2_device_string(const struct rdf_device *device, const char *name, size_t index,
                         uint8_t *buffer, size_t capacity, size_t *written);
extern int32_t rdf_api_v2_device_bytes(const struct rdf_device *device, const char *name, uint8_t *buffer,
                        size_t capacity, size_t *written);
extern size_t rdf_api_v2_device_property_len(const struct rdf_device *device, const char *name);
extern int32_t rdf_api_v2_device_resource(const struct rdf_device *device, uint32_t kind, size_t index,
                           uint64_t *start, uint64_t *size, uint32_t *flags);
extern int32_t rdf_api_v2_device_fwnode(const struct rdf_device *device, uint32_t *kind, uint64_t *token);

extern int32_t rdf_api_v2_bus_register(const struct rdf_module *module, const char *name,
                        const struct rdf_bus_def *definition, const struct rdf_bus **out);
extern int32_t rdf_api_v2_bus_unregister(const struct rdf_bus *bus);
extern int32_t rdf_api_v2_bus_find(const char *name, const struct rdf_bus **out);
extern int32_t rdf_api_v2_bus_set_dma_ops(const struct rdf_bus *bus, const struct rdf_dma_ops *ops);
extern int32_t rdf_api_v2_driver_register(const struct rdf_module *module,
                           const struct rdf_driver_def *definition,
                           const struct rdf_driver **out);
extern int32_t rdf_api_v2_driver_unregister(const struct rdf_driver *driver);
extern int32_t rdf_api_v2_iface_publish(const struct rdf_module *module, const struct rdf_device *device,
                         const char *name, uint32_t version, uint32_t flags, const void *ops,
                         size_t ops_size, void *context, const struct rdf_iface **out);
extern int32_t rdf_api_v2_iface_withdraw(const struct rdf_iface *iface);
extern int32_t rdf_api_v2_iface_bind(const struct rdf_module *module, const struct rdf_device *device,
                      uint32_t scope, const char *name, uint32_t min_version,
                      struct rdf_iface_binding **binding, void **ops, void **context);
extern void rdf_api_v2_iface_unbind(struct rdf_iface_binding *binding);
extern int32_t rdf_api_v2_iface_available(const char *name, uint32_t min_version);
extern size_t rdf_api_v2_iface_count(const char *name, uint32_t min_version);
extern struct rdf_iface *rdf_api_v2_iface_provider(const char *name, uint32_t min_version, size_t index);
extern void rdf_api_v2_probe_retrigger(void);

extern int32_t rdf_api_v2_irq_domain_register(const struct rdf_module *module, const char *name,
                               uint32_t flags, uint32_t hwirq_count,
                               const struct rdf_irq_domain_def *definition,
                               const struct rdf_irq_domain **out);
extern int32_t rdf_api_v2_irq_domain_unregister(const struct rdf_irq_domain *domain);
extern int32_t rdf_api_v2_irq_map(const struct rdf_irq_domain *domain, uint64_t hwirq, uint32_t flags,
                   uint32_t *out_virq);
extern int32_t rdf_api_v2_irq_unmap(const struct rdf_irq_domain *domain, uint64_t hwirq);
extern int32_t rdf_api_v2_irq_of_device(const struct rdf_device *device, size_t index, uint32_t *out_virq);
extern int32_t rdf_api_v2_irq_request(const struct rdf_module *module, const struct rdf_device *device,
                       uint32_t virq, const char *name, uint32_t flags,
                       rdf_irq_handler_fn handler, rdf_irq_thread_fn thread, void *context,
                       struct rdf_irq **out);
extern int32_t rdf_api_v2_irq_release(struct rdf_irq *irq);
extern int32_t rdf_api_v2_irq_mask(uint32_t virq);
extern int32_t rdf_api_v2_irq_unmask(uint32_t virq);
extern int32_t rdf_api_v2_irq_set_affinity(uint32_t virq, uint32_t cpu);
extern int32_t rdf_api_v2_irq_alloc_vector(uint32_t virq, uint32_t *out_vector);
extern int32_t rdf_api_v2_irq_free_vector(uint32_t vector);
extern int32_t rdf_api_v2_irq_compose_message(const struct rdf_irq_domain *domain, uint64_t hwirq,
                               uint64_t *out_address, uint32_t *out_data);

extern int32_t rdf_api_v2_mmio_map(const struct rdf_module *module, uint64_t physical, size_t length,
                    uint32_t flags, struct rdf_mmio *out);
extern int32_t rdf_api_v2_mmio_unmap(struct rdf_mmio *window);
extern int32_t rdf_api_v2_mmio_direct(uint64_t physical, void **out);
extern int32_t rdf_api_v2_dma_alloc(const struct rdf_module *module, const struct rdf_device *device,
                     size_t size, size_t align, uint32_t flags, struct rdf_dma *out);
extern int32_t rdf_api_v2_dma_free(struct rdf_dma *buffer);
extern int32_t rdf_api_v2_dma_map(const struct rdf_device *device, void *address, size_t size,
                   uint32_t direction, uint64_t *out);
extern void rdf_api_v2_dma_unmap(const struct rdf_device *device, uint64_t address, size_t size,
                  uint32_t direction);
extern void rdf_api_v2_dma_sync(void *address, size_t size, uint32_t direction);

extern int32_t rdf_api_v2_work_create(const struct rdf_module *module, const char *name,
                       struct rdf_work **out);
extern int32_t rdf_api_v2_work_queue(struct rdf_work *queue, rdf_work_fn callback, void *context,
                      uint64_t argument);
extern int32_t rdf_api_v2_work_flush(struct rdf_work *queue);
extern int32_t rdf_api_v2_work_destroy(struct rdf_work *queue);
extern int32_t rdf_api_v2_timer_create(const struct rdf_module *module, rdf_work_fn callback, void *context,
                        uint64_t argument, struct rdf_timer **out);
extern int32_t rdf_api_v2_timer_arm(struct rdf_timer *timer, uint64_t delay_ns, uint64_t period_ns);
extern int32_t rdf_api_v2_timer_cancel(struct rdf_timer *timer);
extern int32_t rdf_api_v2_timer_destroy(struct rdf_timer *timer);
extern int32_t rdf_api_v2_event_create(const struct rdf_module *module, rdf_event_t *out);
extern int32_t rdf_api_v2_event_destroy(rdf_event_t event);
extern int32_t rdf_api_v2_event_wait(rdf_event_t event);
extern int32_t rdf_api_v2_event_signal(rdf_event_t event);
extern int32_t rdf_api_v2_event_reset(rdf_event_t event);

extern uint64_t rdf_api_v2_time_monotonic(void);
extern void rdf_api_v2_time_delay(uint64_t nanoseconds);
extern void rdf_api_v2_time_sleep(uint64_t nanoseconds);
extern void rdf_api_v2_random_fill(uint8_t *buffer, size_t length);
extern void rdf_api_v2_random_mix(const uint8_t *buffer, size_t length);
extern uint32_t rdf_api_v2_cpu_count(void);
extern uint32_t rdf_api_v2_cpu_current(void);
extern int32_t rdf_api_v2_cpu_platform_id(uint32_t cpu, uint64_t *out);
extern int32_t rdf_api_v2_in_interrupt(void);

extern int32_t rdf_api_v2_firmware_acpi(uint8_t *buffer, size_t capacity, size_t *written);
extern int32_t rdf_api_v2_firmware_devicetree(uint8_t *buffer, size_t capacity, size_t *written);

extern int32_t rdf_api_v2_devfs_root(rdf_devnode_t *out);
extern int32_t rdf_api_v2_devfs_mkdir(const struct rdf_module *module, rdf_devnode_t parent,
                       const char *name, uint16_t mode, rdf_devnode_t *out);
extern int32_t rdf_api_v2_devfs_create(const struct rdf_module *module, rdf_devnode_t parent,
                        const char *name, uint32_t kind, uint16_t mode,
                        const struct rdf_node_ops *ops, rdf_devnode_t *out);
extern int32_t rdf_api_v2_devfs_remove(const struct rdf_module *module, rdf_devnode_t node);
extern int32_t rdf_api_v2_devfs_lookup(const char *path, rdf_devnode_t *out);
extern int32_t rdf_api_v2_block_register(const struct rdf_module *module, const char *name,
                           const struct rdf_block_ops *ops,
                           const struct rdf_block_device **out);
extern int32_t rdf_api_v2_block_unregister(const struct rdf_block_device *device);
extern int32_t rdf_api_v2_block_rescan(const struct rdf_block_device *device);
extern int32_t rdf_api_v2_tty_register(const struct rdf_module *module, rdf_devnode_t parent,
                        const char *name, uint16_t mode, uint32_t baud,
                        const struct rdf_console_ops *ops, struct rdf_tty **out);
extern int32_t rdf_api_v2_tty_unregister(struct rdf_tty *tty);

extern uint32_t rdf_api_v2_klog_level(void);
extern uint32_t rdf_api_v2_klog_set_level(uint32_t level);
extern uint32_t rdf_api_v2_klog_console_level(void);
extern uint32_t rdf_api_v2_klog_set_console_level(uint32_t level);

/* Event fan-in and module-owned worker services. */
extern int32_t rdf_api_v2_event_wait_any(const rdf_event_t *events, size_t count, size_t *out_index);
extern int32_t rdf_api_v2_event_wait_timeout(rdf_event_t event, uint64_t nanoseconds,
                              uint8_t *out_signalled);
extern int32_t rdf_api_v2_worker_spawn(const struct rdf_module *module, rdf_worker_fn callback,
                        void *context, struct rdf_worker **out);
extern int32_t rdf_api_v2_worker_join(struct rdf_worker *worker);

/* Native terminal state exposed by device-node operations. */
_Static_assert(sizeof(struct rdf_terminal_state) == 12, "rdf terminal state ABI");
_Static_assert(offsetof(struct rdf_node_ops, terminal_state) == 112, "rdf node ops ABI");
_Static_assert(sizeof(struct rdf_node_ops) == 120, "rdf node ops ABI");
_Static_assert(sizeof(struct rdf_block_ops) == 64, "rdf block ops ABI");

/* Module entry point resolved as the image's ELF entry. */
struct rdf_module_def {
    uint32_t size;
    uint16_t abi_major;
    uint16_t abi_reserved;
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
