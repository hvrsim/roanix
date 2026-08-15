/*
 * Roanix driver framework: the driver-facing API.
 *
 * Include this one header to write a driver. Everything below is either a
 * static inline wrapper over the kernel service table or a compile-time
 * constant, so a module needs no support library at all.
 *
 * A module looks like this:
 *
 *     #include <roanix/driver.h>
 *
 *     static int32_t probe(void *ctx, struct rdf_device *dev, uintptr_t data)
 *     {
 *         ...
 *         return RDF_OK;
 *     }
 *
 *     static const struct rdf_match matches[] = {
 *         RDF_MATCH_COMPATIBLE("vendor,widget"),
 *     };
 *
 *     static int32_t init(struct rdf_module *self)
 *     {
 *         static struct rdf_driver_def def = { ... };
 *         return rdf_driver_register(&def, NULL);
 *     }
 *
 *     RDF_MODULE("widget", "Widget controller", init, NULL);
 */
#ifndef ROANIX_DRIVER_H
#define ROANIX_DRIVER_H

#include <roanix/api.h>
#include <roanix/io.h>
#include <roanix/print.h>
#include <roanix/string.h>
#include <roanix/sync.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Constants ---------------------------------------------------------- */

/* Match entry kinds. */
#define RDF_MATCH_KIND_COMPATIBLE 1u
#define RDF_MATCH_KIND_NAME 2u
#define RDF_MATCH_KIND_ACPI 3u
#define RDF_MATCH_KIND_PROPERTY 4u
#define RDF_MATCH_KIND_ID 5u
#define RDF_MATCH_KIND_ANY 6u

/* Match entry flags. */
#define RDF_MATCH_NO_BONUS (UINT32_C(1) << 0)

/* Well-known property names used by matching. */
#define RDF_PROP_COMPATIBLE "compatible"
#define RDF_PROP_ID "id"
#define RDF_PROP_CLASS "class"
#define RDF_PROP_ACPI_HID "acpi.hid"
#define RDF_PROP_ACPI_CID "acpi.cid"

/* Property widths for the integer entry points. */
#define RDF_WIDTH_32 0u
#define RDF_WIDTH_64 1u

/* Resource kinds. */
#define RDF_RES_MEM 1u
#define RDF_RES_IO 2u
#define RDF_RES_IRQ 3u
#define RDF_RES_DMA 4u
#define RDF_RES_BUS 5u

/* Resource flags. */
#define RDF_RES_READONLY (UINT32_C(1) << 0)
#define RDF_RES_PREFETCH (UINT32_C(1) << 1)
#define RDF_RES_CACHEABLE (UINT32_C(1) << 2)
#define RDF_RES_WIDE (UINT32_C(1) << 3)
#define RDF_RES_SHARED (UINT32_C(1) << 4)

/* Firmware description formats. */
#define RDF_FW_DEVICETREE 1u
#define RDF_FW_ACPI 2u
#define RDF_FW_SYNTHETIC 3u

/* Interface behaviour flags. */
#define RDF_IFACE_IRQ_SAFE (UINT32_C(1) << 0)
#define RDF_IFACE_MAY_SLEEP (UINT32_C(1) << 1)
#define RDF_IFACE_CONCURRENT (UINT32_C(1) << 2)
#define RDF_IFACE_SINGLETON (UINT32_C(1) << 3)

/* Interface binding scopes. */
#define RDF_SCOPE_GLOBAL 0u
#define RDF_SCOPE_DEVICE 1u
#define RDF_SCOPE_ANCESTOR 2u

/* Interrupt handler results. */
#define RDF_IRQ_NONE 0u
#define RDF_IRQ_HANDLED (UINT32_C(1) << 0)
#define RDF_IRQ_WAKE_THREAD (UINT32_C(1) << 1)
#define RDF_IRQ_RESCHEDULE (UINT32_C(1) << 2)

/* Interrupt trigger and sharing flags. */
#define RDF_IRQ_EDGE (UINT32_C(1) << 0)
#define RDF_IRQ_LEVEL (UINT32_C(1) << 1)
#define RDF_IRQ_ACTIVE_HIGH (UINT32_C(1) << 2)
#define RDF_IRQ_ACTIVE_LOW (UINT32_C(1) << 3)
#define RDF_IRQ_SHARED (UINT32_C(1) << 4)
#define RDF_IRQ_START_MASKED (UINT32_C(1) << 5)

/* Interrupt domain capabilities. */
#define RDF_IRQ_DOMAIN_ROOT (UINT32_C(1) << 0)
#define RDF_IRQ_DOMAIN_MESSAGE (UINT32_C(1) << 1)
#define RDF_IRQ_DOMAIN_DEFAULT (UINT32_C(1) << 2)

/* Register window attributes. */
#define RDF_MMIO_DEVICE 0u
#define RDF_MMIO_WRITE_COMBINE (UINT32_C(1) << 0)
#define RDF_MMIO_CACHED (UINT32_C(1) << 1)
#define RDF_MMIO_READONLY (UINT32_C(1) << 2)

/* DMA transfer directions. */
#define RDF_DMA_FROM_DEVICE 1u
#define RDF_DMA_TO_DEVICE 2u
#define RDF_DMA_BIDIRECTIONAL 3u

/* DMA allocation attributes. */
#define RDF_DMA_ZERO (UINT32_C(1) << 0)
#define RDF_DMA_ADDRESS32 (UINT32_C(1) << 1)

/* Device node kinds. */
#define RDF_NODE_CHARACTER 1u
#define RDF_NODE_BLOCK 2u

/* Open flags visible to node operations. */
#define RDF_OPEN_READ (UINT32_C(1) << 0)
#define RDF_OPEN_WRITE (UINT32_C(1) << 1)
#define RDF_OPEN_APPEND (UINT32_C(1) << 5)
#define RDF_OPEN_NONBLOCK (UINT32_C(1) << 8)

/* Poll event bits. */
#define RDF_POLL_IN UINT16_C(0x0001)
#define RDF_POLL_PRI UINT16_C(0x0002)
#define RDF_POLL_OUT UINT16_C(0x0004)
#define RDF_POLL_ERR UINT16_C(0x0008)
#define RDF_POLL_HUP UINT16_C(0x0010)
#define RDF_POLL_NVAL UINT16_C(0x0020)
#define RDF_POLL_RDNORM UINT16_C(0x0040)
#define RDF_POLL_RDBAND UINT16_C(0x0080)
#define RDF_POLL_WRNORM UINT16_C(0x0100)
#define RDF_POLL_WRBAND UINT16_C(0x0200)

/* Console behaviour flags. */
#define RDF_CONSOLE_RESET_ON_LAST_CLOSE (UINT64_C(1) << 0)

/* Name of the bus carrying firmware-described devices. */
#define RDF_BUS_PLATFORM "platform"
/* Name of the class every terminal joins. */
#define RDF_CLASS_TTY "tty"

/* --- Module state ------------------------------------------------------- */

/*
 * The service table and this module's own handle.
 *
 * Both are hidden, so references from any translation unit of the module are
 * resolved at link time without a global offset table, which is what lets the
 * kernel loader accept an image with no symbol relocations.
 */
extern const struct rdf_api *rdf_api __attribute__((visibility("hidden")));
extern struct rdf_module *rdf_self __attribute__((visibility("hidden")));

/* --- Diagnostics and memory --------------------------------------------- */

/* Writes a preformatted message to the kernel log. */
static inline void rdf_log_raw(uint32_t level, const char *message)
{
    rdf_api->log(level, rdf_api->module_name(rdf_self), message);
}

/* Writes a formatted message to the kernel log. */
RDF_UNUSED RDF_PRINTF(2, 3) static void rdf_log(uint32_t level, const char *format, ...)
{
    char line[RDF_LOG_LINE];
    va_list args;
    va_start(args, format);
    rdf_vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    rdf_log_raw(level, line);
}

/* Convenience wrappers for the common log levels. */
#define RDF_ERROR(...) rdf_log(RDF_LOG_ERROR, __VA_ARGS__)
#define RDF_WARN(...) rdf_log(RDF_LOG_WARN, __VA_ARGS__)
#define RDF_INFO(...) rdf_log(RDF_LOG_INFO, __VA_ARGS__)
#define RDF_DEBUG(...) rdf_log(RDF_LOG_DEBUG, __VA_ARGS__)
#define RDF_TRACE(...) rdf_log(RDF_LOG_TRACE, __VA_ARGS__)

/* Allocates uninitialized kernel memory. */
static inline void *rdf_alloc(size_t size, size_t align)
{
    return rdf_api->alloc(size, align);
}

/* Allocates zeroed kernel memory. */
static inline void *rdf_zalloc(size_t size, size_t align)
{
    return rdf_api->alloc_zeroed(size, align);
}

/* Releases memory with the layout it was allocated with. */
static inline void rdf_free(void *pointer, size_t size, size_t align)
{
    rdf_api->free(pointer, size, align);
}

/* Allocates one zeroed, correctly aligned object of type `type`. */
#define RDF_NEW(type) ((type *)rdf_zalloc(sizeof(type), __alignof__(type)))
/* Releases an object allocated with RDF_NEW. */
#define RDF_DELETE(type, pointer) rdf_free((pointer), sizeof(type), __alignof__(type))

/* --- Modules ------------------------------------------------------------ */

/* Returns a module's name. */
static inline const char *rdf_module_name(const struct rdf_module *module)
{
    return rdf_api->module_name(module);
}

/* Finds a loaded module by name. */
static inline int32_t rdf_module_find(const char *name, const struct rdf_module **out)
{
    return rdf_api->module_find(name, out);
}

/* --- Device construction ------------------------------------------------ */

/* Starts describing a device. */
static inline int32_t rdf_device_new(const char *name, struct rdf_device_builder **out)
{
    return rdf_api->device_new(rdf_self, name, out);
}

/* Sets the parent of a device being described. */
static inline int32_t rdf_device_set_parent(struct rdf_device_builder *builder,
                                            const struct rdf_device *parent)
{
    return rdf_api->device_set_parent(builder, parent);
}

/* Sets the bus of a device being described. */
static inline int32_t rdf_device_set_bus(struct rdf_device_builder *builder,
                                         const struct rdf_bus *bus)
{
    return rdf_api->device_set_bus(builder, bus);
}

/* Records the firmware entry a device came from. */
static inline int32_t rdf_device_set_fwnode(struct rdf_device_builder *builder, uint32_t kind,
                                            uint64_t token, const char *provider,
                                            const char *path)
{
    return rdf_api->device_set_fwnode(builder, kind, token, provider, path);
}

/* Restricts the addressing limit a device can reach with DMA. */
static inline int32_t rdf_device_set_dma_mask(struct rdf_device_builder *builder, uint64_t mask)
{
    return rdf_api->device_set_dma_mask(builder, mask);
}

/* Adds a 32-bit integer property. */
static inline int32_t rdf_device_add_u32(struct rdf_device_builder *builder, const char *name,
                                         uint32_t value)
{
    return rdf_api->device_add_int(builder, name, value, RDF_WIDTH_32);
}

/* Adds a 64-bit integer property. */
static inline int32_t rdf_device_add_u64(struct rdf_device_builder *builder, const char *name,
                                         uint64_t value)
{
    return rdf_api->device_add_int(builder, name, value, RDF_WIDTH_64);
}

/* Adds a string property. */
static inline int32_t rdf_device_add_string(struct rdf_device_builder *builder, const char *name,
                                            const char *value)
{
    return rdf_api->device_add_string(builder, name, value);
}

/* Adds a string-list property such as `compatible`. */
static inline int32_t rdf_device_add_strings(struct rdf_device_builder *builder,
                                             const char *name, const char *const *values,
                                             size_t count)
{
    return rdf_api->device_add_strings(builder, name, values, count);
}

/* Adds a 32-bit integer-list property. */
static inline int32_t rdf_device_add_u32_list(struct rdf_device_builder *builder,
                                              const char *name, const uint64_t *values,
                                              size_t count)
{
    return rdf_api->device_add_cells(builder, name, values, count, RDF_WIDTH_32);
}

/* Adds a 64-bit integer-list property. */
static inline int32_t rdf_device_add_u64_list(struct rdf_device_builder *builder,
                                              const char *name, const uint64_t *values,
                                              size_t count)
{
    return rdf_api->device_add_cells(builder, name, values, count, RDF_WIDTH_64);
}

/* Adds an opaque byte property. */
static inline int32_t rdf_device_add_bytes(struct rdf_device_builder *builder, const char *name,
                                           const uint8_t *values, size_t count)
{
    return rdf_api->device_add_bytes(builder, name, values, count);
}

/* Adds a hardware resource. */
static inline int32_t rdf_device_add_resource(struct rdf_device_builder *builder, uint32_t kind,
                                              uint32_t flags, uint64_t start, uint64_t size,
                                              const char *name)
{
    return rdf_api->device_add_resource(builder, kind, flags, start, size, name);
}

/* Adds a firmware-described interrupt specifier. */
static inline int32_t rdf_device_add_irq(struct rdf_device_builder *builder,
                                         const struct rdf_irq_domain *domain,
                                         const uint32_t *cells, size_t count)
{
    return rdf_api->device_add_irq(builder, domain, cells, count);
}

/* Publishes a described device. The builder is consumed either way. */
static inline int32_t rdf_device_add(struct rdf_device_builder *builder, struct rdf_device **out)
{
    return rdf_api->device_add(builder, out);
}

/* Discards a description without publishing it. */
static inline void rdf_device_discard(struct rdf_device_builder *builder)
{
    rdf_api->device_discard(builder);
}

/* Removes a device and everything below it. */
static inline int32_t rdf_device_remove(const struct rdf_device *device)
{
    return rdf_api->device_remove(device);
}

/* --- Device inspection -------------------------------------------------- */

/* Returns the root of the device tree. */
static inline int32_t rdf_device_root(struct rdf_device **out)
{
    return rdf_api->device_root(out);
}

/* Returns a device's name. */
static inline const char *rdf_device_name(const struct rdf_device *device)
{
    return rdf_api->device_name(device);
}

/* Returns a device's parent, or NULL at the root. */
static inline struct rdf_device *rdf_device_parent(const struct rdf_device *device)
{
    return rdf_api->device_parent(device);
}

/* Returns the number of children a device has. */
static inline size_t rdf_device_child_count(const struct rdf_device *device)
{
    return rdf_api->device_child_count(device);
}

/* Returns one child of a device. */
static inline struct rdf_device *rdf_device_child(const struct rdf_device *device, size_t index)
{
    return rdf_api->device_child(device, index);
}

/* Returns the driver-private instance pointer. */
static inline void *rdf_device_data(const struct rdf_device *device)
{
    return rdf_api->device_data(device);
}

/* Stores the driver-private instance pointer. */
static inline void rdf_device_set_data(const struct rdf_device *device, void *value)
{
    rdf_api->device_set_data(device, value);
}

/* Reads an integer property. */
static inline int32_t rdf_device_u64(const struct rdf_device *device, const char *name,
                                     uint64_t *out)
{
    return rdf_api->device_int(device, name, out);
}

/* Reads one cell of an integer-list property. */
static inline int32_t rdf_device_cell(const struct rdf_device *device, const char *name,
                                      size_t index, uint64_t *out)
{
    return rdf_api->device_cell(device, name, index, out);
}

/* Reads one string of a string property into a NUL-terminated buffer. */
static inline int32_t rdf_device_string(const struct rdf_device *device, const char *name,
                                        size_t index, char *buffer, size_t capacity)
{
    return rdf_api->device_string(device, name, index, (uint8_t *)buffer, capacity, NULL);
}

/* Reads an opaque byte property. */
static inline int32_t rdf_device_bytes(const struct rdf_device *device, const char *name,
                                       uint8_t *buffer, size_t capacity, size_t *written)
{
    return rdf_api->device_bytes(device, name, buffer, capacity, written);
}

/* Returns the number of elements in a property, or zero if it is absent. */
static inline size_t rdf_device_property_len(const struct rdf_device *device, const char *name)
{
    return rdf_api->device_property_len(device, name);
}

/* Reads the index-th resource of a kind. */
static inline int32_t rdf_device_resource(const struct rdf_device *device, uint32_t kind,
                                          size_t index, uint64_t *start, uint64_t *size,
                                          uint32_t *flags)
{
    return rdf_api->device_resource(device, kind, index, start, size, flags);
}

/* Returns the firmware token recorded for a device. */
static inline int32_t rdf_device_fwnode(const struct rdf_device *device, uint32_t *kind,
                                        uint64_t *token)
{
    return rdf_api->device_fwnode(device, kind, token);
}

/* --- Buses, drivers, and classes ---------------------------------------- */

/* Registers a bus type. */
static inline int32_t rdf_bus_register(const char *name, const struct rdf_bus_def *definition,
                                       const struct rdf_bus **out)
{
    return rdf_api->bus_register(rdf_self, name, definition, out);
}

/* Unregisters a bus type. */
static inline int32_t rdf_bus_unregister(const struct rdf_bus *bus)
{
    return rdf_api->bus_unregister(bus);
}

/* Finds a registered bus by name. */
static inline int32_t rdf_bus_find(const char *name, const struct rdf_bus **out)
{
    return rdf_api->bus_find(name, out);
}

/* Installs a DMA translation for devices on a bus. */
static inline int32_t rdf_bus_set_dma_ops(const struct rdf_bus *bus,
                                          const struct rdf_dma_ops *ops)
{
    return rdf_api->bus_set_dma_ops(bus, ops);
}

/* Registers a driver and offers it every unbound device. */
static inline int32_t rdf_driver_register(const struct rdf_driver_def *definition,
                                          const struct rdf_driver **out)
{
    return rdf_api->driver_register(rdf_self, definition, out);
}

/* Unregisters a driver, unbinding every device it claimed. */
static inline int32_t rdf_driver_unregister(const struct rdf_driver *driver)
{
    return rdf_api->driver_unregister(driver);
}

/* Registers a device class. */
static inline int32_t rdf_class_register(const char *name,
                                         const struct rdf_class_def *definition,
                                         const struct rdf_class **out)
{
    return rdf_api->class_register(rdf_self, name, definition, out);
}

/* Unregisters a device class. */
static inline int32_t rdf_class_unregister(const struct rdf_class *class_object)
{
    return rdf_api->class_unregister(class_object);
}

/* Finds a registered class by name. */
static inline int32_t rdf_class_find(const char *name, const struct rdf_class **out)
{
    return rdf_api->class_find(name, out);
}

/* Adds a device to a class. */
static inline int32_t rdf_class_add(const struct rdf_class *class_object,
                                    const struct rdf_device *device, const char *name,
                                    const void *ops, size_t ops_size, void *context,
                                    const struct rdf_class_device **out)
{
    return rdf_api->class_add(rdf_self, class_object, device, name, ops, ops_size, context, out);
}

/* Removes a class membership. */
static inline void rdf_class_remove(const struct rdf_class_device *member)
{
    rdf_api->class_remove(member);
}

/* Returns a membership's operation table and context. */
static inline int32_t rdf_class_member_ops(const struct rdf_class_device *member,
                                           const void **ops, void **context)
{
    return rdf_api->class_member_ops(member, ops, context);
}

/* Returns a membership's name. */
static inline const char *rdf_class_member_name(const struct rdf_class_device *member)
{
    return rdf_api->class_member_name(member);
}

/* Returns a membership's device. */
static inline struct rdf_device *rdf_class_member_device(const struct rdf_class_device *member)
{
    return rdf_api->class_member_device(member);
}

/* Returns the class-private value stored on a membership. */
static inline uintptr_t rdf_class_member_data(const struct rdf_class_device *member)
{
    return rdf_api->class_member_data(member);
}

/* Stores a class-private value on a membership. */
static inline void rdf_class_member_set_data(const struct rdf_class_device *member,
                                             uintptr_t value)
{
    rdf_api->class_member_set_data(member, value);
}

/* Returns the number of members in a class. */
static inline size_t rdf_class_member_count(const struct rdf_class *class_object)
{
    return rdf_api->class_member_count(class_object);
}

/* Returns one member of a class. */
static inline struct rdf_class_device *rdf_class_member(const struct rdf_class *class_object,
                                                        size_t index)
{
    return rdf_api->class_member(class_object, index);
}

/* --- Interfaces --------------------------------------------------------- */

/* Publishes a system-wide interface. */
static inline int32_t rdf_iface_publish(const char *name, uint32_t version, uint32_t flags,
                                        const void *ops, size_t ops_size, void *context,
                                        const struct rdf_iface **out)
{
    return rdf_api->iface_publish(rdf_self, NULL, name, version, flags, ops, ops_size, context,
                                  out);
}

/* Publishes an interface describing one device's capability. */
static inline int32_t rdf_iface_publish_device(const struct rdf_device *device, const char *name,
                                               uint32_t version, uint32_t flags, const void *ops,
                                               size_t ops_size, void *context,
                                               const struct rdf_iface **out)
{
    return rdf_api->iface_publish(rdf_self, device, name, version, flags, ops, ops_size, context,
                                  out);
}

/* Withdraws a published interface. */
static inline int32_t rdf_iface_withdraw(const struct rdf_iface *iface)
{
    return rdf_api->iface_withdraw(iface);
}

/*
 * Binds to a system-wide interface.
 *
 * Returns RDF_EDEFER when no compatible provider is loaded yet. A probe
 * callback should return that status unchanged so the framework retries once
 * the provider appears.
 */
static inline int32_t rdf_iface_bind(const char *name, uint32_t min_version,
                                     struct rdf_iface_binding **binding, void **ops,
                                     void **context)
{
    return rdf_api->iface_bind(rdf_self, NULL, RDF_SCOPE_GLOBAL, name, min_version, binding, ops,
                               context);
}

/* Binds to an interface published by one device. */
static inline int32_t rdf_iface_bind_device(const struct rdf_device *device, const char *name,
                                            uint32_t min_version,
                                            struct rdf_iface_binding **binding, void **ops,
                                            void **context)
{
    return rdf_api->iface_bind(rdf_self, device, RDF_SCOPE_DEVICE, name, min_version, binding,
                               ops, context);
}

/* Binds to an interface published by a device or any of its ancestors. */
static inline int32_t rdf_iface_bind_ancestor(const struct rdf_device *device, const char *name,
                                              uint32_t min_version,
                                              struct rdf_iface_binding **binding, void **ops,
                                              void **context)
{
    return rdf_api->iface_bind(rdf_self, device, RDF_SCOPE_ANCESTOR, name, min_version, binding,
                               ops, context);
}

/* Releases a binding. */
static inline void rdf_iface_unbind(struct rdf_iface_binding *binding)
{
    rdf_api->iface_unbind(binding);
}

/* Returns whether a compatible global provider exists. */
static inline int32_t rdf_iface_available(const char *name, uint32_t min_version)
{
    return rdf_api->iface_available(name, min_version);
}

/* Returns the number of global providers of a name. */
static inline size_t rdf_iface_count(const char *name, uint32_t min_version)
{
    return rdf_api->iface_count(name, min_version);
}

/* Returns one global provider of a name. */
static inline struct rdf_iface *rdf_iface_provider(const char *name, uint32_t min_version,
                                                   size_t index)
{
    return rdf_api->iface_provider(name, min_version, index);
}

/* Requests a rescan of devices waiting for a prerequisite. */
static inline void rdf_probe_retrigger(void)
{
    rdf_api->probe_retrigger();
}

/* --- Interrupts --------------------------------------------------------- */

/* Registers an interrupt controller domain. */
static inline int32_t rdf_irq_domain_register(const char *name, uint32_t flags,
                                              uint32_t hwirq_count,
                                              const struct rdf_irq_domain_def *definition,
                                              const struct rdf_irq_domain **out)
{
    return rdf_api->irq_domain_register(rdf_self, name, flags, hwirq_count, definition, out);
}

/* Unregisters an interrupt controller domain. */
static inline int32_t rdf_irq_domain_unregister(const struct rdf_irq_domain *domain)
{
    return rdf_api->irq_domain_unregister(domain);
}

/* Maps a hardware interrupt into the virtual interrupt space. */
static inline int32_t rdf_irq_map(const struct rdf_irq_domain *domain, uint64_t hwirq,
                                  uint32_t flags, uint32_t *out_virq)
{
    return rdf_api->irq_map(domain, hwirq, flags, out_virq);
}

/* Resolves a device's index-th firmware interrupt. */
static inline int32_t rdf_irq_of_device(const struct rdf_device *device, size_t index,
                                        uint32_t *out_virq)
{
    return rdf_api->irq_of_device(device, index, out_virq);
}

/* Attaches a handler to a virtual interrupt. */
static inline int32_t rdf_irq_request(const struct rdf_device *device, uint32_t virq,
                                      const char *name, uint32_t flags,
                                      rdf_irq_handler_fn handler, void *context,
                                      struct rdf_irq **out)
{
    return rdf_api->irq_request(rdf_self, device, virq, name, flags, handler, NULL, context, out);
}

/*
 * Attaches a handler with a threaded second half.
 *
 * The top half runs in interrupt context and must acknowledge or mask the
 * device before returning RDF_IRQ_WAKE_THREAD. The thread callback may block
 * and should drain all pending work before returning.
 *
 * Masking with rdf_irq_mask() from inside the top half is supported: the
 * framework does not hold the descriptor lock across a handler.
 */
static inline int32_t rdf_irq_request_threaded(const struct rdf_device *device, uint32_t virq,
                                               const char *name, uint32_t flags,
                                               rdf_irq_handler_fn handler,
                                               rdf_irq_thread_fn thread, void *context,
                                               struct rdf_irq **out)
{
    return rdf_api->irq_request(rdf_self, device, virq, name, flags, handler, thread, context,
                                out);
}

/* Detaches a handler and waits for its thread to stop. */
static inline int32_t rdf_irq_release(struct rdf_irq *irq)
{
    return rdf_api->irq_release(irq);
}

/* Masks a virtual interrupt. */
static inline int32_t rdf_irq_mask(uint32_t virq)
{
    return rdf_api->irq_mask(virq);
}

/* Unmasks a virtual interrupt. */
static inline int32_t rdf_irq_unmask(uint32_t virq)
{
    return rdf_api->irq_unmask(virq);
}

/* Redirects a virtual interrupt to a CPU. */
static inline int32_t rdf_irq_set_affinity(uint32_t virq, uint32_t cpu)
{
    return rdf_api->irq_set_affinity(virq, cpu);
}

/* Reserves an architecture vector for a controller. */
static inline int32_t rdf_irq_alloc_vector(uint32_t virq, uint32_t *out_vector)
{
    return rdf_api->irq_alloc_vector(virq, out_vector);
}

/* Releases an architecture vector. */
static inline int32_t rdf_irq_free_vector(uint32_t vector)
{
    return rdf_api->irq_free_vector(vector);
}

/* Returns the address and payload a device must write to raise an interrupt. */
static inline int32_t rdf_irq_compose_message(const struct rdf_irq_domain *domain, uint64_t hwirq,
                                              uint64_t *out_address, uint32_t *out_data)
{
    return rdf_api->irq_compose_message(domain, hwirq, out_address, out_data);
}

/* --- Register windows and DMA ------------------------------------------- */

/* Maps a physical range for register access. */
static inline int32_t rdf_mmio_map(uint64_t physical, size_t length, uint32_t flags,
                                   struct rdf_mmio *out)
{
    return rdf_api->mmio_map(rdf_self, physical, length, flags, out);
}

/* Unmaps a register window. */
static inline int32_t rdf_mmio_unmap(struct rdf_mmio *window)
{
    return rdf_api->mmio_unmap(window);
}

/* Returns the direct-map address of boot-mapped physical memory. */
static inline int32_t rdf_mmio_direct(uint64_t physical, void **out)
{
    return rdf_api->mmio_direct(physical, out);
}

/*
 * Spins until a masked register field reaches `want`, or the timeout expires.
 *
 * Returns RDF_OK on success and RDF_ETIMEDOUT otherwise. Controller resets and
 * doorbell handshakes all have this shape.
 */
static inline int32_t rdf_mmio_wait32(const struct rdf_mmio *window, size_t offset,
                                      uint32_t mask, uint32_t want, uint64_t timeout_ns)
{
    uint64_t deadline = rdf_api->time_monotonic() + timeout_ns;
    for (;;) {
        if ((rdf_read32(window, offset) & mask) == want)
            return RDF_OK;
        if (rdf_api->time_monotonic() >= deadline)
            return RDF_ETIMEDOUT;
        rdf_cpu_relax();
    }
}

/* Allocates a coherent buffer a device can address. */
static inline int32_t rdf_dma_alloc(const struct rdf_device *device, size_t size, size_t align,
                                    uint32_t flags, struct rdf_dma *out)
{
    return rdf_api->dma_alloc(rdf_self, device, size, align, flags, out);
}

/* Releases a coherent buffer. */
static inline int32_t rdf_dma_free(struct rdf_dma *buffer)
{
    return rdf_api->dma_free(buffer);
}

/* Resolves the device address of an existing kernel buffer. */
static inline int32_t rdf_dma_map(const struct rdf_device *device, void *address, size_t size,
                                  uint32_t direction, uint64_t *out)
{
    return rdf_api->dma_map(device, address, size, direction, out);
}

/* Releases a mapping created by rdf_dma_map(). */
static inline void rdf_dma_unmap(const struct rdf_device *device, uint64_t address, size_t size,
                                 uint32_t direction)
{
    rdf_api->dma_unmap(device, address, size, direction);
}

/* Orders CPU and device views of a buffer. */
static inline void rdf_dma_sync(void *address, size_t size, uint32_t direction)
{
    rdf_api->dma_sync(address, size, direction);
}

/* --- Deferred work, timers, and events ---------------------------------- */

/* Creates a work queue served by its own kernel thread. */
static inline int32_t rdf_work_create(const char *name, struct rdf_work **out)
{
    return rdf_api->work_create(rdf_self, name, out);
}

/* Appends a callback to a work queue. */
static inline int32_t rdf_work_queue(struct rdf_work *queue, rdf_work_fn callback, void *context,
                                     uint64_t argument)
{
    return rdf_api->work_queue(queue, callback, context, argument);
}

/* Waits until a work queue has no pending or running callbacks. */
static inline int32_t rdf_work_flush(struct rdf_work *queue)
{
    return rdf_api->work_flush(queue);
}

/* Destroys a work queue after its worker thread exits. */
static inline int32_t rdf_work_destroy(struct rdf_work *queue)
{
    return rdf_api->work_destroy(queue);
}

/* Creates a timer bound to its own kernel thread. */
static inline int32_t rdf_timer_create(rdf_work_fn callback, void *context, uint64_t argument,
                                       struct rdf_timer **out)
{
    return rdf_api->timer_create(rdf_self, callback, context, argument, out);
}

/* Arms a timer, repeating when period_ns is non-zero. */
static inline int32_t rdf_timer_arm(struct rdf_timer *timer, uint64_t delay_ns,
                                    uint64_t period_ns)
{
    return rdf_api->timer_arm(timer, delay_ns, period_ns);
}

/* Disarms a timer without destroying it. */
static inline int32_t rdf_timer_cancel(struct rdf_timer *timer)
{
    return rdf_api->timer_cancel(timer);
}

/* Destroys a timer after its thread exits. */
static inline int32_t rdf_timer_destroy(struct rdf_timer *timer)
{
    return rdf_api->timer_destroy(timer);
}

/* Creates a level-triggered event. */
static inline int32_t rdf_event_create(rdf_event_t *out)
{
    return rdf_api->event_create(rdf_self, out);
}

/* Destroys an event. */
static inline int32_t rdf_event_destroy(rdf_event_t event)
{
    return rdf_api->event_destroy(event);
}

/* Waits until an event is signalled. */
static inline int32_t rdf_event_wait(rdf_event_t event)
{
    return rdf_api->event_wait(event);
}

/* Signals an event, waking every waiter. */
static inline int32_t rdf_event_signal(rdf_event_t event)
{
    return rdf_api->event_signal(event);
}

/* Clears an event's signal. */
static inline int32_t rdf_event_reset(rdf_event_t event)
{
    return rdf_api->event_reset(event);
}

/* --- Time, entropy, and topology ---------------------------------------- */

/* Returns nanoseconds since boot. */
static inline uint64_t rdf_time_ns(void)
{
    return rdf_api->time_monotonic();
}

/* Busy-waits for a duration. Safe in interrupt context. */
static inline void rdf_delay_ns(uint64_t nanoseconds)
{
    rdf_api->time_delay(nanoseconds);
}

/* Sleeps the calling thread. Not valid in interrupt context. */
static inline void rdf_sleep_ns(uint64_t nanoseconds)
{
    rdf_api->time_sleep(nanoseconds);
}

/* Fills a buffer with random bytes. */
static inline void rdf_random_fill(uint8_t *buffer, size_t length)
{
    rdf_api->random_fill(buffer, length);
}

/* Mixes entropy into the random pool. */
static inline void rdf_random_mix(const uint8_t *buffer, size_t length)
{
    rdf_api->random_mix(buffer, length);
}

/* Returns the number of CPUs. */
static inline uint32_t rdf_cpu_count(void)
{
    return rdf_api->cpu_count();
}

/* Returns the calling CPU's identifier. */
static inline uint32_t rdf_cpu_current(void)
{
    return rdf_api->cpu_current();
}

/* Returns a CPU's firmware identifier. */
static inline int32_t rdf_cpu_platform_id(uint32_t cpu, uint64_t *out)
{
    return rdf_api->cpu_platform_id(cpu, out);
}

/* Returns whether the caller is in interrupt context. */
static inline int32_t rdf_in_interrupt(void)
{
    return rdf_api->in_interrupt();
}

/* --- Firmware ----------------------------------------------------------- */

/* Copies the ACPI root pointer, when the platform provides one. */
static inline int32_t rdf_firmware_acpi(uint8_t *buffer, size_t capacity, size_t *written)
{
    return rdf_api->firmware_acpi(buffer, capacity, written);
}

/* Copies the device tree, when the platform provides one. */
static inline int32_t rdf_firmware_devicetree(uint8_t *buffer, size_t capacity, size_t *written)
{
    return rdf_api->firmware_devicetree(buffer, capacity, written);
}

/* --- Device nodes and terminals ----------------------------------------- */

/* Returns the device filesystem root. */
static inline int32_t rdf_devfs_root(rdf_devnode_t *out)
{
    return rdf_api->devfs_root(out);
}

/* Creates a directory in the device filesystem. */
static inline int32_t rdf_devfs_mkdir(rdf_devnode_t parent, const char *name, uint16_t mode,
                                      rdf_devnode_t *out)
{
    return rdf_api->devfs_mkdir(rdf_self, parent, name, mode, out);
}

/* Creates a device node backed by a driver operation table. */
static inline int32_t rdf_devfs_create(const struct rdf_device *device, rdf_devnode_t parent,
                                       const char *name, uint32_t kind, uint16_t mode,
                                       const struct rdf_node_ops *ops, rdf_devnode_t *out)
{
    return rdf_api->devfs_create(rdf_self, device, parent, name, kind, mode, ops, out);
}

/* Removes a device-filesystem node. */
static inline int32_t rdf_devfs_remove(rdf_devnode_t node)
{
    return rdf_api->devfs_remove(rdf_self, node);
}

/* Resolves an absolute device-filesystem path. */
static inline int32_t rdf_devfs_lookup(const char *path, rdf_devnode_t *out)
{
    return rdf_api->devfs_lookup(path, out);
}

/* Registers a terminal backed by a console operation table. */
static inline int32_t rdf_tty_register(const struct rdf_device *device, rdf_devnode_t parent,
                                       const char *name, uint16_t mode, uint32_t baud,
                                       const struct rdf_console_ops *ops, struct rdf_tty **out)
{
    return rdf_api->tty_register(rdf_self, device, parent, name, mode, baud, ops, out);
}

/* Removes a terminal. */
static inline int32_t rdf_tty_unregister(struct rdf_tty *tty)
{
    return rdf_api->tty_unregister(tty);
}

/* --- Kernel log --------------------------------------------------------- */

/* Returns the severity currently recorded into the kernel log. */
static inline uint32_t rdf_klog_level(void)
{
    return rdf_api->klog_level();
}

/* Changes the recorded severity and returns the previous one. */
static inline uint32_t rdf_klog_set_level(uint32_t level)
{
    return rdf_api->klog_set_level(level);
}

/* Returns the severity currently mirrored to the console. */
static inline uint32_t rdf_klog_console_level(void)
{
    return rdf_api->klog_console_level();
}

/* Changes the mirrored severity and returns the previous one. */
static inline uint32_t rdf_klog_set_console_level(uint32_t level)
{
    return rdf_api->klog_set_console_level(level);
}

/* --- Match table helpers ------------------------------------------------ */

/* Matches a device-tree compatible string. */
#define RDF_MATCH_COMPATIBLE(text)                                                             \
    {                                                                                          \
        .kind = RDF_MATCH_KIND_COMPATIBLE, .key = (text)                                       \
    }

/* Matches a compatible string and carries a per-variant cookie. */
#define RDF_MATCH_COMPATIBLE_DATA(text, cookie)                                                \
    {                                                                                          \
        .kind = RDF_MATCH_KIND_COMPATIBLE, .key = (text), .data = (uintptr_t)(cookie)          \
    }

/* Matches a device node name. */
#define RDF_MATCH_NAME(text)                                                                   \
    {                                                                                          \
        .kind = RDF_MATCH_KIND_NAME, .key = (text)                                             \
    }

/* Matches an ACPI hardware or compatible identifier. */
#define RDF_MATCH_ACPI(text)                                                                   \
    {                                                                                          \
        .kind = RDF_MATCH_KIND_ACPI, .key = (text)                                             \
    }

/* Matches a masked numeric identifier held in a named property. */
#define RDF_MATCH_ID(property, value, bits)                                                    \
    {                                                                                          \
        .kind = RDF_MATCH_KIND_ID, .key = (property), .id0 = (value), .mask0 = (bits)          \
    }

/* Matches every device on the driver's bus. */
#define RDF_MATCH_ANY()                                                                        \
    {                                                                                          \
        .kind = RDF_MATCH_KIND_ANY                                                             \
    }

/* --- Module definition -------------------------------------------------- */

/*
 * Defines a module's entry point and its two hidden globals.
 *
 * Place this once, in exactly one translation unit of the module. The image's
 * ELF entry must be `rdf_module_entry`, which the supplied link flags arrange.
 */
#define RDF_MODULE(module_name, module_description, module_init, module_exit)                  \
    const struct rdf_api *rdf_api __attribute__((visibility("hidden")));                       \
    struct rdf_module *rdf_self __attribute__((visibility("hidden")));                         \
    static int32_t rdf_module_start(struct rdf_module *self)                                   \
    {                                                                                          \
        rdf_self = self;                                                                       \
        return (module_init)(self);                                                            \
    }                                                                                          \
    static void rdf_module_stop(struct rdf_module *self)                                       \
    {                                                                                          \
        void (*stop)(struct rdf_module *) = (module_exit);                                     \
        if (stop != NULL)                                                                      \
            stop(self);                                                                        \
    }                                                                                          \
    RDF_EXPORT const struct rdf_module_def *rdf_module_entry(const struct rdf_api *api)         \
    {                                                                                          \
        static const struct rdf_module_def definition = {                                      \
            .size = sizeof(struct rdf_module_def),                                             \
            .abi_major = RDF_ABI_MAJOR,                                                        \
            .abi_minor = RDF_ABI_MINOR,                                                        \
            .flags = 0,                                                                        \
            .name = (module_name),                                                             \
            .description = (module_description),                                               \
            .init = rdf_module_start,                                                          \
            .exit = rdf_module_stop,                                                           \
        };                                                                                     \
        rdf_api = api;                                                                         \
        return &definition;                                                                    \
    }

#ifdef __cplusplus
}
#endif

#endif
