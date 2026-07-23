#include <devkit/devkit.h>

#define DK_SERVICE_CORE UINT64_C(0x524f414e434f5245)
#define DK_SERVICE_REGISTRY UINT64_C(0x524f414e52454749)
#define DK_SERVICE_MEMORY UINT64_C(0x524f414e4d454d4f)
#define DK_SERVICE_INTERRUPT UINT64_C(0x524f414e49525153)
#define DK_SERVICE_BINDING UINT64_C(0x524f414e42494e44)
#define DK_SERVICE_REVISION UINT32_C(1)

struct dk_core_api {
    struct dk_abi_service_header header;
    uint8_t *(*allocate)(size_t size, size_t align);
    uint8_t *(*allocate_zeroed)(size_t size, size_t align);
    int32_t (*deallocate)(uint8_t *data, size_t size, size_t align);
    int32_t (*log)(uint32_t level, struct dk_slice message);
    int32_t (*event_create)(uintptr_t *out_event);
    int32_t (*event_destroy)(uintptr_t event);
    int32_t (*event_wait)(uintptr_t event);
    int32_t (*event_reset)(uintptr_t event);
    int64_t (*event_signal)(uintptr_t event);
    void (*sleep_ns)(uint64_t nanoseconds);
    void (*random_fill)(uint8_t *output, size_t len);
    void (*random_mix)(const uint8_t *input, size_t len);
    uint64_t (*kmsg_start)(void);
    uint64_t (*kmsg_end)(void);
    int64_t (*kmsg_read)(
        uint64_t offset,
        uint8_t *output,
        size_t len,
        uint8_t nonblocking);
    int32_t (*kmsg_append)(const uint8_t *input, size_t len);
    void (*kmsg_disable_console_output)(void);
    void (*kmsg_enable_console_output)(void);
};

struct dk_registry_api {
    struct dk_abi_service_header header;
    int32_t (*root_bus)(uint64_t *out_bus);
    int32_t (*register_bus)(
        uint64_t driver,
        uint64_t parent,
        struct dk_slice name,
        uint64_t *out_bus);
    int32_t (*register_device)(
        uint64_t driver,
        uint64_t parent,
        struct dk_slice name,
        uint64_t *out_device);
    int32_t (*remove_node)(uint64_t driver, uint64_t node);
    int32_t (*set_node_property)(
        uint64_t driver,
        uint64_t node,
        struct dk_resource_key key,
        struct dk_slice value);
    int32_t (*read_node_property)(
        uint64_t node,
        struct dk_resource_key key,
        uint8_t *output,
        size_t output_len,
        size_t *written);
    int32_t (*publish_data_resource)(
        uint64_t driver,
        uint64_t bus,
        struct dk_resource_key key,
        uint64_t flags,
        struct dk_slice data,
        uint64_t *out_resource);
    int32_t (*publish_memory_resource)(
        uint64_t driver,
        uint64_t node,
        struct dk_resource_key key,
        uint64_t flags,
        uint64_t physical,
        uint64_t length,
        uint64_t *out_resource);
    int32_t (*publish_protocol_resource)(
        uint64_t driver,
        uint64_t bus,
        struct dk_resource_key key,
        uint64_t flags,
        uint32_t revision,
        uintptr_t context,
        const uint8_t *operations,
        size_t operations_size,
        uint64_t *out_resource);
    int32_t (*remove_resource)(
        uint64_t driver,
        uint64_t node,
        struct dk_resource_key key);
    int32_t (*acquire_data_resource)(
        uint64_t driver,
        uint64_t node,
        struct dk_resource_key key,
        struct dk_data_resource *out_resource);
    int32_t (*acquire_memory_resource)(
        uint64_t driver,
        uint64_t node,
        struct dk_resource_key key,
        struct dk_memory_resource *out_resource);
    int32_t (*acquire_protocol_resource)(
        uint64_t driver,
        uint64_t node,
        struct dk_resource_key key,
        uint32_t minimum_revision,
        struct dk_protocol_resource *out_resource);
    int32_t (*release_resource)(uint64_t driver, uint64_t lease);
};

struct dk_memory_api {
    struct dk_abi_service_header header;
    int32_t (*map_mmio)(
        uint64_t driver,
        uint64_t physical,
        size_t size,
        uintptr_t *out_address);
    int32_t (*map_mmio_resource)(
        uint64_t driver,
        uint64_t lease,
        uint64_t offset,
        size_t size,
        struct dk_mmio_mapping *out_mapping);
    int32_t (*release_mmio_mapping)(
        uint64_t driver,
        uint64_t mapping);
    int32_t (*firmware_physical_to_virtual)(
        uint64_t physical,
        uintptr_t *out_address);
};

struct dk_interrupt_api {
    struct dk_abi_service_header header;
    int32_t (*register_interrupt_controller)(
        uint64_t driver,
        uint64_t bus,
        const struct dk_interrupt_controller *controller,
        uint64_t *out_controller);
    int32_t (*unregister_interrupt_controller)(
        uint64_t driver,
        uint64_t controller);
    int32_t (*request_interrupt)(
        uint64_t driver,
        uint64_t node,
        struct dk_slice specifier,
        uint64_t flags,
        uint32_t target_cpu,
        dk_interrupt_handler_fn handler,
        uintptr_t context,
        uint64_t *out_interrupt);
    int32_t (*request_threaded_interrupt)(
        uint64_t driver,
        uint64_t node,
        struct dk_slice specifier,
        uint64_t flags,
        uint32_t target_cpu,
        dk_interrupt_handler_fn handler,
        dk_interrupt_thread_fn thread_handler,
        uintptr_t context,
        uint64_t *out_interrupt);
    int32_t (*release_interrupt)(uint64_t driver, uint64_t interrupt);
    int32_t (*mask_interrupt)(uint64_t driver, uint64_t interrupt);
    int32_t (*unmask_interrupt)(uint64_t driver, uint64_t interrupt);
    int32_t (*set_interrupt_affinity)(
        uint64_t driver,
        uint64_t interrupt,
        uint32_t target_cpu);
};

struct dk_binding_api {
    struct dk_abi_service_header header;
    int32_t (*register_class)(
        uint64_t driver,
        const struct dk_driver_class *class,
        uint64_t *out_class);
    int32_t (*unregister_class)(
        uint64_t driver,
        uint64_t class);
};

struct dk_device_frontend_ops {
    uint32_t size;
    uint32_t revision;
    int32_t (*root)(uintptr_t context, uint64_t *out_node);
    int32_t (*create_dir)(
        uintptr_t context,
        uint64_t driver,
        uint64_t parent,
        struct dk_slice name,
        uint16_t mode,
        uint64_t *out_node);
    int32_t (*create_device)(
        uintptr_t context,
        uint64_t driver,
        uint64_t parent,
        struct dk_slice name,
        uint32_t kind,
        uint16_t mode,
        uint64_t device,
        const struct dk_device_ops *operations,
        uint64_t *out_node);
    int32_t (*remove_node)(
        uintptr_t context,
        uint64_t driver,
        uint64_t node);
};

struct dk_console_service_ops {
    uint32_t size;
    uint32_t revision;
    int32_t (*bus)(uintptr_t context, uint64_t *out_bus);
    int32_t (*create_tty)(
        uintptr_t context,
        uint64_t driver,
        uint64_t device,
        uint64_t parent,
        struct dk_slice name,
        uint16_t mode,
        struct dk_slice path,
        uint32_t baud,
        const struct dk_console_ops *operations,
        uint64_t *out_node);
};

static dk_driver_t current_driver;
static const struct dk_core_api *core_api;
static const struct dk_registry_api *registry_api;
static const struct dk_memory_api *memory_api;
static const struct dk_interrupt_api *interrupt_api;
static const struct dk_binding_api *binding_api;
static uint8_t initialized;

static int32_t query_service(
    const struct dk_abi_bootstrap *bootstrap,
    uint64_t identifier,
    size_t minimum_size,
    const struct dk_abi_service_header **out_service)
{
    const struct dk_abi_service_header *service = NULL;
    int32_t status = bootstrap->get_service(
        identifier,
        DK_SERVICE_REVISION,
        &service);
    if (status != DK_OK)
        return status;
    if (service == NULL ||
        service->revision < DK_SERVICE_REVISION ||
        service->size < minimum_size)
        return DK_ENOTSUP;
    *out_service = service;
    return DK_OK;
}

static int abi_supported(const struct dk_abi_bootstrap *bootstrap)
{
    if (bootstrap->abi_major != DK_ABI_MAJOR)
        return 0;
#if DK_ABI_MINOR > 0
    if (bootstrap->abi_minor < DK_ABI_MINOR)
        return 0;
#endif
    return 1;
}

int32_t dk_runtime_start(
    const struct dk_abi_bootstrap *bootstrap,
    dk_driver_t driver,
    const struct dk_driver_definition *definition)
{
    if (bootstrap == NULL ||
        definition == NULL ||
        (definition->start == NULL && definition->class_count == 0) ||
        (definition->class_count != 0 && definition->classes == NULL) ||
        bootstrap->size < sizeof(*bootstrap) ||
        !abi_supported(bootstrap) ||
        bootstrap->get_service == NULL ||
        initialized != 0)
        return DK_EINVAL;

    const struct dk_abi_service_header *service = NULL;
    int32_t status = query_service(
        bootstrap,
        DK_SERVICE_CORE,
        sizeof(*core_api),
        &service);
    if (status == DK_OK) {
        core_api = (const struct dk_core_api *)service;
        status = query_service(
            bootstrap,
            DK_SERVICE_REGISTRY,
            sizeof(*registry_api),
            &service);
    }
    if (status == DK_OK) {
        registry_api = (const struct dk_registry_api *)service;
        status = query_service(
            bootstrap,
            DK_SERVICE_MEMORY,
            sizeof(*memory_api),
            &service);
    }
    if (status == DK_OK) {
        memory_api = (const struct dk_memory_api *)service;
        status = query_service(
            bootstrap,
            DK_SERVICE_INTERRUPT,
            sizeof(*interrupt_api),
            &service);
    }
    if (status == DK_OK)
        interrupt_api = (const struct dk_interrupt_api *)service;
    if (status == DK_OK) {
        status = query_service(
            bootstrap,
            DK_SERVICE_BINDING,
            sizeof(*binding_api),
            &service);
    }
    if (status == DK_OK)
        binding_api = (const struct dk_binding_api *)service;
    if (status != DK_OK)
        return status;

    current_driver = driver;
    initialized = 1;
    status = definition->start != NULL
        ? definition->start(driver, definition->context)
        : DK_OK;
    for (size_t index = 0;
         status == DK_OK && index < definition->class_count;
         ++index) {
        dk_driver_class_t class_id = 0;
        status = dk_driver_class_register(
            &definition->classes[index],
            &class_id);
    }
    if (status != DK_OK) {
        if (definition->stop != NULL)
            definition->stop(driver, definition->context);
        initialized = 0;
        current_driver = 0;
    }
    return status;
}

void dk_runtime_stop(
    dk_driver_t driver,
    const struct dk_driver_definition *definition)
{
    if (initialized == 0 || driver != current_driver || definition == NULL)
        return;
    if (definition->stop != NULL)
        definition->stop(driver, definition->context);
    initialized = 0;
    current_driver = 0;
}

dk_driver_t dk_driver_current(void)
{
    return current_driver;
}

int32_t dk_driver_class_register(
    const struct dk_driver_class *definition,
    dk_driver_class_t *out_class)
{
    if (initialized == 0)
        return DK_EIO;
    return binding_api->register_class(
        current_driver,
        definition,
        out_class);
}

int32_t dk_driver_class_unregister(dk_driver_class_t class_id)
{
    if (initialized == 0)
        return DK_EIO;
    return binding_api->unregister_class(current_driver, class_id);
}

void *dk_allocate(size_t size, size_t align)
{
    return initialized != 0 ? core_api->allocate(size, align) : NULL;
}

void *dk_allocate_zeroed(size_t size, size_t align)
{
    return initialized != 0 ? core_api->allocate_zeroed(size, align) : NULL;
}

int32_t dk_deallocate(void *data, size_t size, size_t align)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->deallocate(data, size, align);
}

int32_t dk_log(uint32_t level, struct dk_slice message)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->log(level, message);
}

int32_t dk_log_literal(uint32_t level, const char *message, size_t length)
{
    return dk_log(
        level,
        (struct dk_slice){
            .data = (const uint8_t *)message,
            .len = length,
        });
}

int32_t dk_event_create(dk_event_t *out_event)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->event_create(out_event);
}

int32_t dk_event_destroy(dk_event_t event)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->event_destroy(event);
}

int32_t dk_event_wait(dk_event_t event)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->event_wait(event);
}

int32_t dk_event_reset(dk_event_t event)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->event_reset(event);
}

int64_t dk_event_signal(dk_event_t event)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->event_signal(event);
}

void dk_sleep_ns(uint64_t nanoseconds)
{
    if (initialized != 0)
        core_api->sleep_ns(nanoseconds);
}

void dk_random_fill(uint8_t *output, size_t len)
{
    if (initialized != 0)
        core_api->random_fill(output, len);
}

void dk_random_mix(const uint8_t *input, size_t len)
{
    if (initialized != 0)
        core_api->random_mix(input, len);
}

uint64_t dk_kmsg_start(void)
{
    return initialized != 0 ? core_api->kmsg_start() : 0;
}

uint64_t dk_kmsg_end(void)
{
    return initialized != 0 ? core_api->kmsg_end() : 0;
}

int64_t dk_kmsg_read(
    uint64_t offset,
    uint8_t *output,
    size_t len,
    uint8_t nonblocking)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->kmsg_read(offset, output, len, nonblocking);
}

int32_t dk_kmsg_append(const uint8_t *input, size_t len)
{
    if (initialized == 0)
        return DK_EIO;
    return core_api->kmsg_append(input, len);
}

void dk_kmsg_disable_console_output(void)
{
    if (initialized != 0)
        core_api->kmsg_disable_console_output();
}

void dk_kmsg_enable_console_output(void)
{
    if (initialized != 0)
        core_api->kmsg_enable_console_output();
}

int32_t dk_root_bus(dk_bus_t *out_bus)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->root_bus(out_bus);
}

int32_t dk_bus_create(
    dk_node_t parent,
    struct dk_slice name,
    dk_bus_t *out_bus)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->register_bus(
        current_driver,
        parent,
        name,
        out_bus);
}

int32_t dk_device_create(
    dk_node_t parent,
    struct dk_slice name,
    dk_device_t *out_device)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->register_device(
        current_driver,
        parent,
        name,
        out_device);
}

int32_t dk_node_remove(dk_node_t node)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->remove_node(current_driver, node);
}

int32_t dk_node_set_property(
    dk_node_t node,
    struct dk_resource_key key,
    struct dk_slice value)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->set_node_property(
        current_driver,
        node,
        key,
        value);
}

int32_t dk_node_read_property(
    dk_node_t node,
    struct dk_resource_key key,
    uint8_t *output,
    size_t output_len,
    size_t *written)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->read_node_property(
        node,
        key,
        output,
        output_len,
        written);
}

int32_t dk_resource_publish_data(
    dk_node_t node,
    struct dk_resource_key key,
    uint64_t flags,
    struct dk_slice data,
    dk_resource_t *out_resource)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->publish_data_resource(
        current_driver,
        node,
        key,
        flags,
        data,
        out_resource);
}

int32_t dk_resource_publish_memory(
    dk_node_t node,
    struct dk_resource_key key,
    uint64_t flags,
    uint64_t physical,
    uint64_t length,
    dk_resource_t *out_resource)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->publish_memory_resource(
        current_driver,
        node,
        key,
        flags,
        physical,
        length,
        out_resource);
}

int32_t dk_resource_publish_protocol(
    dk_node_t node,
    struct dk_resource_key key,
    uint64_t flags,
    uint32_t revision,
    uintptr_t context,
    const void *operations,
    size_t operations_size,
    dk_resource_t *out_resource)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->publish_protocol_resource(
        current_driver,
        node,
        key,
        flags,
        revision,
        context,
        operations,
        operations_size,
        out_resource);
}

int32_t dk_resource_remove(
    dk_node_t node,
    struct dk_resource_key key)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->remove_resource(current_driver, node, key);
}

int32_t dk_resource_acquire_data(
    dk_node_t node,
    struct dk_resource_key key,
    struct dk_data_resource *out_resource)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->acquire_data_resource(
        current_driver,
        node,
        key,
        out_resource);
}

int32_t dk_resource_acquire_memory(
    dk_node_t node,
    struct dk_resource_key key,
    struct dk_memory_resource *out_resource)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->acquire_memory_resource(
        current_driver,
        node,
        key,
        out_resource);
}

int32_t dk_resource_acquire_protocol(
    dk_node_t node,
    struct dk_resource_key key,
    uint32_t minimum_revision,
    struct dk_protocol_resource *out_resource)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->acquire_protocol_resource(
        current_driver,
        node,
        key,
        minimum_revision,
        out_resource);
}

int32_t dk_resource_release(dk_resource_lease_t lease)
{
    if (initialized == 0)
        return DK_EIO;
    return registry_api->release_resource(current_driver, lease);
}

static int32_t acquire_root_protocol(
    struct dk_resource_key key,
    size_t operations_size,
    struct dk_protocol_resource *out_resource)
{
    dk_bus_t root = 0;
    int32_t status = dk_root_bus(&root);
    if (status != DK_OK)
        return status;
    status = dk_resource_acquire_protocol(
        root,
        key,
        DK_SERVICE_REVISION,
        out_resource);
    if (status != DK_OK)
        return status;
    if (out_resource->operations == NULL ||
        out_resource->operations_size < operations_size) {
        (void)dk_resource_release(out_resource->lease);
        return DK_ENOTSUP;
    }
    const struct dk_abi_service_header *header = out_resource->operations;
    if (header->size < operations_size ||
        header->revision < DK_SERVICE_REVISION) {
        (void)dk_resource_release(out_resource->lease);
        return DK_ENOTSUP;
    }
    return DK_OK;
}

static int32_t finish_protocol_call(
    struct dk_protocol_resource *resource,
    int32_t status)
{
    int32_t released = dk_resource_release(resource->lease);
    return status == DK_OK ? released : status;
}

int32_t dk_devfs_root(dk_devnode_t *out_node)
{
    struct dk_protocol_resource resource = {0};
    int32_t status = acquire_root_protocol(
        DK_RESOURCE_DEVICE_FRONTEND,
        sizeof(struct dk_device_frontend_ops),
        &resource);
    if (status != DK_OK)
        return status;
    const struct dk_device_frontend_ops *operations = resource.operations;
    status = operations->root(resource.context, out_node);
    return finish_protocol_call(&resource, status);
}

int32_t dk_devfs_create_dir(
    dk_devnode_t parent,
    struct dk_slice name,
    uint16_t mode,
    dk_devnode_t *out_node)
{
    struct dk_protocol_resource resource = {0};
    int32_t status = acquire_root_protocol(
        DK_RESOURCE_DEVICE_FRONTEND,
        sizeof(struct dk_device_frontend_ops),
        &resource);
    if (status != DK_OK)
        return status;
    const struct dk_device_frontend_ops *operations = resource.operations;
    status = operations->create_dir(
        resource.context,
        current_driver,
        parent,
        name,
        mode,
        out_node);
    return finish_protocol_call(&resource, status);
}

int32_t dk_devfs_create_device(
    dk_devnode_t parent,
    struct dk_slice name,
    uint32_t kind,
    uint16_t mode,
    dk_device_t device,
    const struct dk_device_ops *operations,
    dk_devnode_t *out_node)
{
    struct dk_protocol_resource resource = {0};
    int32_t status = acquire_root_protocol(
        DK_RESOURCE_DEVICE_FRONTEND,
        sizeof(struct dk_device_frontend_ops),
        &resource);
    if (status != DK_OK)
        return status;
    const struct dk_device_frontend_ops *frontend = resource.operations;
    status = frontend->create_device(
        resource.context,
        current_driver,
        parent,
        name,
        kind,
        mode,
        device,
        operations,
        out_node);
    return finish_protocol_call(&resource, status);
}

int32_t dk_devfs_remove_node(dk_devnode_t node)
{
    struct dk_protocol_resource resource = {0};
    int32_t status = acquire_root_protocol(
        DK_RESOURCE_DEVICE_FRONTEND,
        sizeof(struct dk_device_frontend_ops),
        &resource);
    if (status != DK_OK)
        return status;
    const struct dk_device_frontend_ops *operations = resource.operations;
    status = operations->remove_node(
        resource.context,
        current_driver,
        node);
    return finish_protocol_call(&resource, status);
}

int32_t dk_console_bus(dk_bus_t *out_bus)
{
    struct dk_protocol_resource resource = {0};
    int32_t status = acquire_root_protocol(
        DK_RESOURCE_CONSOLE_SERVICE,
        sizeof(struct dk_console_service_ops),
        &resource);
    if (status != DK_OK)
        return status;
    const struct dk_console_service_ops *operations = resource.operations;
    status = operations->bus(resource.context, out_bus);
    return finish_protocol_call(&resource, status);
}

int32_t dk_console_create_tty(
    dk_device_t device,
    dk_devnode_t parent,
    struct dk_slice name,
    uint16_t mode,
    struct dk_slice path,
    uint32_t baud,
    const struct dk_console_ops *operations,
    dk_devnode_t *out_node)
{
    struct dk_protocol_resource resource = {0};
    int32_t status = acquire_root_protocol(
        DK_RESOURCE_CONSOLE_SERVICE,
        sizeof(struct dk_console_service_ops),
        &resource);
    if (status != DK_OK)
        return status;
    const struct dk_console_service_ops *console = resource.operations;
    status = console->create_tty(
        resource.context,
        current_driver,
        device,
        parent,
        name,
        mode,
        path,
        baud,
        operations,
        out_node);
    return finish_protocol_call(&resource, status);
}

int32_t dk_mmio_map(
    uint64_t physical,
    size_t size,
    uintptr_t *out_address)
{
    if (initialized == 0)
        return DK_EIO;
    return memory_api->map_mmio(
        current_driver,
        physical,
        size,
        out_address);
}

int32_t dk_mmio_map_resource(
    const struct dk_memory_resource *resource,
    uint64_t offset,
    size_t size,
    struct dk_mmio_mapping *out_mapping)
{
    if (initialized == 0)
        return DK_EIO;
    if (resource == NULL ||
        resource->size < sizeof(*resource) ||
        (resource->flags & DK_RESOURCE_MMIO) == 0 ||
        offset > resource->length ||
        size > resource->length - offset ||
        out_mapping == NULL)
        return DK_EINVAL;
    return memory_api->map_mmio_resource(
        current_driver,
        resource->lease,
        offset,
        size,
        out_mapping);
}

int32_t dk_mmio_unmap(dk_mmio_mapping_t mapping)
{
    if (initialized == 0)
        return DK_EIO;
    return memory_api->release_mmio_mapping(current_driver, mapping);
}

int32_t dk_firmware_physical_to_virtual(
    uint64_t physical,
    uintptr_t *out_address)
{
    if (initialized == 0)
        return DK_EIO;
    return memory_api->firmware_physical_to_virtual(
        physical,
        out_address);
}

int32_t dk_irq_controller_register(
    dk_bus_t bus,
    const struct dk_interrupt_controller *controller,
    dk_irq_controller_t *out_controller)
{
    if (initialized == 0)
        return DK_EIO;
    return interrupt_api->register_interrupt_controller(
        current_driver,
        bus,
        controller,
        out_controller);
}

int32_t dk_irq_controller_unregister(dk_irq_controller_t controller)
{
    if (initialized == 0)
        return DK_EIO;
    return interrupt_api->unregister_interrupt_controller(
        current_driver,
        controller);
}

int32_t dk_irq_request(
    dk_node_t node,
    struct dk_slice specifier,
    uint64_t flags,
    uint32_t target_cpu,
    dk_interrupt_handler_fn handler,
    uintptr_t context,
    dk_irq_t *out_interrupt)
{
    if (initialized == 0)
        return DK_EIO;
    return interrupt_api->request_interrupt(
        current_driver,
        node,
        specifier,
        flags,
        target_cpu,
        handler,
        context,
        out_interrupt);
}

int32_t dk_irq_request_threaded(
    dk_node_t node,
    struct dk_slice specifier,
    uint64_t flags,
    uint32_t target_cpu,
    dk_interrupt_handler_fn handler,
    dk_interrupt_thread_fn thread_handler,
    uintptr_t context,
    dk_irq_t *out_interrupt)
{
    if (initialized == 0)
        return DK_EIO;
    if (thread_handler == NULL)
        return DK_EINVAL;
    return interrupt_api->request_threaded_interrupt(
        current_driver,
        node,
        specifier,
        flags,
        target_cpu,
        handler,
        thread_handler,
        context,
        out_interrupt);
}

int32_t dk_irq_release(dk_irq_t interrupt)
{
    if (initialized == 0)
        return DK_EIO;
    return interrupt_api->release_interrupt(current_driver, interrupt);
}

int32_t dk_irq_mask(dk_irq_t interrupt)
{
    if (initialized == 0)
        return DK_EIO;
    return interrupt_api->mask_interrupt(current_driver, interrupt);
}

int32_t dk_irq_unmask(dk_irq_t interrupt)
{
    if (initialized == 0)
        return DK_EIO;
    return interrupt_api->unmask_interrupt(current_driver, interrupt);
}

int32_t dk_irq_set_affinity(dk_irq_t interrupt, uint32_t target_cpu)
{
    if (initialized == 0)
        return DK_EIO;
    return interrupt_api->set_interrupt_affinity(
        current_driver,
        interrupt,
        target_cpu);
}
