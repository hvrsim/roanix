#include <devkit/devkit.h>

#define FDT_MAGIC UINT32_C(0xd00dfeed)
#define FDT_BEGIN_NODE UINT32_C(1)
#define FDT_END_NODE UINT32_C(2)
#define FDT_PROP UINT32_C(3)
#define FDT_NOP UINT32_C(4)
#define FDT_END UINT32_C(9)

#define MAX_DEPTH 32u
#define MAX_UARTS 16u
#define FIFO_CAPACITY 16u

struct uart_descriptor {
    uint64_t physical;
    size_t size;
    uint32_t register_shift;
    uint32_t register_width;
    uint32_t clock_hz;
    uint32_t baud;
};

struct node_state {
    uint32_t parent_address_cells;
    uint32_t parent_size_cells;
    uint32_t child_address_cells;
    uint32_t child_size_cells;
    const uint8_t *reg;
    size_t reg_len;
    uint32_t register_shift;
    uint32_t register_width;
    uint32_t clock_hz;
    uint32_t baud;
    uint8_t enabled;
    uint8_t compatible;
    uint8_t traversable;
    uint8_t identity_ranges;
};

struct uart_state {
    struct dk_spinlock lock;
    uintptr_t base;
    size_t size;
    uint32_t register_shift;
    uint32_t register_width;
    uint32_t clock_hz;
};

static struct uart_state uarts[MAX_UARTS];

static size_t align4(size_t value)
{
    return (value + 3u) & ~(size_t)3u;
}

static int string_equals(const uint8_t *value, size_t length, const char *text)
{
    size_t text_length = strlen(text);
    return length == text_length && memcmp(value, text, length) == 0;
}

static int compatible_uart(const uint8_t *value, size_t length)
{
    size_t offset = 0;
    while (offset < length) {
        size_t end = offset;
        while (end < length && value[end] != 0)
            ++end;
        size_t item_length = end - offset;
        if (string_equals(value + offset, item_length, "ns16550a") ||
            string_equals(value + offset, item_length, "ns16550") ||
            string_equals(value + offset, item_length, "uart8250") ||
            string_equals(value + offset, item_length, "snps,dw-apb-uart"))
            return 1;
        offset = end + 1u;
    }
    return 0;
}

static int status_enabled(const uint8_t *value, size_t length)
{
    size_t text_length = 0;
    while (text_length < length && value[text_length] != 0)
        ++text_length;
    return string_equals(value, text_length, "ok") ||
           string_equals(value, text_length, "okay");
}

static int read_cells(
    const uint8_t *bytes,
    size_t length,
    uint32_t cells,
    uint64_t *value)
{
    if (cells > 2 || length < (size_t)cells * 4u)
        return 0;
    uint64_t result = 0;
    for (uint32_t index = 0; index < cells; ++index)
        result = (result << 32) | dk_read_be32(bytes + index * 4u);
    *value = result;
    return 1;
}

static int finish_node(
    const struct node_state *node,
    struct uart_descriptor *descriptors,
    size_t *count)
{
    if (!node->enabled || !node->traversable || !node->compatible)
        return 1;
    if (*count == MAX_UARTS || node->reg == NULL)
        return 0;

    uint64_t physical = 0;
    uint64_t size = 0;
    size_t address_bytes = (size_t)node->parent_address_cells * 4u;
    if (!read_cells(
            node->reg,
            node->reg_len,
            node->parent_address_cells,
            &physical) ||
        !read_cells(
            node->reg + address_bytes,
            node->reg_len - address_bytes,
            node->parent_size_cells,
            &size) ||
        size == 0)
        return 0;
    if (node->register_shift > 8 ||
        (node->register_width != 1 && node->register_width != 4) ||
        node->clock_hz == 0)
        return 0;

    descriptors[*count] = (struct uart_descriptor){
        .physical = physical,
        .size = (size_t)size,
        .register_shift = node->register_shift,
        .register_width = node->register_width,
        .clock_hz = node->clock_hz,
        .baud = node->baud,
    };
    ++*count;
    return 1;
}

static int parse_dtb(
    const uint8_t *blob,
    size_t blob_length,
    struct uart_descriptor *descriptors,
    size_t *descriptor_count)
{
    if (blob_length < 40 || dk_read_be32(blob) != FDT_MAGIC)
        return 0;
    size_t total = dk_read_be32(blob + 4);
    size_t structure_offset = dk_read_be32(blob + 8);
    size_t strings_offset = dk_read_be32(blob + 12);
    size_t strings_size = dk_read_be32(blob + 32);
    size_t structure_size = dk_read_be32(blob + 36);
    if (total > blob_length ||
        structure_offset > total ||
        structure_size > total - structure_offset ||
        strings_offset > total ||
        strings_size > total - strings_offset)
        return 0;

    const uint8_t *structure = blob + structure_offset;
    const uint8_t *strings = blob + strings_offset;
    size_t offset = 0;
    size_t depth = 0;
    struct node_state stack[MAX_DEPTH];
    memset(stack, 0, sizeof(stack));
    *descriptor_count = 0;

    while (offset + 4u <= structure_size) {
        uint32_t token = dk_read_be32(structure + offset);
        offset += 4u;
        if (token == FDT_BEGIN_NODE) {
            if (depth == MAX_DEPTH)
                return 0;
            size_t name_end = offset;
            while (name_end < structure_size && structure[name_end] != 0)
                ++name_end;
            if (name_end == structure_size)
                return 0;
            offset = align4(name_end + 1u);
            if (offset > structure_size)
                return 0;

            struct node_state node = {
                .parent_address_cells = depth == 0
                    ? 2
                    : stack[depth - 1u].child_address_cells,
                .parent_size_cells = depth == 0
                    ? 1
                    : stack[depth - 1u].child_size_cells,
                .child_address_cells = 2,
                .child_size_cells = 1,
                .register_shift = 0,
                .register_width = 1,
                .clock_hz = UINT32_C(3686400),
                .baud = UINT32_C(115200),
                .enabled = 1,
                .traversable = depth == 0
                    ? 1
                    : (uint8_t)(stack[depth - 1u].traversable &&
                                stack[depth - 1u].enabled &&
                                (depth == 1 ||
                                 stack[depth - 1u].identity_ranges)),
            };
            stack[depth++] = node;
            continue;
        }
        if (token == FDT_END_NODE) {
            if (depth == 0)
                return 0;
            if (!finish_node(
                    &stack[depth - 1u],
                    descriptors,
                    descriptor_count))
                return 0;
            --depth;
            continue;
        }
        if (token == FDT_PROP) {
            if (depth == 0 || offset + 8u > structure_size)
                return 0;
            size_t length = dk_read_be32(structure + offset);
            size_t name_offset = dk_read_be32(structure + offset + 4u);
            offset += 8u;
            if (length > structure_size - offset ||
                name_offset >= strings_size)
                return 0;
            const uint8_t *value = structure + offset;
            offset = align4(offset + length);
            if (offset > structure_size)
                return 0;
            const char *name = (const char *)(strings + name_offset);
            size_t maximum = strings_size - name_offset;
            size_t name_length = 0;
            while (name_length < maximum && name[name_length] != '\0')
                ++name_length;
            if (name_length == maximum)
                return 0;

            struct node_state *node = &stack[depth - 1u];
            if (string_equals(
                    (const uint8_t *)name,
                    name_length,
                    "#address-cells") &&
                length == 4)
                node->child_address_cells = dk_read_be32(value);
            else if (string_equals(
                         (const uint8_t *)name,
                         name_length,
                         "#size-cells") &&
                     length == 4)
                node->child_size_cells = dk_read_be32(value);
            else if (string_equals(
                         (const uint8_t *)name,
                         name_length,
                         "compatible"))
                node->compatible = compatible_uart(value, length);
            else if (string_equals(
                         (const uint8_t *)name,
                         name_length,
                         "status"))
                node->enabled = status_enabled(value, length);
            else if (string_equals(
                         (const uint8_t *)name,
                         name_length,
                         "ranges"))
                node->identity_ranges = length == 0;
            else if (string_equals(
                         (const uint8_t *)name,
                         name_length,
                         "reg")) {
                node->reg = value;
                node->reg_len = length;
            } else if (string_equals(
                           (const uint8_t *)name,
                           name_length,
                           "reg-shift") &&
                       length == 4)
                node->register_shift = dk_read_be32(value);
            else if (string_equals(
                           (const uint8_t *)name,
                           name_length,
                           "reg-io-width") &&
                       length == 4)
                node->register_width = dk_read_be32(value);
            else if (string_equals(
                           (const uint8_t *)name,
                           name_length,
                           "clock-frequency") &&
                       length == 4)
                node->clock_hz = dk_read_be32(value);
            else if (string_equals(
                           (const uint8_t *)name,
                           name_length,
                           "current-speed") &&
                       length == 4)
                node->baud = dk_read_be32(value);
            continue;
        }
        if (token == FDT_NOP)
            continue;
        if (token == FDT_END)
            return depth == 0;
        return 0;
    }
    return 0;
}

static uintptr_t register_address(const struct uart_state *uart, size_t reg)
{
    return uart->base + (reg << uart->register_shift);
}

static uint8_t uart_read(const struct uart_state *uart, size_t reg)
{
    uintptr_t address = register_address(uart, reg);
    if (uart->register_width == 1)
        return dk_mmio_read8(address);
    return (uint8_t)dk_mmio_read32(address);
}

static void uart_write(
    const struct uart_state *uart,
    size_t reg,
    uint8_t value)
{
    uintptr_t address = register_address(uart, reg);
    if (uart->register_width == 1)
        dk_mmio_write8(address, value);
    else
        dk_mmio_write32(address, value);
}

static int configure_locked(
    struct uart_state *uart,
    const struct dk_serial_settings *settings,
    int clear_fifo)
{
    uint64_t denominator = UINT64_C(16) * settings->baud;
    if (denominator == 0 || denominator > uart->clock_hz)
        return 0;
    uint64_t divisor = (uart->clock_hz + denominator / 2u) / denominator;
    if (divisor == 0 || divisor > UINT16_MAX)
        return 0;

    uint8_t line;
    switch (settings->data_bits) {
    case 5:
        line = 0;
        break;
    case 6:
        line = 1;
        break;
    case 7:
        line = 2;
        break;
    case 8:
        line = 3;
        break;
    default:
        return 0;
    }
    if (settings->stop_bits == 2)
        line |= UINT8_C(1) << 2;
    else if (settings->stop_bits != 1)
        return 0;
    if (settings->parity != 0) {
        line |= UINT8_C(1) << 3;
        if (settings->odd_parity == 0)
            line |= UINT8_C(1) << 4;
    }

    uart_write(uart, 1, 0);
    uart_write(uart, 3, UINT8_C(0x80));
    uart_write(uart, 0, (uint8_t)divisor);
    uart_write(uart, 1, (uint8_t)(divisor >> 8));
    uart_write(uart, 3, line);
    uart_write(uart, 2, clear_fifo != 0 ? UINT8_C(0x07) : UINT8_C(0x01));
    uart_write(uart, 4, UINT8_C(0x0b));
    return 1;
}

static int64_t backend_read(
    uintptr_t context,
    uint8_t *output,
    size_t length)
{
    struct uart_state *uart = (struct uart_state *)context;
    if (output == NULL && length != 0)
        return DK_EINVAL;
    if (length == 0)
        return 0;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    size_t read = 0;
    while (read < length && (uart_read(uart, 5) & 1u) != 0)
        output[read++] = uart_read(uart, 0);
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return read > INT64_MAX ? INT64_MAX : (int64_t)read;
}

static int32_t backend_try_read(uintptr_t context, uint8_t *output)
{
    return backend_read(context, output, 1) == 1;
}

static int32_t backend_write(
    uintptr_t context,
    const uint8_t *data,
    size_t length,
    uint8_t nonblocking)
{
    struct uart_state *uart = (struct uart_state *)context;
    if (nonblocking != 0) {
        if (length > FIFO_CAPACITY)
            return DK_EAGAIN;
        uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
        if ((uart_read(uart, 5) & UINT8_C(0x20)) == 0) {
            dk_spin_unlock_irqrestore(&uart->lock, irq_state);
            return DK_EAGAIN;
        }
        for (size_t index = 0; index < length; ++index)
            uart_write(uart, 0, data[index]);
        dk_spin_unlock_irqrestore(&uart->lock, irq_state);
        return DK_OK;
    }
    size_t offset = 0;
    while (offset < length) {
        while ((uart_read(uart, 5) & UINT8_C(0x20)) == 0)
            dk_cpu_relax();
        uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
        if ((uart_read(uart, 5) & UINT8_C(0x20)) != 0) {
            size_t chunk = length - offset;
            if (chunk > FIFO_CAPACITY)
                chunk = FIFO_CAPACITY;
            for (size_t index = 0; index < chunk; ++index)
                uart_write(uart, 0, data[offset + index]);
            offset += chunk;
        }
        dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    }
    return DK_OK;
}

static int32_t backend_configure(
    uintptr_t context,
    const struct dk_serial_settings *settings)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    int configured = configure_locked(uart, settings, 0);
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return configured != 0 ? DK_OK : DK_EINVAL;
}

static int32_t backend_flush(uintptr_t context)
{
    struct uart_state *uart = (struct uart_state *)context;
    while ((uart_read(uart, 5) & UINT8_C(0x40)) == 0)
        dk_cpu_relax();
    return DK_OK;
}

static int32_t backend_writable(uintptr_t context)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    int writable = (uart_read(uart, 5) & UINT8_C(0x20)) != 0;
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return writable;
}

static int32_t backend_flush_input(uintptr_t context)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    while ((uart_read(uart, 5) & 1u) != 0)
        (void)uart_read(uart, 0);
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return DK_OK;
}

static int64_t backend_queued_output(uintptr_t context)
{
    (void)context;
    return 0;
}

static int32_t backend_break(uintptr_t context, uint64_t duration_ms)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    uart_write(uart, 3, (uint8_t)(uart_read(uart, 3) | (UINT8_C(1) << 6)));
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    if (duration_ms == 0)
        duration_ms = 250;
    dk_sleep_ns(duration_ms * UINT64_C(1000000));
    irq_state = dk_spin_lock_irqsave(&uart->lock);
    uart_write(uart, 3, (uint8_t)(uart_read(uart, 3) & ~(UINT8_C(1) << 6)));
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return DK_OK;
}

static int32_t publish_uart(
    size_t index,
    const struct uart_descriptor *descriptor)
{
    struct uart_state *uart = &uarts[index];
    uintptr_t mapped = 0;
    int32_t status = dk_mmio_map(
        descriptor->physical,
        descriptor->size,
        &mapped);
    if (status != DK_OK)
        return status;

    size_t stride = (size_t)1u << descriptor->register_shift;
    size_t required = 7u * stride + descriptor->register_width;
    if (stride < descriptor->register_width ||
        required > descriptor->size ||
        mapped % descriptor->register_width != 0)
        return DK_EINVAL;

    atomic_flag_clear_explicit(&uart->lock.held, memory_order_relaxed);
    uart->base = mapped;
    uart->size = descriptor->size;
    uart->register_shift = descriptor->register_shift;
    uart->register_width = descriptor->register_width;
    uart->clock_hz = descriptor->clock_hz;

    struct dk_serial_settings settings = {
        .baud = descriptor->baud,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = 0,
        .odd_parity = 0,
    };
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    int configured = configure_locked(uart, &settings, 1);
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    if (!configured)
        return DK_EINVAL;

    dk_bus_t console = 0;
    dk_devnode_t devfs = 0;
    dk_device_t device = 0;
    dk_devnode_t node = 0;
    char device_name[] = "serial0";
    char tty_name[] = "ttyS0";
    char path[] = "/dev/ttyS0";
    device_name[6] = (char)('0' + index);
    tty_name[4] = (char)('0' + index);
    path[9] = (char)('0' + index);

    status = dk_console_bus(&console);
    if (status != DK_OK)
        return status;
    status = dk_devfs_root(&devfs);
    if (status != DK_OK)
        return status;
    status = dk_device_create(
        console,
        (struct dk_slice){
            .data = (const uint8_t *)device_name,
            .len = sizeof(device_name) - 1u,
        },
        &device);
    if (status != DK_OK)
        return status;
    status = dk_node_set_property(
        device,
        DK_PROPERTY_SUBSYSTEM,
        DK_SLICE_LITERAL("tty"));
    if (status != DK_OK)
        return status;

    struct dk_console_ops operations = {
        .size = sizeof(operations),
        .flags = 0,
        .context = (uintptr_t)uart,
        .open = NULL,
        .close = NULL,
        .try_read = backend_try_read,
        .write = backend_write,
        .configure = backend_configure,
        .flush = backend_flush,
        .send_break = backend_break,
        .writable = backend_writable,
        .hung_up = NULL,
        .destroy = NULL,
        .read = backend_read,
        .flush_input = backend_flush_input,
        .flush_output = NULL,
        .queued_output = backend_queued_output,
        .readable_event = 0,
        .writable_event = 0,
        .hangup_event = 0,
    };
    return dk_console_create_tty(
        device,
        devfs,
        (struct dk_slice){
            .data = (const uint8_t *)tty_name,
            .len = sizeof(tty_name) - 1u,
        },
        0660,
        (struct dk_slice){
            .data = (const uint8_t *)path,
            .len = sizeof(path) - 1u,
        },
        descriptor->baud,
        &operations,
        &node);
}

static int32_t uart_start(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;

    dk_bus_t root = 0;
    int32_t status = dk_root_bus(&root);
    if (status != DK_OK)
        return status;
    struct dk_data_resource blob = {0};
    status = dk_resource_acquire_data(
        root,
        DK_RESOURCE_FIRMWARE_DTB,
        &blob);
    if (status != DK_OK)
        return status;
    struct uart_descriptor descriptors[MAX_UARTS];
    size_t count = 0;
    int parsed = parse_dtb(
        blob.data.data,
        blob.data.len,
        descriptors,
        &count);
    status = dk_resource_release(blob.lease);
    if (status != DK_OK)
        return status;
    if (!parsed || count == 0)
        return DK_OK;
    for (size_t index = 0; index < count; ++index) {
        status = publish_uart(index, &descriptors[index]);
        if (status != DK_OK)
            return status;
    }
    return DK_LOG_LITERAL(
        DK_LOG_INFO,
        "uart8250: published serial consoles");
}

static const struct dk_driver_definition driver = {
    .name = DK_SLICE_LITERAL("uart8250"),
    .context = 0,
    .start = uart_start,
    .stop = NULL,
};

DK_DRIVER_EXPORT(driver)
