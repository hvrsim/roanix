#include <devkit/devkit.h>

#define UART_COUNT 4u
#define UART_CLOCK UINT32_C(1843200)
#define RX_CAPACITY 4096u
#define FIFO_CAPACITY 16u
#define MAX_ISR_PASSES 64u
#define MAX_ISR_BYTES 256u

#define IER_RECEIVE (UINT8_C(1) << 0)
#define IER_LINE_STATUS (UINT8_C(1) << 2)
#define IER_RX (IER_RECEIVE | IER_LINE_STATUS)

struct byte_ring {
    uint8_t bytes[RX_CAPACITY];
    size_t head;
    size_t length;
};

struct uart_state {
    struct dk_spinlock lock;
    uint16_t base;
    uint32_t irq;
    uint8_t present;
    uint8_t interrupts_enabled;
    struct byte_ring receive;
};

static const uint16_t uart_bases[UART_COUNT] = {
    UINT16_C(0x3f8),
    UINT16_C(0x2f8),
    UINT16_C(0x3e8),
    UINT16_C(0x2e8),
};

static const uint32_t uart_irqs[UART_COUNT] = {4, 3, 4, 3};
static struct uart_state uarts[UART_COUNT];
static uint64_t irq_routes[5];

static uint8_t uart_read(const struct uart_state *uart, uint16_t reg)
{
    return dk_in8((uint16_t)(uart->base + reg));
}

static void uart_write(
    const struct uart_state *uart,
    uint16_t reg,
    uint8_t value)
{
    dk_out8((uint16_t)(uart->base + reg), value);
}

static void ring_push(struct byte_ring *ring, uint8_t byte)
{
    if (ring->length == RX_CAPACITY) {
        ring->head = (ring->head + 1u) % RX_CAPACITY;
        --ring->length;
    }
    size_t tail = (ring->head + ring->length) % RX_CAPACITY;
    ring->bytes[tail] = byte;
    ++ring->length;
}

static int ring_pop(struct byte_ring *ring, uint8_t *byte)
{
    if (ring->length == 0)
        return 0;
    *byte = ring->bytes[ring->head];
    ring->head = (ring->head + 1u) % RX_CAPACITY;
    --ring->length;
    return 1;
}

static int configure_locked(
    struct uart_state *uart,
    const struct dk_serial_settings *settings,
    int clear_fifo)
{
    uint64_t denominator = UINT64_C(16) * settings->baud;
    if (denominator == 0 || denominator > UART_CLOCK)
        return 0;
    uint64_t divisor = (UART_CLOCK + denominator / 2u) / denominator;
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

    uint8_t ier = uart->interrupts_enabled != 0 ? IER_RX : 0;
    uart_write(uart, 1, 0);
    uart_write(uart, 3, UINT8_C(0x80));
    uart_write(uart, 0, (uint8_t)divisor);
    uart_write(uart, 1, (uint8_t)(divisor >> 8));
    uart_write(uart, 3, line);
    uart_write(uart, 1, ier);
    uart_write(uart, 2, clear_fifo != 0 ? UINT8_C(0x07) : UINT8_C(0x01));
    uart_write(uart, 4, UINT8_C(0x0b));
    return 1;
}

static int probe_uart(struct uart_state *uart)
{
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    uint8_t original = uart_read(uart, 7);
    uart_write(uart, 7, UINT8_C(0x5a));
    uint8_t first = uart_read(uart, 7);
    uart_write(uart, 7, UINT8_C(0xa5));
    uint8_t second = uart_read(uart, 7);
    uart_write(uart, 7, original);

    struct dk_serial_settings settings = {
        .baud = 9600,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = 0,
        .odd_parity = 0,
    };
    int present = first == UINT8_C(0x5a) &&
                  second == UINT8_C(0xa5) &&
                  uart_read(uart, 5) != UINT8_C(0xff) &&
                  configure_locked(uart, &settings, 1);
    uart->present = present != 0;
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return present;
}

static void drain_receive_locked(struct uart_state *uart)
{
    for (size_t pass = 0; pass < MAX_ISR_BYTES; ++pass) {
        uint8_t status = uart_read(uart, 5);
        if (status == UINT8_C(0xff) || (status & 1u) == 0)
            break;
        ring_push(&uart->receive, uart_read(uart, 0));
    }
}

static void service_interrupt_locked(struct uart_state *uart)
{
    for (size_t pass = 0; pass < MAX_ISR_PASSES; ++pass) {
        uint8_t identification = uart_read(uart, 2);
        if (identification == UINT8_C(0xff) ||
            (identification & 1u) != 0)
            break;

        switch (identification & UINT8_C(0x0e)) {
        case UINT8_C(0x04):
        case UINT8_C(0x0c):
            drain_receive_locked(uart);
            break;
        case UINT8_C(0x06): {
            uint8_t status = uart_read(uart, 5);
            if ((status & 1u) != 0) {
                ring_push(&uart->receive, uart_read(uart, 0));
                drain_receive_locked(uart);
            }
            break;
        }
        case UINT8_C(0x02): {
            uint8_t ier = uart_read(uart, 1);
            uart_write(uart, 1, (uint8_t)(ier & ~(UINT8_C(1) << 1)));
            break;
        }
        case UINT8_C(0x00):
            (void)uart_read(uart, 6);
            break;
        default:
            return;
        }
    }
}

static uint32_t uart_interrupt(uintptr_t context, uint64_t interrupt)
{
    (void)interrupt;
    uint32_t irq = (uint32_t)context;
    for (size_t index = 0; index < UART_COUNT; ++index) {
        struct uart_state *uart = &uarts[index];
        if (uart->irq != irq)
            continue;
        uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
        if (uart->present != 0 && uart->interrupts_enabled != 0)
            service_interrupt_locked(uart);
        dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    }
    return 0;
}

static int32_t backend_try_read(uintptr_t context, uint8_t *output)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    int available = ring_pop(&uart->receive, output);
    if (!available && (uart_read(uart, 5) & 1u) != 0) {
        *output = uart_read(uart, 0);
        available = 1;
    }
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return available;
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

static int32_t enable_interrupts(struct uart_state *uart, uint64_t device)
{
    uint32_t irq = uart->irq;
    if (irq_routes[irq] == 0) {
        uint8_t specifier[8] = {
            1, 0, 0, 0,
            (uint8_t)irq,
            (uint8_t)(irq >> 8),
            (uint8_t)(irq >> 16),
            (uint8_t)(irq >> 24),
        };
        int32_t status = dk_irq_request(
            device,
            (struct dk_slice){
                .data = specifier,
                .len = sizeof(specifier),
            },
            DK_INTERRUPT_EDGE | DK_INTERRUPT_ACTIVE_HIGH,
            0,
            uart_interrupt,
            irq,
            &irq_routes[irq]);
        if (status != DK_OK)
            return status;
    }

    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    drain_receive_locked(uart);
    (void)uart_read(uart, 5);
    (void)uart_read(uart, 6);
    uart->interrupts_enabled = 1;
    uart_write(uart, 1, IER_RX);
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return DK_OK;
}

static int32_t publish_uart(size_t index)
{
    struct uart_state *uart = &uarts[index];
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

    int32_t status = dk_console_bus(&console);
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
    status = enable_interrupts(uart, device);
    if (status != DK_OK &&
        status != DK_ENOENT &&
        status != DK_ENOTSUP)
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
        9600,
        &operations,
        &node);
}

static int32_t uart_start(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;

    size_t published = 0;
    for (size_t index = 0; index < UART_COUNT; ++index) {
        struct uart_state *uart = &uarts[index];
        atomic_flag_clear_explicit(&uart->lock.held, memory_order_relaxed);
        uart->base = uart_bases[index];
        uart->irq = uart_irqs[index];
        if (!probe_uart(uart))
            continue;
        int32_t status = publish_uart(index);
        if (status != DK_OK)
            return status;
        ++published;
    }
    if (published == 0) {
        (void)DK_LOG_LITERAL(
            DK_LOG_WARN,
            "uart16550: no legacy UART detected");
        return DK_OK;
    }
    return DK_LOG_LITERAL(
        DK_LOG_INFO,
        "uart16550: published serial consoles");
}

static void uart_stop(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;
    for (size_t index = 0; index < UART_COUNT; ++index) {
        struct uart_state *uart = &uarts[index];
        if (uart->present == 0)
            continue;
        uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
        uart_write(uart, 1, 0);
        uart->interrupts_enabled = 0;
        dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    }
}

static const struct dk_driver_definition driver = {
    .name = DK_SLICE_LITERAL("uart16550"),
    .context = 0,
    .start = uart_start,
    .stop = uart_stop,
};

DK_DRIVER_EXPORT(driver)
