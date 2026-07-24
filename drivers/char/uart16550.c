#include <devkit/devkit.h>

#define UART_COUNT 4u
#define UART_CLOCK UINT32_C(1843200)
#define RX_CAPACITY 8192u
#define TX_CAPACITY (64u * 1024u)
#define FIFO_CAPACITY 16u
#define MAX_ISR_PASSES 64u
#define MAX_ISR_BYTES 256u

#define IER_RECEIVE (UINT8_C(1) << 0)
#define IER_TRANSMIT (UINT8_C(1) << 1)
#define IER_LINE_STATUS (UINT8_C(1) << 2)
#define IER_RX (IER_RECEIVE | IER_LINE_STATUS)

#define WAKE_READABLE (UINT32_C(1) << 0)
#define WAKE_WRITABLE (UINT32_C(1) << 1)
#define WAKE_DRAINED (UINT32_C(1) << 2)

struct byte_ring {
    uint8_t *bytes;
    size_t capacity;
    size_t head;
    size_t length;
};

struct uart_state {
    struct dk_spinlock lock;
    uint16_t base;
    uint32_t irq;
    uint8_t present;
    uint8_t interrupts_enabled;
    uint8_t ier;
    struct byte_ring receive;
    struct byte_ring transmit;
    dk_event_t readable;
    dk_event_t writable;
    dk_event_t drained;
    uint8_t receive_storage[RX_CAPACITY];
    uint8_t transmit_storage[TX_CAPACITY];
};

static const uint16_t uart_bases[UART_COUNT] = {
    UINT16_C(0x3f8),
    UINT16_C(0x2f8),
    UINT16_C(0x3e8),
    UINT16_C(0x2e8),
};

static const uint32_t uart_irqs[UART_COUNT] = {4, 3, 4, 3};
static _Alignas(64) struct uart_state uarts[UART_COUNT];
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
    if (ring->length == ring->capacity) {
        ring->head = (ring->head + 1u) % ring->capacity;
        --ring->length;
    }
    size_t tail = (ring->head + ring->length) % ring->capacity;
    ring->bytes[tail] = byte;
    ++ring->length;
}

static size_t ring_available(const struct byte_ring *ring)
{
    return ring->capacity - ring->length;
}

static size_t ring_read(
    struct byte_ring *ring,
    uint8_t *output,
    size_t length)
{
    if (length > ring->length)
        length = ring->length;
    size_t first = length;
    if (first > ring->capacity - ring->head)
        first = ring->capacity - ring->head;
    memcpy(output, ring->bytes + ring->head, first);
    memcpy(output + first, ring->bytes, length - first);
    ring->head = (ring->head + length) % ring->capacity;
    ring->length -= length;
    return length;
}

static size_t ring_write(
    struct byte_ring *ring,
    const uint8_t *input,
    size_t length)
{
    size_t available = ring_available(ring);
    if (length > available)
        length = available;
    size_t tail = (ring->head + ring->length) % ring->capacity;
    size_t first = length;
    if (first > ring->capacity - tail)
        first = ring->capacity - tail;
    memcpy(ring->bytes + tail, input, first);
    memcpy(ring->bytes, input + first, length - first);
    ring->length += length;
    return length;
}

static void set_ier_locked(struct uart_state *uart, uint8_t ier)
{
    if (uart->ier == ier)
        return;
    uart->ier = ier;
    uart_write(uart, 1, ier);
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

    uint8_t ier = uart->interrupts_enabled != 0 ? uart->ier : 0;
    uart_write(uart, 1, 0);
    uart_write(uart, 3, UINT8_C(0x80));
    uart_write(uart, 0, (uint8_t)divisor);
    uart_write(uart, 1, (uint8_t)(divisor >> 8));
    uart_write(uart, 3, line);
    uart->ier = ier;
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
        .baud = 115200,
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

static size_t drain_receive_locked(struct uart_state *uart)
{
    size_t received = 0;
    for (size_t pass = 0; pass < MAX_ISR_BYTES; ++pass) {
        uint8_t status = uart_read(uart, 5);
        if (status == UINT8_C(0xff) || (status & 1u) == 0)
            break;
        ring_push(&uart->receive, uart_read(uart, 0));
        ++received;
    }
    return received;
}

static uint32_t fill_transmit_locked(struct uart_state *uart)
{
    uint32_t wake = 0;
    int was_full = ring_available(&uart->transmit) == 0;
    int had_data = uart->transmit.length != 0;
    if ((uart_read(uart, 5) & UINT8_C(0x20)) != 0) {
        size_t count = uart->transmit.length;
        if (count > FIFO_CAPACITY)
            count = FIFO_CAPACITY;
        for (size_t index = 0; index < count; ++index) {
            uint8_t byte = 0;
            (void)ring_read(&uart->transmit, &byte, 1);
            uart_write(uart, 0, byte);
        }
    }

    if (was_full && ring_available(&uart->transmit) != 0)
        wake |= WAKE_WRITABLE;
    if (uart->transmit.length == 0) {
        set_ier_locked(uart, (uint8_t)(uart->ier & ~IER_TRANSMIT));
        if (had_data)
            wake |= WAKE_DRAINED;
    } else {
        set_ier_locked(uart, (uint8_t)(uart->ier | IER_TRANSMIT));
    }
    return wake;
}

static uint32_t service_interrupt_locked(struct uart_state *uart)
{
    uint32_t wake = 0;
    for (size_t pass = 0; pass < MAX_ISR_PASSES; ++pass) {
        uint8_t identification = uart_read(uart, 2);
        if (identification == UINT8_C(0xff) ||
            (identification & 1u) != 0)
            break;

        switch (identification & UINT8_C(0x0e)) {
        case UINT8_C(0x04):
        case UINT8_C(0x0c): {
            int was_empty = uart->receive.length == 0;
            if (drain_receive_locked(uart) != 0 && was_empty)
                wake |= WAKE_READABLE;
            break;
        }
        case UINT8_C(0x06): {
            uint8_t status = uart_read(uart, 5);
            if ((status & 1u) != 0) {
                int was_empty = uart->receive.length == 0;
                ring_push(&uart->receive, uart_read(uart, 0));
                (void)drain_receive_locked(uart);
                if (was_empty)
                    wake |= WAKE_READABLE;
            }
            break;
        }
        case UINT8_C(0x02):
            wake |= fill_transmit_locked(uart);
            break;
        case UINT8_C(0x00):
            (void)uart_read(uart, 6);
            break;
        default:
            return wake;
        }
    }
    return wake;
}

static uint32_t uart_interrupt(uintptr_t context, uint64_t interrupt)
{
    (void)interrupt;
    uint32_t irq = (uint32_t)context;
    uint32_t result = 0;
    for (size_t index = 0; index < UART_COUNT; ++index) {
        struct uart_state *uart = &uarts[index];
        if (uart->irq != irq)
            continue;
        uint32_t wake = 0;
        uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
        if (uart->present != 0 && uart->interrupts_enabled != 0)
            wake = service_interrupt_locked(uart);
        dk_spin_unlock_irqrestore(&uart->lock, irq_state);
        if ((wake & WAKE_READABLE) != 0)
            result |= dk_event_signal(uart->readable) > 0
                ? DK_INTERRUPT_RESCHEDULE
                : 0;
        if ((wake & WAKE_WRITABLE) != 0)
            result |= dk_event_signal(uart->writable) > 0
                ? DK_INTERRUPT_RESCHEDULE
                : 0;
        if ((wake & WAKE_DRAINED) != 0)
            result |= dk_event_signal(uart->drained) > 0
                ? DK_INTERRUPT_RESCHEDULE
                : 0;
    }
    return result;
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
    (void)drain_receive_locked(uart);
    size_t read = ring_read(&uart->receive, output, length);
    if (uart->receive.length == 0)
        (void)dk_event_reset(uart->readable);
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return read > INT64_MAX ? INT64_MAX : (int64_t)read;
}

static int32_t backend_try_read(uintptr_t context, uint8_t *output)
{
    return backend_read(context, output, 1) == 1;
}

static int32_t write_polled(
    struct uart_state *uart,
    const uint8_t *data,
    size_t length,
    uint8_t nonblocking)
{
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
        if ((uart_read(uart, 5) & UINT8_C(0x20)) == 0) {
            dk_cpu_relax();
            continue;
        }
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

static int32_t backend_write(
    uintptr_t context,
    const uint8_t *data,
    size_t length,
    uint8_t nonblocking)
{
    struct uart_state *uart = (struct uart_state *)context;
    if (data == NULL && length != 0)
        return DK_EINVAL;
    if (length == 0)
        return DK_OK;
    if (uart->interrupts_enabled == 0)
        return write_polled(uart, data, length, nonblocking);
    if (nonblocking != 0) {
        uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
        if (ring_available(&uart->transmit) < length) {
            dk_spin_unlock_irqrestore(&uart->lock, irq_state);
            return DK_EAGAIN;
        }
        (void)ring_write(&uart->transmit, data, length);
        (void)dk_event_reset(uart->drained);
        uint32_t wake = fill_transmit_locked(uart);
        if (ring_available(&uart->transmit) == 0)
            (void)dk_event_reset(uart->writable);
        dk_spin_unlock_irqrestore(&uart->lock, irq_state);
        if ((wake & WAKE_WRITABLE) != 0)
            (void)dk_event_signal(uart->writable);
        if ((wake & WAKE_DRAINED) != 0)
            (void)dk_event_signal(uart->drained);
        return DK_OK;
    }

    size_t offset = 0;
    while (offset < length) {
        uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
        size_t available = ring_available(&uart->transmit);
        if (available != 0) {
            size_t chunk = length - offset;
            if (chunk > available)
                chunk = available;
            offset += ring_write(
                &uart->transmit,
                data + offset,
                chunk);
            (void)dk_event_reset(uart->drained);
            uint32_t wake = fill_transmit_locked(uart);
            if (ring_available(&uart->transmit) == 0)
                (void)dk_event_reset(uart->writable);
            dk_spin_unlock_irqrestore(&uart->lock, irq_state);
            if ((wake & WAKE_WRITABLE) != 0)
                (void)dk_event_signal(uart->writable);
            if ((wake & WAKE_DRAINED) != 0)
                (void)dk_event_signal(uart->drained);
            continue;
        }
        (void)dk_event_reset(uart->writable);
        dk_spin_unlock_irqrestore(&uart->lock, irq_state);
        (void)dk_event_wait(uart->writable);
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
    for (;;) {
        uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
        if (uart->transmit.length == 0) {
            dk_spin_unlock_irqrestore(&uart->lock, irq_state);
            break;
        }
        (void)dk_event_reset(uart->drained);
        dk_spin_unlock_irqrestore(&uart->lock, irq_state);
        (void)dk_event_wait(uart->drained);
    }
    while ((uart_read(uart, 5) & UINT8_C(0x40)) == 0)
        dk_sleep_ns(UINT64_C(50000));
    return DK_OK;
}

static int32_t backend_writable(uintptr_t context)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    int writable = uart->interrupts_enabled != 0
        ? ring_available(&uart->transmit) != 0
        : (uart_read(uart, 5) & UINT8_C(0x20)) != 0;
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return writable;
}

static int32_t backend_flush_input(uintptr_t context)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    uart->receive.head = 0;
    uart->receive.length = 0;
    uart_write(uart, 2, UINT8_C(0x03));
    (void)dk_event_reset(uart->readable);
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return DK_OK;
}

static int32_t backend_flush_output(uintptr_t context)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    uart->transmit.head = 0;
    uart->transmit.length = 0;
    set_ier_locked(uart, (uint8_t)(uart->ier & ~IER_TRANSMIT));
    uart_write(uart, 2, UINT8_C(0x05));
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    (void)dk_event_signal(uart->writable);
    (void)dk_event_signal(uart->drained);
    return DK_OK;
}

static int64_t backend_queued_output(uintptr_t context)
{
    struct uart_state *uart = (struct uart_state *)context;
    uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
    size_t queued = uart->interrupts_enabled != 0
        ? uart->transmit.length
        : 0;
    dk_spin_unlock_irqrestore(&uart->lock, irq_state);
    return queued > INT64_MAX ? INT64_MAX : (int64_t)queued;
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
    if (drain_receive_locked(uart) != 0)
        (void)dk_event_signal(uart->readable);
    (void)uart_read(uart, 5);
    (void)uart_read(uart, 6);
    uart->interrupts_enabled = 1;
    set_ier_locked(uart, IER_RX);
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
        .read = backend_read,
        .flush_input = backend_flush_input,
        .flush_output = backend_flush_output,
        .queued_output = backend_queued_output,
        .readable_event = uart->interrupts_enabled != 0
            ? uart->readable
            : 0,
        .writable_event = uart->interrupts_enabled != 0
            ? uart->writable
            : 0,
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
        115200,
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
        uart->receive = (struct byte_ring){
            .bytes = uart->receive_storage,
            .capacity = RX_CAPACITY,
        };
        uart->transmit = (struct byte_ring){
            .bytes = uart->transmit_storage,
            .capacity = TX_CAPACITY,
        };
        int32_t status = dk_event_create(&uart->readable);
        if (status != DK_OK)
            return status;
        status = dk_event_create(&uart->writable);
        if (status != DK_OK)
            return status;
        status = dk_event_create(&uart->drained);
        if (status != DK_OK)
            return status;
        (void)dk_event_signal(uart->writable);
        (void)dk_event_signal(uart->drained);
        if (!probe_uart(uart))
            continue;
        status = publish_uart(index);
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
        if (uart->present != 0) {
            uintptr_t irq_state = dk_spin_lock_irqsave(&uart->lock);
            uart_write(uart, 1, 0);
            uart->ier = 0;
            uart->interrupts_enabled = 0;
            dk_spin_unlock_irqrestore(&uart->lock, irq_state);
        }
        if (uart->readable != 0) {
            (void)dk_event_destroy(uart->readable);
            uart->readable = 0;
        }
        if (uart->writable != 0) {
            (void)dk_event_destroy(uart->writable);
            uart->writable = 0;
        }
        if (uart->drained != 0) {
            (void)dk_event_destroy(uart->drained);
            uart->drained = 0;
        }
    }
}

static const struct dk_driver_definition driver = {
    .name = DK_SLICE_LITERAL("uart16550"),
    .context = 0,
    .start = uart_start,
    .stop = uart_stop,
};

DK_DRIVER_EXPORT(driver)
