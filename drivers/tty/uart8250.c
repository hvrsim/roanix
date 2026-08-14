/*
 * 8250 and 16550 compatible serial ports.
 *
 * One driver covers both the port-mapped legacy PC UART and the
 * memory-mapped variants that device trees describe, because the register set
 * is identical and only the access method differs. Which one applies is
 * decided from the device's resources, so the same code serves both supported
 * architectures.
 *
 * Transmission is interrupt driven when a line is available and falls back to
 * polling otherwise, which keeps the console usable before an interrupt
 * controller has bound.
 */
#include <roanix/driver.h>

#define REG_DATA 0u
#define REG_INTERRUPT_ENABLE 1u
#define REG_INTERRUPT_ID 2u
#define REG_FIFO_CONTROL 2u
#define REG_LINE_CONTROL 3u
#define REG_MODEM_CONTROL 4u
#define REG_LINE_STATUS 5u
#define REG_MODEM_STATUS 6u
#define REG_SCRATCH 7u

#define IER_RECEIVE (UINT8_C(1) << 0)
#define IER_TRANSMIT (UINT8_C(1) << 1)
#define IER_LINE_STATUS (UINT8_C(1) << 2)
#define IER_RX (IER_RECEIVE | IER_LINE_STATUS)

#define LSR_DATA_READY (UINT8_C(1) << 0)
#define LSR_TRANSMIT_EMPTY (UINT8_C(1) << 5)
#define LSR_TRANSMIT_IDLE (UINT8_C(1) << 6)

#define LCR_DIVISOR_LATCH (UINT8_C(0x80))
#define LCR_BREAK (UINT8_C(1) << 6)

#define FIFO_CAPACITY 16u
#define RX_CAPACITY 8192u
#define TX_CAPACITY (64u * 1024u)
#define MAX_ISR_PASSES 64u
#define MAX_ISR_BYTES 256u
#define DEFAULT_CLOCK UINT32_C(1843200)
#define MAX_PORTS 8u

#define WAKE_READABLE (UINT32_C(1) << 0)
#define WAKE_WRITABLE (UINT32_C(1) << 1)
#define WAKE_DRAINED (UINT32_C(1) << 2)
/* Set when the port actually had an interrupt pending, even if nothing woke. */
#define WAKE_SERVICED (UINT32_C(1) << 3)

struct byte_ring {
    uint8_t *bytes;
    size_t capacity;
    size_t head;
    size_t length;
};

struct port {
    struct rdf_spinlock lock;
    struct rdf_mmio window;
    uint16_t io_base;
    uint8_t mapped;
    uint8_t memory_mapped;
    uint32_t shift;
    uint32_t width;
    uint32_t clock;
    uint32_t virq;
    struct rdf_irq *irq;
    uint8_t interrupts_enabled;
    uint8_t ier;
    struct rdf_tty *tty;
    struct byte_ring receive;
    struct byte_ring transmit;
    rdf_event_t readable;
    rdf_event_t writable;
    rdf_event_t drained;
    uint8_t receive_storage[RX_CAPACITY];
    uint8_t transmit_storage[TX_CAPACITY];
};

static struct port ports[MAX_PORTS];
static size_t port_count;
static struct rdf_spinlock allocation_lock = RDF_SPINLOCK_INIT;

/* --- Register access ---------------------------------------------------- */

static uint8_t port_read(const struct port *port, uint32_t reg)
{
    if (port->memory_mapped == 0)
        return rdf_in8((uint16_t)(port->io_base + reg));

    size_t offset = (size_t)reg << port->shift;
    switch (port->width) {
    case 4:
        return (uint8_t)rdf_read32(&port->window, offset);
    case 2:
        return (uint8_t)rdf_read16(&port->window, offset);
    default:
        return rdf_read8(&port->window, offset);
    }
}

static void port_write(const struct port *port, uint32_t reg, uint8_t value)
{
    if (port->memory_mapped == 0) {
        rdf_out8((uint16_t)(port->io_base + reg), value);
        return;
    }

    size_t offset = (size_t)reg << port->shift;
    switch (port->width) {
    case 4:
        rdf_write32(&port->window, offset, value);
        break;
    case 2:
        rdf_write16(&port->window, offset, value);
        break;
    default:
        rdf_write8(&port->window, offset, value);
        break;
    }
}

/* --- Byte rings --------------------------------------------------------- */

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

static size_t ring_read(struct byte_ring *ring, uint8_t *output, size_t length)
{
    if (length > ring->length)
        length = ring->length;
    size_t first = length;
    if (first > ring->capacity - ring->head)
        first = ring->capacity - ring->head;
    memcpy(output, ring->bytes + ring->head, first);
    if (length > first)
        memcpy(output + first, ring->bytes, length - first);
    ring->head = (ring->head + length) % ring->capacity;
    ring->length -= length;
    return length;
}

static size_t ring_write(struct byte_ring *ring, const uint8_t *input, size_t length)
{
    size_t available = ring_available(ring);
    if (length > available)
        length = available;
    size_t tail = (ring->head + ring->length) % ring->capacity;
    size_t first = length;
    if (first > ring->capacity - tail)
        first = ring->capacity - tail;
    memcpy(ring->bytes + tail, input, first);
    if (length > first)
        memcpy(ring->bytes, input + first, length - first);
    ring->length += length;
    return length;
}

/* --- Line configuration ------------------------------------------------- */

static void set_ier_locked(struct port *port, uint8_t ier)
{
    if (port->ier == ier)
        return;
    port->ier = ier;
    port_write(port, REG_INTERRUPT_ENABLE, ier);
}

static int configure_locked(struct port *port, const struct rdf_serial_framing *framing,
                            int clear_fifo)
{
    uint64_t denominator = UINT64_C(16) * framing->baud;
    if (denominator == 0 || denominator > port->clock)
        return 0;
    uint64_t divisor = (port->clock + denominator / 2u) / denominator;
    if (divisor == 0 || divisor > UINT16_MAX)
        return 0;

    uint8_t line;
    switch (framing->data_bits) {
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
    if (framing->stop_bits == 2)
        line |= UINT8_C(1) << 2;
    else if (framing->stop_bits != 1)
        return 0;
    if (framing->parity != 0) {
        line |= UINT8_C(1) << 3;
        if (framing->odd_parity == 0)
            line |= UINT8_C(1) << 4;
    }

    uint8_t ier = port->interrupts_enabled != 0 ? port->ier : 0;
    port_write(port, REG_INTERRUPT_ENABLE, 0);
    port_write(port, REG_LINE_CONTROL, LCR_DIVISOR_LATCH);
    port_write(port, REG_DATA, (uint8_t)divisor);
    port_write(port, REG_INTERRUPT_ENABLE, (uint8_t)(divisor >> 8));
    port_write(port, REG_LINE_CONTROL, line);
    port->ier = ier;
    port_write(port, REG_INTERRUPT_ENABLE, ier);
    port_write(port, REG_FIFO_CONTROL, clear_fifo != 0 ? UINT8_C(0x07) : UINT8_C(0x01));
    port_write(port, REG_MODEM_CONTROL, UINT8_C(0x0b));
    return 1;
}

/* Confirms a UART is present by round-tripping the scratch register. */
static int detect(struct port *port)
{
    static const struct rdf_serial_framing framing = {
        .baud = 115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = 0,
        .odd_parity = 0,
    };

    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    uint8_t original = port_read(port, REG_SCRATCH);
    port_write(port, REG_SCRATCH, UINT8_C(0x5a));
    uint8_t first = port_read(port, REG_SCRATCH);
    port_write(port, REG_SCRATCH, UINT8_C(0xa5));
    uint8_t second = port_read(port, REG_SCRATCH);
    port_write(port, REG_SCRATCH, original);

    int present = first == UINT8_C(0x5a) && second == UINT8_C(0xa5) &&
                  port_read(port, REG_LINE_STATUS) != UINT8_C(0xff) &&
                  configure_locked(port, &framing, 1);
    rdf_spin_unlock_irqrestore(&port->lock, state);
    return present;
}

/* --- Interrupt service -------------------------------------------------- */

static size_t drain_receive_locked(struct port *port)
{
    size_t received = 0;
    for (size_t pass = 0; pass < MAX_ISR_BYTES; ++pass) {
        uint8_t status = port_read(port, REG_LINE_STATUS);
        if (status == UINT8_C(0xff) || (status & LSR_DATA_READY) == 0)
            break;
        ring_push(&port->receive, port_read(port, REG_DATA));
        ++received;
    }
    return received;
}

static uint32_t fill_transmit_locked(struct port *port)
{
    uint32_t wake = 0;
    int was_full = ring_available(&port->transmit) == 0;
    int had_data = port->transmit.length != 0;

    if ((port_read(port, REG_LINE_STATUS) & LSR_TRANSMIT_EMPTY) != 0) {
        size_t count = port->transmit.length;
        if (count > FIFO_CAPACITY)
            count = FIFO_CAPACITY;
        for (size_t index = 0; index < count; ++index) {
            uint8_t byte = 0;
            ring_read(&port->transmit, &byte, 1);
            port_write(port, REG_DATA, byte);
        }
    }

    if (was_full && ring_available(&port->transmit) != 0)
        wake |= WAKE_WRITABLE;
    if (port->transmit.length == 0) {
        set_ier_locked(port, (uint8_t)(port->ier & ~IER_TRANSMIT));
        if (had_data)
            wake |= WAKE_DRAINED;
    } else {
        set_ier_locked(port, (uint8_t)(port->ier | IER_TRANSMIT));
    }
    return wake;
}

static uint32_t service_locked(struct port *port)
{
    uint32_t wake = 0;
    for (size_t pass = 0; pass < MAX_ISR_PASSES; ++pass) {
        uint8_t identification = port_read(port, REG_INTERRUPT_ID);
        if (identification == UINT8_C(0xff) || (identification & 1u) != 0)
            break;
        wake |= WAKE_SERVICED;

        switch (identification & UINT8_C(0x0e)) {
        case UINT8_C(0x04):
        case UINT8_C(0x0c): {
            int was_empty = port->receive.length == 0;
            if (drain_receive_locked(port) != 0 && was_empty)
                wake |= WAKE_READABLE;
            break;
        }
        case UINT8_C(0x06): {
            uint8_t status = port_read(port, REG_LINE_STATUS);
            if ((status & LSR_DATA_READY) != 0) {
                int was_empty = port->receive.length == 0;
                ring_push(&port->receive, port_read(port, REG_DATA));
                drain_receive_locked(port);
                if (was_empty)
                    wake |= WAKE_READABLE;
            }
            break;
        }
        case UINT8_C(0x02):
            wake |= fill_transmit_locked(port);
            break;
        case UINT8_C(0x00):
            port_read(port, REG_MODEM_STATUS);
            break;
        default:
            return wake;
        }
    }
    return wake;
}

static uint32_t port_interrupt(void *context, uint32_t virq)
{
    struct port *port = (struct port *)context;
    uint32_t wake = 0;

    (void)virq;

    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    if (port->interrupts_enabled != 0)
        wake = service_locked(port);
    rdf_spin_unlock_irqrestore(&port->lock, state);

    if (wake == 0)
        return RDF_IRQ_NONE;
    if ((wake & ~WAKE_SERVICED) == 0)
        return RDF_IRQ_HANDLED;

    if ((wake & WAKE_READABLE) != 0)
        rdf_event_signal(port->readable);
    if ((wake & WAKE_WRITABLE) != 0)
        rdf_event_signal(port->writable);
    if ((wake & WAKE_DRAINED) != 0)
        rdf_event_signal(port->drained);
    return RDF_IRQ_HANDLED | RDF_IRQ_RESCHEDULE;
}

/* --- Console backend ---------------------------------------------------- */

static int64_t console_read(void *context, uint8_t *output, size_t length)
{
    struct port *port = (struct port *)context;
    if (output == NULL && length != 0)
        return RDF_EINVAL;
    if (length == 0)
        return 0;

    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    drain_receive_locked(port);
    size_t read = ring_read(&port->receive, output, length);
    if (port->receive.length == 0)
        rdf_event_reset(port->readable);
    rdf_spin_unlock_irqrestore(&port->lock, state);
    return (int64_t)read;
}

static int32_t console_try_read(void *context, uint8_t *output)
{
    return console_read(context, output, 1) == 1;
}

static int32_t write_polled(struct port *port, const uint8_t *data, size_t length,
                            uint8_t nonblocking)
{
    if (nonblocking != 0) {
        if (length > FIFO_CAPACITY)
            return RDF_EAGAIN;
        uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
        if ((port_read(port, REG_LINE_STATUS) & LSR_TRANSMIT_EMPTY) == 0) {
            rdf_spin_unlock_irqrestore(&port->lock, state);
            return RDF_EAGAIN;
        }
        for (size_t index = 0; index < length; ++index)
            port_write(port, REG_DATA, data[index]);
        rdf_spin_unlock_irqrestore(&port->lock, state);
        return RDF_OK;
    }

    size_t offset = 0;
    while (offset < length) {
        if ((port_read(port, REG_LINE_STATUS) & LSR_TRANSMIT_EMPTY) == 0) {
            rdf_cpu_relax();
            continue;
        }
        uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
        if ((port_read(port, REG_LINE_STATUS) & LSR_TRANSMIT_EMPTY) != 0) {
            size_t chunk = length - offset;
            if (chunk > FIFO_CAPACITY)
                chunk = FIFO_CAPACITY;
            for (size_t index = 0; index < chunk; ++index)
                port_write(port, REG_DATA, data[offset + index]);
            offset += chunk;
        }
        rdf_spin_unlock_irqrestore(&port->lock, state);
    }
    return RDF_OK;
}

static int32_t console_write(void *context, const uint8_t *data, size_t length,
                             uint8_t nonblocking)
{
    struct port *port = (struct port *)context;
    if (data == NULL && length != 0)
        return RDF_EINVAL;
    if (length == 0)
        return RDF_OK;
    if (port->interrupts_enabled == 0)
        return write_polled(port, data, length, nonblocking);

    if (nonblocking != 0) {
        uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
        if (ring_available(&port->transmit) < length) {
            rdf_spin_unlock_irqrestore(&port->lock, state);
            return RDF_EAGAIN;
        }
        ring_write(&port->transmit, data, length);
        rdf_event_reset(port->drained);
        uint32_t wake = fill_transmit_locked(port);
        if (ring_available(&port->transmit) == 0)
            rdf_event_reset(port->writable);
        rdf_spin_unlock_irqrestore(&port->lock, state);
        if ((wake & WAKE_WRITABLE) != 0)
            rdf_event_signal(port->writable);
        if ((wake & WAKE_DRAINED) != 0)
            rdf_event_signal(port->drained);
        return RDF_OK;
    }

    size_t offset = 0;
    while (offset < length) {
        uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
        size_t available = ring_available(&port->transmit);
        if (available != 0) {
            size_t chunk = length - offset;
            if (chunk > available)
                chunk = available;
            offset += ring_write(&port->transmit, data + offset, chunk);
            rdf_event_reset(port->drained);
            uint32_t wake = fill_transmit_locked(port);
            if (ring_available(&port->transmit) == 0)
                rdf_event_reset(port->writable);
            rdf_spin_unlock_irqrestore(&port->lock, state);
            if ((wake & WAKE_WRITABLE) != 0)
                rdf_event_signal(port->writable);
            if ((wake & WAKE_DRAINED) != 0)
                rdf_event_signal(port->drained);
            continue;
        }
        rdf_event_reset(port->writable);
        rdf_spin_unlock_irqrestore(&port->lock, state);
        rdf_event_wait(port->writable);
    }
    return RDF_OK;
}

static int32_t console_configure(void *context, const struct rdf_serial_framing *framing)
{
    struct port *port = (struct port *)context;
    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    int configured = configure_locked(port, framing, 0);
    rdf_spin_unlock_irqrestore(&port->lock, state);
    return configured != 0 ? RDF_OK : RDF_EINVAL;
}

static int32_t console_flush(void *context)
{
    struct port *port = (struct port *)context;
    for (;;) {
        uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
        if (port->transmit.length == 0) {
            rdf_spin_unlock_irqrestore(&port->lock, state);
            break;
        }
        rdf_event_reset(port->drained);
        rdf_spin_unlock_irqrestore(&port->lock, state);
        rdf_event_wait(port->drained);
    }
    while ((port_read(port, REG_LINE_STATUS) & LSR_TRANSMIT_IDLE) == 0)
        rdf_sleep_ns(UINT64_C(50000));
    return RDF_OK;
}

static int32_t console_flush_input(void *context)
{
    struct port *port = (struct port *)context;
    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    port->receive.head = 0;
    port->receive.length = 0;
    port_write(port, REG_FIFO_CONTROL, UINT8_C(0x03));
    rdf_event_reset(port->readable);
    rdf_spin_unlock_irqrestore(&port->lock, state);
    return RDF_OK;
}

static int32_t console_flush_output(void *context)
{
    struct port *port = (struct port *)context;
    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    port->transmit.head = 0;
    port->transmit.length = 0;
    set_ier_locked(port, (uint8_t)(port->ier & ~IER_TRANSMIT));
    port_write(port, REG_FIFO_CONTROL, UINT8_C(0x05));
    rdf_spin_unlock_irqrestore(&port->lock, state);
    rdf_event_signal(port->writable);
    rdf_event_signal(port->drained);
    return RDF_OK;
}

static int64_t console_queued_output(void *context)
{
    struct port *port = (struct port *)context;
    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    size_t queued = port->interrupts_enabled != 0 ? port->transmit.length : 0;
    rdf_spin_unlock_irqrestore(&port->lock, state);
    return (int64_t)queued;
}

static int32_t console_writable(void *context)
{
    struct port *port = (struct port *)context;
    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    int writable = port->interrupts_enabled != 0
                       ? ring_available(&port->transmit) != 0
                       : (port_read(port, REG_LINE_STATUS) & LSR_TRANSMIT_EMPTY) != 0;
    rdf_spin_unlock_irqrestore(&port->lock, state);
    return writable;
}

static int32_t console_break(void *context, uint64_t duration_ms)
{
    struct port *port = (struct port *)context;

    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    port_write(port, REG_LINE_CONTROL, (uint8_t)(port_read(port, REG_LINE_CONTROL) | LCR_BREAK));
    rdf_spin_unlock_irqrestore(&port->lock, state);

    if (duration_ms == 0)
        duration_ms = 250;
    rdf_sleep_ns(duration_ms * UINT64_C(1000000));

    state = rdf_spin_lock_irqsave(&port->lock);
    port_write(port, REG_LINE_CONTROL, (uint8_t)(port_read(port, REG_LINE_CONTROL) & ~LCR_BREAK));
    rdf_spin_unlock_irqrestore(&port->lock, state);
    return RDF_OK;
}

/* --- Binding ------------------------------------------------------------ */

static struct port *claim_port(void)
{
    struct port *port = NULL;
    uintptr_t state = rdf_spin_lock_irqsave(&allocation_lock);
    if (port_count < MAX_PORTS)
        port = &ports[port_count++];
    rdf_spin_unlock_irqrestore(&allocation_lock, state);
    return port;
}

static int32_t map_registers(struct port *port, struct rdf_device *device)
{
    uint64_t start = 0;
    uint64_t size = 0;

    if (rdf_device_resource(device, RDF_RES_MEM, 0, &start, &size, NULL) == RDF_OK) {
        uint64_t value = 0;
        port->memory_mapped = 1;
        port->shift = rdf_device_u64(device, "reg-shift", &value) == RDF_OK ? (uint32_t)value : 0;
        port->width = rdf_device_u64(device, "reg-io-width", &value) == RDF_OK ? (uint32_t)value
                                                                              : 1;
        if (port->shift > 8 || (port->width != 1 && port->width != 2 && port->width != 4))
            return RDF_EINVAL;
        int32_t status = rdf_mmio_map(start, (size_t)size, RDF_MMIO_DEVICE, &port->window);
        if (status != RDF_OK)
            return status;
        port->mapped = 1;
        return RDF_OK;
    }

    if (rdf_device_resource(device, RDF_RES_IO, 0, &start, &size, NULL) == RDF_OK) {
        if (RDF_HAVE_PORT_IO == 0 || start > UINT16_MAX)
            return RDF_ENOTSUP;
        port->memory_mapped = 0;
        port->io_base = (uint16_t)start;
        return RDF_OK;
    }
    return RDF_ENOENT;
}

static void enable_interrupts(struct port *port, struct rdf_device *device)
{
    int32_t status = rdf_irq_of_device(device, 0, &port->virq);
    if (status != RDF_OK)
        return;

    status = rdf_irq_request(device, port->virq, "uart", RDF_IRQ_SHARED, port_interrupt, port,
                             &port->irq);
    if (status != RDF_OK) {
        RDF_WARN("%s: interrupt %u unavailable, falling back to polling",
                 rdf_device_name(device), port->virq);
        return;
    }

    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    if (drain_receive_locked(port) != 0)
        rdf_event_signal(port->readable);
    port_read(port, REG_LINE_STATUS);
    port_read(port, REG_MODEM_STATUS);
    port->interrupts_enabled = 1;
    set_ier_locked(port, IER_RX);
    rdf_spin_unlock_irqrestore(&port->lock, state);
}

static int32_t uart_probe(void *context, struct rdf_device *device, uintptr_t match_data)
{
    struct port *port;
    uint64_t value = 0;
    rdf_devnode_t root = 0;
    char name[16];
    int32_t status;

    (void)context;
    (void)match_data;

    port = claim_port();
    if (port == NULL)
        return RDF_ENOSPC;

    rdf_spin_init(&port->lock);
    port->clock = rdf_device_u64(device, "clock-frequency", &value) == RDF_OK && value != 0
                      ? (uint32_t)value
                      : DEFAULT_CLOCK;
    port->receive.bytes = port->receive_storage;
    port->receive.capacity = RX_CAPACITY;
    port->transmit.bytes = port->transmit_storage;
    port->transmit.capacity = TX_CAPACITY;

    status = map_registers(port, device);
    if (status != RDF_OK)
        return status;
    if (!detect(port)) {
        if (port->mapped != 0)
            rdf_mmio_unmap(&port->window);
        return RDF_ENODEV;
    }

    status = rdf_event_create(&port->readable);
    if (status == RDF_OK)
        status = rdf_event_create(&port->writable);
    if (status == RDF_OK)
        status = rdf_event_create(&port->drained);
    if (status != RDF_OK)
        goto fail;
    rdf_event_signal(port->writable);
    rdf_event_signal(port->drained);

    /*
     * Interrupts are an optimization here: if no controller has claimed this
     * line the port still works by polling, which is what keeps a console
     * available on platforms with no interrupt controller driver loaded.
     */
    enable_interrupts(port, device);

    status = rdf_devfs_root(&root);
    if (status != RDF_OK)
        goto fail;

    if (rdf_device_u64(device, "index", &value) != RDF_OK)
        value = (uint64_t)(port - ports);
    rdf_snprintf(name, sizeof(name), "ttyS%u", (uint32_t)value);

    struct rdf_console_ops operations = {
        .size = sizeof(operations),
        .flags = 0,
        .context = port,
        .try_read = console_try_read,
        .read = console_read,
        .write = console_write,
        .configure = console_configure,
        .flush = console_flush,
        .flush_input = console_flush_input,
        .flush_output = console_flush_output,
        .send_break = console_break,
        .writable = console_writable,
        .queued_output = console_queued_output,
        .readable_event = port->interrupts_enabled != 0 ? port->readable : 0,
        .writable_event = port->interrupts_enabled != 0 ? port->writable : 0,
    };
    status = rdf_tty_register(device, root, name, 0660, 115200, &operations, &port->tty);
    if (status != RDF_OK)
        goto fail;

    rdf_device_set_data(device, port);
    RDF_INFO("%s: %s at %s interrupt", rdf_device_name(device), name,
             port->interrupts_enabled != 0 ? "with" : "without");
    return RDF_OK;

fail:
    if (port->irq != NULL) {
        rdf_irq_release(port->irq);
        port->irq = NULL;
    }
    if (port->readable != 0) {
        rdf_event_destroy(port->readable);
        port->readable = 0;
    }
    if (port->writable != 0) {
        rdf_event_destroy(port->writable);
        port->writable = 0;
    }
    if (port->drained != 0) {
        rdf_event_destroy(port->drained);
        port->drained = 0;
    }
    if (port->mapped != 0) {
        rdf_mmio_unmap(&port->window);
        port->mapped = 0;
    }
    return status;
}

static void uart_remove(void *context, struct rdf_device *device)
{
    struct port *port = (struct port *)rdf_device_data(device);
    (void)context;
    if (port == NULL)
        return;

    if (port->tty != NULL) {
        rdf_tty_unregister(port->tty);
        port->tty = NULL;
    }

    uintptr_t state = rdf_spin_lock_irqsave(&port->lock);
    port_write(port, REG_INTERRUPT_ENABLE, 0);
    port->ier = 0;
    port->interrupts_enabled = 0;
    rdf_spin_unlock_irqrestore(&port->lock, state);

    if (port->irq != NULL) {
        rdf_irq_release(port->irq);
        port->irq = NULL;
    }
    if (port->readable != 0) {
        rdf_event_destroy(port->readable);
        port->readable = 0;
    }
    if (port->writable != 0) {
        rdf_event_destroy(port->writable);
        port->writable = 0;
    }
    if (port->drained != 0) {
        rdf_event_destroy(port->drained);
        port->drained = 0;
    }
    if (port->mapped != 0) {
        rdf_mmio_unmap(&port->window);
        port->mapped = 0;
    }
    rdf_device_set_data(device, NULL);
}

static const struct rdf_match matches[] = {
    RDF_MATCH_COMPATIBLE("ns16550a"),
    RDF_MATCH_COMPATIBLE("ns16550"),
    RDF_MATCH_COMPATIBLE("uart8250"),
    RDF_MATCH_COMPATIBLE("snps,dw-apb-uart"),
    RDF_MATCH_COMPATIBLE("pnp,16550a"),
};

static struct rdf_driver_def driver = {
    .size = sizeof(struct rdf_driver_def),
    .name = "uart8250",
    .matches = matches,
    .match_count = RDF_COUNT(matches),
    .probe = uart_probe,
    .remove = uart_remove,
};

static const struct rdf_driver *registration;

static int32_t uart_init(struct rdf_module *self)
{
    int32_t status;
    (void)self;

    status = rdf_bus_find(RDF_BUS_PLATFORM, &driver.bus);
    if (status != RDF_OK)
        return status;
    return rdf_driver_register(&driver, &registration);
}

static void uart_exit(struct rdf_module *self)
{
    (void)self;
    if (registration != NULL)
        rdf_driver_unregister(registration);
}

RDF_MODULE("uart8250", "8250 and 16550 compatible serial ports", uart_init, uart_exit)
