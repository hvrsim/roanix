#include <roanix/driver.h>

#define PTY_SLOT_COUNT 64u
#define PTY_BUFFER_CAPACITY (64u * 1024u)
#define PTY_STORAGE_SIZE (PTY_BUFFER_CAPACITY * 2u)
#define PTY_STORAGE_ALIGNMENT 64u

#define TIOCGPTN UINT64_C(0x80045430)
#define TIOCSPTLCK UINT64_C(0x40045431)
#define FIONREAD UINT64_C(0x541b)

enum slot_state {
    SLOT_FREE,
    SLOT_RESERVED,
    SLOT_ACTIVE,
};

struct byte_ring {
    uint8_t *bytes;
    size_t head;
    size_t length;
};

struct pty_slot {
    struct rdf_spinlock lock;
    enum slot_state state;
    uint32_t index;
    uint32_t slave_opens;
    uint8_t master_open;
    uint8_t slave_seen;
    uint8_t locked;
    uint8_t *storage;
    struct byte_ring to_master;
    struct byte_ring to_slave;
    rdf_event_t master_state;
    rdf_event_t master_readable;
    rdf_event_t master_writable;
    rdf_event_t slave_readable;
    rdf_event_t slave_writable;
};

struct cleanup_resources {
    uint8_t *storage;
};

static struct pty_slot slots[PTY_SLOT_COUNT];

static size_t ring_available(const struct byte_ring *ring)
{
    return PTY_BUFFER_CAPACITY - ring->length;
}

static size_t ring_read(
    struct byte_ring *ring,
    uint8_t *output,
    size_t length)
{
    if (length > ring->length)
        length = ring->length;
    size_t first = length;
    if (first > PTY_BUFFER_CAPACITY - ring->head)
        first = PTY_BUFFER_CAPACITY - ring->head;
    memcpy(output, ring->bytes + ring->head, first);
    if (length > first)
        memcpy(output + first, ring->bytes, length - first);
    ring->head = (ring->head + length) % PTY_BUFFER_CAPACITY;
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
    size_t tail = (ring->head + ring->length) % PTY_BUFFER_CAPACITY;
    size_t first = length;
    if (first > PTY_BUFFER_CAPACITY - tail)
        first = PTY_BUFFER_CAPACITY - tail;
    memcpy(ring->bytes + tail, input, first);
    if (length > first)
        memcpy(ring->bytes, input + first, length - first);
    ring->length += length;
    return length;
}

static struct pty_slot *slot_from_context(uintptr_t context)
{
    uintptr_t first = (uintptr_t)&slots[0];
    uintptr_t end = (uintptr_t)&slots[PTY_SLOT_COUNT];
    if (context < first ||
        context >= end ||
        (context - first) % sizeof(struct pty_slot) != 0)
        return NULL;
    return (struct pty_slot *)context;
}

static int take_cleanup_locked(
    struct pty_slot *slot,
    struct cleanup_resources *resources)
{
    if (slot->state != SLOT_ACTIVE ||
        slot->master_open != 0 ||
        slot->slave_opens != 0)
        return 0;

    *resources = (struct cleanup_resources){
        .storage = slot->storage,
    };
    slot->state = SLOT_FREE;
    slot->locked = 1;
    slot->storage = NULL;
    slot->to_master = (struct byte_ring){0};
    slot->to_slave = (struct byte_ring){0};
    return 1;
}

static void release_resources(const struct cleanup_resources *resources)
{
    if (resources->storage != NULL) {
        rdf_free(
            resources->storage,
            PTY_STORAGE_SIZE,
            PTY_STORAGE_ALIGNMENT);
    }
}

static int activate_slot(struct pty_slot *slot)
{
    uint8_t *storage = rdf_zalloc(
        PTY_STORAGE_SIZE,
        PTY_STORAGE_ALIGNMENT);
    if (storage == NULL) {
        return 0;
    }
    (void)rdf_event_reset(slot->master_state);
    (void)rdf_event_reset(slot->master_readable);
    (void)rdf_event_reset(slot->slave_readable);

    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    if (slot->state != SLOT_RESERVED) {
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
        release_resources(&(struct cleanup_resources){
            .storage = storage,
        });
        return 0;
    }
    slot->state = SLOT_ACTIVE;
    slot->slave_opens = 0;
    slot->master_open = 1;
    slot->slave_seen = 0;
    slot->locked = 1;
    slot->storage = storage;
    slot->to_master = (struct byte_ring){
        .bytes = storage,
        .head = 0,
        .length = 0,
    };
    slot->to_slave = (struct byte_ring){
        .bytes = storage + PTY_BUFFER_CAPACITY,
        .head = 0,
        .length = 0,
    };
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    (void)rdf_event_signal(slot->master_writable);
    (void)rdf_event_signal(slot->slave_writable);
    return 1;
}

static int32_t master_open(
    void *context,
    uint32_t flags,
    uintptr_t *out_file_context)
{
    (void)context;
    (void)flags;
    if (out_file_context == NULL)
        return RDF_EINVAL;

    for (size_t index = 0; index < PTY_SLOT_COUNT; ++index) {
        struct pty_slot *slot = &slots[index];
        uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_FREE) {
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            continue;
        }
        slot->state = SLOT_RESERVED;
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);

        if (activate_slot(slot)) {
            *out_file_context = (uintptr_t)slot;
            return RDF_OK;
        }

        irq_state = rdf_spin_lock_irqsave(&slot->lock);
        if (slot->state == SLOT_RESERVED)
            slot->state = SLOT_FREE;
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
        return RDF_ENOMEM;
    }
    return RDF_ENOSPC;
}

static void master_close(
    void *context,
    uintptr_t file_context,
    uint32_t flags)
{
    (void)context;
    (void)flags;
    struct pty_slot *slot = slot_from_context(file_context);
    if (slot == NULL)
        return;

    struct cleanup_resources resources = {0};
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    if (slot->state == SLOT_ACTIVE && slot->master_open != 0) {
        slot->master_open = 0;
        (void)rdf_event_signal(slot->master_writable);
        (void)rdf_event_signal(slot->slave_readable);
        (void)rdf_event_signal(slot->slave_writable);
        (void)take_cleanup_locked(slot, &resources);
    }
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    release_resources(&resources);
}

static int64_t master_read(
    void *context,
    uintptr_t file_context,
    uint64_t offset,
    uint8_t *data,
    size_t length,
    uint32_t flags)
{
    (void)context;
    (void)offset;
    struct pty_slot *slot = slot_from_context(file_context);
    if (slot == NULL || (data == NULL && length != 0))
        return RDF_EINVAL;
    if (length == 0)
        return 0;

    for (;;) {
        uintptr_t writable = 0;
        uintptr_t readable = 0;
        uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_ACTIVE || slot->master_open == 0) {
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            return RDF_EIO;
        }
        if (slot->to_master.length != 0) {
            size_t read = ring_read(&slot->to_master, data, length);
            writable = slot->master_writable;
            if (slot->to_master.length == 0)
                (void)rdf_event_reset(slot->master_readable);
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            (void)rdf_event_signal(writable);
            return (int64_t)read;
        }
        if (slot->slave_opens == 0) {
            if (slot->slave_seen == 0) {
                if ((flags & RDF_OPEN_NONBLOCK) != 0) {
                    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
                    return RDF_EAGAIN;
                }
                readable = slot->master_state;
                (void)rdf_event_reset(readable);
                rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
                (void)rdf_event_wait(readable);
                continue;
            }
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            return RDF_EIO;
        }
        if ((flags & RDF_OPEN_NONBLOCK) != 0) {
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            return RDF_EAGAIN;
        }
        readable = slot->master_readable;
        (void)rdf_event_reset(readable);
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
        (void)rdf_event_wait(readable);
    }
}

static int64_t master_write(
    void *context,
    uintptr_t file_context,
    uint64_t offset,
    const uint8_t *data,
    size_t length,
    uint32_t flags)
{
    (void)context;
    (void)offset;
    struct pty_slot *slot = slot_from_context(file_context);
    if (slot == NULL || (data == NULL && length != 0))
        return RDF_EINVAL;
    if (length == 0)
        return 0;

    for (;;) {
        uintptr_t writable = 0;
        uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_ACTIVE ||
            slot->master_open == 0) {
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            return RDF_EIO;
        }
        if (slot->slave_opens == 0) {
            if (slot->slave_seen != 0) {
                rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
                return RDF_EIO;
            }
            if ((flags & RDF_OPEN_NONBLOCK) != 0) {
                rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
                return RDF_EAGAIN;
            }
            writable = slot->master_state;
            (void)rdf_event_reset(writable);
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            (void)rdf_event_wait(writable);
            continue;
        }
        if (ring_available(&slot->to_slave) != 0) {
            size_t written = ring_write(&slot->to_slave, data, length);
            uintptr_t readable = slot->slave_readable;
            if (ring_available(&slot->to_slave) == 0)
                (void)rdf_event_reset(slot->slave_writable);
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            (void)rdf_event_signal(readable);
            return (int64_t)written;
        }
        if ((flags & RDF_OPEN_NONBLOCK) != 0) {
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            return RDF_EAGAIN;
        }
        writable = slot->slave_writable;
        (void)rdf_event_reset(writable);
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
        (void)rdf_event_wait(writable);
    }
}

static int64_t master_poll(
    void *context,
    uintptr_t file_context,
    uint64_t offset,
    uint16_t events,
    uint32_t flags)
{
    (void)context;
    (void)offset;
    (void)flags;
    struct pty_slot *slot = slot_from_context(file_context);
    if (slot == NULL)
        return RDF_POLL_NVAL;

    uint16_t ready = 0;
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    if (slot->state != SLOT_ACTIVE || slot->master_open == 0)
        ready = RDF_POLL_HUP;
    else {
        if (slot->to_master.length != 0)
            ready |= events & (RDF_POLL_IN | RDF_POLL_RDNORM);
        if (slot->slave_seen != 0 && slot->slave_opens == 0)
            ready |= RDF_POLL_HUP;
        else if (slot->slave_opens != 0 &&
                 ring_available(&slot->to_slave) != 0)
            ready |= events & (RDF_POLL_OUT | RDF_POLL_WRNORM);
    }
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    return ready;
}

static int64_t master_ioctl(
    void *context,
    uintptr_t file_context,
    const struct rdf_ioctl_identity *identity,
    uint64_t request,
    uint64_t value,
    uint8_t *argument,
    size_t argument_len)
{
    (void)context;
    (void)identity;
    (void)value;
    struct pty_slot *slot = slot_from_context(file_context);
    if (slot == NULL || argument == NULL || argument_len != 4)
        return RDF_EINVAL;

    if (request == TIOCGPTN) {
        uint32_t index = slot->index;
        memcpy(argument, &index, sizeof(index));
        return RDF_OK;
    }
    if (request == TIOCSPTLCK) {
        int32_t locked = 0;
        memcpy(&locked, argument, sizeof(locked));
        uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_ACTIVE || slot->master_open == 0) {
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            return RDF_EIO;
        }
        slot->locked = locked != 0;
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
        return RDF_OK;
    }
    if (request == FIONREAD) {
        int32_t pending = 0;
        uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
        if (slot->to_master.length > INT32_MAX)
            pending = INT32_MAX;
        else
            pending = (int32_t)slot->to_master.length;
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
        memcpy(argument, &pending, sizeof(pending));
        return RDF_OK;
    }
    return RDF_ENOTTY;
}

static uintptr_t master_readable_event(
    void *context,
    uintptr_t file_context)
{
    (void)context;
    struct pty_slot *slot = slot_from_context(file_context);
    return slot != NULL ? slot->master_readable : 0;
}

static uintptr_t master_writable_event(
    void *context,
    uintptr_t file_context)
{
    (void)context;
    struct pty_slot *slot = slot_from_context(file_context);
    return slot != NULL ? slot->slave_writable : 0;
}

static uintptr_t master_hangup_event(
    void *context,
    uintptr_t file_context)
{
    return master_readable_event(context, file_context);
}

static int32_t slave_open(void *context)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL)
        return RDF_EINVAL;
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    if (slot->state != SLOT_ACTIVE ||
        slot->master_open == 0 ||
        slot->locked != 0) {
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
        return RDF_EIO;
    }
    slot->slave_seen = 1;
    ++slot->slave_opens;
    if (slot->to_master.length == 0)
        (void)rdf_event_reset(slot->master_readable);
    (void)rdf_event_signal(slot->master_state);
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    return RDF_OK;
}

static void slave_close(void *context)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL)
        return;

    struct cleanup_resources resources = {0};
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    if (slot->state == SLOT_ACTIVE && slot->slave_opens != 0) {
        --slot->slave_opens;
        if (slot->slave_opens == 0) {
            slot->to_slave.head = 0;
            slot->to_slave.length = 0;
            (void)rdf_event_reset(slot->slave_readable);
        }
        (void)rdf_event_signal(slot->master_state);
        (void)rdf_event_signal(slot->master_readable);
        (void)rdf_event_signal(slot->slave_writable);
        (void)take_cleanup_locked(slot, &resources);
    }
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    release_resources(&resources);
}

static int64_t slave_read(
    void *context,
    uint8_t *output,
    size_t length)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL || (output == NULL && length != 0))
        return RDF_EINVAL;
    if (length == 0)
        return 0;
    uintptr_t writable = 0;
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    size_t read = 0;
    if (slot->state == SLOT_ACTIVE && slot->to_slave.length != 0) {
        read = ring_read(&slot->to_slave, output, length);
        writable = slot->slave_writable;
        if (slot->to_slave.length == 0)
            (void)rdf_event_reset(slot->slave_readable);
    }
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    if (writable != 0)
        (void)rdf_event_signal(writable);
    return (int64_t)read;
}

static int32_t slave_try_read(void *context, uint8_t *output)
{
    return slave_read(context, output, 1) == 1;
}

static int32_t slave_write(
    void *context,
    const uint8_t *data,
    size_t length,
    uint8_t nonblocking)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL || (data == NULL && length != 0))
        return RDF_EINVAL;
    if (length == 0)
        return RDF_OK;

    size_t offset = 0;
    while (offset < length) {
        uintptr_t readable = 0;
        uintptr_t writable = 0;
        uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_ACTIVE || slot->master_open == 0) {
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            return RDF_EIO;
        }
        if (nonblocking != 0 &&
            ring_available(&slot->to_master) < length - offset) {
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            return RDF_EAGAIN;
        }
        if (ring_available(&slot->to_master) != 0) {
            offset += ring_write(
                &slot->to_master,
                data + offset,
                length - offset);
            readable = slot->master_readable;
            if (ring_available(&slot->to_master) == 0)
                (void)rdf_event_reset(slot->master_writable);
            rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
            (void)rdf_event_signal(readable);
            continue;
        }
        writable = slot->master_writable;
        (void)rdf_event_reset(writable);
        rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
        (void)rdf_event_wait(writable);
    }
    return RDF_OK;
}

static int32_t slave_hung_up(void *context)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL)
        return 1;
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    int hung_up = slot->state != SLOT_ACTIVE || slot->master_open == 0;
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    return hung_up;
}

static int32_t slave_writable(void *context)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL)
        return 0;
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    int writable = slot->state == SLOT_ACTIVE &&
                   slot->master_open != 0 &&
                   ring_available(&slot->to_master) != 0;
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    return writable;
}

static int32_t slave_flush_input(void *context)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL)
        return RDF_EINVAL;
    uintptr_t writable = 0;
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    if (slot->state == SLOT_ACTIVE) {
        slot->to_slave.head = 0;
        slot->to_slave.length = 0;
        (void)rdf_event_reset(slot->slave_readable);
        writable = slot->slave_writable;
    }
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    if (writable != 0)
        (void)rdf_event_signal(writable);
    return RDF_OK;
}

static int32_t slave_flush_output(void *context)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL)
        return RDF_EINVAL;
    uintptr_t writable = 0;
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    if (slot->state == SLOT_ACTIVE) {
        slot->to_master.head = 0;
        slot->to_master.length = 0;
        if (slot->slave_opens != 0)
            (void)rdf_event_reset(slot->master_readable);
        writable = slot->master_writable;
    }
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    if (writable != 0)
        (void)rdf_event_signal(writable);
    return RDF_OK;
}

static int64_t slave_queued_output(void *context)
{
    struct pty_slot *slot = slot_from_context((uintptr_t)context);
    if (slot == NULL)
        return 0;
    uintptr_t irq_state = rdf_spin_lock_irqsave(&slot->lock);
    size_t queued = slot->state == SLOT_ACTIVE
        ? slot->to_master.length
        : 0;
    rdf_spin_unlock_irqrestore(&slot->lock, irq_state);
    return queued > INT64_MAX ? INT64_MAX : (int64_t)queued;
}

static const struct rdf_node_ops master_operations = {
    .size = sizeof(master_operations),
    .context = NULL,
    .open = master_open,
    .close = master_close,
    .initial_offset = NULL,
    .read = master_read,
    .write = master_write,
    .size_bytes = NULL,
    .sync = NULL,
    .poll = master_poll,
    .ioctl = master_ioctl,
    .readable_event = master_readable_event,
    .writable_event = master_writable_event,
    .hangup_event = master_hangup_event,
};

static size_t decimal_name(uint32_t value, char *output)
{
    char reverse[10];
    size_t length = 0;
    do {
        reverse[length++] = (char)('0' + value % 10u);
        value /= 10u;
    } while (value != 0);
    for (size_t index = 0; index < length; ++index)
        output[index] = reverse[length - index - 1u];
    return length;
}

static int32_t publish_slave(
    rdf_devnode_t pts,
    struct pty_slot *slot)
{
    char name[11];
    size_t name_length = decimal_name(slot->index, name);
    name[name_length] = '\0';

    struct rdf_console_ops operations = {
        .size = sizeof(operations),
        .flags = RDF_CONSOLE_RESET_ON_LAST_CLOSE,
        .context = slot,
        .open = slave_open,
        .close = slave_close,
        .try_read = slave_try_read,
        .write = slave_write,
        .configure = NULL,
        .flush = NULL,
        .send_break = NULL,
        .writable = slave_writable,
        .hung_up = slave_hung_up,
        .destroy = NULL,
        .read = slave_read,
        .flush_input = slave_flush_input,
        .flush_output = slave_flush_output,
        .queued_output = slave_queued_output,
        .readable_event = slot->slave_readable,
        .writable_event = slot->master_writable,
        .hangup_event = slot->slave_readable,
    };
    struct rdf_tty *tty = NULL;
    return rdf_tty_register(NULL, pts, name, 0620, 38400, &operations, &tty);
}

static int32_t pty_init(struct rdf_module *self)
{
    (void)self;

    rdf_devnode_t devfs = 0;
    rdf_devnode_t pts = 0;
    rdf_devnode_t ptmx_node = 0;
    int32_t status = rdf_devfs_root(&devfs);
    if (status != RDF_OK)
        return status;
    status = rdf_devfs_mkdir(
        devfs,
        "pts",
        0755,
        &pts);
    if (status != RDF_OK)
        return status;

    for (size_t index = 0; index < PTY_SLOT_COUNT; ++index) {
        struct pty_slot *slot = &slots[index];
        rdf_spin_init(&slot->lock);
        slot->state = SLOT_FREE;
        slot->index = (uint32_t)index;
        slot->locked = 1;
        status = rdf_event_create(&slot->master_state);
        if (status != RDF_OK)
            goto fail_events;
        status = rdf_event_create(&slot->master_readable);
        if (status != RDF_OK)
            goto fail_events;
        status = rdf_event_create(&slot->master_writable);
        if (status != RDF_OK)
            goto fail_events;
        status = rdf_event_create(&slot->slave_readable);
        if (status != RDF_OK)
            goto fail_events;
        status = rdf_event_create(&slot->slave_writable);
        if (status != RDF_OK)
            goto fail_events;
        status = publish_slave(pts, slot);
        if (status != RDF_OK)
            goto fail_events;
    }

    status = rdf_devfs_create(
        NULL,
        devfs,
        "ptmx",
        RDF_NODE_CHARACTER,
        0666,
        &master_operations,
        &ptmx_node);
    if (status != RDF_OK)
        goto fail_events;

    RDF_INFO("pty: published /dev/ptmx and /dev/pts");
    return RDF_OK;

fail_events:
    for (size_t index = 0; index < PTY_SLOT_COUNT; ++index) {
        struct pty_slot *slot = &slots[index];
        if (slot->master_state != 0) {
            (void)rdf_event_destroy(slot->master_state);
            slot->master_state = 0;
        }
        if (slot->master_readable != 0) {
            (void)rdf_event_destroy(slot->master_readable);
            slot->master_readable = 0;
        }
        if (slot->master_writable != 0) {
            (void)rdf_event_destroy(slot->master_writable);
            slot->master_writable = 0;
        }
        if (slot->slave_readable != 0) {
            (void)rdf_event_destroy(slot->slave_readable);
            slot->slave_readable = 0;
        }
        if (slot->slave_writable != 0) {
            (void)rdf_event_destroy(slot->slave_writable);
            slot->slave_writable = 0;
        }
    }
    return status;
}

static void pty_exit(struct rdf_module *self)
{
    (void)self;
    for (size_t index = 0; index < PTY_SLOT_COUNT; ++index) {
        struct pty_slot *slot = &slots[index];
        if (slot->master_state != 0)
            (void)rdf_event_destroy(slot->master_state);
        if (slot->master_readable != 0)
            (void)rdf_event_destroy(slot->master_readable);
        if (slot->master_writable != 0)
            (void)rdf_event_destroy(slot->master_writable);
        if (slot->slave_readable != 0)
            (void)rdf_event_destroy(slot->slave_readable);
        if (slot->slave_writable != 0)
            (void)rdf_event_destroy(slot->slave_writable);
    }
}

RDF_MODULE("pty", "Pseudo-terminal (ptmx/pts) driver", pty_init, pty_exit);
