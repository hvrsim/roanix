#include <devkit/devkit.h>

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
    struct dk_spinlock lock;
    enum slot_state state;
    uint32_t index;
    uint32_t slave_opens;
    uint8_t master_open;
    uint8_t slave_seen;
    uint8_t locked;
    uint8_t *storage;
    struct byte_ring to_master;
    struct byte_ring to_slave;
    uintptr_t master_readable;
    uintptr_t master_writable;
    uintptr_t slave_writable;
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
        (void)dk_deallocate(
            resources->storage,
            PTY_STORAGE_SIZE,
            PTY_STORAGE_ALIGNMENT);
    }
}

static int activate_slot(struct pty_slot *slot)
{
    uint8_t *storage = dk_allocate_zeroed(
        PTY_STORAGE_SIZE,
        PTY_STORAGE_ALIGNMENT);
    if (storage == NULL) {
        return 0;
    }
    (void)dk_event_reset(slot->master_readable);
    (void)dk_event_reset(slot->master_writable);
    (void)dk_event_reset(slot->slave_writable);

    uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
    if (slot->state != SLOT_RESERVED) {
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);
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
    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
    return 1;
}

static int32_t master_open(
    uintptr_t context,
    uint32_t flags,
    uintptr_t *out_file_context)
{
    (void)context;
    (void)flags;
    if (out_file_context == NULL)
        return DK_EINVAL;

    for (size_t index = 0; index < PTY_SLOT_COUNT; ++index) {
        struct pty_slot *slot = &slots[index];
        uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_FREE) {
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            continue;
        }
        slot->state = SLOT_RESERVED;
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);

        if (activate_slot(slot)) {
            *out_file_context = (uintptr_t)slot;
            return DK_OK;
        }

        irq_state = dk_spin_lock_irqsave(&slot->lock);
        if (slot->state == SLOT_RESERVED)
            slot->state = SLOT_FREE;
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);
        return DK_ENOMEM;
    }
    return DK_ENOSPC;
}

static void master_close(
    uintptr_t context,
    uintptr_t file_context,
    uint32_t flags)
{
    (void)context;
    (void)flags;
    struct pty_slot *slot = slot_from_context(file_context);
    if (slot == NULL)
        return;

    struct cleanup_resources resources = {0};
    uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
    if (slot->state == SLOT_ACTIVE && slot->master_open != 0) {
        slot->master_open = 0;
        (void)dk_event_signal(slot->master_writable);
        (void)dk_event_signal(slot->slave_writable);
        (void)take_cleanup_locked(slot, &resources);
    }
    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
    release_resources(&resources);
}

static int64_t master_read(
    uintptr_t context,
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
        return DK_EINVAL;
    if (length == 0)
        return 0;

    for (;;) {
        uintptr_t writable = 0;
        uintptr_t readable = 0;
        uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_ACTIVE || slot->master_open == 0) {
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return DK_EIO;
        }
        if (slot->to_master.length != 0) {
            size_t read = ring_read(&slot->to_master, data, length);
            writable = slot->master_writable;
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            (void)dk_event_signal(writable);
            return (int64_t)read;
        }
        if (slot->slave_opens == 0) {
            if (slot->slave_seen == 0) {
                if ((flags & DK_OPEN_NONBLOCK) != 0) {
                    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
                    return DK_EAGAIN;
                }
                readable = slot->master_readable;
                (void)dk_event_reset(readable);
                dk_spin_unlock_irqrestore(&slot->lock, irq_state);
                (void)dk_event_wait(readable);
                continue;
            }
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return DK_EIO;
        }
        if ((flags & DK_OPEN_NONBLOCK) != 0) {
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return DK_EAGAIN;
        }
        readable = slot->master_readable;
        (void)dk_event_reset(readable);
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);
        (void)dk_event_wait(readable);
    }
}

static int64_t master_write(
    uintptr_t context,
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
        return DK_EINVAL;
    if (length == 0)
        return 0;

    for (;;) {
        uintptr_t writable = 0;
        uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_ACTIVE ||
            slot->master_open == 0) {
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return DK_EIO;
        }
        if (slot->slave_opens == 0) {
            if (slot->slave_seen != 0) {
                dk_spin_unlock_irqrestore(&slot->lock, irq_state);
                return DK_EIO;
            }
            if ((flags & DK_OPEN_NONBLOCK) != 0) {
                dk_spin_unlock_irqrestore(&slot->lock, irq_state);
                return DK_EAGAIN;
            }
            writable = slot->master_readable;
            (void)dk_event_reset(writable);
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            (void)dk_event_wait(writable);
            continue;
        }
        if (ring_available(&slot->to_slave) != 0) {
            size_t written = ring_write(&slot->to_slave, data, length);
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return (int64_t)written;
        }
        if ((flags & DK_OPEN_NONBLOCK) != 0) {
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return DK_EAGAIN;
        }
        writable = slot->slave_writable;
        (void)dk_event_reset(writable);
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);
        (void)dk_event_wait(writable);
    }
}

static int64_t master_poll(
    uintptr_t context,
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
        return DK_POLL_NVAL;

    uint16_t ready = 0;
    uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
    if (slot->state != SLOT_ACTIVE || slot->master_open == 0)
        ready = DK_POLL_HUP;
    else {
        if (slot->to_master.length != 0)
            ready |= events & (DK_POLL_IN | DK_POLL_RDNORM);
        if (slot->slave_seen != 0 && slot->slave_opens == 0)
            ready |= DK_POLL_HUP;
        else if (slot->slave_opens != 0 &&
                 ring_available(&slot->to_slave) != 0)
            ready |= events & (DK_POLL_OUT | DK_POLL_WRNORM);
    }
    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
    return ready;
}

static int64_t master_ioctl(
    uintptr_t context,
    uintptr_t file_context,
    uintptr_t process_id,
    int32_t process_group,
    int32_t session_id,
    uint8_t is_session_leader,
    uint64_t request,
    uint64_t value,
    uint8_t *argument,
    size_t argument_len)
{
    (void)context;
    (void)process_id;
    (void)process_group;
    (void)session_id;
    (void)is_session_leader;
    (void)value;
    struct pty_slot *slot = slot_from_context(file_context);
    if (slot == NULL || argument == NULL || argument_len != 4)
        return DK_EINVAL;

    if (request == TIOCGPTN) {
        uint32_t index = slot->index;
        memcpy(argument, &index, sizeof(index));
        return DK_OK;
    }
    if (request == TIOCSPTLCK) {
        int32_t locked = 0;
        memcpy(&locked, argument, sizeof(locked));
        uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_ACTIVE || slot->master_open == 0) {
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return DK_EIO;
        }
        slot->locked = locked != 0;
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);
        return DK_OK;
    }
    if (request == FIONREAD) {
        int32_t pending = 0;
        uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
        if (slot->to_master.length > INT32_MAX)
            pending = INT32_MAX;
        else
            pending = (int32_t)slot->to_master.length;
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);
        memcpy(argument, &pending, sizeof(pending));
        return DK_OK;
    }
    return DK_ENOTTY;
}

static int32_t slave_open(uintptr_t context)
{
    struct pty_slot *slot = slot_from_context(context);
    if (slot == NULL)
        return DK_EINVAL;
    uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
    if (slot->state != SLOT_ACTIVE ||
        slot->master_open == 0 ||
        slot->locked != 0) {
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);
        return DK_EIO;
    }
    slot->slave_seen = 1;
    ++slot->slave_opens;
    (void)dk_event_signal(slot->master_readable);
    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
    return DK_OK;
}

static void slave_close(uintptr_t context)
{
    struct pty_slot *slot = slot_from_context(context);
    if (slot == NULL)
        return;

    struct cleanup_resources resources = {0};
    uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
    if (slot->state == SLOT_ACTIVE && slot->slave_opens != 0) {
        --slot->slave_opens;
        if (slot->slave_opens == 0) {
            slot->to_slave.head = 0;
            slot->to_slave.length = 0;
        }
        (void)dk_event_signal(slot->master_readable);
        (void)dk_event_signal(slot->slave_writable);
        (void)take_cleanup_locked(slot, &resources);
    }
    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
    release_resources(&resources);
}

static int32_t slave_try_read(uintptr_t context, uint8_t *output)
{
    struct pty_slot *slot = slot_from_context(context);
    if (slot == NULL || output == NULL)
        return 0;
    uintptr_t writable = 0;
    uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
    int available = slot->state == SLOT_ACTIVE &&
                    slot->to_slave.length != 0;
    if (available) {
        (void)ring_read(&slot->to_slave, output, 1);
        writable = slot->slave_writable;
    }
    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
    if (writable != 0)
        (void)dk_event_signal(writable);
    return available;
}

static int32_t slave_write(
    uintptr_t context,
    const uint8_t *data,
    size_t length,
    uint8_t nonblocking)
{
    struct pty_slot *slot = slot_from_context(context);
    if (slot == NULL || (data == NULL && length != 0))
        return DK_EINVAL;
    if (length == 0)
        return DK_OK;

    size_t offset = 0;
    while (offset < length) {
        uintptr_t readable = 0;
        uintptr_t writable = 0;
        uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
        if (slot->state != SLOT_ACTIVE || slot->master_open == 0) {
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return DK_EIO;
        }
        if (nonblocking != 0 &&
            ring_available(&slot->to_master) < length - offset) {
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            return DK_EAGAIN;
        }
        if (ring_available(&slot->to_master) != 0) {
            offset += ring_write(
                &slot->to_master,
                data + offset,
                length - offset);
            readable = slot->master_readable;
            dk_spin_unlock_irqrestore(&slot->lock, irq_state);
            (void)dk_event_signal(readable);
            continue;
        }
        writable = slot->master_writable;
        (void)dk_event_reset(writable);
        dk_spin_unlock_irqrestore(&slot->lock, irq_state);
        (void)dk_event_wait(writable);
    }
    return DK_OK;
}

static int32_t slave_hung_up(uintptr_t context)
{
    struct pty_slot *slot = slot_from_context(context);
    if (slot == NULL)
        return 1;
    uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
    int hung_up = slot->state != SLOT_ACTIVE || slot->master_open == 0;
    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
    return hung_up;
}

static int32_t slave_writable(uintptr_t context)
{
    struct pty_slot *slot = slot_from_context(context);
    if (slot == NULL)
        return 0;
    uintptr_t irq_state = dk_spin_lock_irqsave(&slot->lock);
    int writable = slot->state == SLOT_ACTIVE &&
                   slot->master_open != 0 &&
                   ring_available(&slot->to_master) != 0;
    dk_spin_unlock_irqrestore(&slot->lock, irq_state);
    return writable;
}

static const struct dk_device_ops master_operations = {
    .size = sizeof(master_operations),
    .context = 0,
    .open = master_open,
    .close = master_close,
    .initial_offset = NULL,
    .read = master_read,
    .write = master_write,
    .size_bytes = NULL,
    .sync = NULL,
    .poll = master_poll,
    .ioctl = master_ioctl,
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
    dk_bus_t console,
    dk_devnode_t pts,
    struct pty_slot *slot)
{
    char name[10];
    size_t name_length = decimal_name(slot->index, name);
    char device_name[16] = "pty";
    memcpy(device_name + 3, name, name_length);
    size_t device_name_length = 3u + name_length;
    char path[24] = "/dev/pts/";
    memcpy(path + 9, name, name_length);
    size_t path_length = 9u + name_length;

    dk_device_t device = 0;
    dk_devnode_t node = 0;
    int32_t status = dk_device_create(
        console,
        (struct dk_slice){
            .data = (const uint8_t *)device_name,
            .len = device_name_length,
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
        .flags = DK_CONSOLE_RESET_ON_LAST_CLOSE,
        .context = (uintptr_t)slot,
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
    };
    return dk_console_create_tty(
        device,
        pts,
        (struct dk_slice){
            .data = (const uint8_t *)name,
            .len = name_length,
        },
        0620,
        (struct dk_slice){
            .data = (const uint8_t *)path,
            .len = path_length,
        },
        38400,
        &operations,
        &node);
}

static int32_t pty_start(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;

    dk_bus_t console = 0;
    dk_devnode_t devfs = 0;
    dk_bus_t bus = 0;
    dk_devnode_t pts = 0;
    dk_device_t ptmx_device = 0;
    dk_devnode_t ptmx_node = 0;
    int32_t status = dk_console_bus(&console);
    if (status != DK_OK)
        return status;
    status = dk_devfs_root(&devfs);
    if (status != DK_OK)
        return status;
    status = dk_bus_create(console, DK_SLICE_LITERAL("pty"), &bus);
    if (status != DK_OK)
        return status;
    status = dk_devfs_create_dir(
        devfs,
        DK_SLICE_LITERAL("pts"),
        0755,
        &pts);
    if (status != DK_OK)
        return status;

    for (size_t index = 0; index < PTY_SLOT_COUNT; ++index) {
        struct pty_slot *slot = &slots[index];
        atomic_flag_clear_explicit(&slot->lock.held, memory_order_relaxed);
        slot->state = SLOT_FREE;
        slot->index = (uint32_t)index;
        slot->locked = 1;
        status = dk_event_create(&slot->master_readable);
        if (status != DK_OK)
            goto fail_events;
        status = dk_event_create(&slot->master_writable);
        if (status != DK_OK)
            goto fail_events;
        status = dk_event_create(&slot->slave_writable);
        if (status != DK_OK)
            goto fail_events;
        status = publish_slave(console, pts, slot);
        if (status != DK_OK)
            goto fail_events;
    }

    status = dk_device_create(
        bus,
        DK_SLICE_LITERAL("ptmx"),
        &ptmx_device);
    if (status != DK_OK)
        goto fail_events;
    status = dk_node_set_property(
        ptmx_device,
        DK_PROPERTY_SUBSYSTEM,
        DK_SLICE_LITERAL("tty"));
    if (status != DK_OK)
        goto fail_events;
    status = dk_devfs_create_device(
        devfs,
        DK_SLICE_LITERAL("ptmx"),
        DK_DEVICE_CHARACTER,
        0666,
        ptmx_device,
        &master_operations,
        &ptmx_node);
    if (status != DK_OK)
        goto fail_events;

    (void)DK_LOG_LITERAL(
        DK_LOG_INFO,
        "pty: published /dev/ptmx and /dev/pts");
    return DK_OK;

fail_events:
    for (size_t index = 0; index < PTY_SLOT_COUNT; ++index) {
        struct pty_slot *slot = &slots[index];
        if (slot->master_readable != 0) {
            (void)dk_event_destroy(slot->master_readable);
            slot->master_readable = 0;
        }
        if (slot->master_writable != 0) {
            (void)dk_event_destroy(slot->master_writable);
            slot->master_writable = 0;
        }
        if (slot->slave_writable != 0) {
            (void)dk_event_destroy(slot->slave_writable);
            slot->slave_writable = 0;
        }
    }
    return status;
}

static void pty_stop(dk_driver_t driver, uintptr_t context)
{
    (void)driver;
    (void)context;
    for (size_t index = 0; index < PTY_SLOT_COUNT; ++index) {
        struct pty_slot *slot = &slots[index];
        if (slot->master_readable != 0)
            (void)dk_event_destroy(slot->master_readable);
        if (slot->master_writable != 0)
            (void)dk_event_destroy(slot->master_writable);
        if (slot->slave_writable != 0)
            (void)dk_event_destroy(slot->slave_writable);
    }
}

static const struct dk_driver_definition driver = {
    .name = DK_SLICE_LITERAL("pty"),
    .context = 0,
    .start = pty_start,
    .stop = pty_stop,
};

DK_DRIVER_EXPORT(driver)
