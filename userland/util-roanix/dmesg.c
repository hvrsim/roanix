/*
 * dmesg - print or control the Roanix kernel log.
 *
 * The kernel exposes its log as a stream of binary packets on /dev/klog. Each
 * packet is a fixed header followed by the subsystem tag, the source path and
 * the message text. Formatting lives here rather than in the kernel so that
 * filtering, colouring and follow mode do not depend on the kernel guessing how
 * the output will be consumed.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <libgen.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define KLOG_DEVICE "/dev/klog"

#define KLOG_MAGIC 0x474f4c4bu
#define KLOG_VERSION 1u

#define KLOG_HEADER_SIZE 40u
#define KLOG_PAYLOAD_CAPACITY 256u
#define KLOG_MAX_PACKET (KLOG_HEADER_SIZE + KLOG_PAYLOAD_CAPACITY)

/* Message text was cut short because it did not fit in a record. */
#define KLOG_FLAG_TRUNCATED (1u << 0)
/* Record was submitted by userspace. */
#define KLOG_FLAG_USERSPACE (1u << 1)
/* Record was emitted by the panic path. */
#define KLOG_FLAG_EMERGENCY (1u << 2)

#define KLOG_LEVEL_EMERGENCY 0u
#define KLOG_LEVEL_ERROR 1u
#define KLOG_LEVEL_WARN 2u
#define KLOG_LEVEL_INFO 3u
#define KLOG_LEVEL_DEBUG 4u
#define KLOG_LEVEL_TRACE 5u
#define KLOG_LEVEL_COUNT 6u

/* Linux-compatible ioctl encoding, matching what the kernel decodes. */
#define KLOG_IOC(direction, number, size) \
    (((unsigned long)(direction) << 30) | ((unsigned long)(size) << 16) | \
     ((unsigned long)'K' << 8) | (unsigned long)(number))
#define KLOG_IOC_READ 2u
#define KLOG_IOC_WRITE 1u
#define KLOG_IOC_NONE 0u

#define KLOG_GET_LEVEL KLOG_IOC(KLOG_IOC_READ, 1u, sizeof(uint32_t))
#define KLOG_SET_LEVEL KLOG_IOC(KLOG_IOC_WRITE, 2u, sizeof(uint32_t))
#define KLOG_GET_CONSOLE_LEVEL KLOG_IOC(KLOG_IOC_READ, 3u, sizeof(uint32_t))
#define KLOG_SET_CONSOLE_LEVEL KLOG_IOC(KLOG_IOC_WRITE, 4u, sizeof(uint32_t))
#define KLOG_CLEAR KLOG_IOC(KLOG_IOC_NONE, 5u, 0u)
#define KLOG_GET_STATS KLOG_IOC(KLOG_IOC_READ, 6u, sizeof(struct klog_stats))
#define KLOG_SEEK_FIRST KLOG_IOC(KLOG_IOC_NONE, 7u, 0u)
#define KLOG_SEEK_LAST KLOG_IOC(KLOG_IOC_NONE, 8u, 0u)

struct klog_header {
    uint32_t magic;
    uint16_t length;
    uint8_t level;
    uint8_t version;
    uint64_t sequence;
    uint64_t timestamp_ns;
    uint16_t cpu;
    uint16_t line;
    uint32_t thread;
    uint8_t subsystem_len;
    uint8_t file_len;
    uint16_t message_len;
    uint8_t flags;
    uint8_t reserved[3];
};

struct klog_stats {
    uint64_t next_sequence;
    uint64_t first_sequence;
    uint64_t overwritten;
    uint64_t truncated;
    uint32_t slots;
    uint32_t slot_payload;
    uint8_t record_level;
    uint8_t console_level;
    uint8_t reserved[6];
};

_Static_assert(sizeof(struct klog_header) == KLOG_HEADER_SIZE,
               "kernel log header layout drifted from the kernel");
_Static_assert(sizeof(struct klog_stats) == 48,
               "kernel log statistics layout drifted from the kernel");

/* Records are read in batches; the kernel only ever returns whole packets. */
#define READ_BUFFER_SIZE (64u * 1024u)

struct level_alias {
    const char *name;
    unsigned value;
};

static const struct level_alias LEVEL_ALIASES[] = {
    {"emerg", KLOG_LEVEL_EMERGENCY}, {"panic", KLOG_LEVEL_EMERGENCY},
    {"err", KLOG_LEVEL_ERROR},       {"error", KLOG_LEVEL_ERROR},
    {"warn", KLOG_LEVEL_WARN},       {"warning", KLOG_LEVEL_WARN},
    {"info", KLOG_LEVEL_INFO},       {"debug", KLOG_LEVEL_DEBUG},
    {"trace", KLOG_LEVEL_TRACE},
};

static const char *const LEVEL_LABEL[KLOG_LEVEL_COUNT] = {
    "emerg", "err", "warn", "info", "debug", "trace",
};

/* SGR attributes used to colour each severity; empty means no colour. */
static const char *const LEVEL_COLOR[KLOG_LEVEL_COUNT] = {
    "1;41;97", "1;31", "1;33", "", "2", "2;35",
};

struct options {
    bool follow;
    bool show_time;
    bool show_level;
    bool show_source;
    bool show_cpu;
    bool raw;
    bool color;
    unsigned level_mask;
    unsigned long tail;
};

/* Fixed-size ring holding the most recent records for --tail. */
struct tail_ring {
    unsigned char *packets;
    size_t *lengths;
    unsigned long capacity;
    unsigned long stored;
    unsigned long next;
};

static const char *program_name = "dmesg";

static void fail(const char *format, ...)
{
    va_list args;
    fprintf(stderr, "%s: ", program_name);
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);
    fputc('\n', stderr);
    exit(EXIT_FAILURE);
}

static void usage(FILE *stream)
{
    fprintf(stream,
            "usage: %s [options]\n"
            "\n"
            "print or control the kernel log.\n"
            "\n"
            "output:\n"
            "  -f, --follow           keep printing records as they arrive\n"
            "  -t, --notime           do not print the timestamp\n"
            "  -x, --decode           print the severity of each record\n"
            "  -u, --cpu              print the CPU and thread of each record\n"
            "  -o, --source           print the source location of each record\n"
            "  -H, --human            same as --decode --source --color=always\n"
            "  -r, --raw              copy raw packets to standard output\n"
            "      --color[=WHEN]     colourise output: auto, always or never\n"
            "\n"
            "filtering:\n"
            "  -l, --level LIST       only show these severities, comma separated\n"
            "      --tail COUNT       only show the last COUNT records\n"
            "\n"
            "control:\n"
            "  -n, --console-level L  set the severity mirrored to the console\n"
            "  -N, --kernel-level L   set the severity the kernel records\n"
            "  -C, --clear            discard the kernel log and print nothing\n"
            "  -c, --read-clear       print the kernel log, then discard it\n"
            "  -S, --stats            print kernel log buffer statistics\n"
            "\n"
            "  -h, --help             display this help and exit\n"
            "  -V, --version          display version information and exit\n"
            "\n"
            "severities, most to least urgent: %s %s %s %s %s %s\n",
            program_name, LEVEL_LABEL[0], LEVEL_LABEL[1], LEVEL_LABEL[2],
            LEVEL_LABEL[3], LEVEL_LABEL[4], LEVEL_LABEL[5]);
}

/* Resolves a severity given by name or by number. */
static bool parse_level(const char *text, unsigned *out)
{
    if (text == NULL || *text == '\0')
        return false;

    for (size_t index = 0; index < sizeof(LEVEL_ALIASES) / sizeof(*LEVEL_ALIASES); index++) {
        if (strcmp(text, LEVEL_ALIASES[index].name) == 0) {
            *out = LEVEL_ALIASES[index].value;
            return true;
        }
    }

    char *end = NULL;
    errno = 0;
    unsigned long value = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value >= KLOG_LEVEL_COUNT)
        return false;
    *out = (unsigned)value;
    return true;
}

/* Builds a severity bitmask from a comma-separated list. */
static unsigned parse_level_list(const char *list)
{
    char buffer[128];
    if (strlen(list) >= sizeof(buffer))
        fail("severity list is too long");
    strcpy(buffer, list);

    unsigned mask = 0;
    for (char *item = strtok(buffer, ","); item != NULL; item = strtok(NULL, ",")) {
        unsigned level = 0;
        if (!parse_level(item, &level))
            fail("unknown severity '%s'", item);
        mask |= 1u << level;
    }

    if (mask == 0)
        fail("severity list is empty");
    return mask;
}

static unsigned long parse_count(const char *text)
{
    char *end = NULL;
    errno = 0;
    unsigned long value = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value == 0)
        fail("invalid record count '%s'", text);
    return value;
}

static void set_level(int fd, unsigned long request, unsigned level, const char *what)
{
    uint32_t value = (uint32_t)level;
    if (ioctl(fd, request, &value) < 0)
        fail("cannot set the %s severity: %s", what, strerror(errno));
}

static void clear_log(int fd)
{
    if (ioctl(fd, KLOG_CLEAR, 0) < 0)
        fail("cannot clear the kernel log: %s", strerror(errno));
}

static void print_stats(int fd)
{
    struct klog_stats stats;
    memset(&stats, 0, sizeof(stats));
    if (ioctl(fd, KLOG_GET_STATS, &stats) < 0)
        fail("cannot read log statistics: %s", strerror(errno));

    unsigned record = stats.record_level < KLOG_LEVEL_COUNT ? stats.record_level : 0;
    unsigned console = stats.console_level < KLOG_LEVEL_COUNT ? stats.console_level : 0;

    printf("records written    %" PRIu64 "\n", stats.next_sequence);
    printf("records retained   %" PRIu64 "\n",
           stats.next_sequence - stats.first_sequence);
    printf("records evicted    %" PRIu64 "\n", stats.overwritten);
    printf("records truncated  %" PRIu64 "\n", stats.truncated);
    printf("buffer capacity    %" PRIu32 " records of %" PRIu32 " payload bytes\n",
           stats.slots, stats.slot_payload);
    printf("kernel severity    %s\n", LEVEL_LABEL[record]);
    printf("console severity   %s\n", LEVEL_LABEL[console]);
}

static void write_all(int fd, const unsigned char *bytes, size_t length)
{
    while (length > 0) {
        ssize_t written = write(fd, bytes, length);
        if (written < 0) {
            if (errno == EINTR)
                continue;
            fail("write failed: %s", strerror(errno));
        }
        /* A zero-length write would otherwise spin here forever. */
        if (written == 0)
            fail("write made no progress");
        bytes += (size_t)written;
        length -= (size_t)written;
    }
}

/* Prints kernel-supplied bytes, escaping anything that would break a line. */
static void print_escaped(const unsigned char *bytes, size_t length)
{
    for (size_t index = 0; index < length; index++) {
        unsigned char byte = bytes[index];
        if (byte >= 0x20 && byte != 0x7f)
            putchar((int)byte);
        else if (byte == '\t')
            putchar('\t');
        else
            printf("\\x%02x", byte);
    }
}

/* Validates one packet and reports its parts. */
static void decode(
        const unsigned char *packet,
        size_t available,
        struct klog_header *header,
        const unsigned char **subsystem,
        const unsigned char **file,
        const unsigned char **message)
{
    memcpy(header, packet, sizeof(*header));

    if (header->magic != KLOG_MAGIC)
        fail("kernel log packet is corrupt: bad magic 0x%08" PRIx32, header->magic);
    if (header->version != KLOG_VERSION)
        fail("kernel log packet version %u is not supported", header->version);
    if (header->length < KLOG_HEADER_SIZE || (size_t)header->length > available)
        fail("kernel log packet is corrupt: bad length %u", header->length);

    size_t payload = (size_t)header->subsystem_len + header->file_len + header->message_len;
    if (KLOG_HEADER_SIZE + payload > (size_t)header->length)
        fail("kernel log packet is corrupt: payload overruns the packet");

    *subsystem = packet + KLOG_HEADER_SIZE;
    *file = *subsystem + header->subsystem_len;
    *message = *file + header->file_len;
}

static void print_record(
        const unsigned char *packet,
        size_t available,
        const struct options *options)
{
    struct klog_header header;
    const unsigned char *subsystem = NULL;
    const unsigned char *file = NULL;
    const unsigned char *message = NULL;
    decode(packet, available, &header, &subsystem, &file, &message);

    unsigned level = header.level < KLOG_LEVEL_COUNT ? header.level : KLOG_LEVEL_TRACE;
    const char *color = options->color ? LEVEL_COLOR[level] : "";
    bool colored = color[0] != '\0';

    if (options->show_time) {
        uint64_t seconds = header.timestamp_ns / 1000000000u;
        uint64_t microseconds = (header.timestamp_ns % 1000000000u) / 1000u;
        if (options->color)
            printf("\033[2m[%5" PRIu64 ".%06" PRIu64 "]\033[0m ", seconds, microseconds);
        else
            printf("[%5" PRIu64 ".%06" PRIu64 "] ", seconds, microseconds);
    }

    if (options->show_level)
        printf("%-5s ", LEVEL_LABEL[level]);

    if (options->show_cpu)
        printf("cpu%-2u tid%-6" PRIu32 " ", header.cpu, header.thread);

    if (colored)
        printf("\033[%sm", color);

    if (header.subsystem_len > 0) {
        print_escaped(subsystem, header.subsystem_len);
        fputs(": ", stdout);
    }
    print_escaped(message, header.message_len);

    if ((header.flags & KLOG_FLAG_TRUNCATED) != 0)
        fputs("...", stdout);

    if (colored)
        fputs("\033[0m", stdout);

    if (options->show_source && header.file_len > 0) {
        fputs(options->color ? " \033[2m(" : " (", stdout);
        print_escaped(file, header.file_len);
        printf(":%u)", header.line);
        if (options->color)
            fputs("\033[0m", stdout);
    }

    putchar('\n');
}

static bool wanted(const unsigned char *packet, const struct options *options)
{
    struct klog_header header;
    memcpy(&header, packet, sizeof(header));
    unsigned level = header.level < KLOG_LEVEL_COUNT ? header.level : KLOG_LEVEL_TRACE;
    return (options->level_mask & (1u << level)) != 0;
}

static void emit(const unsigned char *packet, size_t length, const struct options *options)
{
    if (options->raw)
        write_all(STDOUT_FILENO, packet, length);
    else
        print_record(packet, length, options);
}

static void tail_push(
        struct tail_ring *ring,
        const unsigned char *packet,
        size_t length)
{
    memcpy(ring->packets + ring->next * KLOG_MAX_PACKET, packet, length);
    ring->lengths[ring->next] = length;
    ring->next = (ring->next + 1) % ring->capacity;
    if (ring->stored < ring->capacity)
        ring->stored++;
}

static void tail_flush(const struct tail_ring *ring, const struct options *options)
{
    unsigned long start = (ring->next + ring->capacity - ring->stored) % ring->capacity;
    for (unsigned long index = 0; index < ring->stored; index++) {
        unsigned long slot = (start + index) % ring->capacity;
        emit(ring->packets + slot * KLOG_MAX_PACKET, ring->lengths[slot], options);
    }
}

/*
 * Streams the log until it runs dry, or forever when following.
 *
 * When `ring` is present, records are buffered instead of printed: the kernel
 * serves the log oldest first, so the tail can only be known once the whole
 * buffer has been read.
 */
static void stream(int fd, const struct options *options, struct tail_ring *ring)
{
    unsigned char *buffer = malloc(READ_BUFFER_SIZE);
    if (buffer == NULL)
        fail("cannot allocate a read buffer");

    for (;;) {
        ssize_t count = read(fd, buffer, READ_BUFFER_SIZE);
        if (count < 0) {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            fail("cannot read %s: %s", KLOG_DEVICE, strerror(errno));
        }
        if (count == 0)
            break;

        size_t offset = 0;
        while (offset + KLOG_HEADER_SIZE <= (size_t)count) {
            struct klog_header header;
            memcpy(&header, buffer + offset, sizeof(header));
            if (header.length < KLOG_HEADER_SIZE ||
                    offset + header.length > (size_t)count)
                fail("kernel log stream lost framing");

            const unsigned char *packet = buffer + offset;
            if (wanted(packet, options)) {
                if (ring != NULL)
                    tail_push(ring, packet, header.length);
                else
                    emit(packet, header.length, options);
            }
            offset += header.length;
        }

        if (ring == NULL && !options->raw)
            fflush(stdout);
    }

    free(buffer);
}

int main(int argc, char *argv[])
{
    if (argc > 0 && argv[0] != NULL && argv[0][0] != '\0')
        program_name = basename(argv[0]);

    struct options options = {
        .show_time = true,
        .color = isatty(STDOUT_FILENO) == 1,
        .level_mask = (1u << KLOG_LEVEL_COUNT) - 1u,
    };

    bool set_console = false;
    bool set_kernel = false;
    bool clear = false;
    bool read_clear = false;
    bool stats = false;
    unsigned console_level = 0;
    unsigned kernel_level = 0;

    enum { OPT_COLOR = 0x100, OPT_TAIL };
    static const struct option long_options[] = {
        {"follow", no_argument, NULL, 'f'},
        {"notime", no_argument, NULL, 't'},
        {"decode", no_argument, NULL, 'x'},
        {"cpu", no_argument, NULL, 'u'},
        {"source", no_argument, NULL, 'o'},
        {"human", no_argument, NULL, 'H'},
        {"raw", no_argument, NULL, 'r'},
        {"level", required_argument, NULL, 'l'},
        {"console-level", required_argument, NULL, 'n'},
        {"kernel-level", required_argument, NULL, 'N'},
        {"clear", no_argument, NULL, 'C'},
        {"read-clear", no_argument, NULL, 'c'},
        {"stats", no_argument, NULL, 'S'},
        {"color", optional_argument, NULL, OPT_COLOR},
        {"tail", required_argument, NULL, OPT_TAIL},
        {"help", no_argument, NULL, 'h'},
        {"version", no_argument, NULL, 'V'},
        {NULL, 0, NULL, 0},
    };

    int option = 0;
    while ((option = getopt_long(argc, argv, "ftxuoHrl:n:N:CcShV", long_options, NULL)) != -1) {
        switch (option) {
        case 'f':
            options.follow = true;
            break;
        case 't':
            options.show_time = false;
            break;
        case 'x':
            options.show_level = true;
            break;
        case 'u':
            options.show_cpu = true;
            break;
        case 'o':
            options.show_source = true;
            break;
        case 'H':
            options.show_level = true;
            options.show_source = true;
            options.color = true;
            break;
        case 'r':
            options.raw = true;
            break;
        case 'l':
            options.level_mask = parse_level_list(optarg);
            break;
        case 'n':
            if (!parse_level(optarg, &console_level))
                fail("unknown severity '%s'", optarg);
            set_console = true;
            break;
        case 'N':
            if (!parse_level(optarg, &kernel_level))
                fail("unknown severity '%s'", optarg);
            set_kernel = true;
            break;
        case 'C':
            clear = true;
            break;
        case 'c':
            read_clear = true;
            break;
        case 'S':
            stats = true;
            break;
        case OPT_COLOR:
            if (optarg == NULL || strcmp(optarg, "always") == 0)
                options.color = true;
            else if (strcmp(optarg, "never") == 0)
                options.color = false;
            else if (strcmp(optarg, "auto") == 0)
                options.color = isatty(STDOUT_FILENO) == 1;
            else
                fail("unknown colour mode '%s'", optarg);
            break;
        case OPT_TAIL:
            options.tail = parse_count(optarg);
            break;
        case 'h':
            usage(stdout);
            return EXIT_SUCCESS;
        case 'V':
            printf("dmesg (roanix)\n");
            return EXIT_SUCCESS;
        default:
            usage(stderr);
            return EXIT_FAILURE;
        }
    }

    if (optind < argc)
        fail("unexpected argument '%s'", argv[optind]);
    if (options.follow && options.tail != 0)
        fail("--follow and --tail cannot be combined");
    if (options.raw)
        options.color = false;

    int fd = open(KLOG_DEVICE, O_RDWR);
    if (fd < 0 && errno == EACCES)
        fd = open(KLOG_DEVICE, O_RDONLY);
    if (fd < 0)
        fail("cannot open %s: %s", KLOG_DEVICE, strerror(errno));

    if (set_kernel)
        set_level(fd, KLOG_SET_LEVEL, kernel_level, "kernel");
    if (set_console)
        set_level(fd, KLOG_SET_CONSOLE_LEVEL, console_level, "console");

    if (stats) {
        print_stats(fd);
        close(fd);
        return EXIT_SUCCESS;
    }
    if (clear) {
        clear_log(fd);
        close(fd);
        return EXIT_SUCCESS;
    }
    /* Adjusting a level is a control action, not a request to dump the log. */
    if ((set_console || set_kernel) && !options.follow) {
        close(fd);
        return EXIT_SUCCESS;
    }

    /* Without --follow the dump has to end, so reads must not block. */
    if (!options.follow && fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
        fail("cannot configure %s: %s", KLOG_DEVICE, strerror(errno));

    if (options.tail != 0) {
        struct tail_ring ring = {
            .packets = calloc(options.tail, KLOG_MAX_PACKET),
            .lengths = calloc(options.tail, sizeof(size_t)),
            .capacity = options.tail,
        };
        if (ring.packets == NULL || ring.lengths == NULL)
            fail("cannot buffer %lu records", options.tail);

        stream(fd, &options, &ring);
        tail_flush(&ring, &options);
        free(ring.lengths);
        free(ring.packets);
    } else {
        stream(fd, &options, NULL);
    }

    if (read_clear)
        clear_log(fd);

    fflush(stdout);
    close(fd);
    return EXIT_SUCCESS;
}
