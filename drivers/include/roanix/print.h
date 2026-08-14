/*
 * Roanix driver framework: message formatting.
 *
 * The kernel log service takes a finished string, so formatting happens in the
 * module. This is a deliberately small formatter covering what driver
 * diagnostics need: %s, %c, %d, %i, %u, %x, %X, %p, %%, an optional field
 * width with zero padding, and the l/ll/z length modifiers.
 */
#ifndef ROANIX_PRINT_H
#define ROANIX_PRINT_H

#include <roanix/string.h>
#include <roanix/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Largest formatted log line. Longer messages are truncated. */
#define RDF_LOG_LINE 512

struct rdf_sink {
    char *buffer;
    size_t capacity;
    size_t length;
};

RDF_UNUSED static void rdf_sink_char(struct rdf_sink *sink, char value)
{
    if (sink->length + 1 < sink->capacity)
        sink->buffer[sink->length] = value;
    ++sink->length;
}

RDF_UNUSED static void rdf_sink_text(struct rdf_sink *sink, const char *text, size_t length)
{
    for (size_t index = 0; index < length; ++index)
        rdf_sink_char(sink, text[index]);
}

RDF_UNUSED static void rdf_sink_unsigned(struct rdf_sink *sink, uint64_t value, unsigned base,
                              int uppercase, unsigned width, int zero_pad)
{
    const char *digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";
    char scratch[24];
    unsigned length = 0;

    do {
        scratch[length++] = digits[value % base];
        value /= base;
    } while (value != 0 && length < sizeof(scratch));

    while (length < width && length < sizeof(scratch))
        scratch[length++] = zero_pad ? '0' : ' ';
    while (length > 0)
        rdf_sink_char(sink, scratch[--length]);
}

RDF_UNUSED static void rdf_sink_signed(struct rdf_sink *sink, int64_t value, unsigned width, int zero_pad)
{
    if (value < 0) {
        rdf_sink_char(sink, '-');
        if (width > 0)
            --width;
        rdf_sink_unsigned(sink, (uint64_t)(-(value + 1)) + 1u, 10, 0, width, zero_pad);
        return;
    }
    rdf_sink_unsigned(sink, (uint64_t)value, 10, 0, width, zero_pad);
}

/* Formats `format` into `buffer`, returning the length that would be written. */
RDF_UNUSED static size_t rdf_vsnprintf(char *buffer, size_t capacity, const char *format, va_list args)
{
    struct rdf_sink sink = {buffer, capacity, 0};

    for (const char *cursor = format; *cursor != '\0'; ++cursor) {
        if (*cursor != '%') {
            rdf_sink_char(&sink, *cursor);
            continue;
        }
        ++cursor;
        if (*cursor == '\0')
            break;

        int zero_pad = 0;
        if (*cursor == '0') {
            zero_pad = 1;
            ++cursor;
        }
        unsigned width = 0;
        while (*cursor >= '0' && *cursor <= '9') {
            width = width * 10u + (unsigned)(*cursor - '0');
            ++cursor;
        }
        int wide = 0;
        while (*cursor == 'l' || *cursor == 'z') {
            wide = 1;
            ++cursor;
        }

        switch (*cursor) {
        case 's': {
            const char *text = va_arg(args, const char *);
            if (text == NULL)
                text = "(null)";
            rdf_sink_text(&sink, text, strlen(text));
            break;
        }
        case 'c':
            rdf_sink_char(&sink, (char)va_arg(args, int));
            break;
        case 'd':
        case 'i':
            rdf_sink_signed(&sink, wide ? va_arg(args, int64_t) : va_arg(args, int32_t), width,
                            zero_pad);
            break;
        case 'u':
            rdf_sink_unsigned(&sink, wide ? va_arg(args, uint64_t) : va_arg(args, uint32_t), 10,
                              0, width, zero_pad);
            break;
        case 'x':
            rdf_sink_unsigned(&sink, wide ? va_arg(args, uint64_t) : va_arg(args, uint32_t), 16,
                              0, width, zero_pad);
            break;
        case 'X':
            rdf_sink_unsigned(&sink, wide ? va_arg(args, uint64_t) : va_arg(args, uint32_t), 16,
                              1, width, zero_pad);
            break;
        case 'p':
            rdf_sink_text(&sink, "0x", 2);
            rdf_sink_unsigned(&sink, (uint64_t)(uintptr_t)va_arg(args, void *), 16, 0, 0, 0);
            break;
        case '%':
            rdf_sink_char(&sink, '%');
            break;
        default:
            rdf_sink_char(&sink, '%');
            rdf_sink_char(&sink, *cursor);
            break;
        }
    }

    if (sink.capacity != 0) {
        size_t terminator = sink.length < sink.capacity ? sink.length : sink.capacity - 1;
        sink.buffer[terminator] = '\0';
    }
    return sink.length;
}

/* Formats into a bounded buffer. */
RDF_UNUSED static size_t rdf_snprintf(char *buffer, size_t capacity, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    size_t length = rdf_vsnprintf(buffer, capacity, format, args);
    va_end(args);
    return length;
}

#ifdef __cplusplus
}
#endif

#endif
