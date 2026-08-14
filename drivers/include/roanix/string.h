/*
 * Roanix driver framework: freestanding string and memory routines.
 *
 * A freestanding compiler may still emit calls to memcpy, memset, memmove, and
 * memcmp for ordinary structure assignments, so every module needs its own
 * copies. Defining them here keeps modules self-contained; unused copies are
 * removed by section garbage collection at link time.
 */
#ifndef ROANIX_STRING_H
#define ROANIX_STRING_H

#include <roanix/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Copies `length` bytes between non-overlapping buffers. */
RDF_USED static void *memcpy(void *restrict destination, const void *restrict source, size_t length)
{
    uint8_t *out = (uint8_t *)destination;
    const uint8_t *in = (const uint8_t *)source;
    for (size_t index = 0; index < length; ++index)
        out[index] = in[index];
    return destination;
}

/* Copies `length` bytes between possibly overlapping buffers. */
RDF_USED static void *memmove(void *destination, const void *source, size_t length)
{
    uint8_t *out = (uint8_t *)destination;
    const uint8_t *in = (const uint8_t *)source;
    if (out == in || length == 0)
        return destination;
    if (out < in) {
        for (size_t index = 0; index < length; ++index)
            out[index] = in[index];
    } else {
        for (size_t index = length; index > 0; --index)
            out[index - 1] = in[index - 1];
    }
    return destination;
}

/* Fills `length` bytes with `value`. */
RDF_USED static void *memset(void *destination, int value, size_t length)
{
    uint8_t *out = (uint8_t *)destination;
    for (size_t index = 0; index < length; ++index)
        out[index] = (uint8_t)value;
    return destination;
}

/* Compares two byte ranges. */
RDF_USED static int memcmp(const void *left, const void *right, size_t length)
{
    const uint8_t *a = (const uint8_t *)left;
    const uint8_t *b = (const uint8_t *)right;
    for (size_t index = 0; index < length; ++index) {
        if (a[index] != b[index])
            return a[index] < b[index] ? -1 : 1;
    }
    return 0;
}

/* Returns the length of a NUL-terminated string. */
RDF_USED static size_t strlen(const char *text)
{
    size_t length = 0;
    while (text[length] != '\0')
        ++length;
    return length;
}

/* Compares two NUL-terminated strings. */
RDF_UNUSED static int strcmp(const char *left, const char *right)
{
    while (*left != '\0' && *left == *right) {
        ++left;
        ++right;
    }
    return (int)(unsigned char)*left - (int)(unsigned char)*right;
}

/* Compares at most `length` characters of two strings. */
RDF_UNUSED static int strncmp(const char *left, const char *right, size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        unsigned char a = (unsigned char)left[index];
        unsigned char b = (unsigned char)right[index];
        if (a != b)
            return (int)a - (int)b;
        if (a == '\0')
            break;
    }
    return 0;
}

/* Returns whether a length-delimited byte range equals a C string. */
RDF_UNUSED static int rdf_str_equals(const uint8_t *bytes, size_t length, const char *text)
{
    size_t expected = strlen(text);
    return length == expected && memcmp(bytes, text, length) == 0;
}

/*
 * Returns whether a NUL-separated string list contains `text`.
 *
 * Device-tree `compatible` values arrive in exactly this form.
 */
RDF_UNUSED static int rdf_strlist_contains(const uint8_t *bytes, size_t length, const char *text)
{
    size_t offset = 0;
    while (offset < length) {
        size_t end = offset;
        while (end < length && bytes[end] != 0)
            ++end;
        if (rdf_str_equals(bytes + offset, end - offset, text))
            return 1;
        offset = end + 1;
    }
    return 0;
}

/* Copies a NUL-terminated string into a bounded buffer. */
RDF_UNUSED static size_t rdf_strlcpy(char *destination, const char *source, size_t capacity)
{
    size_t length = strlen(source);
    if (capacity != 0) {
        size_t copied = length < capacity - 1 ? length : capacity - 1;
        memcpy(destination, source, copied);
        destination[copied] = '\0';
    }
    return length;
}

#ifdef __cplusplus
}
#endif

#endif
