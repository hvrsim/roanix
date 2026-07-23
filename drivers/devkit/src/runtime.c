#include <devkit/base.h>

void *memcpy(void *restrict destination, const void *restrict source, size_t length)
{
    uint8_t *dst = destination;
    const uint8_t *src = source;
    for (size_t index = 0; index < length; ++index)
        dst[index] = src[index];
    return destination;
}

void *memmove(void *destination, const void *source, size_t length)
{
    uint8_t *dst = destination;
    const uint8_t *src = source;
    uintptr_t dst_address = (uintptr_t)dst;
    uintptr_t src_address = (uintptr_t)src;
    if (dst_address <= src_address ||
        dst_address - src_address >= length) {
        for (size_t index = 0; index < length; ++index)
            dst[index] = src[index];
    } else {
        for (size_t index = length; index != 0; --index)
            dst[index - 1u] = src[index - 1u];
    }
    return destination;
}

void *memset(void *destination, int value, size_t length)
{
    uint8_t *dst = destination;
    for (size_t index = 0; index < length; ++index)
        dst[index] = (uint8_t)value;
    return destination;
}

int memcmp(const void *left, const void *right, size_t length)
{
    const uint8_t *lhs = left;
    const uint8_t *rhs = right;
    for (size_t index = 0; index < length; ++index) {
        if (lhs[index] != rhs[index])
            return lhs[index] < rhs[index] ? -1 : 1;
    }
    return 0;
}

size_t strlen(const char *string)
{
    size_t length = 0;
    while (string[length] != '\0')
        ++length;
    return length;
}
