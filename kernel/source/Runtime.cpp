//===-- Runtime.cpp - Runtime Support Library -----------------------------===//
//
// Part of the Roanix Project, under the Mozilla Public License 2.0.
// See LICENSE in the project root for license information.
// SPDX-License-Identifier: MPL-2.0
//
//===----------------------------------------------------------------------===//
//
// Implementation of a C++ language runtime.
//
//===----------------------------------------------------------------------===//

#include <cstddef>
#include <cstdint>
#include <utility>

extern "C" {

void *memcpy(void *dest, const void *src, std::size_t n) {
  auto *pdest = static_cast<uint8_t *>(dest);
  const auto *psrc = static_cast<const uint8_t *>(src);

  for (std::size_t i = 0; i < n; i++) {
    pdest[i] = psrc[i];
  }

  return dest;
}

void *memset(void *s, int c, std::size_t n) {
  auto *p = static_cast<uint8_t *>(s);

  for (std::size_t i = 0; i < n; i++) {
    p[i] = static_cast<uint8_t>(c);
  }

  return s;
}

void *memmove(void *dest, const void *src, std::size_t n) {
  auto *pdest = static_cast<uint8_t *>(dest);
  const auto *psrc = static_cast<const uint8_t *>(src);

  if (src > dest) {
    for (std::size_t i = 0; i < n; i++) {
      pdest[i] = psrc[i];
    }
  } else if (src < dest) {
    for (std::size_t i = n; i > 0; i--) {
      pdest[i - 1] = psrc[i - 1];
    }
  }

  return dest;
}

int memcmp(const void *s1, const void *s2, std::size_t n) {
  const auto *p1 = static_cast<const uint8_t *>(s1);
  const auto *p2 = static_cast<const uint8_t *>(s2);

  for (std::size_t i = 0; i < n; i++) {
    if (p1[i] != p2[i]) {
      return p1[i] < p2[i] ? -1 : 1;
    }
  }

  return 0;
}

std::size_t strlen(const char *str) {
  std::size_t len;

  for (len = 0; str[len] != 0; len++)
    ;

  return len;
}

[[gnu::visibility("hidden")]] void *__dso_handle;

int __cxa_atexit(void (*)(void *), void *, void *) { return 0; }

void __cxa_pure_virtual() {
  __builtin_unreachable();
}

void abort(void) {
  __builtin_unreachable();
}

} // extern "C"
