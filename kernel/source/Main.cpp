//===-- Main.cpp - Kernel Entrypoint  -------------------------------------===//
//
// Part of the Roanix Project, under the Mozilla Public License 2.0.
// See LICENSE in the project root for license information.
// SPDX-License-Identifier: MPL-2.0
//
//===----------------------------------------------------------------------===//
//
// This file defines the kernel entry-point.
//
//===----------------------------------------------------------------------===//

#include <cstddef>

#include <limine.h>

namespace {

__attribute__((used, section(".limine_requests")))
volatile LIMINE_BASE_REVISION(3);

__attribute__((used, section(".limine_requests_start")))
volatile LIMINE_REQUESTS_START_MARKER;

__attribute__((used, section(".limine_requests_end")))
volatile LIMINE_REQUESTS_END_MARKER;

} // namespace

extern void (*__init_array_start[])();
extern void (*__init_array_end[])();

extern "C" void rmain() {
  const auto hcf = []() {
    for (;;) {
#if defined(__x86_64__)
      asm volatile ("hlt");
#elif defined(__riscv)
      asm volatile ("wfi");
#endif
    }
  };

  // Ensure limine supports our minimum version.
  if (LIMINE_BASE_REVISION_SUPPORTED == false) {
    hcf();
  }

  // Run the global constructors.
  for (std::size_t i = 0; &__init_array_start[i] != __init_array_end; i++) {
    __init_array_start[i]();
  }

  hcf();
}