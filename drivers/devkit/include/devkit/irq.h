#ifndef DEVKIT_IRQ_H
#define DEVKIT_IRQ_H

#include <devkit/resource.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DK_INTERRUPT_CONTROLLER_ROOT (UINT64_C(1) << 0)

#define DK_INTERRUPT_EDGE (UINT64_C(1) << 0)
#define DK_INTERRUPT_LEVEL (UINT64_C(1) << 1)
#define DK_INTERRUPT_ACTIVE_HIGH (UINT64_C(1) << 2)
#define DK_INTERRUPT_ACTIVE_LOW (UINT64_C(1) << 3)
#define DK_INTERRUPT_START_MASKED (UINT64_C(1) << 4)

#define DK_INTERRUPT_RESCHEDULE (UINT32_C(1) << 0)
#define DK_INTERRUPT_WAKE_THREAD (UINT32_C(1) << 1)

typedef uint64_t dk_irq_controller_t;
typedef uint64_t dk_irq_t;

struct dk_interrupt_route {
    uint32_t size;
    uint64_t interrupt;
    uint32_t vector;
    uint32_t target_cpu;
    uint64_t target_platform_id;
    uint64_t flags;
    struct dk_slice specifier;
};

typedef int32_t (*dk_interrupt_connect_fn)(
    uintptr_t context,
    const struct dk_interrupt_route *route,
    uint64_t *out_cookie);
typedef int32_t (*dk_interrupt_disconnect_fn)(
    uintptr_t context,
    uint64_t cookie);
typedef int32_t (*dk_interrupt_line_fn)(
    uintptr_t context,
    uint64_t cookie);
typedef int32_t (*dk_interrupt_set_affinity_fn)(
    uintptr_t context,
    uint64_t cookie,
    const struct dk_interrupt_route *route);
typedef int32_t (*dk_interrupt_claim_fn)(
    uintptr_t context,
    uint32_t cpu,
    uint64_t platform_id,
    uint64_t *out_interrupt,
    uint64_t *out_cookie);
typedef void (*dk_interrupt_complete_fn)(
    uintptr_t context,
    uint32_t cpu,
    uint64_t platform_id,
    uint64_t interrupt,
    uint64_t cookie);
typedef uint32_t (*dk_interrupt_handler_fn)(
    uintptr_t context,
    uint64_t interrupt);
typedef void (*dk_interrupt_thread_fn)(
    uintptr_t context,
    uint64_t interrupt);

struct dk_interrupt_controller {
    uint32_t size;
    uint64_t flags;
    uintptr_t context;
    dk_interrupt_connect_fn connect;
    dk_interrupt_disconnect_fn disconnect;
    dk_interrupt_line_fn mask;
    dk_interrupt_line_fn unmask;
    dk_interrupt_set_affinity_fn set_affinity;
    dk_interrupt_claim_fn claim;
    dk_interrupt_complete_fn complete;
};

int32_t dk_irq_controller_register(
    dk_bus_t bus,
    const struct dk_interrupt_controller *controller,
    dk_irq_controller_t *out_controller);
int32_t dk_irq_controller_unregister(dk_irq_controller_t controller);
int32_t dk_irq_request(
    dk_node_t node,
    struct dk_slice specifier,
    uint64_t flags,
    uint32_t target_cpu,
    dk_interrupt_handler_fn handler,
    uintptr_t context,
    dk_irq_t *out_interrupt);
/*
 * The optional top half must acknowledge or mask level-triggered hardware
 * before returning DK_INTERRUPT_WAKE_THREAD. The managed thread callback may
 * block and should drain all pending device work before returning.
 */
int32_t dk_irq_request_threaded(
    dk_node_t node,
    struct dk_slice specifier,
    uint64_t flags,
    uint32_t target_cpu,
    dk_interrupt_handler_fn handler,
    dk_interrupt_thread_fn thread_handler,
    uintptr_t context,
    dk_irq_t *out_interrupt);
int32_t dk_irq_release(dk_irq_t interrupt);
int32_t dk_irq_mask(dk_irq_t interrupt);
int32_t dk_irq_unmask(dk_irq_t interrupt);
int32_t dk_irq_set_affinity(dk_irq_t interrupt, uint32_t target_cpu);

#ifdef __cplusplus
}
#endif

#endif
