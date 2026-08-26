/*
 * Roanix driver framework: basic types and status codes.
 *
 * This header, and every other header under <roanix/>, is self-contained. A
 * driver links against no support library: whatever is not a kernel service is
 * inlined here, and whatever is a kernel service is reached through the table
 * in <roanix/api.h>.
 */
#ifndef ROANIX_TYPES_H
#define ROANIX_TYPES_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Incompatible ABI generation. */
#define RDF_ABI_MAJOR 1u
/* Append-only ABI revision. */
#define RDF_ABI_MINOR 3u

/* Success. */
#define RDF_OK 0
/* An argument was malformed or out of range. */
#define RDF_EINVAL (-1)
/* The requested object does not exist. */
#define RDF_ENOENT (-2)
/* An object with the same identity already exists. */
#define RDF_EEXIST (-3)
/* The object has the wrong type for this operation. */
#define RDF_EKIND (-4)
/* The caller lacks the required rights. */
#define RDF_EPERM (-5)
/* The object is in use. */
#define RDF_EBUSY (-6)
/* The operation is not implemented. */
#define RDF_ENOTSUP (-7)
/* A table, identifier space, or address range is exhausted. */
#define RDF_ENOSPC (-8)
/* Out of memory. */
#define RDF_ENOMEM (-9)
/* Hardware or transport failure. */
#define RDF_EIO (-10)
/* A non-blocking operation would have blocked. */
#define RDF_EAGAIN (-11)
/* A blocking operation was interrupted. */
#define RDF_EINTR (-12)
/* The operation would deadlock or create a dependency cycle. */
#define RDF_EDEADLK (-13)
/*
 * A prerequisite is not available yet.
 *
 * Returning this from a probe callback parks the device and retries it once
 * the topology changes. It is the mechanism a driver uses to wait for another
 * driver's service without caring about module load order.
 */
#define RDF_EDEFER (-14)
/* The subsystem is not initialized. */
#define RDF_ENOINIT (-15)
/* A bounded wait expired. */
#define RDF_ETIMEDOUT (-16)
/* The device was removed while the operation was in flight. */
#define RDF_ENODEV (-17)
/* A caller-supplied buffer was too small. */
#define RDF_E2BIG (-18)
/* The target is not a terminal. */
#define RDF_ENOTTY (-19)
/* The object does not support seeking. */
#define RDF_ESPIPE (-20)
/* The filesystem layer rejected the operation. */
#define RDF_EFS (-21)
/* A filesystem operation required a directory. */
#define RDF_ENOTDIR (-22)
/* A filesystem operation cannot operate on a directory. */
#define RDF_EISDIR (-23)
/* A directory still contains entries. */
#define RDF_ENOTEMPTY (-24)
/* A filesystem operation crosses mount boundaries. */
#define RDF_EXDEV (-25)
/* A filesystem traversal exceeded its symbolic-link bound. */
#define RDF_ELOOP (-26)
/* A filesystem name exceeds its supported length. */
#define RDF_ENAMETOOLONG (-27)
/* A file offset or size exceeds its supported range. */
#define RDF_EFBIG (-28)
/* A filesystem is read-only. */
#define RDF_EROFS (-29)
/* An open file does not permit the requested operation. */
#define RDF_EBADF (-30)

/* Log levels accepted by rdf_log(). */
#define RDF_LOG_ERROR 1u
#define RDF_LOG_WARN 2u
#define RDF_LOG_INFO 3u
#define RDF_LOG_DEBUG 4u
#define RDF_LOG_TRACE 5u

/* Opaque framework objects. Handles are validated by the kernel. */
struct rdf_module;
struct rdf_device;
struct rdf_driver;
struct rdf_bus;
struct rdf_class;
struct rdf_class_device;
struct rdf_iface;
struct rdf_irq_domain;
struct rdf_tty_provider;
struct rdf_fs_provider;
struct rdf_fs_page_account;
struct rdf_fs_memory_object;
struct rdf_devfs_broker;
struct rdf_devfs_endpoint;

/* Opaque receipts returned by the call that creates a resource. */
struct rdf_device_builder;
struct rdf_iface_binding;
struct rdf_irq;
struct rdf_work;
struct rdf_timer;
struct rdf_tty;
struct rdf_worker;

/* Identifier of a node in the device filesystem. */
typedef uint64_t rdf_devnode_t;
/* Address of a kernel event object. */
typedef uintptr_t rdf_event_t;

#if defined(__GNUC__) || defined(__clang__)
#define RDF_PRINTF(fmt, args) __attribute__((format(printf, fmt, args)))
#define RDF_UNUSED __attribute__((unused))
#define RDF_USED __attribute__((used))
#define RDF_EXPORT __attribute__((used, visibility("default")))
#define RDF_LIKELY(x) __builtin_expect(!!(x), 1)
#define RDF_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#error "The Roanix driver framework requires GCC or Clang"
#endif

/* Number of elements in a fixed-size array. */
#define RDF_COUNT(array) (sizeof(array) / sizeof((array)[0]))

#ifdef __cplusplus
}
#endif

#endif
