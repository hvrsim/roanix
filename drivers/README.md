# Roanix hardware drivers

Only hardware-facing code is a loadable module. Every in-tree module is a Rust
crate whose `build.rs` asks the DDK to link a freestanding `.ko` image. The
kernel loads those images from `/usr/lib/roanix/drivers`.

Modules use ABI generation 2 from `include/roanix/`: they enter through
`rdf_module_entry`, import only the `rdf_api_v2_*` functions they call, and
expose size-prefixed operation records made of plain function pointers. There
is no general kernel symbol namespace.

## Layout

| Directory          | Module            | Purpose                                  |
| ------------------ | ----------------- | ---------------------------------------- |
| `ddk/`             | —                 | The no_std driver development kit        |
| `platform/acpi/`   | `acpi.ko`         | x86_64 ACPI platform enumerator           |
| `platform/fdt/`    | `fdt.ko`          | riscv64 device-tree enumerator            |
| `irqchip/ioapic/`  | `ioapic.ko`       | I/O APIC interrupt controller             |
| `irqchip/plic/`    | `plic.ko`         | RISC-V interrupt controller               |
| `bus/pci/`         | `pci.ko`          | PCI/PCIe enumeration and configuration    |
| `storage/nvme/`    | `storage-nvme.ko` | NVMe hardware driver                      |
| `tty/uart8250/`    | `uart8250.ko`     | 8250/16550 serial hardware driver         |
| `input/evdev/`     | —                | Shared evdev keyboard queues and ioctls   |
| `input/i8042/`     | `i8042.ko`        | PC keyboard controller                   |
| `usb/usb/`         | —                | USB host service ABI                     |
| `usb/xhci/`        | `usb-xhci.ko`     | xHCI root ports, control and interrupt IN |
| `usb/keyboard/`    | `usb-keyboard.ko` | USB HID boot keyboards                   |

Enumerators publish devices on the kernel `platform` bus; drivers bind through
match tables. Kernel policy and always-present services are deliberately not
modules: tmpfs/devfs live in `kernel/src/fs`, while TTY, console, PTY, and
special-device services live in `kernel/src/dev`. A UART module supplies only
the hardware backend to the native terminal layer.

## Building

```bash
./x.py run
./x.py run --machine qemu-riscv64-virt
./x.py build drivers
./x.py build driver nvme
```

The workspace targets `x86_64-unknown-none` and `riscv64gc-unknown-none-elf`
with the pinned nightly toolchain in the [root toolchain file](../rust-toolchain.toml). RISC-V builds
rebuild the Rust standard-library components needed by the modules for
position-independent code. Cargo keeps intermediates in `build/cargo/drivers`;
finished modules are staged under `build/work/<arch>/drivers-rust`.

Drivers use the release profile. Add modules to the appropriate machine in
[`tools/machines.toml`](../tools/machines.toml). See the
[build guide](../book/src/guide/BUILDING.md) for package and image workflows.

## Writing a driver

Depend on `ddk`, implement probe/remove callbacks over its safe wrappers,
and finish with:

```rust
ddk::module!("my-driver", "What it does", init, exit);
```

The kernel resolves every used `rdf_api_v2_*` import before calling the module
entry point; unknown or internal kernel symbols make the module fail to load.

## Binding and removal

Device, bus, module and event handles are opaque identities. Copying a handle
does not retain registration or grant permission to dereference its numeric
value. Kernel lookups validate identities before accessing retained storage;
retired identities are not reused.

`OwnedEvent` owns registration and revokes it on Drop. `Event` is a Copy,
non-owning handle obtained with `OwnedEvent::event()` or from a callback identity;
it cannot destroy registration. Copying it does not extend registration lifetime.
Revocation wakes admitted waits; subsequent operations report a stale identity.
Create and drop owners in thread context, outside IRQ locks. Move an owner out
under the lock and drop it after releasing the guard. Leaked owners are revoked
by module teardown after callbacks and workers stop.

Finish hardware configuration before publishing callback contexts. A failed
probe must withdraw everything that can still invoke those contexts. If an
external user acquired a successfully published resource and withdrawal is busy,
retain a recoverable binding with its data pointer and report the reduced
availability. Do not return probe failure while orphaning that context.

New drivers that can refuse removal use `DriverDef::try_remove` and leave
`remove` empty. This is the optional 88-byte extension to the historical
80-byte ABI v2 record. The legacy `remove` callback remains supported and must
always finish disconnection. The two callbacks are mutually exclusive.

A checked callback returns an error while retaining every still-published
context and its required hardware state. The framework then preserves the
binding, data pointer, IRQ actions and provider dependencies. Successful removal
must withdraw external operations, drain callbacks and release context before
returning. `BlockRegistration::unregister` and `Tty::unregister` retain their
receipts on error so the driver can retry; never reclaim their contexts or reuse
static slots after a busy result. Best-effort Drop is not proof of withdrawal.

Removal claims close child admission and exclude competing removers. A failed
descendant restores the failing binding and unprocessed ancestors; descendants
and resources already removed stay removed. Driver unregister also rejects an
active probe transaction. Module unload closes new callback admission and tries
checked detach before invoking exit. A refusal keeps executable backing resident
and permits a later retry.

Storage drivers register `raw::BlockOps` through `BlockRegistration`. The
callbacks operate on whole logical blocks only; the kernel owns unaligned byte
I/O, devfs nodes, transfer splitting, partition parsing, and mount exclusion.
After the controller and callback context are fully published, call
`BlockRegistration::rescan()` once to discover GPT or MBR partitions. Dropping
the receipt attempts withdrawal; explicitly unregister it and check success
before destroying context. Withdrawal requires idle nodes and mounted filesystems.
Whole-disk and partition writes share range exclusion, including partial-sector
read/modify/write operations, discard and flush.

## Firmware topology

FDT enumeration publishes enabled nodes beneath a `devicetree` root with their
firmware parent relationships intact, including containers without registers.
Disabled status suppresses the entire subtree. Address/size cell defaults apply
locally to each bus. Every `reg` tuple is retained and each mapped register block
is published, using `reg-names` when present (`regs`, `regs1`, ... otherwise).
CPU identifiers with zero size cells remain metadata, without an MMIO window.

Register translation checks the full span through all enclosing `ranges`.
Empty ranges preserve identity; missing ranges provide no CPU mapping. Raw `reg`
properties remain available when translation fails, and
`roanix,unmapped-reg-count` records the number of windows left unmapped. PCI
configuration addresses are retained as metadata; memory and I/O spaces are
matched separately when explicit windows map them into CPU address space.
The parser accepts FDT v17-compatible headers and rejects malformed lengths,
overflowing spans, duplicate phandles and unterminated strings.

Each bus also exports translated windows in `roanix,bus-ranges`, a list of u64
rows `(kind, flags, bus_start, cpu_start, length)`. This preserves addresses above
4 GiB. Raw ranges and interrupt-map properties remain available. Adopting these
windows in PCI BAR translation/allocation remains part of the active review work.

## PCI resources

PCI resources retain BAR names (`bar0` through `bar5`) across memory and I/O
kinds. Use `Device::resource_named` to select a specific BAR. MMIO consumers use
`ddk::pci::memory_bar(device, index)`, which rejects I/O or unassigned BARs with
`ENOTSUP` and reports an absent BAR with `ENOENT`. NVMe and xHCI require memory
BAR0; another memory BAR cannot substitute for it. UART port access requires
x86_64 and a complete register range within the 16-bit port address space.

The enumerator sizes and restores I/O, 32-bit memory and 64-bit memory BARs.
Both slots of a 64-bit BAR are consumed even when no usable address is available.
Firmware-assigned I/O resources are published with their original port numbers.
Unassigned BARs are recorded in the `pci.unassigned-bars` bitmap and are excluded
from physical resources. The current QEMU virt MMIO allocator can assign memory
BARs in its supported window; it does not invent I/O assignments. Firmware bus
window translation and allocation are tracked with the remaining FDT work.

BAR sizing restores the original command register; bus preparation enables the
resource types present while preserving firmware bridge decode. Interrupt mode
requirements belong to each consumer. NVMe's MSI requirement on x86_64 leaves
the PCI bus usable by devices with other interrupt modes or no interrupt pin.

## Synchronization

Use `ddk::Mutex<T>` for thread operations that allocate, publish devices, access
services, or wait. It uses the kernel's adaptive mutex and priority inheritance;
its guard must be released on the acquiring thread. The receipt is created lazily,
so static initialization is supported. Uncontended firmware enumeration also
works during single-CPU bootstrap, before the scheduler exists. `try_lock` avoids
waiting for the protected value; lazy setup and receipt bookkeeping can still
sleep.

Use `TicketLock::lock_irqsave` for short register/shared-state sections. The plain
ticket-lock API has been removed. Move owned resources out under the guard and
release the guard before unregistering, dropping allocated storage or invoking
thread-only kernel services. Local IRQ state must be restored on the same thread.

Dropping an idle DDK mutex destroys its kernel receipt. Forgetting a guard can
strand the acquisition; a busy Drop retains the receipt, and module unload
refuses while any holder or waiter remains. The final module sweep closes mutex
admission and reclaims idle receipts after callbacks and workers have stopped.
Busy teardown preserves receipts so callers can release them and retry.

## Service tables

Use a shared `repr(C)` operation record for each service. PCI MSI is defined in
`ddk::pci::MsiOps` and `<roanix/pci.h>`; USB host operations live in the USB ABI
crate. `TypedInterface<T>` retains the provider and checks published byte extent,
alignment, revision and required behavior flags before constructing `&T`.
Binding metadata comes from the kernel publication, independently of a table's
own size prefix. The unsafe constructor still requires the service name and
revisions to identify valid `T` values and callback signatures.

A service revision defines its layout and callback semantics. Compatible tables
may append fields while preserving the existing prefix; incompatible changes
require a new revision. Consumers specify an inclusive supported revision range;
the kernel chooses the newest provider inside that range. Current PCI MSI and
USB consumers accept revision 1 only. The historical minimum-only bind remains
available for consumers that explicitly assume all newer revisions are compatible.

`MAY_SLEEP` calls require thread context, and consumers that share a binding
across concurrent calls must require `CONCURRENT`. Flags describe the provider's
contract; binding does not serialize its callbacks. Keep the binding alive through
all calls and drain them before releasing it.

## Interrupt lifetime

Virtual IRQ numbers are opaque identities. Retired identities are not reused;
metadata storage is reclaimed after IRQ and thread readers drain. Do not index
arrays by a virtual IRQ value. Firmware IRQ resolutions belong to a device
binding and share routing until the final binding releases it.

IRQ actions retain provider dependencies (or leases for kernel consumers).
Domain withdrawal closes callback admission, drains calls and root-controller
readers, then permits the owner to free context. Domain registration is unsafe:
callbacks and context must satisfy their documented lifetime, concurrency and
IRQ-context requirements. Do not withdraw a domain/current mapping from its own
callback or recursively register mappings/handlers from setup.

## USB keyboards

`usb-xhci` claims PCI xHCI functions, owns their coherent command/event rings,
and enumerates directly attached USB devices. It publishes USB interfaces on
its `usb` bus and supplies a versioned `usb.host` service for control requests
and interrupt IN transfers. x86 uses one MSI/MSI-X interrupter; RISC-V uses
PLIC-routed INTx. Enumeration and waits run in worker context; the hard
handler acknowledges and drains completion events without allocating.

Host calls may run concurrently. Control and configuration transactions are
serialized per controller; interrupt IN admission is per endpoint and remains
held through the completed DMA copy. An overlapping receive on that endpoint
returns `EBUSY`, while a cancelled or disconnected endpoint returns `ENODEV`.
Failed endpoint configuration retains its DMA until controller teardown and
rejects further receives; an ambiguous timeout never turns into cached success.

`usb-keyboard` binds HID boot keyboard interfaces, selects boot protocol, and
translates modifier and six-key reports into Linux evdev keycodes. It shares
the evdev implementation with i8042. Up to eight keyboard endpoints, including
readers retaining disconnected devices, can coexist. Disconnection cancels
pending reads before freeing controller DMA state. A descriptor kept open after
unplug retains its static evdev endpoint until it can be reclaimed safely.

The initial host service supports root-port keyboards. External hubs, bulk and
isochronous transfers, HID report-protocol-only devices, and other USB classes
are not implemented. These limits do not require changes in kmscon's input path.

Modules may return `EDEFERRED` when a required bus is unavailable. The packaged
module loader retries them after other modules have loaded, stopping if a pass
makes no progress. Device-probe deferral remains a separate mechanism.

DMA descriptor publication and completion consumption use `ddk::device_barrier()`
(or `rdf_device_barrier()` from `<roanix/io.h>`). It orders coherent DMA memory
with device I/O, including RISC-V input/output domains. MMIO accessors perform
volatile accesses; callers place the device barrier at the protocol boundary.
Ordinary atomic/shared-memory fences are separate. These barriers neither
perform noncoherent cache maintenance nor flush posted device writes.

Device-filesystem helpers are namespace functions under `ddk::devfs`. Lookup
and root queries need no module receipt; registration and removal resolve the
calling module when they perform the operation.

Module teardown closes ordinary callback and registration admission before
withdrawing bindings and owned devices. A checked refusal before exit preserves
the failed receipt, callback context and executable backing; an explicit retry
can resume after its users release it. Successful descendant removals stay
removed even if a later ancestor refuses. Drivers must retain the context for
every unregister operation that returns an error.

After exit completes, a cleanup failure leaves the module retired. Cleanup can
continue while backing remains resident, but ordinary callbacks and publication
stay closed. Unload retries the framework cleanup without calling exit again.
The final transition closes cleanup callback admission and checks outstanding
pins before freeing the image. Kernel consumers of module-owned buses and
interface tables retain provider leases; metadata handles alone do not pin code.

Device removal closes both interface publication and binding admission. Retained
bindings can reject removal before the driver destroys its context. Subtree
removal processes descendants before ancestors, so child drivers can release
references to parent services. A refused remove callback restores admission.
Publications attributed to the bound driver retire on unbind, including for
kernel drivers. Independent capabilities published before binding retain their
device lifetime and can survive driver replacement. Every provider still keeps
its table and context valid until checked withdrawal succeeds.

Object and module callback handles are const pointers. Mutable pointers identify
consuming receipts or writable output storage; constness does not prevent
calling framework services that modify an object's logical state.

Registration cleanup follows one policy. Provider receipts have explicit checked
cleanup and no `Drop` cleanup. A refusal leaves both receipt and provider context
owned by the driver. Store them together and retry after users drain. Module
sweeps are the framework backstop; dropping a receipt does not disconnect users
or authorize freeing its callback context.

| Owned receipt | Completion operation | Refusal |
| --- | --- | --- |
| `DeviceRegistration` | `remove` | Preserves receipt; acknowledges already-removed ancestors |
| `BusRegistration` | `unregister` | Preserves receipt |
| `DriverRegistration` | `unregister` | Preserves receipt and remaining bindings |
| `InterfacePublication` | `withdraw` | Preserves receipt and table/context |
| `IrqDomain` | `unregister` | Preserves receipt and controller state |
| `BlockRegistration` | `unregister` | Preserves receipt and backend context |
| `Tty` | `unregister` | Preserves receipt and terminal context |

`DeviceBuilder::publish` returns a unique `DeviceRegistration`. Its `device()`
method yields a copyable `Device` view for resource queries, parenting, IRQ
resolution and driver calls. Probe arguments are views too. Views are revocable
identities; retaining one does not retain a registration, provider image or
callback context. An owner can acknowledge parent removal through `remove`
even after its view begins returning `ENODEV`. PCI keeps publication receipts
while using views for removal callbacks that reenter its configuration state.

IRQ actions and workers use consuming `release`/`join` operations. Their ABI
consumes valid receipts and drains callbacks; call them from an outside thread,
then free their contexts. They do not perform implicit cleanup from `Drop`.
Owned events, interface bindings, MMIO windows and DMA allocations instead own
kernel resources whose valid release does not require busy-user negotiation,
so they use RAII. The controller owns the hardware protocol separately: halt
DMA and join device callbacks before allowing its buffers or mappings to drop.
The native mutex retains a busy kernel receipt if a forgotten guard prevents
its destructor from completing; module unload refuses until that use drains.

FDT publication and PCI share `firmware_tree`, the FDT package's pure Rust
library. Interrupt resolution follows nested interrupt-map/mask entries and
retains terminal controller identities and multi-cell specifiers. Nonzero
pass-through requires matching source/target formats; unsupported transformations
fail explicitly. PCI consults each bridge's explicit interrupt map before INTx
swizzling toward the root nexus. A present map without a matching entry stops
resolution; only an absent map inherits the ancestor route.
A controller initializes its state, then calls `IrqDomain::set_device` to publish
its firmware identity. This may synchronously retry consumers. Firmware builders
use `add_firmware_irq` so the kernel defers until the named controller exists;
legacy `add_irq` retains its explicit-domain/default-domain behavior.

PCI ECAM ranges have separate configuration and BAR allocation ownership. FDT
host callbacks enumerate and remove only their own functions and mapping; other
roots remain usable. Segment/bus ranges and physical ECAM windows must not
overlap. Separate roots may reuse bus addresses when their CPU apertures are
disjoint. Identity hosts have no allocation aperture and reserve only known
firmware assignments. ACPI MCFG bases describe bus zero, so mapping an entry
with a nonzero first bus includes that bus's ECAM offset.

PCI bridges use firmware-configured bus numbers and positive-decode forwarding
apertures. Each BAR must fit the root translation and every ancestor window;
MSI-X table access uses the same check. Missing apertures leave explicit
unassigned/unmapped BAR metadata. Optional I/O and prefetch capabilities are
probed with decoding disabled and restored, using word writes beside W1C status.
Subtractive bridge modes report `ENOTSUP`. PCI publications follow bridge
ancestry, while host configuration/allocation ownership remains separate.
