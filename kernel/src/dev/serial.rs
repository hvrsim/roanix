//! Platform UART enumeration below the console bus.

use alloc::{boxed::Box, format, sync::Arc, vec::Vec};
use core::time::Duration;

use log::info;

use crate::{
    fs::{
        Error as FsError, Result as FsResult,
        devtempfs::{self, DeviceNodeKind, DeviceNodeOps},
    },
    sys::sync::Once,
};

use super::{
    DeviceNodeId, Error, KERNEL_DRIVER, Result,
    console::{ConsoleBackend, SerialSettings, Tty},
    platform::CONSOLE_RESOURCE,
};

struct DiscoveredUart {
    backend: Arc<dyn ConsoleBackend>,
    description: Box<str>,
    baud: u32,
}

static DISCOVERED: Once<Vec<DiscoveredUart>> = Once::new();
static STARTED: Once<()> = Once::new();

/// Discovers and initializes UART hardware before secondary CPUs start.
pub(super) fn discover() -> Result<usize> {
    if let Some(discovered) = DISCOVERED.get() {
        return Ok(discovered.len());
    }
    let discovered = discover_inner()?;
    let count = discovered.len();
    DISCOVERED.call_once(|| discovered);
    Ok(count)
}

/// Probes platform UARTs and publishes their TTY nodes.
pub(super) fn start() -> Result<usize> {
    if STARTED.get().is_some() {
        return Ok(0);
    }
    let discovered = DISCOVERED.get().ok_or(Error::NotInitialized)?;
    let console = super::platform_buses()?.console;
    for (index, uart) in discovered.iter().enumerate() {
        publish_uart(console, index, uart.backend.clone(), uart.baud)?;
        info!("dev: serial{index} {}", uart.description);
    }
    let count = discovered.len();
    STARTED.call_once(|| ());
    Ok(count)
}

#[cfg(target_arch = "x86_64")]
fn discover_inner() -> Result<Vec<DiscoveredUart>> {
    let mut discovered = Vec::new();
    for com_index in 0..4 {
        let Some(uart) = crate::arch::serial::LegacyUart::probe(com_index) else {
            continue;
        };
        let backend: Arc<dyn ConsoleBackend> = Arc::new(X86Backend { uart });
        discovered.push(DiscoveredUart {
            backend,
            description: format!("COM{} at I/O port 0x{:x}", com_index + 1, uart.base())
                .into_boxed_str(),
            baud: 9600,
        });
    }
    Ok(discovered)
}

#[cfg(target_arch = "riscv64")]
fn discover_inner() -> Result<Vec<DiscoveredUart>> {
    let tree = super::dtb::parse().ok_or(Error::NotFound)?;
    let mut discovered = Vec::new();
    let root = tree.find_node("/").ok_or(Error::NotFound)?;
    let mut uart_nodes = Vec::new();
    collect_identity_uart_nodes(root, &mut uart_nodes);
    for node in uart_nodes {
        let Some(region) = node.reg().and_then(|mut regions| regions.next()) else {
            continue;
        };
        let physical = region.starting_address as usize as u64;
        let Some(size) = region.size else {
            continue;
        };
        let shift = property_u32(node, "reg-shift").unwrap_or(0);
        let width = property_u32(node, "reg-io-width").unwrap_or(1);
        if shift > 8 || !matches!(width, 1 | 4) {
            continue;
        }
        let clock_hz = property_u32(node, "clock-frequency").unwrap_or(3_686_400);
        let baud = property_u32(node, "current-speed").unwrap_or(115_200);
        ensure_device_mapping(physical, size)?;
        let backend = Arc::new(Mmio8250::new(
            physical,
            size,
            shift as u8,
            width as u8,
            clock_hz,
        )?);
        backend
            .initialize(SerialSettings {
                baud,
                data_bits: 8,
                stop_bits: 1,
                parity: false,
                odd_parity: false,
            })
            .map_err(fs_error)?;
        discovered.push(DiscoveredUart {
            backend,
            description: format!("{} at MMIO 0x{:x}", node.name, physical).into_boxed_str(),
            baud,
        });
    }
    Ok(discovered)
}

fn publish_uart(
    console: super::BusId,
    index: usize,
    backend: Arc<dyn ConsoleBackend>,
    baud: u32,
) -> Result<()> {
    let device = super::register_device(KERNEL_DRIVER, console, &format!("serial{index}"))?;
    validate_console_capability(device.node())?;

    let tty: Arc<dyn DeviceNodeOps> = Tty::new(backend, index, baud)?;
    let filesystem = devtempfs::global().map_err(|_| Error::Filesystem)?;
    let name = format!("ttys{index}");
    filesystem
        .create_device(
            KERNEL_DRIVER,
            filesystem.root_id(),
            name.as_bytes(),
            DeviceNodeKind::Character,
            0o660,
            device.node(),
            tty,
        )
        .map_err(fs_error)
        .or_else(|error| {
            let _ = super::remove_node(KERNEL_DRIVER, device.node());
            Err(error)
        })?;
    Ok(())
}

fn validate_console_capability(device: DeviceNodeId) -> Result<()> {
    let resource = super::resolve_resource(device, CONSOLE_RESOURCE)?;
    let bytes = resource.data()?;
    if bytes.len() != 8
        || u32::from_le_bytes(
            bytes[..4]
                .try_into()
                .expect("console capability version width"),
        ) != 1
    {
        return Err(Error::AbiMismatch);
    }
    Ok(())
}

fn fs_error(error: FsError) -> Error {
    match error {
        FsError::NotFound => Error::NotFound,
        FsError::AlreadyExists => Error::AlreadyExists,
        FsError::Busy | FsError::NotEmpty => Error::Busy,
        FsError::PermissionDenied | FsError::ReadOnly => Error::PermissionDenied,
        FsError::NoSpace => Error::NoSpace,
        FsError::OutOfMemory => Error::OutOfMemory,
        FsError::InvalidArgument | FsError::NameTooLong => Error::InvalidArgument,
        _ => Error::Filesystem,
    }
}

#[cfg(target_arch = "x86_64")]
struct X86Backend {
    uart: crate::arch::serial::LegacyUart,
}

#[cfg(target_arch = "x86_64")]
impl ConsoleBackend for X86Backend {
    fn try_read(&self) -> Option<u8> {
        self.uart.try_read()
    }

    fn write(&self, bytes: &[u8]) {
        self.uart.write(bytes);
    }

    fn configure(&self, settings: SerialSettings) -> FsResult<()> {
        self.uart
            .configure(settings)
            .then_some(())
            .ok_or(FsError::InvalidArgument)
    }

    fn flush(&self) -> FsResult<()> {
        self.uart.flush();
        Ok(())
    }

    fn send_break(&self, duration: u64) -> FsResult<()> {
        self.uart.set_break(true);
        crate::sys::clock::sleep(Duration::from_millis(if duration == 0 {
            250
        } else {
            duration
        }));
        self.uart.set_break(false);
        Ok(())
    }
}

#[cfg(target_arch = "riscv64")]
struct Mmio8250 {
    base: usize,
    register_shift: u8,
    register_width: u8,
    clock_hz: u32,
    lock: crate::sys::smp::IrqSpinLock<()>,
}

#[cfg(target_arch = "riscv64")]
impl Mmio8250 {
    fn new(
        physical: u64,
        size: usize,
        register_shift: u8,
        register_width: u8,
        clock_hz: u32,
    ) -> Result<Self> {
        if clock_hz == 0 {
            return Err(Error::InvalidArgument);
        }
        let stride = 1usize
            .checked_shl(register_shift.into())
            .ok_or(Error::InvalidArgument)?;
        let required = 7usize
            .checked_mul(stride)
            .and_then(|offset| offset.checked_add(register_width as usize))
            .ok_or(Error::InvalidArgument)?;
        if stride < register_width as usize || required > size {
            return Err(Error::InvalidArgument);
        }
        let base = crate::mem::phys_to_virt(crate::mem::PhysAddr::new(physical)).as_u64() as usize;
        if !base.is_multiple_of(register_width as usize) {
            return Err(Error::InvalidArgument);
        }
        Ok(Self {
            base,
            register_shift,
            register_width,
            clock_hz,
            lock: crate::sys::smp::IrqSpinLock::new(()),
        })
    }

    unsafe fn read(&self, register: usize) -> u8 {
        let address = self.base + (register << self.register_shift);
        // SAFETY: the DTB described this mapped UART register range and the
        // caller holds the UART lock.
        unsafe {
            match self.register_width {
                1 => core::ptr::read_volatile(address as *const u8),
                4 => core::ptr::read_volatile(address as *const u32) as u8,
                _ => unreachable!(),
            }
        }
    }

    unsafe fn write(&self, register: usize, value: u8) {
        let address = self.base + (register << self.register_shift);
        // SAFETY: the DTB described this mapped UART register range and the
        // caller holds the UART lock.
        unsafe {
            match self.register_width {
                1 => core::ptr::write_volatile(address as *mut u8, value),
                4 => core::ptr::write_volatile(address as *mut u32, value as u32),
                _ => unreachable!(),
            }
        }
    }

    fn set_break(&self, enabled: bool) {
        let _lock = self.lock.lock();
        // SAFETY: the UART lock serializes line-control access.
        unsafe {
            let mut line = self.read(3);
            if enabled {
                line |= 1 << 6;
            } else {
                line &= !(1 << 6);
            }
            self.write(3, line);
        }
    }

    fn initialize(&self, settings: SerialSettings) -> FsResult<()> {
        self.configure_inner(settings, true)
    }

    fn configure_inner(&self, settings: SerialSettings, clear_fifos: bool) -> FsResult<()> {
        let denominator = 16u64 * u64::from(settings.baud);
        if denominator == 0 || denominator > u64::from(self.clock_hz) {
            return Err(FsError::InvalidArgument);
        }
        let divisor = (u64::from(self.clock_hz) + denominator / 2) / denominator;
        if divisor == 0 || divisor > u64::from(u16::MAX) {
            return Err(FsError::InvalidArgument);
        }
        let divisor = divisor as u16;
        let mut line = match settings.data_bits {
            5 => 0,
            6 => 1,
            7 => 2,
            8 => 3,
            _ => return Err(FsError::InvalidArgument),
        };
        if settings.stop_bits == 2 {
            line |= 1 << 2;
        }
        if settings.parity {
            line |= 1 << 3;
            if !settings.odd_parity {
                line |= 1 << 4;
            }
        }

        let _lock = self.lock.lock();
        // SAFETY: the UART lock serializes the complete DLAB sequence.
        unsafe {
            self.write(1, 0);
            self.write(3, 0x80);
            self.write(0, divisor as u8);
            self.write(1, (divisor >> 8) as u8);
            self.write(3, line);
            self.write(2, if clear_fifos { 0x07 } else { 0x01 });
            self.write(4, 0x0B);
        }
        Ok(())
    }
}

#[cfg(target_arch = "riscv64")]
impl ConsoleBackend for Mmio8250 {
    fn try_read(&self) -> Option<u8> {
        let _lock = self.lock.lock();
        // SAFETY: the UART lock serializes register access.
        unsafe { (self.read(5) & 0x01 != 0).then(|| self.read(0)) }
    }

    fn write(&self, bytes: &[u8]) {
        for byte in bytes {
            loop {
                // SAFETY: status polling is read-only.
                unsafe {
                    while self.read(5) & 0x20 == 0 {
                        core::hint::spin_loop();
                    }
                }
                let _lock = self.lock.lock();
                // SAFETY: the UART lock serializes the transmit-register write.
                unsafe {
                    if self.read(5) & 0x20 != 0 {
                        self.write(0, *byte);
                        break;
                    }
                }
            }
        }
    }

    fn configure(&self, settings: SerialSettings) -> FsResult<()> {
        self.configure_inner(settings, false)
    }

    fn flush(&self) -> FsResult<()> {
        // SAFETY: status polling is read-only.
        unsafe {
            while self.read(5) & 0x40 == 0 {
                core::hint::spin_loop();
            }
        }
        Ok(())
    }

    fn send_break(&self, duration: u64) -> FsResult<()> {
        self.set_break(true);
        crate::sys::clock::sleep(Duration::from_millis(if duration == 0 {
            250
        } else {
            duration
        }));
        self.set_break(false);
        Ok(())
    }
}

#[cfg(target_arch = "riscv64")]
fn node_is_enabled(node: fdt::node::FdtNode<'_, '_>) -> bool {
    node.property("status")
        .and_then(|property| property.as_str())
        .is_none_or(|status| matches!(status, "ok" | "okay"))
}

#[cfg(target_arch = "riscv64")]
fn node_is_8250(node: fdt::node::FdtNode<'_, '_>) -> bool {
    node.compatible().is_some_and(|compatible| {
        compatible.all().any(|name| {
            matches!(
                name,
                "ns16550a" | "ns16550" | "uart8250" | "snps,dw-apb-uart"
            )
        })
    })
}

#[cfg(target_arch = "riscv64")]
fn collect_identity_uart_nodes<'b, 'a: 'b>(
    bus: fdt::node::FdtNode<'b, 'a>,
    output: &mut Vec<fdt::node::FdtNode<'b, 'a>>,
) {
    for child in bus.children() {
        if node_is_enabled(child) && node_is_8250(child) {
            output.push(child);
            continue;
        }
        let identity_ranges = child
            .property("ranges")
            .is_some_and(|ranges| ranges.value.is_empty());
        if node_is_enabled(child) && identity_ranges {
            collect_identity_uart_nodes(child, output);
        }
    }
}

#[cfg(target_arch = "riscv64")]
fn property_u32(node: fdt::node::FdtNode<'_, '_>, name: &str) -> Option<u32> {
    let property = node.property(name)?;
    let bytes: [u8; 4] = property.value.try_into().ok()?;
    Some(u32::from_be_bytes(bytes))
}

#[cfg(target_arch = "riscv64")]
fn ensure_device_mapping(physical: u64, size: usize) -> Result<()> {
    use crate::{
        arch,
        mem::{self, PAGE_SIZE, PhysAddr, VmFlags},
    };

    let size = u64::try_from(size).map_err(|_| Error::InvalidArgument)?;
    let end = physical.checked_add(size).ok_or(Error::InvalidArgument)?;
    let start_page = PhysAddr::new(physical).align_down();
    let end_page = PhysAddr::new(end).align_up();
    let root = arch::paging::active_root();
    let flags = VmFlags::READ | VmFlags::WRITE | VmFlags::GLOBAL | VmFlags::DEVICE;
    let mut page = start_page;
    while page < end_page {
        let virtual_page = mem::phys_to_virt(page);
        // SAFETY: discovery runs before secondary harts start. It only queries
        // and installs global, device-typed mappings in the active kernel root.
        if unsafe { arch::paging::translate(root, virtual_page).is_none() } {
            // SAFETY: the DTB identifies this physical page as device MMIO and
            // discovery is the sole page-table mutator for this range.
            unsafe { arch::paging::map_page(root, virtual_page, page, flags) }
                .map_err(|_| Error::OutOfMemory)?;
        }
        page = page.checked_add(PAGE_SIZE).ok_or(Error::InvalidArgument)?;
    }
    Ok(())
}
