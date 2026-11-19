//!
//! # xHCI Ports and USB Devices
//!
//! Port work resets newly connected devices, addresses a controller slot,
//! and publishes USB interfaces. Enumeration completes the hardware
//! transaction before publishing children so class probes may submit
//! control transfers.
//!
//! Disconnect cancels transfers and removes child registrations before
//! releasing the slot. Busy child registrations remain available for retry.
//!

use crate::{
    controller::{Controller, Endpoint, RESERVED_TRANSFER, Slot, error, get32, put32, put64},
    ring::{self, Trb},
};
use alloc::{ffi::CString, format, sync::Arc, vec::Vec};
use core::{ptr, sync::atomic::Ordering};
use ddk::{Bus, DeviceBuilder, Error, Result};

/// USB descriptor values needed to configure one xHCI endpoint.
#[derive(Clone, Copy, Default)]
pub struct EndpointDescriptor {
    /// USB endpoint address including its direction bit.
    pub address: u8,

    /// Maximum packet payload.
    pub packet: u16,

    /// Number of additional packets in a `SuperSpeed` bulk burst.
    pub burst: u8,

    /// Polling interval for interrupt endpoints.
    pub interval: u8,
}

impl EndpointDescriptor {
    /// Maps the USB address to an xHCI device context index.
    pub fn dci(self) -> usize {
        usize::from(self.address & 15) * 2 + usize::from(self.address >> 7)
    }
}

/// Caller storage for one interrupt or bulk transfer.
pub enum Payload<'a> {
    /// Mutable destination for IN data.
    In(&'a mut [u8]),

    /// Readable source for OUT data.
    Out(&'a [u8]),
}

impl Controller {
    /// Enumerates connected ports and cleans up slots for disconnected devices.
    pub fn scan_ports(&self) {
        let count = { self.hardware.lock_irqsave().max_ports };
        for port in 0..count {
            if self.stopping.load(Ordering::Acquire) {
                break;
            }

            let (connected, slot, online) = {
                let h = self.hardware.lock_irqsave();
                (
                    h.registers.read32(h.portsc(port)) & 1 != 0,
                    h.ports[port],
                    h.slots[usize::from(h.ports[port])]
                        .as_ref()
                        .is_some_and(|slot| slot.online),
                )
            };

            if (!connected || !online) && slot != 0 {
                let _ = self.disconnect(port, usize::from(slot));
            } else if connected && slot == 0 {
                if let Err(_error) = self.enumerate(port) {
                    ddk::log(
                        ddk::LOG_WARN,
                        c"USB root-port enumeration failed and will be retried",
                    );
                }
            }
        }
    }

    /// Resets a connected port, addresses a slot, and publishes its USB interfaces.
    fn enumerate(&self, port: usize) -> Result<()> {
        let guard = self.transaction.lock()?;
        let enumeration = (|| {
            let speed = self.reset_port(port)?;
            let slot = self.address_slot(port, speed)?;
            Ok((slot, self.configuration(slot, speed)?))
        })();
        // Class probes call back into control transfers during publication.
        drop(guard);
        let result = match enumeration {
            Ok((slot, interfaces)) => self.publish_interfaces(slot, interfaces),
            Err(error) => Err(error),
        };

        if result.is_err() {
            let slot = { self.hardware.lock_irqsave().ports[port] };
            if slot != 0 {
                let _ = self.disconnect(port, usize::from(slot));
            }
        }
        result
    }

    /// Resets the port and returns the device speed reported by PORTSC.
    fn reset_port(&self, port: usize) -> Result<u32> {
        // Debounce connection before resetting a USB 2 port or a USB 3 link.
        ddk::sleep_ns(20_000_000);
        {
            let h = self.hardware.lock_irqsave();
            let offset = h.portsc(port);
            let value = h.registers.read32(offset);
            if value & 1 == 0 {
                return Err(Error::from_status(ddk::ENODEV));
            }

            let speed = (value >> 10) & 15;
            if value & 2 == 0 {
                // PP is read/write and change bits are W1C. Do not echo PED or link state.
                h.registers.write32(
                    offset,
                    (value & (1 << 9)) | if speed >= 4 { 1 << 31 } else { 1 << 4 },
                );
            }
        }

        let start = ddk::monotonic_ns();
        let speed = loop {
            let value = {
                let h = self.hardware.lock_irqsave();
                h.registers.read32(h.portsc(port))
            };

            if value & 1 == 0 {
                return Err(Error::from_status(ddk::ENODEV));
            }

            if value & 2 != 0 && value & (1 << 4) == 0 {
                break (value >> 10) & 15;
            }

            if ddk::monotonic_ns().wrapping_sub(start) > 1_000_000_000 {
                return Err(error());
            }
            ddk::sleep_ns(1_000_000);
        };

        if !(1..=4).contains(&speed) {
            return Err(Error::from_status(ddk::ENOTSUP));
        }

        Ok(speed)
    }

    /// Enables an xHCI slot and issues Address Device with its initialized contexts.
    fn address_slot(&self, port: usize, speed: u32) -> Result<usize> {
        let (flags, context_size) = {
            let h = self.hardware.lock_irqsave();
            (h.flags, h.context_size)
        };

        let slot = Slot::new(self.device, flags, speed)?;
        put32(&slot.input, 1, 3); // Add slot + default control endpoint.
        let slot_word = context_size / 4;
        put32(&slot.input, slot_word, (speed << 20) | (1 << 27));
        put32(&slot.input, slot_word + 1, ((port + 1) as u32) << 16);
        let max_packet = match speed {
            3 => 64,
            4..=15 => 512,
            _ => 8,
        };

        let ep0_word = context_size / 4 * 2;
        put32(
            &slot.input,
            ep0_word + 1,
            (3 << 1) | (4 << 3) | (max_packet << 16),
        );
        let ep0 = slot.endpoints[1].as_ref().ok_or_else(error)?;
        put32(&slot.input, ep0_word + 2, ep0.ring.address() as u32 | 1);
        put32(&slot.input, ep0_word + 3, (ep0.ring.address() >> 32) as u32);
        put32(&slot.input, ep0_word + 4, 8);
        let input_address = slot.input.device_address();
        let response = self.command(Trb {
            control: 9 << 10,
            ..Trb::default()
        })?;
        let slot_id = (response.control >> 24) as usize;
        {
            let mut h = self.hardware.lock_irqsave();
            if slot_id == 0 || slot_id >= h.slots.len() || h.slots[slot_id].is_some() {
                return Err(error());
            }
            put64(&h.dcbaa, slot_id, slot.context.device_address());
            h.slots[slot_id] = Some(slot);
            h.ports[port] = slot_id as u8;
        }
        ring::barrier();
        self.command(Trb {
            parameter: input_address,
            control: (11 << 10) | ((slot_id as u32) << 24),
            ..Trb::default()
        })?;
        Ok(slot_id)
    }

    /// Reads and selects a USB configuration, returning its supported interfaces.
    fn configuration(&self, slot_id: usize, speed: u32) -> Result<Vec<Interface>> {
        let (input_address, context_size) = {
            let h = self.hardware.lock_irqsave();
            (
                h.slots[slot_id]
                    .as_ref()
                    .ok_or_else(error)?
                    .input
                    .device_address(),
                h.context_size,
            )
        };

        let ep0_word = context_size / 4 * 2;
        let mut descriptor = [0; 18];
        self.control(slot_id, setup(0x80, 6, 0x100, 0, 8), &mut descriptor[..8])?;
        if descriptor[1] != 1 || descriptor[0] < 18 {
            return Err(error());
        }

        if speed == 1 && descriptor[7] != 8 {
            let packet = u32::from(descriptor[7]);
            if !matches!(packet, 16 | 32 | 64) {
                return Err(error());
            }
            {
                let h = self.hardware.lock_irqsave();
                let slot = h.slots[slot_id].as_ref().ok_or_else(error)?;
                put32(&slot.input, 1, 2);
                for word in 0..context_size / 4 {
                    put32(
                        &slot.input,
                        ep0_word + word,
                        get32(&slot.context, context_size / 4 + word),
                    );
                }
                put32(
                    &slot.input,
                    ep0_word + 1,
                    (get32(&slot.input, ep0_word + 1) & 0xffff) | (packet << 16),
                );
            }
            self.command(Trb {
                parameter: input_address,
                control: (13 << 10) | ((slot_id as u32) << 24),
                ..Trb::default()
            })?;
        }
        self.control(slot_id, setup(0x80, 6, 0x100, 0, 18), &mut descriptor)?;
        let mut head = [0; 9];
        self.control(slot_id, setup(0x80, 6, 0x200, 0, 9), &mut head)?;
        let length = usize::from(u16::from_le_bytes([head[2], head[3]]));
        if head[1] != 2 || !(9..=4096).contains(&length) {
            return Err(error());
        }

        let mut configuration = alloc::vec![0; length];
        let actual = self.control(
            slot_id,
            setup(0x80, 6, 0x200, 0, length as u16),
            &mut configuration,
        )?;
        if actual != length {
            return Err(error());
        }

        let interfaces = interfaces(&configuration)?;
        self.control(slot_id, setup(0, 9, u16::from(head[5]), 0, 0), &mut [])?;
        Ok(interfaces)
    }

    /// Publishes the USB interfaces as child devices of the controller.
    fn publish_interfaces(&self, slot_id: usize, interfaces: Vec<Interface>) -> Result<()> {
        let children = Vec::with_capacity(interfaces.len());
        self.hardware.lock_irqsave().slots[slot_id]
            .as_mut()
            .ok_or_else(error)?
            .children = children;
        for interface in interfaces {
            let name = CString::new(format!("usb-{}-{}-{}", self.id, slot_id, interface.number))
                .map_err(|_| error())?;
            let mut builder = DeviceBuilder::new(&name)?;
            builder.set_bus(Bus::find(c"usb")?)?;
            builder.set_parent(self.device)?;
            for (key, value) in [
                (c"usb.controller", self.id),
                (c"usb.slot", slot_id as u32),
                (c"usb.interface", u32::from(interface.number)),
                (c"class", interface.class),
            ] {
                builder.add_u32(key, value)?;
            }

            if let Some(endpoint) = interface.interrupt {
                builder.add_u32(c"usb.endpoint", u32::from(endpoint.address))?;
                builder.add_u32(c"usb.max-packet", u32::from(endpoint.packet))?;
                builder.add_u32(c"usb.interval", u32::from(endpoint.interval))?;
            }

            if let (Some(input), Some(output)) = (interface.bulk_in, interface.bulk_out) {
                for (key, endpoint) in [(c"usb.bulk-in", input), (c"usb.bulk-out", output)] {
                    let packed = u32::from(endpoint.address)
                        | (u32::from(endpoint.packet) << 8)
                        | (u32::from(endpoint.burst) << 24);
                    builder.add_u32(key, packed)?;
                }
            }

            let child = builder.publish()?;
            // Capacity was reserved before acquiring the IRQ lock.
            self.hardware.lock_irqsave().slots[slot_id]
                .as_mut()
                .ok_or_else(error)?
                .children
                .push(child);
        }
        ddk::log(ddk::LOG_INFO, c"USB device enumerated");
        Ok(())
    }

    /// Checks the endpoint state without taking the command gate on the I/O path.
    fn endpoint_configured(&self, slot_id: usize, dci: usize, capacity: usize) -> Result<bool> {
        let h = self.hardware.lock_irqsave();
        let slot = h
            .slots
            .get(slot_id)
            .and_then(Option::as_ref)
            .filter(|slot| slot.online && slot.cancelled & (1 << dci) == 0)
            .ok_or(Error::from_status(ddk::ENODEV))?;
        match &slot.endpoints[dci] {
            Some(endpoint) if endpoint.configured && endpoint.buffer.0.len() == capacity => {
                Ok(true)
            }
            Some(_) => Err(error()),
            None => Ok(false),
        }
    }

    /// Installs a transfer ring and its xHCI endpoint context on first use.
    fn configure_endpoint(
        &self,
        slot_id: usize,
        descriptor: EndpointDescriptor,
        bulk: bool,
    ) -> Result<usize> {
        let dci = descriptor.dci();
        let input_direction = dci & 1 != 0;
        if dci < 2
            || descriptor.packet == 0
            || descriptor.packet > 1024
            || (!bulk && (!input_direction || descriptor.interval == 0))
        {
            return Err(Error::from_status(ddk::EINVAL));
        }

        let capacity = if bulk { usb::BULK_MAX_TRANSFER } else { 4096 };
        if self.endpoint_configured(slot_id, dci, capacity)? {
            return Ok(dci);
        }

        let _guard = self.transaction.lock()?;
        if self.endpoint_configured(slot_id, dci, capacity)? {
            return Ok(dci);
        }

        let flags = self.hardware.lock_irqsave().flags;
        let endpoint = Endpoint::new(self.device, flags, capacity)?;
        let input = {
            let mut h = self.hardware.lock_irqsave();
            let context_size = h.context_size;
            let slot = h.slots[slot_id]
                .as_mut()
                .filter(|s| s.online)
                .ok_or(Error::from_status(ddk::ENODEV))?;
            if slot.cancelled & (1 << dci) != 0 {
                return Err(Error::from_status(ddk::ENODEV));
            }

            let dequeue = endpoint.ring.producer();
            let input = endpoint_context(slot, context_size, descriptor, dequeue, bulk, false);
            slot.endpoints[dci] = Some(endpoint);
            input
        };
        self.submit_endpoint_context(slot_id, input)?;
        self.hardware
            .lock_irqsave()
            .endpoint_mut(slot_id, dci)?
            .configured = true;
        Ok(dci)
    }

    /// Transfers data through an interrupt or bulk endpoint with one TD in flight.
    pub fn transfer(
        &self,
        slot: usize,
        descriptor: EndpointDescriptor,
        bulk: bool,
        payload: Payload<'_>,
    ) -> Result<usize> {
        let (length, input) = match &payload {
            Payload::In(buffer) => (buffer.len(), true),
            Payload::Out(buffer) => (buffer.len(), false),
        };

        let limit = if bulk { usb::BULK_MAX_TRANSFER } else { 4096 };
        if length == 0 || length > limit || input != (descriptor.address & 0x80 != 0) {
            return Err(Error::from_status(ddk::EINVAL));
        }

        let dci = self.configure_endpoint(slot, descriptor, bulk)?;
        let (event, dma) = {
            let mut h = self.hardware.lock_irqsave();
            let endpoint = h.endpoint_mut(slot, dci)?;
            let event = endpoint.prepare(length)?;
            endpoint.pending = RESERVED_TRANSFER; // Reserve admission before staging OUT data.
            (event, Arc::clone(&endpoint.buffer))
        };

        if let Payload::Out(buffer) = &payload {
            // SAFETY: the reserved endpoint owns this DMA buffer until completion.
            // The transfer has not yet been published to the controller.
            unsafe { ptr::copy_nonoverlapping(buffer.as_ptr(), dma.0.as_ptr(), length) };
        }
        {
            let mut h = self.hardware.lock_irqsave();
            let endpoint = h.endpoint_mut(slot, dci)?;
            if endpoint.cancelled {
                return Err(Error::from_status(ddk::ENODEV));
            }
            endpoint.pending = endpoint.ring.push(Trb {
                parameter: dma.0.device_address(),
                status: length as u32,
                control: (1 << 10) | (1 << 5) | (u32::from(input) << 2),
            });
            h.registers.write32(h.doorbell + slot * 4, dci as u32);
        }

        let count = self.wait_transfer(slot, dci, event, bulk.then_some(10_000_000_000))?;
        if let Payload::In(buffer) = payload {
            // SAFETY: completion bounds the copy and releases controller access.
            // The endpoint retains admission through the copy, outside the IRQ lock.
            unsafe { ptr::copy_nonoverlapping(dma.0.as_ptr(), buffer.as_mut_ptr(), count) };
        }
        self.hardware
            .lock_irqsave()
            .endpoint_mut(slot, dci)?
            .pending = 0;
        Ok(count)
    }

    /// Issues Configure Endpoint after publishing its input context.
    fn submit_endpoint_context(&self, slot: usize, input: u64) -> Result<()> {
        ring::barrier();
        self.command(Trb {
            parameter: input,
            control: (12 << 10) | ((slot as u32) << 24),
            ..Trb::default()
        })
        .map(|_| ())
    }

    /// Resets host and device data sequencing with no other transfer admitted.
    pub fn clear_bulk_halt(&self, slot: usize, descriptor: EndpointDescriptor) -> Result<()> {
        let dci = self.configure_endpoint(slot, descriptor, true)?;
        let _guard = self.transaction.lock()?;
        let halted = {
            let mut h = self.hardware.lock_irqsave();
            let endpoint = h.endpoint_mut(slot, dci)?;
            if endpoint.cancelled {
                return Err(Error::from_status(ddk::ENODEV));
            }

            if endpoint.pending != 0 {
                return Err(Error::from_status(ddk::EBUSY));
            }
            endpoint.pending = RESERVED_TRANSFER;
            endpoint.halted
        };

        let recovery = (|| {
            if halted {
                self.recover_halt(slot, dci)?;
            } else {
                self.reset_bulk_toggle(slot, descriptor)?;
            }
            self.control(
                slot,
                setup(0x02, 1, 0, u16::from(descriptor.address), 0),
                &mut [],
            )
            .map(|_| ())
        })();
        let mut h = self.hardware.lock_irqsave();
        let endpoint = h.endpoint_mut(slot, dci)?;
        endpoint.pending = 0;
        endpoint.cancelled |= recovery.is_err();
        recovery
    }

    /// Reinstalls an idle bulk endpoint to reset its host-side data toggle.
    fn reset_bulk_toggle(&self, slot: usize, descriptor: EndpointDescriptor) -> Result<()> {
        let dci = descriptor.dci();
        let input = {
            let mut h = self.hardware.lock_irqsave();
            let context_size = h.context_size;
            let state = h.slots[slot]
                .as_mut()
                .filter(|s| s.online)
                .ok_or(Error::from_status(ddk::ENODEV))?;
            let dequeue = state.endpoints[dci]
                .as_ref()
                .ok_or_else(error)?
                .ring
                .producer();
            endpoint_context(state, context_size, descriptor, dequeue, true, true)
        };
        self.submit_endpoint_context(slot, input)
    }

    /// Retires a slot and releases its DMA only after children and hardware are gone.
    pub fn disconnect(&self, port: usize, slot_id: usize) -> Result<()> {
        let children = {
            let mut h = self.hardware.lock_irqsave();
            let Some(slot) = h.slots[slot_id].as_mut() else {
                return Ok(());
            };
            slot.retire();
            core::mem::take(&mut slot.children)
        };

        let mut result = Ok(());
        let remaining: Vec<_> = children
            .into_iter()
            .filter_map(|mut child| match child.remove() {
                Ok(()) => None,
                Err(error) => {
                    result = Err(error);
                    Some(child)
                }
            })
            .collect();
        if result.is_err() {
            self.hardware.lock_irqsave().slots[slot_id]
                .as_mut()
                .expect("retiring slot")
                .children = remaining;
            return result;
        }

        let _guard = self.transaction.lock()?;
        self.command(Trb {
            control: (10 << 10) | ((slot_id as u32) << 24),
            ..Trb::default()
        })?;
        let slot = {
            let mut h = self.hardware.lock_irqsave();
            put64(&h.dcbaa, slot_id, 0);
            h.ports[port] = 0;
            h.slots[slot_id].take()
        };
        drop(slot); // DMA frees and event destruction may sleep.
        Ok(())
    }
}

/// Builds a Configure Endpoint input context for a new or idle endpoint.
fn endpoint_context(
    slot: &mut Slot,
    context_size: usize,
    descriptor: EndpointDescriptor,
    dequeue: u64,
    bulk: bool,
    reset: bool,
) -> u64 {
    let dci = descriptor.dci();
    let slot_word = context_size / 4;
    slot.input.fill_zero();
    put32(&slot.input, 0, u32::from(reset) << dci);
    put32(&slot.input, 1, 1 | (1 << dci));
    for word in 0..slot_word {
        put32(&slot.input, slot_word + word, get32(&slot.context, word));
    }

    let slot_context = get32(&slot.input, slot_word);
    let entries = (slot_context >> 27).max(dci as u32);
    put32(
        &slot.input,
        slot_word,
        (slot_context & !(31 << 27)) | (entries << 27),
    );

    let offset = (dci + 1) * slot_word;
    let interval = u32::from(descriptor.interval);
    let rate = if bulk {
        0
    } else if slot.speed >= 3 {
        (interval - 1).min(15)
    } else {
        (interval.ilog2() + 3).min(10)
    };

    let burst = if bulk && slot.speed >= 4 {
        u32::from(descriptor.burst)
    } else {
        0
    };

    let kind = if bulk {
        if dci & 1 == 0 { 2 } else { 6 }
    } else {
        7
    };

    let packet = u32::from(descriptor.packet);
    put32(&slot.input, offset, rate << 16);
    put32(
        &slot.input,
        offset + 1,
        (3 << 1) | (kind << 3) | (burst << 8) | (packet << 16),
    );
    put32(&slot.input, offset + 2, dequeue as u32);
    put32(&slot.input, offset + 3, (dequeue >> 32) as u32);
    put32(
        &slot.input,
        offset + 4,
        packet | if bulk { 0 } else { packet << 16 },
    );
    slot.input.device_address()
}

/// Packs an eight-byte USB setup packet into the controller's immediate-data format.
pub fn setup(kind: u8, request: u8, value: u16, index: u16, length: u16) -> u64 {
    u64::from(kind)
        | (u64::from(request) << 8)
        | (u64::from(value) << 16)
        | (u64::from(index) << 32)
        | (u64::from(length) << 48)
}

/// USB interface descriptor data used when publishing a class-driver device.
struct Interface {
    /// USB interface number used in control requests.
    number: u8,

    /// Packed class, subclass, and protocol bytes.
    class: u32,

    /// Interrupt IN endpoint used by HID interfaces.
    interrupt: Option<EndpointDescriptor>,

    /// Bulk IN endpoint used by the class protocol.
    bulk_in: Option<EndpointDescriptor>,

    /// Bulk OUT endpoint used by the class protocol.
    bulk_out: Option<EndpointDescriptor>,
}

/// Decodes endpoints on default alternate settings and their `SuperSpeed` burst sizes.
fn interfaces(configuration: &[u8]) -> Result<Vec<Interface>> {
    let mut interfaces = Vec::new();
    let mut current = None;
    let mut remaining = configuration;
    while let Some(&size) = remaining.first() {
        let size = usize::from(size);
        if size < 2 || size > remaining.len() {
            return Err(error());
        }

        let (desc, rest) = remaining.split_at(size);
        remaining = rest;
        match desc[1] {
            4 => {
                current = None;
                if size < 9 {
                    return Err(error());
                }

                if desc[3] == 0 {
                    let class = u32::from_be_bytes([0, desc[5], desc[6], desc[7]]);
                    interfaces.push(Interface {
                        number: desc[2],
                        class,
                        interrupt: None,
                        bulk_in: None,
                        bulk_out: None,
                    });
                    current = Some(interfaces.len() - 1);
                }
            }
            5 if size >= 7 => {
                let Some(index) = current else { continue };
                let interface = &mut interfaces[index];
                let packet = u16::from_le_bytes([desc[4], desc[5]]) & 0x7ff;
                if !(1..=1024).contains(&packet) || desc[2].trailing_zeros() >= 4 {
                    continue;
                }

                let input = desc[2] & 0x80 != 0;
                let kind = desc[3] & 3;
                let present = match (kind, input) {
                    (2, true) => &mut interface.bulk_in,
                    (2, false) => &mut interface.bulk_out,
                    (3, true) => &mut interface.interrupt,
                    _ => continue,
                };

                let burst = match rest {
                    [size, 0x30, burst, ..]
                        if kind == 2 && *size >= 6 && usize::from(*size) <= rest.len() =>
                    {
                        (*burst).min(15)
                    }
                    _ => 0,
                };
                present.get_or_insert(EndpointDescriptor {
                    address: desc[2],
                    packet,
                    burst,
                    interval: desc[6],
                });
            }
            _ => {}
        }
    }
    interfaces.retain(|interface| {
        interface.interrupt.is_some()
            || (interface.bulk_in.is_some() && interface.bulk_out.is_some())
    });
    Ok(interfaces)
}
