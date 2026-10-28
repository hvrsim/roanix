//! Root-port enumeration and endpoint configuration.
use crate::{
    controller::{Controller, Endpoint, Slot, error, get32, put32, put64},
    ring::{self, Trb},
};
use alloc::{ffi::CString, format, vec::Vec};
use core::{ptr, sync::atomic::Ordering};
use ddk::{Bus, DeviceBuilder, Error, Result};

impl Controller {
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
                self.disconnect(port, usize::from(slot));
            } else if connected && slot == 0 {
                if let Err(_error) = self.enumerate(port) {
                    ddk::log(
                        ddk::LOG_WARN,
                        c"USB root-port enumeration failed; will retry",
                    );
                }
            }
        }
    }
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
                self.disconnect(port, usize::from(slot));
            }
        }
        result
    }
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
                // PP is RW; change bits are W1C. Do not echo PED or link state.
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
                (c"usb.endpoint", u32::from(interface.endpoint)),
                (c"usb.max-packet", u32::from(interface.packet)),
                (c"usb.interval", u32::from(interface.interval)),
            ] {
                builder.add_u32(key, value)?;
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
    #[allow(clippy::verbose_bit_mask)] // USB endpoint number occupies the low nibble.
    pub fn configure_interrupt(
        &self,
        slot_id: usize,
        endpoint: u8,
        packet: u16,
        interval: u8,
    ) -> Result<usize> {
        if endpoint & 0x80 == 0
            || endpoint & 15 == 0
            || interval == 0
            || packet == 0
            || packet > 1024
        {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let dci = usize::from(endpoint & 15) * 2 + 1;
        let _guard = self.transaction.lock()?;
        let flags = {
            let h = self.hardware.lock_irqsave();
            let slot = h
                .slots
                .get(slot_id)
                .and_then(Option::as_ref)
                .filter(|s| s.online)
                .ok_or(Error::from_status(ddk::ENODEV))?;
            if slot.cancelled & (1 << dci) != 0 {
                return Err(Error::from_status(ddk::ENODEV));
            }
            if let Some(endpoint) = &slot.endpoints[dci] {
                return if endpoint.configured {
                    Ok(dci)
                } else {
                    Err(error())
                };
            }
            h.flags
        };
        let endpoint = Endpoint::new(self.device, flags)?;
        let input = {
            let mut h = self.hardware.lock_irqsave();
            let context_size = h.context_size;
            let slot = h.slots[slot_id]
                .as_mut()
                .filter(|s| s.online)
                .ok_or(Error::from_status(ddk::ENODEV))?;
            slot.input.fill_zero();
            put32(&slot.input, 1, 1 | (1 << dci));
            let slot_word = context_size / 4;
            for word in 0..slot_word {
                put32(&slot.input, slot_word + word, get32(&slot.context, word));
            }
            put32(
                &slot.input,
                slot_word,
                (get32(&slot.input, slot_word) & !(31 << 27)) | ((dci as u32) << 27),
            );
            let offset = (dci + 1) * slot_word;
            let rate = if slot.speed >= 3 {
                u32::from(interval.saturating_sub(1)).min(15)
            } else {
                (u32::from(interval).ilog2() + 3).min(10)
            };
            put32(&slot.input, offset, rate << 16);
            put32(
                &slot.input,
                offset + 1,
                (3 << 1) | (7 << 3) | (u32::from(packet) << 16),
            );
            put32(&slot.input, offset + 2, endpoint.ring.address() as u32 | 1);
            put32(
                &slot.input,
                offset + 3,
                (endpoint.ring.address() >> 32) as u32,
            );
            put32(
                &slot.input,
                offset + 4,
                u32::from(packet) | (u32::from(packet) << 16),
            );
            slot.endpoints[dci] = Some(endpoint);
            slot.input.device_address()
        };
        ring::barrier();
        self.command(Trb {
            parameter: input,
            control: (12 << 10) | ((slot_id as u32) << 24),
            ..Trb::default()
        })?;
        self.hardware
            .lock_irqsave()
            .endpoint_mut(slot_id, dci)?
            .configured = true;
        Ok(dci)
    }
    pub fn receive_interrupt(
        &self,
        slot: usize,
        endpoint: u8,
        packet: u16,
        interval: u8,
        buffer: &mut [u8],
    ) -> Result<usize> {
        if buffer.is_empty() || buffer.len() > 4096 {
            return Err(Error::from_status(ddk::EINVAL));
        }
        let dci = self.configure_interrupt(slot, endpoint, packet, interval)?;
        let event = {
            let mut h = self.hardware.lock_irqsave();
            let endpoint = h.endpoint_mut(slot, dci)?;
            let event = endpoint.prepare(buffer.len())?;
            endpoint.pending = endpoint.ring.push(Trb {
                parameter: endpoint.buffer.device_address(),
                status: buffer.len() as u32,
                control: (1 << 10) | (1 << 5) | 4,
            });
            h.registers.write32(h.doorbell + slot * 4, dci as u32);
            event
        };
        let count = self.wait_transfer(slot, dci, event)?;
        let mut h = self.hardware.lock_irqsave();
        let endpoint = h.endpoint_mut(slot, dci)?;
        // SAFETY: completion publishes at most the requested buffer length;
        // pending retains admission while this hardware lock protects the copy.
        unsafe {
            ptr::copy_nonoverlapping(endpoint.buffer.as_ptr(), buffer.as_mut_ptr(), count);
        }
        endpoint.pending = 0;
        Ok(count)
    }

    pub fn disconnect(&self, port: usize, slot_id: usize) {
        self.cancel(slot_id);
        let children = {
            let mut h = self.hardware.lock_irqsave();
            let Some(slot) = h.slots[slot_id].as_mut() else {
                return;
            };
            slot.online = false;
            core::mem::take(&mut slot.children)
        };
        let remaining: Vec<_> = children
            .into_iter()
            .filter_map(|mut child| child.remove().err().map(|_| child))
            .collect();
        if !remaining.is_empty() {
            self.hardware.lock_irqsave().slots[slot_id]
                .as_mut()
                .expect("retiring slot")
                .children = remaining;
            return;
        }
        let Ok(_guard) = self.transaction.lock() else {
            return;
        };
        if self
            .command(Trb {
                control: (10 << 10) | ((slot_id as u32) << 24),
                ..Trb::default()
            })
            .is_err()
        {
            return;
        }
        let slot = {
            let mut h = self.hardware.lock_irqsave();
            put64(&h.dcbaa, slot_id, 0);
            h.ports[port] = 0;
            h.slots[slot_id].take()
        };
        drop(slot); // DMA frees and event destruction may sleep.
    }
}
pub fn setup(kind: u8, request: u8, value: u16, index: u16, length: u16) -> u64 {
    u64::from(kind)
        | (u64::from(request) << 8)
        | (u64::from(value) << 16)
        | (u64::from(index) << 32)
        | (u64::from(length) << 48)
}

struct Interface {
    number: u8,
    class: u32,
    endpoint: u8,
    packet: u16,
    interval: u8,
}

fn interfaces(configuration: &[u8]) -> Result<Vec<Interface>> {
    let mut interfaces: Vec<Interface> = Vec::new();
    let mut current = None;
    let mut offset = 0;
    while offset + 2 <= configuration.len() {
        let size = usize::from(configuration[offset]);
        if size < 2 || offset + size > configuration.len() {
            return Err(error());
        }
        let desc = &configuration[offset..offset + size];
        match desc[1] {
            4 if size >= 9 => {
                current = (desc[3] == 0).then_some((
                    desc[2],
                    (u32::from(desc[5]) << 16) | (u32::from(desc[6]) << 8) | u32::from(desc[7]),
                ));
            }
            5 if size >= 7 && desc[2] & 0x80 != 0 && desc[3] & 3 == 3 => {
                if let Some((number, class)) = current {
                    let packet = u16::from_le_bytes([desc[4], desc[5]]) & 0x7ff;
                    if packet != 0
                        && packet <= 1024
                        && !interfaces
                            .iter()
                            .any(|interface| interface.number == number)
                    {
                        interfaces.push(Interface {
                            number,
                            class,
                            endpoint: desc[2],
                            packet,
                            interval: desc[6],
                        });
                    }
                }
            }
            _ => {}
        }
        offset += size;
    }
    if offset != configuration.len() {
        return Err(error());
    }
    Ok(interfaces)
}
