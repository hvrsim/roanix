// Included by `api.rs`. These are the entry points in its export list; each one
// validates its arguments, performs the framework operation, and converts the
// result into the ABI's status convention.

fn module_ref(handle: *const c_void) -> Result<Option<Arc<Module>>> {
    if handle.is_null() {
        return Ok(None);
    }
    // SAFETY: the handle came from the framework, which only hands out module
    // pointers, and the tag check rejects anything else.
    unsafe { obj::upgrade::<Module>(handle) }.map(Some)
}

fn object<T: Object>(handle: *const c_void) -> Result<Arc<T>> {
    // SAFETY: the handle came from the framework and the tag check rejects a
    // pointer of the wrong type or to a poisoned object.
    unsafe { obj::upgrade::<T>(handle) }
}

fn optional_object<T: Object>(handle: *const c_void) -> Result<Option<Arc<T>>> {
    if handle.is_null() {
        return Ok(None);
    }
    object::<T>(handle).map(Some)
}

fn status<T>(result: Result<T>) -> i32 {
    match result {
        Ok(_) => STATUS_OK,
        Err(error) => error.to_status(),
    }
}

// --- Diagnostics and memory -------------------------------------------------

unsafe extern "C" fn shim_log(level: u32, module: *const c_char, message: *const c_char) {
    let level = clamp_level(level);
    if level > klog::record_level() {
        return;
    }

    // SAFETY: the ABI requires both arguments to be NUL-terminated strings.
    let name = unsafe { borrow_opt_str(module) }
        .ok()
        .flatten()
        .unwrap_or("driver");
    // SAFETY: as above.
    let Ok(text) = (unsafe { borrow_str(message) }) else {
        return;
    };
    klog::emit(level, name, "", 0, text.as_bytes(), 0);
}

fn layout(size: usize, align: usize) -> Option<core::alloc::Layout> {
    let align = align.max(1);
    core::alloc::Layout::from_size_align(size.max(1), align.next_power_of_two()).ok()
}

unsafe extern "C" fn shim_alloc(size: usize, align: usize) -> *mut c_void {
    let Some(layout) = layout(size, align) else {
        return core::ptr::null_mut();
    };
    // SAFETY: the layout has a non-zero size and a power-of-two alignment.
    unsafe { alloc::alloc::alloc(layout) }.cast()
}

unsafe extern "C" fn shim_alloc_zeroed(size: usize, align: usize) -> *mut c_void {
    let Some(layout) = layout(size, align) else {
        return core::ptr::null_mut();
    };
    // SAFETY: as above.
    unsafe { alloc::alloc::alloc_zeroed(layout) }.cast()
}

unsafe extern "C" fn shim_free(pointer: *mut c_void, size: usize, align: usize) {
    if pointer.is_null() {
        return;
    }
    let Some(layout) = layout(size, align) else {
        return;
    };
    // SAFETY: the ABI requires the caller to pass the layout the allocation was
    // made with.
    unsafe { alloc::alloc::dealloc(pointer.cast(), layout) };
}

// --- Modules ----------------------------------------------------------------

unsafe extern "C" fn shim_module_name(handle: *const c_void) -> *const c_char {
    // SAFETY: the handle came from the framework.
    match unsafe { obj::borrow::<Module>(handle) } {
        Ok(module) => module.name_c_ptr(),
        Err(_) => core::ptr::null(),
    }
}

unsafe extern "C" fn shim_module_find(name: *const c_char, out: *mut *const c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let module = super::super::core::module::find(name)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, obj::into_handle(module)) };
        Ok(())
    })())
}

// --- Device construction ----------------------------------------------------

unsafe extern "C" fn shim_device_new(
    module: *const c_void,
    name: *const c_char,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let builder = DeviceBuilder::new(owner.as_ref(), name)?;
        let boxed = alloc::boxed::Box::new(builder);
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, alloc::boxed::Box::into_raw(boxed).cast()) };
        Ok(())
    })())
}

fn builder<'a>(handle: *mut c_void) -> Result<&'a mut DeviceBuilder> {
    if handle.is_null() {
        return Err(Error::InvalidArgument);
    }
    // SAFETY: the handle came from `shim_device_new` and the ABI forbids using
    // it after it is consumed or discarded.
    Ok(unsafe { &mut *handle.cast::<DeviceBuilder>() })
}

unsafe extern "C" fn shim_device_set_parent(handle: *mut c_void, parent: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        let parent = object::<Device>(parent)?;
        replace_builder(target, |value| value.parent(&parent));
        Ok(())
    })())
}

fn replace_builder<F: FnOnce(DeviceBuilder) -> DeviceBuilder>(
    target: &mut DeviceBuilder,
    apply: F,
) {
    // The builder's setters consume and return the value, so swap through a
    // placeholder to apply one in place.
    let placeholder = DeviceBuilder::new(None, "placeholder")
        .expect("driver/abi: placeholder builder is always valid");
    let current = core::mem::replace(target, placeholder);
    *target = apply(current);
}

unsafe extern "C" fn shim_device_set_bus(handle: *mut c_void, bus: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        let bus = object::<Bus>(bus)?;
        replace_builder(target, |value| value.bus(&bus));
        Ok(())
    })())
}

unsafe extern "C" fn shim_device_set_fwnode(
    handle: *mut c_void,
    kind: u32,
    token: u64,
    provider: *const c_char,
    path: *const c_char,
) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        // SAFETY: the ABI requires NUL-terminated strings or null.
        let provider = unsafe { borrow_opt_str(provider) }?;
        // SAFETY: as above.
        let path = unsafe { borrow_opt_str(path) }?;
        let node = fwnode::create(kind, token, provider, path)?;
        replace_builder(target, |value| value.fwnode(&node));
        Ok(())
    })())
}

unsafe extern "C" fn shim_device_set_dma_mask(handle: *mut c_void, mask: u64) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        replace_builder(target, |value| value.dma_mask(mask));
        Ok(())
    })())
}

/// Property width selectors used by the integer entry points.
const WIDTH32: u32 = 0;

unsafe extern "C" fn shim_device_add_int(
    handle: *mut c_void,
    name: *const c_char,
    value: u64,
    width: u32,
) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let property = if width == WIDTH32 {
            PropValue::U32(u32::try_from(value).map_err(|_| Error::InvalidArgument)?)
        } else {
            PropValue::U64(value)
        };
        target.property(name, property)
    })())
}

unsafe extern "C" fn shim_device_add_string(
    handle: *mut c_void,
    name: *const c_char,
    value: *const c_char,
) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        // SAFETY: the ABI requires NUL-terminated strings.
        let name = unsafe { borrow_str(name) }?;
        // SAFETY: as above.
        let value = unsafe { borrow_str(value) }?;
        target.property(name, PropValue::Str(alloc::string::String::from(value).into()))
    })())
}

unsafe extern "C" fn shim_device_add_strings(
    handle: *mut c_void,
    name: *const c_char,
    values: *const *const c_char,
    count: usize,
) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        // SAFETY: the ABI requires `count` readable pointers.
        let entries = unsafe { borrow_slice(values, count) }?;
        let mut list = Vec::with_capacity(count);
        for entry in entries {
            // SAFETY: each element is required to be a NUL-terminated string.
            list.push(alloc::string::String::from(unsafe { borrow_str(*entry) }?).into());
        }
        target.property(name, PropValue::StrList(list.into_boxed_slice()))
    })())
}

unsafe extern "C" fn shim_device_add_cells(
    handle: *mut c_void,
    name: *const c_char,
    values: *const u64,
    count: usize,
    width: u32,
) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        // SAFETY: the ABI requires `count` readable elements.
        let cells = unsafe { borrow_slice(values, count) }?;
        let property = if width == WIDTH32 {
            let mut narrow = Vec::with_capacity(count);
            for value in cells {
                narrow.push(u32::try_from(*value).map_err(|_| Error::InvalidArgument)?);
            }
            PropValue::U32List(narrow.into_boxed_slice())
        } else {
            PropValue::U64List(cells.to_vec().into_boxed_slice())
        };
        target.property(name, property)
    })())
}

unsafe extern "C" fn shim_device_add_bytes(
    handle: *mut c_void,
    name: *const c_char,
    values: *const u8,
    count: usize,
) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        // SAFETY: the ABI requires `count` readable bytes.
        let bytes = unsafe { borrow_slice(values, count) }?;
        target.property(name, PropValue::Bytes(bytes.to_vec().into_boxed_slice()))
    })())
}

unsafe extern "C" fn shim_device_add_resource(
    handle: *mut c_void,
    kind: u32,
    flags: u32,
    start: u64,
    size: u64,
    name: *const c_char,
) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        let mut resource = Resource::new(kind, flags, start, size)?;
        // SAFETY: the ABI requires a NUL-terminated string or null.
        if let Some(label) = unsafe { borrow_opt_str(name) }? {
            resource = resource.with_name(label);
        }
        target.resource(resource).map(|_| ())
    })())
}

unsafe extern "C" fn shim_device_add_irq(
    handle: *mut c_void,
    domain: *const c_void,
    cells: *const u32,
    count: usize,
) -> i32 {
    status((|| -> Result<()> {
        let target = builder(handle)?;
        let domain = optional_object::<IrqDomain>(domain)?;
        // SAFETY: the ABI requires `count` readable cells.
        let cells = unsafe { borrow_slice(cells, count) }?;
        target.irq(domain, cells).map(|_| ())
    })())
}

unsafe extern "C" fn shim_device_add(handle: *mut c_void, out: *mut *const c_void) -> i32 {
    status((|| -> Result<()> {
        if handle.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the handle came from `shim_device_new` and is consumed here.
        let builder = unsafe { alloc::boxed::Box::from_raw(handle.cast::<DeviceBuilder>()) };
        let device = device::add(*builder)?;
        // SAFETY: the ABI requires a writable output pointer or null.
        unsafe { write_out(out, obj::into_handle(device)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_device_discard(handle: *mut c_void) {
    if handle.is_null() {
        return;
    }
    // SAFETY: the handle came from `shim_device_new` and is consumed here.
    drop(unsafe { alloc::boxed::Box::from_raw(handle.cast::<DeviceBuilder>()) });
}

unsafe extern "C" fn shim_device_remove(handle: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let device = object::<Device>(handle)?;
        device::remove(&device)
    })())
}

// --- Device inspection ------------------------------------------------------

unsafe extern "C" fn shim_device_root(out: *mut *const c_void) -> i32 {
    status((|| -> Result<()> {
        let root = device::root()?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, obj::into_handle(root)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_device_name(handle: *const c_void) -> *const c_char {
    // SAFETY: the handle came from the framework.
    match unsafe { obj::borrow::<Device>(handle) } {
        Ok(device) => device.name_c_ptr(),
        Err(_) => core::ptr::null(),
    }
}

unsafe extern "C" fn shim_device_parent(handle: *const c_void) -> *const c_void {
    // SAFETY: the handle came from the framework.
    let Ok(device) = (unsafe { obj::borrow::<Device>(handle) }) else {
        return core::ptr::null();
    };
    match device.parent() {
        Some(parent) => obj::into_handle(parent),
        None => core::ptr::null(),
    }
}

unsafe extern "C" fn shim_device_child_count(handle: *const c_void) -> usize {
    // SAFETY: the handle came from the framework.
    unsafe { obj::borrow::<Device>(handle) }.map_or(0, |device| device.children().len())
}

unsafe extern "C" fn shim_device_child(handle: *const c_void, index: usize) -> *const c_void {
    // SAFETY: the handle came from the framework.
    let Ok(device) = (unsafe { obj::borrow::<Device>(handle) }) else {
        return core::ptr::null();
    };
    match device.children().get(index) {
        Some(child) => obj::into_handle(child.clone()),
        None => core::ptr::null(),
    }
}

unsafe extern "C" fn shim_device_data(handle: *const c_void) -> *mut c_void {
    // SAFETY: the handle came from the framework.
    unsafe { obj::borrow::<Device>(handle) }
        .map_or(core::ptr::null_mut(), |device| device.drvdata() as *mut c_void)
}

unsafe extern "C" fn shim_device_set_data(handle: *const c_void, value: *mut c_void) {
    // SAFETY: the handle came from the framework.
    if let Ok(device) = unsafe { obj::borrow::<Device>(handle) } {
        device.set_drvdata(value as usize);
    }
}

unsafe extern "C" fn shim_device_int(
    handle: *const c_void,
    name: *const c_char,
    out: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        let device = object::<Device>(handle)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let value = device
            .properties()
            .get(name)
            .and_then(PropValue::as_u64)
            .ok_or(Error::NotFound)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, value) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_device_cell(
    handle: *const c_void,
    name: *const c_char,
    index: usize,
    out: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        let device = object::<Device>(handle)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let value = device
            .properties()
            .get(name)
            .and_then(|property| property.cell_at(index))
            .ok_or(Error::NotFound)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, value) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_device_string(
    handle: *const c_void,
    name: *const c_char,
    index: usize,
    buffer: *mut u8,
    capacity: usize,
    written: *mut usize,
) -> i32 {
    status((|| -> Result<()> {
        let device = object::<Device>(handle)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let value = device
            .properties()
            .get(name)
            .and_then(|property| property.string_at(index))
            .ok_or(Error::NotFound)?;
        // The result is NUL-terminated so C callers can use it directly.
        let mut bytes = Vec::with_capacity(value.len() + 1);
        bytes.extend_from_slice(value.as_bytes());
        bytes.push(0);
        // SAFETY: the ABI requires `capacity` writable bytes.
        unsafe { copy_out(&bytes, buffer, capacity, written) }
    })())
}

unsafe extern "C" fn shim_device_bytes(
    handle: *const c_void,
    name: *const c_char,
    buffer: *mut u8,
    capacity: usize,
    written: *mut usize,
) -> i32 {
    status((|| -> Result<()> {
        let device = object::<Device>(handle)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let value = device
            .properties()
            .get(name)
            .and_then(PropValue::bytes)
            .ok_or(Error::NotFound)?;
        // SAFETY: the ABI requires `capacity` writable bytes.
        unsafe { copy_out(value, buffer, capacity, written) }
    })())
}

unsafe extern "C" fn shim_device_property_len(
    handle: *const c_void,
    name: *const c_char,
) -> usize {
    // SAFETY: the handle came from the framework.
    let Ok(device) = (unsafe { obj::borrow::<Device>(handle) }) else {
        return 0;
    };
    // SAFETY: the ABI requires a NUL-terminated string.
    let Ok(name) = (unsafe { borrow_str(name) }) else {
        return 0;
    };
    device.properties().get(name).map_or(0, PropValue::len)
}

unsafe extern "C" fn shim_device_resource(
    handle: *const c_void,
    kind: u32,
    index: usize,
    start: *mut u64,
    size: *mut u64,
    flags: *mut u32,
) -> i32 {
    status((|| -> Result<()> {
        let device = object::<Device>(handle)?;
        let resource = device.resource(kind, index).ok_or(Error::NotFound)?;
        // SAFETY: the ABI requires writable output pointers or null.
        unsafe {
            write_out(start, resource.start);
            write_out(size, resource.size);
            write_out(flags, resource.flags);
        }
        Ok(())
    })())
}

unsafe extern "C" fn shim_device_fwnode(
    handle: *const c_void,
    kind: *mut u32,
    token: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        let device = object::<Device>(handle)?;
        let node = device.fwnode().ok_or(Error::NotFound)?;
        // SAFETY: the ABI requires writable output pointers or null.
        unsafe {
            write_out(kind, node.kind());
            write_out(token, node.token());
        }
        Ok(())
    })())
}

// --- Buses, drivers, and classes --------------------------------------------

unsafe extern "C" fn shim_bus_register(
    module: *const c_void,
    name: *const c_char,
    registration: *const BusRegistration,
    out: *mut *const c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        if registration.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires a readable registration record.
        let registration = unsafe { &*registration };
        if (registration.size as usize) < size_of::<BusRegistration>() {
            return Err(Error::InvalidArgument);
        }
        let ops = BusOps {
            match_device: registration.match_device,
            prepare: registration.prepare,
            cleanup: registration.cleanup,
            shutdown: registration.shutdown,
            context: registration.context,
        };
        // SAFETY: the module contract requires these callbacks to stay
        // executable until the bus is unregistered.
        let bus = unsafe { bus::register(owner.as_ref(), name, ops) }?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, obj::into_handle(bus)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_bus_unregister(handle: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let bus = object::<Bus>(handle)?;
        bus::unregister(&bus)
    })())
}

unsafe extern "C" fn shim_bus_find(name: *const c_char, out: *mut *const c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let bus = bus::find(name)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, obj::into_handle(bus)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_bus_set_dma_ops(handle: *const c_void, ops: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let bus = object::<Bus>(handle)?;
        // SAFETY: the module contract requires the table to stay valid until
        // the bus is unregistered.
        unsafe { bus.set_dma_ops(ops) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_driver_register(
    module: *const c_void,
    definition: *const DriverDef,
    out: *mut *const c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        if definition.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires a readable definition record.
        let definition = unsafe { &*definition };
        if (definition.size as usize) < size_of::<DriverDef>() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(definition.name) }?;
        let probe = definition.probe.ok_or(Error::InvalidArgument)?;
        // SAFETY: the ABI requires `match_count` readable entries.
        let entries = unsafe { borrow_slice(definition.matches, definition.match_count) }?;

        let mut matches = Vec::with_capacity(entries.len());
        for entry in entries {
            // SAFETY: the ABI requires NUL-terminated strings or null.
            let key = unsafe { borrow_opt_str(entry.key) }?;
            // SAFETY: as above.
            let value = unsafe { borrow_opt_str(entry.value) }?;
            matches.push(MatchEntry {
                kind: entry.kind,
                flags: entry.flags,
                key: key.map(|text| alloc::string::String::from(text).into()),
                value: value.map(|text| alloc::string::String::from(text).into()),
                id0: entry.id0,
                mask0: entry.mask0,
                id1: entry.id1,
                mask1: entry.mask1,
                data: entry.data,
                score: entry.score,
            });
        }

        let bus = optional_object::<Bus>(definition.bus)?;
        let registration: Registration = driver::registration(
            name,
            bus,
            definition.priority,
            matches,
            DriverOps {
                probe,
                remove: definition.remove,
                shutdown: definition.shutdown,
                context: definition.context,
            },
        )?;
        // SAFETY: the module contract requires these callbacks to stay
        // executable until the driver is unregistered.
        let driver = unsafe { driver::register(owner.as_ref(), registration) }?;
        // SAFETY: the ABI requires a writable output pointer or null.
        unsafe { write_out(out, obj::into_handle(driver)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_driver_unregister(handle: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let driver = object::<Driver>(handle)?;
        driver::unregister(&driver)
    })())
}

unsafe extern "C" fn shim_class_register(
    module: *const c_void,
    name: *const c_char,
    registration: *const ClassRegistration,
    out: *mut *const c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        if registration.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires a readable registration record.
        let registration = unsafe { &*registration };
        if (registration.size as usize) < size_of::<ClassRegistration>() {
            return Err(Error::InvalidArgument);
        }
        let ops = ClassOps {
            attach: registration.attach,
            detach: registration.detach,
            context: registration.context,
        };
        // SAFETY: the module contract requires these callbacks to stay
        // executable until the class is unregistered.
        let class = unsafe { class::register(owner.as_ref(), name, ops) }?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, obj::into_handle(class)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_class_unregister(handle: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let class = object::<Class>(handle)?;
        class::unregister(&class)
    })())
}

unsafe extern "C" fn shim_class_find(name: *const c_char, out: *mut *const c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let class = class::find(name)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, obj::into_handle(class)) };
        Ok(())
    })())
}

#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn shim_class_add(
    module: *const c_void,
    class: *const c_void,
    device: *const c_void,
    name: *const c_char,
    ops: *const c_void,
    ops_size: usize,
    context: *mut c_void,
    out: *mut *const c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        let class = object::<Class>(class)?;
        let device = optional_object::<Device>(device)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let membership = Membership {
            name: alloc::string::String::from(name).into(),
            ops,
            ops_size,
            context,
        };
        // SAFETY: the module contract requires the member table to stay valid
        // until the membership is removed.
        let member =
            unsafe { class::add_device(owner.as_ref(), &class, device.as_ref(), membership) }?;
        // SAFETY: the ABI requires a writable output pointer or null.
        unsafe { write_out(out, obj::into_handle(member)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_class_remove(handle: *const c_void) {
    if let Ok(member) = object::<ClassDevice>(handle) {
        class::remove_device(&member);
    }
}

unsafe extern "C" fn shim_class_member_ops(
    handle: *const c_void,
    ops: *mut *const c_void,
    context: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let member = object::<ClassDevice>(handle)?;
        // SAFETY: the ABI requires writable output pointers or null.
        unsafe {
            write_out(ops, member.ops());
            write_out(context, member.context());
        }
        Ok(())
    })())
}

unsafe extern "C" fn shim_class_member_name(handle: *const c_void) -> *const c_char {
    // SAFETY: the handle came from the framework.
    match unsafe { obj::borrow::<ClassDevice>(handle) } {
        Ok(member) => member.name_c_ptr(),
        Err(_) => core::ptr::null(),
    }
}

unsafe extern "C" fn shim_class_member_device(handle: *const c_void) -> *const c_void {
    // SAFETY: the handle came from the framework.
    let Ok(member) = (unsafe { obj::borrow::<ClassDevice>(handle) }) else {
        return core::ptr::null();
    };
    match member.device() {
        Some(device) => obj::into_handle(device),
        None => core::ptr::null(),
    }
}

unsafe extern "C" fn shim_class_member_data(handle: *const c_void) -> usize {
    // SAFETY: the handle came from the framework.
    unsafe { obj::borrow::<ClassDevice>(handle) }.map_or(0, ClassDevice::class_data)
}

unsafe extern "C" fn shim_class_member_set_data(handle: *const c_void, value: usize) {
    // SAFETY: the handle came from the framework.
    if let Ok(member) = unsafe { obj::borrow::<ClassDevice>(handle) } {
        member.set_class_data(value);
    }
}

unsafe extern "C" fn shim_class_member_count(handle: *const c_void) -> usize {
    // SAFETY: the handle came from the framework.
    unsafe { obj::borrow::<Class>(handle) }.map_or(0, |class| class.members().len())
}

unsafe extern "C" fn shim_class_member(handle: *const c_void, index: usize) -> *const c_void {
    // SAFETY: the handle came from the framework.
    let Ok(class) = (unsafe { obj::borrow::<Class>(handle) }) else {
        return core::ptr::null();
    };
    match class.members().get(index) {
        Some(member) => obj::into_handle(member.clone()),
        None => core::ptr::null(),
    }
}

// --- Interfaces -------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn shim_iface_publish(
    module: *const c_void,
    device: *const c_void,
    name: *const c_char,
    version: u32,
    flags: u32,
    ops: *const c_void,
    ops_size: usize,
    context: *mut c_void,
    out: *mut *const c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let publication = Publication {
            name: alloc::string::String::from(name).into(),
            version,
            flags,
            ops,
            ops_size,
            context,
        };
        let interface = match optional_object::<Device>(device)? {
            // SAFETY: the module contract requires the operation table to stay
            // valid until the interface is withdrawn.
            Some(device) => unsafe {
                iface::publish_device(owner.as_ref(), &device, publication)
            }?,
            // SAFETY: the same operation-table lifetime contract applies to a
            // global interface.
            None => unsafe { iface::publish_global(owner.as_ref(), publication) }?,
        };
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, obj::into_handle(interface)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_iface_withdraw(handle: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let interface = object::<Interface>(handle)?;
        iface::withdraw(&interface)
    })())
}

#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn shim_iface_bind(
    module: *const c_void,
    device: *const c_void,
    bind_scope: u32,
    name: *const c_char,
    min_version: u32,
    binding: *mut *const c_void,
    ops: *mut *mut c_void,
    context: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let consumer = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let reference = match bind_scope {
            scope::GLOBAL => iface::bind_global(consumer.as_ref(), name, min_version)?,
            scope::DEVICE => {
                let device = object::<Device>(device)?;
                iface::bind_device(consumer.as_ref(), &device, name, min_version)?
            }
            scope::ANCESTOR => {
                let device = object::<Device>(device)?;
                iface::bind_ancestor(consumer.as_ref(), &device, name, min_version)?
            }
            _ => return Err(Error::InvalidArgument),
        };
        let (table, call_context) = reference.ops();
        let boxed = alloc::boxed::Box::new(reference);
        // SAFETY: the ABI requires writable output pointers or null.
        unsafe {
            write_out(ops, table.cast_mut());
            write_out(context, call_context);
            write_out(
                binding,
                alloc::boxed::Box::into_raw(boxed).cast::<c_void>().cast_const(),
            );
        }
        Ok(())
    })())
}

unsafe extern "C" fn shim_iface_unbind(handle: *mut c_void) {
    if handle.is_null() {
        return;
    }
    // SAFETY: the handle came from `shim_iface_bind` and is consumed here.
    drop(unsafe { alloc::boxed::Box::from_raw(handle.cast::<InterfaceRef>()) });
}

unsafe extern "C" fn shim_iface_available(name: *const c_char, min_version: u32) -> i32 {
    // SAFETY: the ABI requires a NUL-terminated string.
    let Ok(name) = (unsafe { borrow_str(name) }) else {
        return 0;
    };
    i32::from(iface::available(name, min_version))
}

unsafe extern "C" fn shim_iface_count(name: *const c_char, min_version: u32) -> usize {
    // SAFETY: the ABI requires a NUL-terminated string.
    let Ok(name) = (unsafe { borrow_str(name) }) else {
        return 0;
    };
    iface::enumerate(name, min_version).map_or(0, |entries| entries.len())
}

unsafe extern "C" fn shim_iface_provider(
    name: *const c_char,
    min_version: u32,
    index: usize,
) -> *const c_void {
    // SAFETY: the ABI requires a NUL-terminated string.
    let Ok(name) = (unsafe { borrow_str(name) }) else {
        return core::ptr::null();
    };
    let Ok(entries) = iface::enumerate(name, min_version) else {
        return core::ptr::null();
    };
    match entries.get(index) {
        Some(interface) => obj::into_handle(interface.clone()),
        None => core::ptr::null(),
    }
}

unsafe extern "C" fn shim_probe_retrigger() {
    probe::retrigger();
}

// --- Interrupts -------------------------------------------------------------

unsafe extern "C" fn shim_irq_domain_register(
    module: *const c_void,
    name: *const c_char,
    flags: u32,
    hwirq_count: u32,
    registration: *const DomainRegistration,
    out: *mut *const c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        if registration.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires a readable registration record.
        let registration = unsafe { &*registration };
        if (registration.size as usize) < size_of::<DomainRegistration>() {
            return Err(Error::InvalidArgument);
        }
        let ops = DomainOps {
            translate: registration.translate,
            setup: registration.setup,
            teardown: registration.teardown,
            mask: registration.mask,
            unmask: registration.unmask,
            eoi: registration.eoi,
            set_affinity: registration.set_affinity,
            claim: registration.claim,
            complete: registration.complete,
            compose_message: registration.compose_message,
            context: registration.context,
        };
        // SAFETY: the module contract requires these callbacks to stay
        // executable until the domain is unregistered.
        let domain =
            unsafe { irq::register_domain(owner.as_ref(), name, flags, hwirq_count, ops) }?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, obj::into_handle(domain)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_irq_domain_unregister(handle: *const c_void) -> i32 {
    status((|| -> Result<()> {
        let domain = object::<IrqDomain>(handle)?;
        irq::unregister_domain(&domain)
    })())
}

unsafe extern "C" fn shim_irq_map(
    domain: *const c_void,
    hwirq: u64,
    flags: u32,
    out: *mut u32,
) -> i32 {
    status((|| -> Result<()> {
        let domain = object::<IrqDomain>(domain)?;
        let virq = irq::create_mapping(&domain, hwirq, flags)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, virq) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_irq_unmap(domain: *const c_void, hwirq: u64) -> i32 {
    status((|| -> Result<()> {
        let domain = object::<IrqDomain>(domain)?;
        irq::destroy_mapping(&domain, hwirq)
    })())
}

unsafe extern "C" fn shim_irq_of_device(
    device: *const c_void,
    index: usize,
    out: *mut u32,
) -> i32 {
    status((|| -> Result<()> {
        let device = object::<Device>(device)?;
        let virq = irq::device_virq(&device, index)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, virq) };
        Ok(())
    })())
}

#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn shim_irq_request(
    module: *const c_void,
    device: *const c_void,
    virq: u32,
    name: *const c_char,
    flags: u32,
    handler: Option<HandlerFn>,
    thread: Option<ThreadFn>,
    context: *mut c_void,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        let device = optional_object::<Device>(device)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let handler = handler.ok_or(Error::InvalidArgument)?;
        // SAFETY: the module contract requires these callbacks to stay
        // executable until the interrupt is released.
        let action = unsafe {
            irq::request(
                owner.as_ref(),
                device.as_ref(),
                virq,
                name,
                flags,
                handler,
                thread,
                context,
            )
        }?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, receipt(action)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_irq_release(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_irq_request` and is consumed here.
        let action = unsafe { claim_receipt::<IrqAction>(handle) }?;
        irq::release(&action)
    })())
}

unsafe extern "C" fn shim_irq_mask(virq: u32) -> i32 {
    status(irq::mask(virq))
}

unsafe extern "C" fn shim_irq_unmask(virq: u32) -> i32 {
    status(irq::unmask(virq))
}

unsafe extern "C" fn shim_irq_set_affinity(virq: u32, cpu: u32) -> i32 {
    status(irq::set_affinity(virq, cpu))
}

unsafe extern "C" fn shim_irq_alloc_vector(virq: u32, out: *mut u32) -> i32 {
    status((|| -> Result<()> {
        let vector = irq::allocate_vector(virq)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, vector) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_irq_free_vector(vector: u32) -> i32 {
    status(irq::free_vector(vector))
}

unsafe extern "C" fn shim_irq_compose_message(
    domain: *const c_void,
    hwirq: u64,
    address: *mut u64,
    data: *mut u32,
) -> i32 {
    status((|| -> Result<()> {
        let domain = object::<IrqDomain>(domain)?;
        let (message_address, message_data) = domain.compose_message(hwirq)?;
        // SAFETY: the ABI requires writable output pointers or null.
        unsafe {
            write_out(address, message_address);
            write_out(data, message_data);
        }
        Ok(())
    })())
}

// --- Register windows, ports, and DMA ---------------------------------------

unsafe extern "C" fn shim_mmio_map(
    module: *const c_void,
    physical: u64,
    length: usize,
    flags: u32,
    out: *mut MmioWindow,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        if out.is_null() {
            return Err(Error::InvalidArgument);
        }
        let mapping = mmio::map(owner.as_ref(), physical, length, flags)?;
        let window = MmioWindow {
            base: mapping.address().as_mut_ptr::<c_void>(),
            length: mapping.len(),
            token: receipt(mapping),
        };
        // SAFETY: the ABI requires a writable output record.
        unsafe { out.write(window) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_mmio_unmap(window: *mut MmioWindow) -> i32 {
    status((|| -> Result<()> {
        if window.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires a readable and writable window record.
        let token = unsafe { (*window).token };
        // SAFETY: the token came from `shim_mmio_map` and is consumed here.
        let mapping = unsafe { claim_receipt::<Mapping>(token) }?;
        let result = mmio::unmap(&mapping);
        // SAFETY: as above.
        unsafe {
            (*window).base = core::ptr::null_mut();
            (*window).length = 0;
            (*window).token = core::ptr::null_mut();
        }
        result
    })())
}

unsafe extern "C" fn shim_mmio_direct(physical: u64, out: *mut *mut c_void) -> i32 {
    status((|| -> Result<()> {
        let address = mmio::direct_map(physical)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, address.as_mut_ptr::<c_void>()) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_port_read8(port: u16) -> u32 {
    port::read8(port).map_or(u32::MAX, u32::from)
}

unsafe extern "C" fn shim_port_read16(port: u16) -> u32 {
    port::read16(port).map_or(u32::MAX, u32::from)
}

unsafe extern "C" fn shim_port_read32(port: u16) -> u32 {
    port::read32(port).unwrap_or(u32::MAX)
}

unsafe extern "C" fn shim_port_write8(port: u16, value: u8) {
    let _ = port::write8(port, value);
}

unsafe extern "C" fn shim_port_write16(port: u16, value: u16) {
    let _ = port::write16(port, value);
}

unsafe extern "C" fn shim_port_write32(port: u16, value: u32) {
    let _ = port::write32(port, value);
}

unsafe extern "C" fn shim_dma_alloc(
    module: *const c_void,
    device: *const c_void,
    size: usize,
    align: usize,
    flags: u32,
    out: *mut DmaWindow,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        let device = optional_object::<Device>(device)?;
        if out.is_null() {
            return Err(Error::InvalidArgument);
        }
        let buffer = dma::alloc_coherent(owner.as_ref(), device.as_ref(), size, align, flags)?;
        let window = DmaWindow {
            cpu: buffer.address().as_mut_ptr::<c_void>(),
            device: buffer.device_address(),
            length: buffer.len(),
            token: receipt(buffer),
        };
        // SAFETY: the ABI requires a writable output record.
        unsafe { out.write(window) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_dma_free(window: *mut DmaWindow) -> i32 {
    status((|| -> Result<()> {
        if window.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires a readable and writable window record.
        let token = unsafe { (*window).token };
        // SAFETY: the token came from `shim_dma_alloc` and is consumed here.
        let buffer = unsafe { claim_receipt::<DmaBuffer>(token) }?;
        let result = dma::free_coherent(&buffer);
        // SAFETY: as above.
        unsafe {
            (*window).cpu = core::ptr::null_mut();
            (*window).device = 0;
            (*window).length = 0;
            (*window).token = core::ptr::null_mut();
        }
        result
    })())
}

unsafe extern "C" fn shim_dma_map(
    device: *const c_void,
    address: *mut c_void,
    size: usize,
    direction: u32,
    out: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        let device = optional_object::<Device>(device)?;
        let bus_address = dma::map_single(
            device.as_ref(),
            VirtAddr::from_ptr(address.cast_const()),
            size,
            direction,
        )?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, bus_address) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_dma_unmap(device: *const c_void, address: u64, size: usize, direction: u32) {
    let device = optional_object::<Device>(device).ok().flatten();
    dma::unmap_single(device.as_ref(), address, size, direction);
}

unsafe extern "C" fn shim_dma_sync(address: *mut c_void, size: usize, direction: u32) {
    dma::sync(VirtAddr::from_ptr(address.cast_const()), size, direction);
}

// --- Deferred work, timers, and events --------------------------------------

unsafe extern "C" fn shim_work_create(
    module: *const c_void,
    name: *const c_char,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let queue = work::create_queue(owner.as_ref(), name)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, receipt(queue)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_work_queue(
    handle: *mut c_void,
    callback: Option<WorkFn>,
    context: *mut c_void,
    argument: u64,
) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_work_create`.
        let queue = unsafe { borrow_receipt::<WorkQueue>(handle) }?;
        let callback = callback.ok_or(Error::InvalidArgument)?;
        // SAFETY: the module contract requires the callback to stay executable
        // until the queue is destroyed.
        unsafe { work::queue_work(&queue, callback, context, argument) }
    })())
}

unsafe extern "C" fn shim_work_flush(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_work_create`.
        let queue = unsafe { borrow_receipt::<WorkQueue>(handle) }?;
        work::flush_queue(&queue);
        Ok(())
    })())
}

unsafe extern "C" fn shim_work_destroy(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_work_create` and is consumed here.
        let queue = unsafe { claim_receipt::<WorkQueue>(handle) }?;
        work::destroy_queue(&queue)
    })())
}

unsafe extern "C" fn shim_timer_create(
    module: *const c_void,
    callback: Option<WorkFn>,
    context: *mut c_void,
    argument: u64,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        let callback = callback.ok_or(Error::InvalidArgument)?;
        // SAFETY: the module contract requires the callback to stay executable
        // until the timer is destroyed.
        let timer = unsafe { work::create_timer(owner.as_ref(), callback, context, argument) }?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, receipt(timer)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_timer_arm(handle: *mut c_void, delay_ns: u64, period_ns: u64) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_timer_create`.
        let timer = unsafe { borrow_receipt::<Timer>(handle) }?;
        work::arm_timer(&timer, delay_ns, period_ns)
    })())
}

unsafe extern "C" fn shim_timer_cancel(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_timer_create`.
        let timer = unsafe { borrow_receipt::<Timer>(handle) }?;
        work::cancel_timer(&timer);
        Ok(())
    })())
}

unsafe extern "C" fn shim_timer_destroy(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_timer_create` and is consumed here.
        let timer = unsafe { claim_receipt::<Timer>(handle) }?;
        work::destroy_timer(&timer)
    })())
}

unsafe extern "C" fn shim_event_create(module: *const c_void, out: *mut usize) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        let event = super::events::create(owner.as_ref())?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, event) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_event_destroy(handle: usize) -> i32 {
    status(super::events::destroy(handle))
}

unsafe extern "C" fn shim_event_wait(handle: usize) -> i32 {
    status(super::events::with(handle, |event| {
        event.wait();
        Ok(())
    }))
}

unsafe extern "C" fn shim_event_signal(handle: usize) -> i32 {
    status(super::events::with(handle, |event| {
        event.signal();
        Ok(())
    }))
}

unsafe extern "C" fn shim_event_reset(handle: usize) -> i32 {
    status(super::events::with(handle, |event| {
        event.reset();
        Ok(())
    }))
}

unsafe extern "C" fn shim_event_wait_any(
    handles: *const usize,
    count: usize,
    out: *mut usize,
) -> i32 {
    const MAX_EVENTS: usize = 64;

    status((|| -> Result<()> {
        if count == 0 || count > MAX_EVENTS {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires `count` readable event handles.
        let handles = unsafe { borrow_slice(handles, count) }?;
        let mut events = Vec::with_capacity(handles.len());
        for handle in handles {
            events.push(super::events::resolve(*handle).ok_or(Error::InvalidArgument)?);
        }
        let winner = Event::wait_any(&events);
        // SAFETY: the ABI requires a writable output pointer or null.
        unsafe { write_out(out, winner) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_event_wait_timeout(
    handle: usize,
    nanoseconds: u64,
    out_signalled: *mut u8,
) -> i32 {
    status(super::events::with(handle, |event| {
        let signalled = clock::wait_timeout(event, core::time::Duration::from_nanos(nanoseconds));
        // SAFETY: the ABI requires a writable output pointer or null.
        unsafe { write_out(out_signalled, u8::from(signalled)) };
        Ok(())
    }))
}

struct AbiWorker {
    callback: unsafe extern "C" fn(*mut c_void),
    context: *mut c_void,
    finished: Event,
    _lease: ModuleLease,
}

// SAFETY: the worker invokes its raw callback on exactly one kernel thread;
// the callback provider is responsible for synchronizing its context.
unsafe impl Send for AbiWorker {}
// SAFETY: only the immutable callback/context and thread-safe completion event
// are shared with the joining thread.
unsafe impl Sync for AbiWorker {}

unsafe extern "C" fn shim_worker_spawn(
    owner: *const c_void,
    callback: Option<unsafe extern "C" fn(*mut c_void)>,
    context: *mut c_void,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let callback = callback.ok_or(Error::InvalidArgument)?;
        let owner = module_ref(owner)?;
        let worker = Arc::new(AbiWorker {
            callback,
            context,
            finished: Event::new(),
            _lease: module::lease_owner(owner.as_ref())?,
        });
        let task = worker.clone();
        crate::sys::sched::run(move || {
            // SAFETY: the callback remains executable while the worker's
            // module lease is held, and its context follows the spawn ABI.
            unsafe { (task.callback)(task.context) };
            task.finished.signal();
        });
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, receipt(worker)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_worker_join(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_worker_spawn` and is consumed
        // exactly once by this join operation.
        let worker = unsafe { claim_receipt::<AbiWorker>(handle) }?;
        worker.finished.wait();
        Ok(())
    })())
}

// --- Time, entropy, and topology --------------------------------------------

unsafe extern "C" fn shim_time_monotonic() -> u64 {
    clock::monotonic_ns()
}

unsafe extern "C" fn shim_time_delay(nanoseconds: u64) {
    clock::delay(core::time::Duration::from_nanos(nanoseconds));
}

unsafe extern "C" fn shim_time_sleep(nanoseconds: u64) {
    clock::sleep(core::time::Duration::from_nanos(nanoseconds));
}

unsafe extern "C" fn shim_random_fill(buffer: *mut u8, length: usize) {
    if buffer.is_null() || length == 0 {
        return;
    }
    // SAFETY: the ABI requires `length` writable bytes.
    let slice = unsafe { core::slice::from_raw_parts_mut(buffer, length) };
    random::fill_bytes(slice);
}

unsafe extern "C" fn shim_random_mix(buffer: *const u8, length: usize) {
    if buffer.is_null() || length == 0 {
        return;
    }
    // SAFETY: the ABI requires `length` readable bytes.
    let slice = unsafe { core::slice::from_raw_parts(buffer, length) };
    random::mix_bytes(slice);
}

unsafe extern "C" fn shim_cpu_count() -> u32 {
    smp::cpu_count() as u32
}

unsafe extern "C" fn shim_cpu_current() -> u32 {
    crate::arch::thiscpu_opt().map_or(0, |cpu| cpu.id as u32)
}

unsafe extern "C" fn shim_cpu_platform_id(cpu: u32, out: *mut u64) -> i32 {
    status((|| -> Result<()> {
        let id = smp::platform_id(cpu as usize).ok_or(Error::NotFound)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, id) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_in_interrupt() -> i32 {
    i32::from(smp::in_interrupt_context())
}

// --- Firmware ---------------------------------------------------------------

unsafe extern "C" fn shim_firmware_acpi(
    buffer: *mut u8,
    capacity: usize,
    written: *mut usize,
) -> i32 {
    #[cfg(target_arch = "x86_64")]
    let result: Result<()> = crate::sys::firmware::acpi_rsdp()
        .ok_or(Error::NotFound)
        .and_then(|data| {
            // SAFETY: the ABI requires `capacity` writable bytes.
            unsafe { copy_out(data, buffer, capacity, written) }
        });
    #[cfg(not(target_arch = "x86_64"))]
    let result: Result<()> = {
        let _ = (buffer, capacity, written);
        Err(Error::NotFound)
    };
    status(result)
}

unsafe extern "C" fn shim_firmware_devicetree(
    buffer: *mut u8,
    capacity: usize,
    written: *mut usize,
) -> i32 {
    #[cfg(target_arch = "riscv64")]
    let result: Result<()> = crate::sys::firmware::dtb()
        .ok_or(Error::NotFound)
        .and_then(|data| {
            // SAFETY: the ABI requires `capacity` writable bytes.
            unsafe { copy_out(data, buffer, capacity, written) }
        });
    #[cfg(not(target_arch = "riscv64"))]
    let result: Result<()> = {
        let _ = (buffer, capacity, written);
        Err(Error::NotFound)
    };
    status(result)
}

// --- Device nodes and terminals ---------------------------------------------

unsafe extern "C" fn shim_devfs_root(out: *mut u64) -> i32 {
    status((|| -> Result<()> {
        let root = chardev::root()?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, root.get()) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_devfs_mkdir(
    module: *const c_void,
    parent: u64,
    name: *const c_char,
    mode: u16,
    out: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        let node =
            chardev::create_directory(owner.as_ref(), DevNodeId::from_raw(parent), name, mode)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, node.get()) };
        Ok(())
    })())
}

#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn shim_devfs_create(
    module: *const c_void,
    device: *const c_void,
    parent: u64,
    name: *const c_char,
    node_kind: u32,
    mode: u16,
    ops: *const NodeOps,
    out: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        let device = optional_object::<Device>(device)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        if ops.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the ABI requires a readable size-prefixed operation table.
        // The helper copies only the prefix supplied by legacy drivers.
        let table = unsafe { chardev::copy_node_ops(ops) }?;
        // SAFETY: the module contract requires these callbacks to stay
        // executable until the node is removed.
        let (node, _) = unsafe {
            chardev::create_node(
                owner.as_ref(),
                device.as_ref(),
                DevNodeId::from_raw(parent),
                name,
                node_kind,
                mode,
                table,
            )
        }?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, node.get()) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_devfs_remove(module: *const c_void, node: u64) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        chardev::remove(owner.as_ref(), DevNodeId::from_raw(node))
    })())
}

unsafe extern "C" fn shim_devfs_lookup(path: *const c_char, out: *mut u64) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a NUL-terminated string.
        let path = unsafe { borrow_str(path) }?;
        let node = chardev::lookup(path)?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, node.get()) };
        Ok(())
    })())
}

#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn shim_tty_register(
    module: *const c_void,
    device: *const c_void,
    parent: u64,
    name: *const c_char,
    mode: u16,
    baud: u32,
    ops: *const ConsoleOps,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        let device = optional_object::<Device>(device)?;
        // SAFETY: the ABI requires a NUL-terminated string.
        let name = unsafe { borrow_str(name) }?;
        if name.is_empty() {
            return Err(Error::InvalidArgument);
        }
        if ops.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the module contract requires this callback table to stay
        // executable until the terminal receipt is removed.
        let terminal = unsafe {
            console::register(
                owner.as_ref(),
                device.as_ref(),
                DevNodeId::from_raw(parent),
                name.as_ptr().cast(),
                mode,
                baud,
                ops,
            )
        }?;
        // SAFETY: the ABI requires a writable output pointer.
        unsafe { write_out(out, receipt(terminal)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_tty_unregister(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the handle came from `shim_tty_register` and remains valid
        // if the provider reports that its node is still busy.
        let terminal = unsafe { borrow_receipt::<console::Terminal>(handle) }?;
        console::unregister(&terminal)?;
        // SAFETY: unregister succeeded, so consume the receipt exactly once.
        drop(unsafe { claim_receipt::<console::Terminal>(handle) }?);
        Ok(())
    })())
}

unsafe extern "C" fn shim_tty_provider_register(
    module: *const c_void,
    ops: *const TtyProviderOps,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        if ops.is_null() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: only the mandatory size prefix is read before accepting
        // the full append-only provider table.
        if unsafe { (*ops).size as usize } < core::mem::size_of::<TtyProviderOps>() {
            return Err(Error::InvalidArgument);
        }
        // SAFETY: the declared size now proves the full current table is
        // readable, and the broker copies it before returning.
        let ops = unsafe { core::ptr::read(ops) };
        // SAFETY: the module contract keeps the source callback table and
        // context valid until the returned provider receipt is consumed.
        let provider = unsafe { console::register_provider(owner.as_ref(), ops) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, receipt(provider)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_tty_provider_unregister(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: keep the receipt available when live terminals make the
        // provider busy, allowing the module to retry teardown later.
        let provider = unsafe { borrow_receipt::<console::Provider>(handle) }?;
        console::unregister_provider(&provider)?;
        // SAFETY: provider removal succeeded, so consume its receipt.
        drop(unsafe { claim_receipt::<console::Provider>(handle) }?);
        Ok(())
    })())
}

unsafe extern "C" fn shim_process_group_signal(group: i32, signal: u8) -> i32 {
    status((|| -> Result<()> {
        let group = usize::try_from(group).map_err(|_| Error::InvalidArgument)?;
        if group == 0 {
            return Err(Error::InvalidArgument);
        }
        crate::proc::signal::send_kernel_process_group(group, signal);
        Ok(())
    })())
}

// --- Filesystem providers ---------------------------------------------------

unsafe extern "C" fn shim_fs_provider_register(
    module: *const c_void,
    name: *const c_char,
    operations: *const crate::fs::provider::FsProviderOps,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the ABI requires a NUL-terminated provider name.
        let name = unsafe { borrow_str(name) }?;
        // SAFETY: the module supplies a readable, immutable provider table
        // whose callback lifetime is protected by its module registration.
        let provider =
            unsafe { crate::fs::provider::register(owner.as_ref(), name, operations) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, receipt(provider)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_fs_provider_unregister(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: retain the receipt if a malformed removal reports an error.
        let provider = unsafe { borrow_receipt::<crate::fs::provider::Provider>(handle) }?;
        crate::fs::provider::unregister(&provider)?;
        // SAFETY: successful removal consumes the registration receipt.
        drop(unsafe {
            claim_receipt::<crate::fs::provider::Provider>(handle)?
        });
        Ok(())
    })())
}

unsafe extern "C" fn shim_fs_page_account_create(limit: u64, out: *mut *mut c_void) -> i32 {
    let account = crate::fs::provider::page_account_create(limit);
    // SAFETY: the ABI requires writable output storage or null.
    unsafe { write_out(out, account) };
    STATUS_OK
}

unsafe extern "C" fn shim_fs_page_account_release(account: *mut c_void) {
    // SAFETY: page-account receipts are created only by the matching service
    // and consumed exactly once by this ABI operation.
    unsafe { crate::fs::provider::page_account_release(account) };
}

unsafe extern "C" fn shim_fs_page_account_limit(account: *mut c_void, out: *mut u64) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a live account receipt.
        let limit = unsafe { crate::fs::provider::page_account_limit(account) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, limit) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_fs_page_account_used(account: *mut c_void, out: *mut u64) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a live account receipt.
        let used = unsafe { crate::fs::provider::page_account_used(account) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, used) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_fs_memory_object_create(
    account: *mut c_void,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a live account receipt.
        let object = unsafe { crate::fs::provider::memory_object_create(account) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, object) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_fs_memory_object_retain(object: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a live object receipt.
        unsafe { crate::fs::provider::memory_object_retain(object) }?;
        Ok(())
    })())
}

unsafe extern "C" fn shim_fs_memory_object_release(object: *mut c_void) {
    // SAFETY: object receipts are owned values created or retained through the
    // matching page-cache services.
    unsafe { crate::fs::provider::memory_object_release(object) };
}

unsafe extern "C" fn shim_fs_memory_object_read(
    object: *mut c_void,
    offset: u64,
    buffer: *mut u8,
    length: usize,
) -> i64 {
    // SAFETY: the ABI requires a live object and `length` writable bytes.
    match unsafe { crate::fs::provider::memory_object_read(object, offset, buffer, length) } {
        Ok(read) => i64::try_from(read).unwrap_or(i64::MAX),
        Err(error) => i64::from(Error::from(error).to_status()),
    }
}

unsafe extern "C" fn shim_fs_memory_object_write(
    object: *mut c_void,
    offset: u64,
    buffer: *const u8,
    length: usize,
) -> i64 {
    // SAFETY: the ABI requires a live object and `length` readable bytes.
    match unsafe { crate::fs::provider::memory_object_write(object, offset, buffer, length) } {
        Ok(written) => i64::try_from(written).unwrap_or(i64::MAX),
        Err(error) => i64::from(Error::from(error).to_status()),
    }
}

unsafe extern "C" fn shim_fs_memory_object_truncate(
    object: *mut c_void,
    size: u64,
    out: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a live object receipt.
        let removed = unsafe { crate::fs::provider::memory_object_truncate(object, size) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, removed) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_fs_memory_object_page_count(
    object: *mut c_void,
    out: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: the ABI requires a live object receipt.
        let count = unsafe { crate::fs::provider::memory_object_page_count(object) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, count) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_fs_total_physical_pages() -> u64 {
    crate::fs::provider::total_physical_pages()
}

unsafe extern "C" fn shim_devfs_broker_register(
    module: *const c_void,
    operations: *const crate::fs::provider::DevfsBrokerOps,
    out: *mut *mut c_void,
) -> i32 {
    status((|| -> Result<()> {
        let owner = module_ref(module)?;
        // SAFETY: the module supplies a readable, immutable broker table.
        let broker =
            unsafe { crate::fs::provider::register_devfs_broker(owner.as_ref(), operations) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, receipt(broker)) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_devfs_broker_unregister(handle: *mut c_void) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: preserve ownership if broker withdrawal fails.
        let broker = unsafe { borrow_receipt::<crate::fs::provider::DevfsBroker>(handle) }?;
        crate::fs::provider::unregister_devfs_broker(&broker)?;
        // SAFETY: successful removal consumes the registration receipt.
        drop(unsafe {
            claim_receipt::<crate::fs::provider::DevfsBroker>(handle)?
        });
        Ok(())
    })())
}

unsafe extern "C" fn shim_devfs_endpoint_open(
    endpoint: *mut c_void,
    flags: u32,
    out: *mut usize,
) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: devfs owns a live endpoint receipt for this callback.
        let file = unsafe { chardev::endpoint_open(endpoint, flags) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, file) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_devfs_endpoint_close(endpoint: *mut c_void, file: usize, flags: u32) {
    // SAFETY: the endpoint receipt and file context are owned by devfs.
    unsafe { chardev::endpoint_close(endpoint, file, flags) };
}

unsafe extern "C" fn shim_devfs_endpoint_initial_offset(
    endpoint: *mut c_void,
    file: usize,
    flags: u32,
    out: *mut u64,
) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: devfs owns a live endpoint receipt and file context.
        let offset = unsafe { chardev::endpoint_initial_offset(endpoint, file, flags) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, offset) };
        Ok(())
    })())
}

unsafe extern "C" fn shim_devfs_endpoint_read(
    endpoint: *mut c_void,
    file: usize,
    offset: u64,
    buffer: *mut u8,
    length: usize,
    flags: u32,
) -> i64 {
    // SAFETY: the ABI requires `buffer` writable for `length` bytes.
    match unsafe { chardev::endpoint_read(endpoint, file, offset, buffer, length, flags) } {
        Ok(read) => read,
        Err(error) => i64::from(error.to_status()),
    }
}

unsafe extern "C" fn shim_devfs_endpoint_write(
    endpoint: *mut c_void,
    file: usize,
    offset: u64,
    buffer: *const u8,
    length: usize,
    flags: u32,
) -> i64 {
    // SAFETY: the ABI requires `buffer` readable for `length` bytes.
    match unsafe { chardev::endpoint_write(endpoint, file, offset, buffer, length, flags) } {
        Ok(written) => written,
        Err(error) => i64::from(error.to_status()),
    }
}

unsafe extern "C" fn shim_devfs_endpoint_size(endpoint: *mut c_void) -> u64 {
    // SAFETY: the ABI requires a live endpoint receipt.
    unsafe { chardev::endpoint_size(endpoint) }
}

unsafe extern "C" fn shim_devfs_endpoint_sync(endpoint: *mut c_void) -> i32 {
    // SAFETY: devfs owns a live endpoint receipt for this callback.
    status(unsafe { chardev::endpoint_sync(endpoint) })
}

unsafe extern "C" fn shim_devfs_endpoint_poll(
    endpoint: *mut c_void,
    file: usize,
    offset: u64,
    events: u16,
    flags: u32,
) -> i64 {
    // SAFETY: the ABI requires a live endpoint receipt.
    match unsafe { chardev::endpoint_poll(endpoint, file, offset, events, flags) } {
        Ok(ready) => i64::from(ready),
        Err(error) => i64::from(error.to_status()),
    }
}

unsafe extern "C" fn shim_devfs_endpoint_event(
    endpoint: *mut c_void,
    file: usize,
    selector: u32,
) -> usize {
    // SAFETY: the ABI requires a live endpoint receipt.
    unsafe { chardev::endpoint_event(endpoint, file, selector) }
}

unsafe extern "C" fn shim_devfs_endpoint_terminal_state(
    endpoint: *mut c_void,
    out: *mut crate::fs::provider::FsTerminalState,
) -> i32 {
    status((|| -> Result<()> {
        // SAFETY: devfs owns a live endpoint receipt for this callback.
        let state = unsafe { chardev::endpoint_terminal_state(endpoint) }?;
        // SAFETY: the ABI requires writable output storage or null.
        unsafe { write_out(out, state) };
        Ok(())
    })())
}

#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn shim_devfs_endpoint_ioctl(
    endpoint: *mut c_void,
    file: usize,
    process: u64,
    group: i32,
    session: i32,
    session_leader: u8,
    request: u64,
    value: u64,
    argument: *mut u8,
    length: usize,
) -> i64 {
    // SAFETY: the ABI requires `argument` writable for `length` bytes.
    match unsafe {
        chardev::endpoint_ioctl(
            endpoint,
            file,
            process,
            group,
            session,
            session_leader != 0,
            request,
            value,
            argument,
            length,
        )
    } {
        Ok(result) => i64::try_from(result).unwrap_or(i64::MAX),
        Err(error) => i64::from(error.to_status()),
    }
}

unsafe extern "C" fn shim_devfs_endpoint_release(endpoint: *mut c_void) {
    // SAFETY: devfs invokes this exactly once for the endpoint receipt it owns.
    unsafe { chardev::endpoint_release(endpoint) };
}

// --- Kernel log -------------------------------------------------------------

unsafe extern "C" fn shim_klog_level() -> u32 {
    klog::record_level().as_raw().into()
}

unsafe extern "C" fn shim_klog_set_level(level: u32) -> u32 {
    klog::set_record_level(clamp_level(level)).as_raw().into()
}

unsafe extern "C" fn shim_klog_console_level() -> u32 {
    klog::console_level().as_raw().into()
}

unsafe extern "C" fn shim_klog_set_console_level(level: u32) -> u32 {
    klog::set_console_level(clamp_level(level)).as_raw().into()
}

/// Converts a severity supplied by a driver, saturating at the least severe
/// level rather than rejecting the call.
fn clamp_level(level: u32) -> klog::Level {
    klog::Level::from_raw(level.min(u32::from(klog::LEVEL_COUNT) - 1) as u8)
}
