use core::{
    marker::PhantomData,
    mem::{align_of, size_of},
    ops::{Deref, DerefMut},
    ptr::{self, NonNull},
};

use crate::{arch, sys::smp::IrqSpinLock};

use super::{PAGE_SIZE, phys, phys_to_virt, tlb};

struct Header {
    next: *mut Header,
    previous: *mut Header,
    free: *mut u8,
    used: usize,
    owner: usize,
    frame: &'static phys::Page,
}

struct Partial(*mut Header);

// SAFETY: pointers belong to this pool and are accessed under its shard lock.
unsafe impl Send for Partial {}

impl Partial {
    unsafe fn insert(&mut self, page: *mut Header) {
        // SAFETY: the caller owns this shard lock and page is not linked.
        unsafe {
            (*page).previous = ptr::null_mut();
            (*page).next = self.0;
            if !self.0.is_null() {
                (*self.0).previous = page;
            }
            self.0 = page;
        }
    }

    unsafe fn remove(&mut self, page: *mut Header) {
        // SAFETY: the caller owns this shard lock and page is linked in it.
        unsafe {
            let previous = (*page).previous;
            let next = (*page).next;
            if previous.is_null() {
                self.0 = next;
            } else {
                (*previous).next = next;
            }
            if !next.is_null() {
                (*next).previous = previous;
            }
        }
    }
}

/// Fixed-size, nonblocking storage backed directly by physical pages.
/// Allocation and return use only spin locks; neither enters the heap,
/// reclaims memory, changes page tables, or waits for a free page.
pub(super) struct Pool<T> {
    partial: [Shard; tlb::MAX_TLB_CPUS],
    marker: PhantomData<fn() -> T>,
}

#[repr(align(64))]
struct Shard {
    partial: IrqSpinLock<Partial>,
    cache: IrqSpinLock<Cache>,
}

const CACHE_SIZE: usize = 16;
struct Cache {
    slots: [*mut u8; CACHE_SIZE],
    len: usize,
}
// SAFETY: cached slots are exclusively owned under the cache lock.
unsafe impl Send for Cache {}
impl Cache {
    const fn new() -> Self {
        Self {
            slots: [ptr::null_mut(); CACHE_SIZE],
            len: 0,
        }
    }
    fn pop(&mut self) -> Option<*mut u8> {
        if self.len == 0 {
            return None;
        }
        self.len -= 1;
        Some(self.slots[self.len])
    }
    fn push(&mut self, slot: *mut u8) {
        self.slots[self.len] = slot;
        self.len += 1;
    }
}

impl<T: 'static> Pool<T> {
    const ALIGN: usize = if align_of::<T>() > align_of::<*mut u8>() {
        align_of::<T>()
    } else {
        align_of::<*mut u8>()
    };
    const STRIDE: usize = (if size_of::<T>() > size_of::<*mut u8>() {
        size_of::<T>()
    } else {
        size_of::<*mut u8>()
    })
    .next_multiple_of(Self::ALIGN);
    const START: usize = size_of::<Header>().next_multiple_of(Self::ALIGN);
    const CAPACITY: usize = (PAGE_SIZE as usize - Self::START) / Self::STRIDE;

    pub(super) const fn new() -> Self {
        assert!(Self::ALIGN <= PAGE_SIZE as usize / 2 && Self::CAPACITY != 0);
        Self {
            partial: [const {
                Shard {
                    partial: IrqSpinLock::new(Partial(ptr::null_mut())),
                    cache: IrqSpinLock::new(Cache::new()),
                }
            }; tlb::MAX_TLB_CPUS],
            marker: PhantomData,
        }
    }

    pub(super) fn try_alloc(&'static self, value: T) -> core::result::Result<Pooled<T>, T> {
        self.allocate(value).or_else(|value| {
            self.drain();
            self.allocate(value)
        })
    }

    fn allocate(&'static self, value: T) -> core::result::Result<Pooled<T>, T> {
        let owner = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
        if let Some(slot) = self.partial[owner].cache.lock().pop() {
            // SAFETY: cached slots retain their page and contain no live T.
            let pointer = unsafe {
                let pointer = slot.cast::<T>();
                pointer.write(value);
                NonNull::new_unchecked(pointer)
            };
            return Ok(Pooled {
                pointer,
                pool: self,
            });
        }
        let mut partial = self.partial[owner].partial.lock();
        if partial.0.is_null() {
            let Some(frame) =
                phys::alloc_page(phys::PageUse::KernelHeap, phys::PageAllocFlags::USERESERVE)
            else {
                return Err(value);
            };
            let base = phys_to_virt(frame.paddr()).as_mut_ptr::<u8>();
            let header = base.cast::<Header>();
            // SAFETY: this exclusive direct-mapped frame is page aligned.
            // Header and fixed-stride slots are disjoint and fit in the page.
            unsafe {
                let mut free = ptr::null_mut();
                for index in 0..Self::CAPACITY {
                    let slot = base.add(Self::START + index * Self::STRIDE);
                    slot.cast::<*mut u8>().write(free);
                    free = slot;
                }
                header.write(Header {
                    next: ptr::null_mut(),
                    previous: ptr::null_mut(),
                    free,
                    used: 0,
                    owner,
                    frame,
                });
                partial.insert(header);
            }
        }
        // SAFETY: the shard owns this partial page and its free list. Removing
        // a slot gives the returned handle exclusive ownership of its value.
        let pointer = unsafe {
            let header = partial.0;
            let slot = (*header).free;
            (*header).free = slot.cast::<*mut u8>().read();
            (*header).used += 1;
            if (*header).used == Self::CAPACITY {
                partial.remove(header);
            }
            let pointer = slot.cast::<T>();
            pointer.write(value);
            NonNull::new_unchecked(pointer)
        };
        Ok(Pooled {
            pointer,
            pool: self,
        })
    }
}

pub(super) struct Pooled<T: 'static> {
    pointer: NonNull<T>,
    pool: &'static Pool<T>,
}

impl<T> Pooled<T> {
    pub(super) fn into_inner(this: Self) -> T {
        let this = core::mem::ManuallyDrop::new(this);
        // SAFETY: consuming the unique handle moves T out once. Its storage
        // is returned without invoking T's destructor a second time.
        let value = unsafe { this.pointer.as_ptr().read() };
        this.pool.recycle(this.pointer.as_ptr().cast());
        value
    }
}

// SAFETY: intrusive collections return each pointer exactly once. The adapter
// retains the same pool across conversion, insertion, removal, and destruction.
unsafe impl<T> intrusive_collections::PointerOps for &'static Pool<T> {
    type Value = T;
    type Pointer = Pooled<T>;

    unsafe fn from_raw(&self, value: *const T) -> Pooled<T> {
        Pooled {
            // SAFETY: the collection supplies a live pointer from into_raw.
            pointer: unsafe { NonNull::new_unchecked(value.cast_mut()) },
            pool: self,
        }
    }

    fn into_raw(&self, value: Pooled<T>) -> *const T {
        assert!(ptr::eq(*self, value.pool), "mem: intrusive pool mismatch");
        let pointer = value.pointer.as_ptr();
        core::mem::forget(value);
        pointer
    }
}

// SAFETY: the handle exclusively owns T; pool bookkeeping is synchronized.
unsafe impl<T: Send> Send for Pooled<T> {}
// SAFETY: shared handles expose only a shared T reference.
unsafe impl<T: Sync> Sync for Pooled<T> {}

impl<T> Deref for Pooled<T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: the handle owns an initialized value until drop.
        unsafe { self.pointer.as_ref() }
    }
}

impl<T> DerefMut for Pooled<T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: the exclusive handle borrow excludes all other references.
        unsafe { self.pointer.as_mut() }
    }
}

impl<T> Drop for Pooled<T> {
    fn drop(&mut self) {
        // SAFETY: this handle exclusively owns the initialized value. Destroy
        // it before caching its storage, outside every allocator lock.
        unsafe { ptr::drop_in_place(self.pointer.as_ptr()) };
        self.pool.recycle(self.pointer.as_ptr().cast());
    }
}

impl<T: 'static> Pool<T> {
    pub(super) fn drain(&self) {
        for shard in &self.partial {
            let cache = core::mem::replace(&mut *shard.cache.lock(), Cache::new());
            for &slot in &cache.slots[..cache.len] {
                // SAFETY: detaching the cache transfers its dead slots here.
                unsafe { self.release(slot) };
            }
        }
    }

    fn recycle(&self, slot: *mut u8) {
        let owner = arch::thiscpu_opt().map_or(0, |cpu| cpu.id);
        let mut cache = self.partial[owner].cache.lock();
        let mut spill = [ptr::null_mut(); CACHE_SIZE / 2];
        if cache.len == CACHE_SIZE {
            for entry in &mut spill {
                *entry = cache.pop().unwrap();
            }
        }
        cache.push(slot);
        drop(cache);
        for slot in spill.into_iter().filter(|slot| !slot.is_null()) {
            // SAFETY: these dead slots were detached from the local cache.
            unsafe { self.release(slot) };
        }
    }

    unsafe fn release(&self, slot: *mut u8) {
        let header = (slot as usize & !(PAGE_SIZE as usize - 1)) as *mut Header;
        // SAFETY: the allocated slot keeps its immutable owner field live.
        let owner = unsafe { (*header).owner };
        let mut partial = self.partial[owner].partial.lock();
        // SAFETY: the owner's shard serializes header and free-list changes.
        // No other handle can access this slot or release the last live slot.
        let released = unsafe {
            if (*header).used == Pool::<T>::CAPACITY {
                partial.insert(header);
            }
            slot.cast::<*mut u8>().write((*header).free);
            (*header).free = slot;
            (*header).used -= 1;
            if (*header).used == 0 {
                partial.remove(header);
                Some((*header).frame)
            } else {
                None
            }
        };
        drop(partial);
        if let Some(frame) = released {
            // SAFETY: all slots have been returned and the page is unlinked.
            unsafe { phys::free_page(frame) };
        }
    }
}
