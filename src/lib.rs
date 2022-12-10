#![no_std]
#![feature(alloc_error_handler)]
#![feature(type_alias_impl_trait)]
#![feature(let_else)]
#![feature(const_fn_floating_point_arithmetic)]
#![feature(const_float_bits_conv)]
#![feature(adt_const_params)]
#![feature(generic_const_exprs)]
#![feature(mixed_integer_ops)]
#![feature(core_ffi_c)]
#![feature(const_mut_refs)]
#![feature(const_unsafecell_get_mut)]
#![allow(clippy::type_complexity)]

pub mod rendering;
pub mod message;
pub mod flags;
pub mod clock;
pub mod morse;

use core::alloc::Layout;

use alloc_cortex_m::CortexMHeap;
#[cfg(feature = "no-debugger")]
use panic_reset as _;

#[macro_export]
macro_rules! forever {
    ($e:expr) => {
        {
            type A = impl Sized + 'static;
            static X: static_cell::StaticCell<A> = static_cell::StaticCell::new();
            X.init_with(|| $e)
        }
    };
}

#[cfg(feature = "debugger")]
use defmt_rtt as _;
#[cfg(feature = "debugger")]
use panic_probe as _;

use rp_pico as _;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

pub fn init_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024;
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
}


#[alloc_error_handler]
fn oom(layout: Layout) -> ! {
    panic!("OOM allocating {:?}", layout)
}
