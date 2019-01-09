//! Drone for STM32. Drivers.

#![feature(asm)]
#![feature(generators)]
#![feature(never_type)]
#![feature(prelude_import)]
#![feature(uniform_paths)]
#![no_std]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(clippy::inline_always, clippy::module_name_repetitions)]
#![cfg_attr(test, feature(allocator_api, allocator_internals))]
#![cfg_attr(test, default_lib_allocator)]

pub mod exti;
pub mod gpio;
#[cfg(any(
  feature = "stm32l4x1",
  feature = "stm32l4x2",
  feature = "stm32l4x3",
  feature = "stm32l4x5",
  feature = "stm32l4x6",
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
))]
pub mod rtc;

#[prelude_import]
#[allow(unused_imports)]
use drone_cortex_m::prelude::*;

#[cfg(test)]
drone_core::heap! {
  struct Heap;
  size = 0x40000;
  pools = [
    [0x4; 0x4000],
    [0x20; 0x800],
    [0x100; 0x100],
    [0x800; 0x20],
  ];
}

#[cfg(test)]
#[global_allocator]
static mut GLOBAL: Heap = Heap::new();
