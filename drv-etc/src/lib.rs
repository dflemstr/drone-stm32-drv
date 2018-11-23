//! Drone for STM32. Drivers.

#![feature(asm)]
#![feature(generators)]
#![feature(never_type)]
#![feature(prelude_import)]
#![no_std]
#![warn(missing_docs)]
#![allow(clippy::precedence)]
#![cfg_attr(test, feature(allocator_api, allocator_internals))]
#![cfg_attr(test, default_lib_allocator)]

#[macro_use]
extern crate drone_core;
extern crate drone_cortex_m;
extern crate drone_stm32_map;
extern crate failure;
#[macro_use]
extern crate failure_derive;
extern crate futures;
#[cfg(test)]
extern crate test;

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
use drone_core::heap;

#[cfg(test)]
heap! {
  struct Heap;
  extern fn alloc_hook;
  extern fn dealloc_hook;
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

#[cfg(test)]
fn alloc_hook(
  _layout: ::core::alloc::Layout,
  _pool: &::drone_core::heap::Pool,
) {
}

#[cfg(test)]
fn dealloc_hook(
  _layout: ::core::alloc::Layout,
  _pool: &::drone_core::heap::Pool,
) {
}
