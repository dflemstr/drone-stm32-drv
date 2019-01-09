//! Drone for STM32. UART driver.

#![feature(generators)]
#![feature(marker_trait_attr)]
#![feature(never_type)]
#![feature(prelude_import)]
#![feature(uniform_paths)]
#![no_std]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(clippy::inline_always, clippy::module_name_repetitions)]
#![cfg_attr(test, feature(allocator_api, allocator_internals))]
#![cfg_attr(test, default_lib_allocator)]

pub mod uart;

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
