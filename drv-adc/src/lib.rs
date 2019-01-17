//! Drone for STM32. ADC driver.

#![feature(futures_api)]
#![feature(generators)]
#![feature(never_type)]
#![feature(prelude_import)]
#![no_std]
#![deny(bare_trait_objects)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(clippy::inline_always, clippy::module_name_repetitions)]
#![cfg_attr(test, feature(allocator_api, allocator_internals))]
#![cfg_attr(test, default_lib_allocator)]

pub mod adc;

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
