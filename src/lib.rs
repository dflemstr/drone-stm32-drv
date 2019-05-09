//! Drone for STM32. Drivers.

#![feature(generators)]
#![feature(prelude_import)]
#![no_std]
#![deny(bare_trait_objects)]
#![deny(elided_lifetimes_in_paths)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(
  clippy::cast_possible_truncation,
  clippy::module_name_repetitions,
  clippy::similar_names
)]
#![cfg_attr(test, feature(allocator_api, allocator_internals))]
#![cfg_attr(test, default_lib_allocator)]

pub mod adc;
pub mod common;
pub mod dma;
pub mod gpio;
pub mod i2c;
pub mod spi;
pub mod tim;
pub mod uart;

mod select3;

pub use drone_cortex_m::drv::*;

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
