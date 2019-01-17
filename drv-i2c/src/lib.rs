//! Drone for STM32. I2C driver.

#![feature(exhaustive_patterns)]
#![feature(futures_api)]
#![feature(generators)]
#![feature(marker_trait_attr)]
#![feature(never_type)]
#![feature(prelude_import)]
#![no_std]
#![deny(bare_trait_objects)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(
  clippy::cast_possible_truncation,
  clippy::inline_always,
  clippy::module_name_repetitions,
  clippy::similar_names
)]
#![cfg_attr(test, feature(allocator_api, allocator_internals))]
#![cfg_attr(test, default_lib_allocator)]

pub mod i2c;
pub mod i2c_sess;

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
