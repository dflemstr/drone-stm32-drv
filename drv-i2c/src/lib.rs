//! Drone for STM32. I2C driver.

#![feature(exhaustive_patterns)]
#![feature(generators)]
#![feature(marker_trait_attr)]
#![feature(never_type)]
#![feature(prelude_import)]
#![feature(self_struct_ctor)]
#![no_std]
#![warn(missing_docs)]
#![allow(clippy::precedence)]
#![cfg_attr(test, feature(allocator_api, allocator_internals))]
#![cfg_attr(test, default_lib_allocator)]

#[allow(unused_imports)]
#[macro_use]
extern crate drone_core;
extern crate drone_stm32;
extern crate drone_stm32_drv_dma;
extern crate drone_stm32_map;
extern crate failure;
#[allow(unused_imports)]
#[macro_use]
extern crate failure_derive;
extern crate futures;
#[cfg(test)]
extern crate test;

pub mod i2c;
pub mod i2c_master;

#[prelude_import]
#[allow(unused_imports)]
use drone_stm32::prelude::*;

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
