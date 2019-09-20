//! STM32 peripheral drivers for Drone, an Embedded Operating System.
//!
//! # Documentation
//!
//! - [Drone Book](https://book.drone-os.com/)
//! - [API documentation](https://docs.rs/drone-stm32-drv/0.10.0)
//!
//! # Usage
//!
//! Place the following to the Cargo.toml:
//!
//! ```toml
//! [dependencies]
//! drone-stm32-drv = { version = "0.10.0", features = [...] }
//! ```

#![feature(prelude_import)]
#![deny(elided_lifetimes_in_paths)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::module_name_repetitions,
    clippy::similar_names
)]
#![cfg_attr(not(feature = "std"), no_std)]

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
