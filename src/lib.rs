//! STM32 peripheral drivers for Drone, an Embedded Operating System.
//!
//! # Documentation
//!
//! - [Drone Book](https://book.drone-os.com/)
//! - [API documentation](https://docs.rs/drone-stm32-drv/0.11.0)
//!
//! # Usage
//!
//! Place the following to the Cargo.toml:
//!
//! ```toml
//! [dependencies]
//! drone-stm32-drv = { version = "0.11.0", features = [...] }
//! ```

#![feature(prelude_import)]
#![deny(elided_lifetimes_in_paths)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::module_name_repetitions,
    clippy::must_use_candidate,
    clippy::similar_names
)]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "adc")]
pub mod adc;
pub mod common;
#[cfg(feature = "dma")]
pub mod dma;
#[cfg(feature = "gpio")]
pub mod gpio;
#[cfg(feature = "i2c")]
pub mod i2c;
#[cfg(feature = "spi")]
pub mod spi;
#[cfg(feature = "tim")]
pub mod tim;
#[cfg(feature = "uart")]
pub mod uart;

mod select3;

pub use drone_cortex_m::drv::*;

#[prelude_import]
#[allow(unused_imports)]
use drone_cortex_m::prelude::*;
