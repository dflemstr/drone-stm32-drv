//! Drone for STM32. Drivers.

#![no_std]
#![deny(bare_trait_objects)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]

pub use drone_cortex_m::drv::*;
pub use drone_stm32_drv_adc::*;
pub use drone_stm32_drv_dma::*;
pub use drone_stm32_drv_etc::*;
pub use drone_stm32_drv_i2c::*;
pub use drone_stm32_drv_spi::*;
pub use drone_stm32_drv_uart::*;
