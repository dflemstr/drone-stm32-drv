//! Direct memory access controller.

use crate::common::DrvRcc;
use drone_core::inventory::{self, Inventory0, Inventory1};
use drone_cortex_m::reg::prelude::*;
use drone_stm32_map::periph::dma::{DmaMap, DmaPeriph};

#[cfg(any(
    stm32_mcu = "stm32l4r5",
    stm32_mcu = "stm32l4r7",
    stm32_mcu = "stm32l4r9",
    stm32_mcu = "stm32l4s5",
    stm32_mcu = "stm32l4s7",
    stm32_mcu = "stm32l4s9"
))]
pub mod mux;

mod ch;

pub use self::ch::*;

/// DMA head driver.
pub struct Dma<T: DmaMap>(Inventory0<DmaEn<T>>);

/// DMA head enabled driver.
pub struct DmaEn<T: DmaMap> {
    periph: DmaPeriph<T>,
}

impl<T: DmaMap> Dma<T> {
    /// Creates a new [`Dma`].
    #[inline]
    pub fn new(periph: DmaPeriph<T>) -> Self {
        Self(Inventory0::new(DmaEn { periph }))
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> DmaPeriph<T> {
        Inventory0::free(self.0).periph
    }

    /// Enables DMA clock.
    pub fn enable(&mut self) -> inventory::Guard<'_, DmaEn<T>> {
        self.setup();
        Inventory0::guard(&mut self.0)
    }

    /// Enables DMA clock.
    pub fn into_enabled(self) -> Inventory1<DmaEn<T>> {
        self.setup();
        let (enabled, token) = self.0.share1();
        // To be recreated in `from_enabled()`.
        drop(token);
        enabled
    }

    /// Disables DMA clock.
    pub fn from_enabled(enabled: Inventory1<DmaEn<T>>) -> Self {
        // Restoring the token dropped in `into_enabled()`.
        let token = unsafe { inventory::Token::new() };
        let mut enabled = enabled.merge1(token);
        Inventory0::teardown(&mut enabled);
        Self(enabled)
    }

    fn setup(&self) {
        let dmaen = &self.0.periph.rcc_busenr_dmaen;
        if dmaen.read_bit() {
            panic!("DMA wasn't turned off");
        }
        dmaen.set_bit();
    }
}

impl<T: DmaMap> inventory::Item for DmaEn<T> {
    fn teardown(&mut self, _token: &mut inventory::GuardToken<Self>) {
        self.periph.rcc_busenr_dmaen.clear_bit()
    }
}

impl<T: DmaMap> DrvRcc for Dma<T> {
    #[inline]
    fn reset(&mut self) {
        self.0.reset();
    }

    #[inline]
    fn disable_stop_mode(&self) {
        self.0.disable_stop_mode();
    }

    #[inline]
    fn enable_stop_mode(&self) {
        self.0.enable_stop_mode();
    }
}

impl<T: DmaMap> DrvRcc for DmaEn<T> {
    fn reset(&mut self) {
        self.periph.rcc_busrstr_dmarst.set_bit();
    }

    fn disable_stop_mode(&self) {
        self.periph.rcc_bussmenr_dmasmen.clear_bit();
    }

    fn enable_stop_mode(&self) {
        self.periph.rcc_bussmenr_dmasmen.set_bit();
    }
}
