//! DMA request multiplexer.

use crate::common::DrvRcc;
use drone_core::inventory::{self, Inventory0, Inventory1};
use drone_cortex_m::reg::prelude::*;
use drone_stm32_map::periph::dma::mux::{DmamuxMap, DmamuxPeriph};

mod ch;
mod rg;

pub use self::{ch::*, rg::*};

/// DMAMUX head driver.
pub struct Dmamux<T: DmamuxMap>(Inventory0<DmamuxEn<T>>);

/// DMAMUX head enabled driver.
pub struct DmamuxEn<T: DmamuxMap> {
    periph: DmamuxPeriph<T>,
}

impl<T: DmamuxMap> Dmamux<T> {
    /// Creates a new [`Dmamux`].
    #[inline]
    pub fn new(periph: DmamuxPeriph<T>) -> Self {
        Self(Inventory0::new(DmamuxEn { periph }))
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> DmamuxPeriph<T> {
        Inventory0::free(self.0).periph
    }

    /// Enables DMAMUX clock.
    pub fn enable(&mut self) -> inventory::Guard<'_, DmamuxEn<T>> {
        self.setup();
        Inventory0::guard(&mut self.0)
    }

    /// Enables DMAMUX clock.
    pub fn into_enabled(self) -> Inventory1<DmamuxEn<T>> {
        self.setup();
        let (enabled, token) = self.0.share1();
        // To be recreated in `from_enabled()`.
        drop(token);
        enabled
    }

    /// Disables DMAMUX clock.
    pub fn from_enabled(enabled: Inventory1<DmamuxEn<T>>) -> Self {
        // Restoring the token dropped in `into_enabled()`.
        let token = unsafe { inventory::Token::new() };
        let mut enabled = enabled.merge1(token);
        Inventory0::teardown(&mut enabled);
        Self(enabled)
    }

    fn setup(&self) {
        let dmamuxen = &self.0.periph.rcc_busenr_dmamuxen;
        if dmamuxen.read_bit() {
            panic!("DMAMUX wasn't turned off");
        }
        dmamuxen.set_bit();
    }
}

impl<T: DmamuxMap> inventory::Item for DmamuxEn<T> {
    fn teardown(&mut self, _token: &mut inventory::GuardToken<Self>) {
        self.periph.rcc_busenr_dmamuxen.clear_bit()
    }
}

impl<T: DmamuxMap> DrvRcc for Dmamux<T> {
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

impl<T: DmamuxMap> DrvRcc for DmamuxEn<T> {
    fn reset(&mut self) {
        self.periph.rcc_busrstr_dmamuxrst.set_bit();
    }

    fn disable_stop_mode(&self) {
        self.periph.rcc_bussmenr_dmamuxsmen.clear_bit();
    }

    fn enable_stop_mode(&self) {
        self.periph.rcc_bussmenr_dmamuxsmen.set_bit();
    }
}
