//! DMA request multiplexer.

use crate::common::DrvRcc;
use drone_core::inventory::{Inventory0, InventoryGuard, InventoryResource};
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
    self.0.free().periph
  }

  /// Enables DMAMUX clock.
  pub fn enable(&mut self) -> InventoryGuard<'_, DmamuxEn<T>> {
    self.setup();
    self.0.guard()
  }

  /// Enables DMAMUX clock.
  pub fn into_enabled(self) -> Inventory0<DmamuxEn<T>> {
    self.setup();
    self.0
  }

  /// Disables DMAMUX clock.
  pub fn from_enabled(mut enabled: Inventory0<DmamuxEn<T>>) -> Self {
    enabled.teardown();
    Self(enabled)
  }

  fn setup(&self) {
    let dmamuxen = &self.0.periph.rcc_ahb1enr_dmamuxen;
    if dmamuxen.read_bit() {
      panic!("DMAMUX wasn't turned off");
    }
    dmamuxen.set_bit();
  }
}

impl<T: DmamuxMap> InventoryResource for DmamuxEn<T> {
  fn teardown(&mut self) {
    self.periph.rcc_ahb1enr_dmamuxen.clear_bit()
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
    self.periph.rcc_ahb1rstr_dmamuxrst.set_bit();
  }

  fn disable_stop_mode(&self) {
    self.periph.rcc_ahb1smenr_dmamuxsmen.clear_bit();
  }

  fn enable_stop_mode(&self) {
    self.periph.rcc_ahb1smenr_dmamuxsmen.set_bit();
  }
}
