//! Direct memory access controller.

use crate::common::DrvRcc;
use drone_core::inventory::{Inventory0, InventoryGuard, InventoryResource};
use drone_cortex_m::reg::prelude::*;
use drone_stm32_map::periph::dma::{DmaMap, DmaPeriph};

#[cfg(any(
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
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
    self.0.free().periph
  }

  /// Enables DMA clock.
  pub fn enable(&mut self) -> InventoryGuard<'_, DmaEn<T>> {
    self.setup();
    self.0.guard()
  }

  /// Enables DMA clock.
  pub fn into_enabled(self) -> Inventory0<DmaEn<T>> {
    self.setup();
    self.0
  }

  /// Disables DMA clock.
  pub fn from_enabled(mut enabled: Inventory0<DmaEn<T>>) -> Self {
    enabled.teardown();
    Self(enabled)
  }

  fn setup(&self) {
    let dmaen = &self.0.periph.rcc_ahb1enr_dmaen;
    if dmaen.read_bit() {
      panic!("DMA wasn't turned off");
    }
    dmaen.set_bit();
  }
}

impl<T: DmaMap> InventoryResource for DmaEn<T> {
  fn teardown(&mut self) {
    self.periph.rcc_ahb1enr_dmaen.clear_bit()
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
    self.periph.rcc_ahb1rstr_dmarst.set_bit();
  }

  fn disable_stop_mode(&self) {
    self.periph.rcc_ahb1smenr_dmasmen.clear_bit();
  }

  fn enable_stop_mode(&self) {
    self.periph.rcc_ahb1smenr_dmasmen.set_bit();
  }
}
