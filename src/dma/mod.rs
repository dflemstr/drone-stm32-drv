//! Direct memory access controller.

use crate::common::DrvRcc;
use core::marker::PhantomData;
use drone_core::shared_guard::{Guard, GuardHandler};
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
pub struct Dma<T: DmaMap>(DmaEn<T>);

/// DMA head enabled driver.
pub struct DmaEn<T: DmaMap> {
  periph: DmaPeriph<T>,
}

/// DMA head enabled guard handler.
pub struct DmaEnGuard<T: DmaMap>(PhantomData<T>);

impl<T: DmaMap> Dma<T> {
  /// Creates a new [`Dma`].
  #[inline]
  pub fn new(periph: DmaPeriph<T>) -> Self {
    Self(DmaEn { periph })
  }

  /// Releases the peripheral.
  #[inline]
  pub fn free(self) -> DmaPeriph<T> {
    self.0.periph
  }

  /// Enables DMA clock.
  pub fn enable(&mut self) -> Guard<'_, DmaEn<T>, DmaEnGuard<T>> {
    let dmaen = &self.0.periph.rcc_ahb1enr_dmaen;
    if dmaen.read_bit() {
      panic!("DMA wasn't turned off");
    }
    dmaen.set_bit();
    Guard::new(&mut self.0, DmaEnGuard(PhantomData))
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

impl<T: DmaMap> GuardHandler<DmaEn<T>> for DmaEnGuard<T> {
  fn teardown(&mut self, dma: &mut DmaEn<T>) {
    dma.periph.rcc_ahb1enr_dmaen.clear_bit()
  }
}
