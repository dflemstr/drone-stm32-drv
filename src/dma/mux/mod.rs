//! DMA request multiplexer.

use crate::common::DrvRcc;
use core::marker::PhantomData;
use drone_core::shared_guard::{Guard, GuardHandler};
use drone_cortex_m::reg::prelude::*;
use drone_stm32_map::periph::dma::mux::{DmamuxMap, DmamuxPeriph};

mod ch;
mod rg;

pub use self::{ch::*, rg::*};

/// DMAMUX head driver.
pub struct Dmamux<T: DmamuxMap>(DmamuxEn<T>);

/// DMAMUX head enabled driver.
pub struct DmamuxEn<T: DmamuxMap> {
  periph: DmamuxPeriph<T>,
}

/// DMAMUX head enabled guard handler.
pub struct DmamuxEnGuard<T: DmamuxMap>(PhantomData<T>);

impl<T: DmamuxMap> Dmamux<T> {
  /// Creates a new [`Dmamux`].
  #[inline(always)]
  pub fn new(periph: DmamuxPeriph<T>) -> Self {
    Self(DmamuxEn { periph })
  }

  /// Releases the peripheral.
  #[inline(always)]
  pub fn free(self) -> DmamuxPeriph<T> {
    self.0.periph
  }

  /// Enables DMAMUX clock.
  pub fn enable(&mut self) -> Guard<'_, DmamuxEn<T>, DmamuxEnGuard<T>> {
    let dmamuxen = &self.0.periph.rcc_ahb1enr_dmamuxen;
    if dmamuxen.read_bit() {
      panic!("DMAMUX wasn't turned off");
    }
    dmamuxen.set_bit();
    Guard::new(&mut self.0, DmamuxEnGuard(PhantomData))
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

impl<T: DmamuxMap> GuardHandler<DmamuxEn<T>> for DmamuxEnGuard<T> {
  fn teardown(&mut self, dmamux: &mut DmamuxEn<T>) {
    dmamux.periph.rcc_ahb1enr_dmamuxen.clear_bit()
  }
}
