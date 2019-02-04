//! General-purpose I/O.

use crate::common::DrvRcc;
use core::marker::PhantomData;
use drone_core::shared_guard::{Guard, GuardHandler};
use drone_cortex_m::reg::prelude::*;
use drone_stm32_map::periph::gpio::head::{GpioHeadMap, GpioHeadPeriph};

/// GPIO port head driver.
pub struct GpioHead<T: GpioHeadMap>(GpioHeadEn<T>);

/// GPIO port head enabled driver.
pub struct GpioHeadEn<T: GpioHeadMap> {
  periph: GpioHeadPeriph<T>,
}

/// GPIO port head enabled guard handler.
pub struct GpioHeadEnGuard<T: GpioHeadMap>(PhantomData<T>);

impl<T: GpioHeadMap> GpioHead<T> {
  /// Creates a new [`GpioHead`].
  #[inline(always)]
  pub fn new(periph: GpioHeadPeriph<T>) -> Self {
    Self(GpioHeadEn { periph })
  }

  /// Releases the peripheral.
  #[inline(always)]
  pub fn free(self) -> GpioHeadPeriph<T> {
    self.0.periph
  }

  /// Enables the port clock.
  pub fn enable(&mut self) -> Guard<'_, GpioHeadEn<T>, GpioHeadEnGuard<T>> {
    let gpioen = &self.0.periph.rcc_ahb2enr_gpioen;
    if gpioen.read_bit() {
      panic!("GPIO wasn't turned off");
    }
    gpioen.set_bit();
    Guard::new(&mut self.0, GpioHeadEnGuard(PhantomData))
  }
}

impl<T: GpioHeadMap> DrvRcc for GpioHead<T> {
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

impl<T: GpioHeadMap> DrvRcc for GpioHeadEn<T> {
  fn reset(&mut self) {
    self.periph.rcc_ahb2rstr_gpiorst.set_bit();
  }

  fn disable_stop_mode(&self) {
    self.periph.rcc_ahb2smenr_gpiosmen.clear_bit();
  }

  fn enable_stop_mode(&self) {
    self.periph.rcc_ahb2smenr_gpiosmen.set_bit();
  }
}

impl<T: GpioHeadMap> GuardHandler<GpioHeadEn<T>> for GpioHeadEnGuard<T> {
  fn teardown(&mut self, gpio_head: &mut GpioHeadEn<T>) {
    gpio_head.periph.rcc_ahb2enr_gpioen.clear_bit()
  }
}
