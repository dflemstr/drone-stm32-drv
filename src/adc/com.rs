use drone_cortex_m::reg::prelude::*;
use drone_stm32_map::periph::adc::com::AdcComPeriph;

/// ADC common driver.
pub struct AdcCom {
  periph: AdcComPeriph,
}

/// Acquires [`AdcCom`].
#[macro_export]
macro_rules! drv_adc_com {
  ($reg:ident) => {
    $crate::adc::AdcCom::new(::drone_stm32_map::periph::adc::periph_adc_com!(
      $reg
    ))
  };
}

impl AdcCom {
  /// Creates a new [`AdcCom`].
  #[inline]
  pub fn new(periph: AdcComPeriph) -> Self {
    Self { periph }
  }

  /// Releases the peripheral.
  #[inline]
  pub fn free(self) -> AdcComPeriph {
    self.periph
  }

  /// Enables the V<sub>BAT</sub> channel.
  pub fn ch18_on(&self) {
    self.periph.adc_common_ccr.ch18sel.set_bit()
  }

  /// Disables the V<sub>BAT</sub> channel.
  pub fn ch18_off(&self) {
    self.periph.adc_common_ccr.ch18sel.clear_bit()
  }

  /// Enables the temperature sensor channel.
  pub fn ch17_on(&self) {
    self.periph.adc_common_ccr.ch17sel.set_bit()
  }

  /// Disables the temperature sensor channel.
  pub fn ch17_off(&self) {
    self.periph.adc_common_ccr.ch17sel.clear_bit()
  }

  /// Enables the V<sub>REFINT</sub> channel.
  pub fn vref_on(&self) {
    self.periph.adc_common_ccr.vrefen.set_bit()
  }

  /// Disables the V<sub>REFINT</sub> channel.
  pub fn vref_off(&self) {
    self.periph.adc_common_ccr.vrefen.clear_bit()
  }
}
