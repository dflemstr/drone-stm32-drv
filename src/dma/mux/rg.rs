use super::DmamuxEnGuard;
use drone_core::shared_guard::GuardToken;
use drone_stm32_map::periph::dma::mux::rg::{DmamuxRgMap, DmamuxRgPeriph};

/// DMAMUX request generator driver.
pub struct DmamuxRg<T: DmamuxRgMap>(DmamuxRgEn<T>);

/// DMAMUX request generator enabled driver.
pub struct DmamuxRgEn<T: DmamuxRgMap> {
  periph: DmamuxRgPeriph<T>,
}

impl<T: DmamuxRgMap> DmamuxRg<T> {
  /// Creates a new [`Dmamux`].
  #[inline]
  pub fn new(periph: DmamuxRgPeriph<T>) -> Self {
    Self(DmamuxRgEn { periph })
  }

  /// Releases the peripheral.
  #[inline]
  pub fn free(self) -> DmamuxRgPeriph<T> {
    self.0.periph
  }

  /// Claims the enabled state.
  #[inline]
  pub fn as_enabled(
    &self,
    _token: &GuardToken<DmamuxEnGuard<T::DmamuxMap>>,
  ) -> &DmamuxRgEn<T> {
    &self.0
  }

  /// Claims the enabled state.
  #[inline]
  pub fn as_enabled_mut(
    &mut self,
    _token: &GuardToken<DmamuxEnGuard<T::DmamuxMap>>,
  ) -> &mut DmamuxRgEn<T> {
    &mut self.0
  }
}
