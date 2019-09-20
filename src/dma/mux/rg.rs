use super::DmamuxEn;
use drone_core::inventory;
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
    pub fn as_enabled(&self, _token: &inventory::Token<DmamuxEn<T::DmamuxMap>>) -> &DmamuxRgEn<T> {
        &self.0
    }

    /// Claims the enabled state.
    #[inline]
    pub fn as_enabled_mut(
        &mut self,
        _token: &inventory::Token<DmamuxEn<T::DmamuxMap>>,
    ) -> &mut DmamuxRgEn<T> {
        &mut self.0
    }

    /// Acquires the enabled state.
    #[inline]
    pub fn into_enabled(self, token: inventory::Token<DmamuxEn<T::DmamuxMap>>) -> DmamuxRgEn<T> {
        // To be recreated in `into_disabled()`.
        drop(token);
        self.0
    }
}

impl<T: DmamuxRgMap> DmamuxRgEn<T> {
    /// Releases the enabled state.
    #[inline]
    pub fn into_disabled(self) -> (DmamuxRg<T>, inventory::Token<DmamuxEn<T::DmamuxMap>>) {
        // Restoring the token dropped in `into_enabled()`.
        let token = unsafe { inventory::Token::new() };
        (DmamuxRg(self), token)
    }
}
