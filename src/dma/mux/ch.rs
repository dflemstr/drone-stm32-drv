use super::DmamuxEn;
use drone_core::inventory::InventoryToken;
use drone_cortex_m::reg::prelude::*;
use drone_stm32_map::periph::dma::mux::ch::{traits::*, DmamuxChMap, DmamuxChPeriph};

/// DMAMUX channel driver.
pub struct DmamuxCh<T: DmamuxChMap>(DmamuxChEn<T>);

/// DMAMUX channel enabled driver.
pub struct DmamuxChEn<T: DmamuxChMap> {
    periph: DmamuxChPeriph<T>,
}

impl<T: DmamuxChMap> DmamuxCh<T> {
    /// Creates a new [`Dmamux`].
    #[inline]
    pub fn new(periph: DmamuxChPeriph<T>) -> Self {
        Self(DmamuxChEn { periph })
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> DmamuxChPeriph<T> {
        self.0.periph
    }

    /// Acquires the enabled state.
    #[inline]
    pub fn as_enabled(&self, _token: &InventoryToken<DmamuxEn<T::DmamuxMap>>) -> &DmamuxChEn<T> {
        &self.0
    }

    /// Acquires the enabled state.
    #[inline]
    pub fn as_enabled_mut(
        &mut self,
        _token: &InventoryToken<DmamuxEn<T::DmamuxMap>>,
    ) -> &mut DmamuxChEn<T> {
        &mut self.0
    }

    /// Acquires the enabled state.
    #[inline]
    pub fn into_enabled(self, token: InventoryToken<DmamuxEn<T::DmamuxMap>>) -> DmamuxChEn<T> {
        // To be recreated in DmamuxChEn::into_disabled.
        drop(token);
        self.0
    }
}

impl<T: DmamuxChMap> DmamuxChEn<T> {
    /// Releases the enabled state.
    #[inline]
    pub fn into_disabled(self) -> (DmamuxCh<T>, InventoryToken<DmamuxEn<T::DmamuxMap>>) {
        // An owned DmamuxChEn can come only from DmamuxCh::into_enabled.
        let token = unsafe { InventoryToken::new() };
        (DmamuxCh(self), token)
    }

    /// Returns DMA request id.
    pub fn dma_req_id(&self) -> u32 {
        self.periph.dmamux_ccr.dmareq_id().read_bits()
    }

    /// Sets DMA request id.
    pub fn set_dma_req_id(&self, id: u32) {
        self.periph.dmamux_ccr.dmareq_id().write_bits(id)
    }
}
