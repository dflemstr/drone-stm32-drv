//! Analog-to-digital converters.

use crate::{
    common::{DrvClockSel, DrvDmaRx, DrvRcc},
    dma::DmaChEn,
};
#[cfg(any(
    stm32_mcu = "stm32l4s5",
    stm32_mcu = "stm32l4s7",
    stm32_mcu = "stm32l4s9"
))]
use core::ptr::read_volatile;
use drone_core::inventory::{self, Inventory0, Inventory1};
use drone_cortex_m::{fib, reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::{
    adc::{traits::*, AdcMap, AdcPeriph},
    dma::ch::DmaChMap,
};
use futures::prelude::*;

#[cfg(any(
    stm32_mcu = "stm32l4r5",
    stm32_mcu = "stm32l4r7",
    stm32_mcu = "stm32l4r9",
    stm32_mcu = "stm32l4s5",
    stm32_mcu = "stm32l4s7",
    stm32_mcu = "stm32l4s9"
))]
mod com;

#[cfg(any(
    stm32_mcu = "stm32l4r5",
    stm32_mcu = "stm32l4r7",
    stm32_mcu = "stm32l4r9",
    stm32_mcu = "stm32l4s5",
    stm32_mcu = "stm32l4s7",
    stm32_mcu = "stm32l4s9"
))]
pub use self::com::*;

/// ADC driver.
pub struct Adc<T: AdcMap, I: IntToken>(Inventory0<AdcEn<T, I>>);

/// ADC enabled driver.
pub struct AdcEn<T: AdcMap, I: IntToken> {
    periph: AdcDiverged<T>,
    int: I,
}

/// ADC diverged peripheral.
#[allow(missing_docs)]
pub struct AdcDiverged<T: AdcMap> {
    pub rcc_busenr_adcen: T::SRccBusenrAdcen,
    pub rcc_bussmenr_adcsmen: T::SRccBussmenrAdcsmen,
    pub rcc_ccipr_adcsel: T::SRccCciprAdcsel,
    pub adc_isr: T::CAdcIsr,
    pub adc_ier: T::SAdcIer,
    pub adc_cr: T::SAdcCr,
    pub adc_cfgr: T::SAdcCfgr,
    pub adc_cfgr2: T::SAdcCfgr2,
    pub adc_smpr1: T::SAdcSmpr1,
    pub adc_smpr2: T::SAdcSmpr2,
    pub adc_tr1: T::SAdcTr1,
    pub adc_tr2: T::SAdcTr2,
    pub adc_tr3: T::SAdcTr3,
    pub adc_sqr1: T::SAdcSqr1,
    pub adc_sqr2: T::SAdcSqr2,
    pub adc_sqr3: T::SAdcSqr3,
    pub adc_sqr4: T::SAdcSqr4,
    pub adc_dr: T::SAdcDr,
    pub adc_jsqr: T::SAdcJsqr,
    pub adc_ofr1: T::SAdcOfr1,
    pub adc_ofr2: T::SAdcOfr2,
    pub adc_ofr3: T::SAdcOfr3,
    pub adc_ofr4: T::SAdcOfr4,
    pub adc_jdr1: T::SAdcJdr1,
    pub adc_jdr2: T::SAdcJdr2,
    pub adc_jdr3: T::SAdcJdr3,
    pub adc_jdr4: T::SAdcJdr4,
    pub adc_awd2cr: T::SAdcAwd2Cr,
    pub adc_awd3cr: T::SAdcAwd3Cr,
    pub adc_difsel: T::SAdcDifsel,
    pub adc_calfact: T::SAdcCalfact,
}

impl<T: AdcMap, I: IntToken> Adc<T, I> {
    /// Creates a new [`Adc`].
    #[inline]
    pub fn new(periph: AdcPeriph<T>, int: I) -> Self {
        let periph = AdcDiverged {
            rcc_busenr_adcen: periph.rcc_busenr_adcen,
            rcc_bussmenr_adcsmen: periph.rcc_bussmenr_adcsmen,
            rcc_ccipr_adcsel: periph.rcc_ccipr_adcsel,
            adc_isr: periph.adc_isr.into_copy(),
            adc_ier: periph.adc_ier,
            adc_cr: periph.adc_cr,
            adc_cfgr: periph.adc_cfgr,
            adc_cfgr2: periph.adc_cfgr2,
            adc_smpr1: periph.adc_smpr1,
            adc_smpr2: periph.adc_smpr2,
            adc_tr1: periph.adc_tr1,
            adc_tr2: periph.adc_tr2,
            adc_tr3: periph.adc_tr3,
            adc_sqr1: periph.adc_sqr1,
            adc_sqr2: periph.adc_sqr2,
            adc_sqr3: periph.adc_sqr3,
            adc_sqr4: periph.adc_sqr4,
            adc_dr: periph.adc_dr,
            adc_jsqr: periph.adc_jsqr,
            adc_ofr1: periph.adc_ofr1,
            adc_ofr2: periph.adc_ofr2,
            adc_ofr3: periph.adc_ofr3,
            adc_ofr4: periph.adc_ofr4,
            adc_jdr1: periph.adc_jdr1,
            adc_jdr2: periph.adc_jdr2,
            adc_jdr3: periph.adc_jdr3,
            adc_jdr4: periph.adc_jdr4,
            adc_awd2cr: periph.adc_awd2cr,
            adc_awd3cr: periph.adc_awd3cr,
            adc_difsel: periph.adc_difsel,
            adc_calfact: periph.adc_calfact,
        };
        Self(Inventory0::new(AdcEn { periph, int }))
    }

    /// Creates a new [`Adc`].
    ///
    /// # Safety
    ///
    /// Some of the `Crt` register tokens can be still in use.
    #[inline]
    pub unsafe fn from_diverged(periph: AdcDiverged<T>, int: I) -> Self {
        Self(Inventory0::new(AdcEn { periph, int }))
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> AdcDiverged<T> {
        Inventory0::free(self.0).periph
    }

    /// Enables ADC clock.
    pub fn enable(&mut self) -> inventory::Guard<'_, AdcEn<T, I>> {
        self.setup();
        Inventory0::guard(&mut self.0)
    }

    /// Enables ADC clock.
    pub fn into_enabled(self) -> Inventory1<AdcEn<T, I>> {
        self.setup();
        let (enabled, token) = self.0.share1();
        // To be recreated in `from_enabled()`.
        drop(token);
        enabled
    }

    /// Disables ADC clock.
    pub fn from_enabled(enabled: Inventory1<AdcEn<T, I>>) -> Self {
        // Restoring the token dropped in `into_enabled()`.
        let token = unsafe { inventory::Token::new() };
        let mut enabled = enabled.merge1(token);
        Inventory0::teardown(&mut enabled);
        Self(enabled)
    }

    fn setup(&self) {
        let adcen = &self.0.periph.rcc_busenr_adcen;
        if adcen.read_bit() {
            panic!("ADC wasn't turned off");
        }
        adcen.set_bit();
    }
}

impl<T: AdcMap, I: IntToken> AdcEn<T, I> {
    /// Returns a future, which resolves on ADC ready event.
    pub fn ready(&self) -> impl Future<Output = ()> {
        let adrdy = *self.periph.adc_isr.adrdy();
        self.int.add_future(fib::new_fn(move || {
            if adrdy.read_bit() {
                adrdy.set_bit();
                fib::Complete(())
            } else {
                fib::Yielded(())
            }
        }))
    }
}

#[allow(missing_docs)]
impl<T: AdcMap, I: IntToken> AdcEn<T, I> {
    #[inline]
    pub fn int(&self) -> &I {
        &self.int
    }

    #[inline]
    pub fn ier(&self) -> &T::SAdcIer {
        &self.periph.adc_ier
    }

    #[inline]
    pub fn cfgr(&self) -> &T::SAdcCfgr {
        &self.periph.adc_cfgr
    }

    #[inline]
    pub fn cr(&self) -> &T::SAdcCr {
        &self.periph.adc_cr
    }

    #[inline]
    pub fn smpr1(&self) -> &T::SAdcSmpr1 {
        &self.periph.adc_smpr1
    }

    #[inline]
    pub fn smpr2(&self) -> &T::SAdcSmpr2 {
        &self.periph.adc_smpr2
    }

    #[inline]
    pub fn sqr1(&self) -> &T::SAdcSqr1 {
        &self.periph.adc_sqr1
    }
}

impl<T: AdcMap, I: IntToken> inventory::Item for AdcEn<T, I> {
    fn teardown(&mut self, _token: &mut inventory::GuardToken<Self>) {
        self.periph.rcc_busenr_adcen.clear_bit()
    }
}

impl<T: AdcMap, I: IntToken, Rx: DmaChMap> DrvDmaRx<Rx> for Adc<T, I> {
    #[inline]
    fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) {
        self.0.dma_rx_paddr_init(dma_rx);
    }
}

impl<T: AdcMap, I: IntToken, Rx: DmaChMap> DrvDmaRx<Rx> for AdcEn<T, I> {
    fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) {
        unsafe { dma_rx.set_paddr(self.periph.adc_dr.as_ptr()) };
    }
}

impl<T: AdcMap, I: IntToken> DrvRcc for Adc<T, I> {
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

impl<T: AdcMap, I: IntToken> DrvRcc for AdcEn<T, I> {
    fn reset(&mut self) {}

    fn disable_stop_mode(&self) {
        self.periph.rcc_bussmenr_adcsmen.clear_bit();
    }

    fn enable_stop_mode(&self) {
        self.periph.rcc_bussmenr_adcsmen.set_bit();
    }
}

impl<T: AdcMap, I: IntToken> DrvClockSel for Adc<T, I> {
    #[inline]
    fn clock_sel(&self, value: u32) {
        self.0.clock_sel(value);
    }
}

impl<T: AdcMap, I: IntToken> DrvClockSel for AdcEn<T, I> {
    fn clock_sel(&self, value: u32) {
        self.periph.rcc_ccipr_adcsel.write_bits(value);
    }
}

#[cfg(any(
    stm32_mcu = "stm32l4s5",
    stm32_mcu = "stm32l4s7",
    stm32_mcu = "stm32l4s9"
))]
/// Reads internal voltage reference (V<sub>REFINT</sub>).
pub fn read_vref_cal() -> u16 {
    unsafe { read_volatile(0x1FFF_75AA as *const u16) }
}
