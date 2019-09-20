use super::{TimDiverged, TimDivergedClockSel, TimPeriph};
use core::num::NonZeroUsize;
use drone_cortex_m::{
    drv::timer::{TimerInterval, TimerOverflow, TimerSleep, TimerStop},
    thr::prelude::*,
};
use drone_stm32_map::periph::tim::low_power::{LowPowerTimMap, LowPowerTimPeriph};

/// Low-power timer diverged peripheral.
#[allow(missing_docs)]
pub struct LowPowerTimDiverged<T: LowPowerTimMap> {
    pub rcc_busenr_lptimen: T::SRccBusenrLptimen,
    pub rcc_busrstr_lptimrst: T::SRccBusrstrLptimrst,
    pub rcc_bussmenr_lptimsmen: T::SRccBussmenrLptimsmen,
    pub rcc_ccipr_lptimsel: T::SRccCciprLptimsel,
    pub lptim_isr: T::SLptimIsr,
    pub lptim_icr: T::SLptimIcr,
    pub lptim_ier: T::SLptimIer,
    pub lptim_cfgr: T::SLptimCfgr,
    pub lptim_cr: T::SLptimCr,
    pub lptim_cmp: T::SLptimCmp,
    pub lptim_arr: T::SLptimArr,
    pub lptim_cnt: T::SLptimCnt,
    pub lptim_or: T::SLptimOr,
}

impl<T: LowPowerTimMap> TimPeriph for LowPowerTimPeriph<T> {
    type Diverged = LowPowerTimDiverged<T>;

    #[inline]
    fn diverge(self) -> Self::Diverged {
        LowPowerTimDiverged {
            rcc_busenr_lptimen: self.rcc_busenr_lptimen,
            rcc_busrstr_lptimrst: self.rcc_busrstr_lptimrst,
            rcc_bussmenr_lptimsmen: self.rcc_bussmenr_lptimsmen,
            rcc_ccipr_lptimsel: self.rcc_ccipr_lptimsel,
            lptim_isr: self.lptim_isr,
            lptim_icr: self.lptim_icr,
            lptim_ier: self.lptim_ier,
            lptim_cfgr: self.lptim_cfgr,
            lptim_cr: self.lptim_cr,
            lptim_cmp: self.lptim_cmp,
            lptim_arr: self.lptim_arr,
            lptim_cnt: self.lptim_cnt,
            lptim_or: self.lptim_or,
        }
    }
}

impl<T: LowPowerTimMap> TimDiverged for LowPowerTimDiverged<T> {
    type RccBusenr = T::SRccBusenr;
    type RccBusenrTimen = T::SRccBusenrLptimen;
    type RccBusrstr = T::SRccBusrstr;
    type RccBusrstrTimrst = T::SRccBusrstrLptimrst;
    type RccBussmenr = T::SRccBussmenr;
    type RccBussmenrTimsmen = T::SRccBussmenrLptimsmen;

    #[inline]
    fn rcc_busenr_timen(&self) -> &Self::RccBusenrTimen {
        &self.rcc_busenr_lptimen
    }

    #[inline]
    fn rcc_busrstr_timrst(&self) -> &Self::RccBusrstrTimrst {
        &self.rcc_busrstr_lptimrst
    }

    #[inline]
    fn rcc_bussmenr_timsmen(&self) -> &Self::RccBussmenrTimsmen {
        &self.rcc_bussmenr_lptimsmen
    }

    #[inline]
    fn presc(&mut self, _value: u32) {
        unimplemented!();
    }

    #[inline]
    fn sleep<I: IntToken>(&mut self, _duration: u32, _int: I) -> TimerSleep<'_, Self> {
        unimplemented!()
    }

    #[inline]
    fn interval<I: IntToken>(
        &mut self,
        _duration: u32,
        _int: I,
    ) -> TimerInterval<'_, Self, Result<NonZeroUsize, TimerOverflow>> {
        unimplemented!()
    }

    #[inline]
    fn interval_skip<I: IntToken>(
        &mut self,
        _duration: u32,
        _int: I,
    ) -> TimerInterval<'_, Self, NonZeroUsize> {
        unimplemented!()
    }
}

impl<T: LowPowerTimMap> TimDivergedClockSel for LowPowerTimDiverged<T> {
    type RccCciprVal = T::RccCciprVal;
    type RccCcipr = T::SRccCcipr;
    type RccCciprTimsel = T::SRccCciprLptimsel;

    #[inline]
    fn rcc_ccipr_timsel(&self) -> &Self::RccCciprTimsel {
        &self.rcc_ccipr_lptimsel
    }
}

impl<T: LowPowerTimMap> TimerStop for LowPowerTimDiverged<T> {
    #[inline]
    fn stop(&mut self) {
        unimplemented!()
    }
}
