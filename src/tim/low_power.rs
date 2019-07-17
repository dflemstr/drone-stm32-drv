use super::{TimDiverged, TimDivergedClockSel, TimPeriph};
use drone_cortex_m::{
    drv::timer::{TimerInterval, TimerOverflow, TimerSleep, TimerStop},
    thr::prelude::*,
};
use drone_stm32_map::periph::tim::low_power::{LowPowerTimMap, LowPowerTimPeriph};

/// Low-power timer diverged peripheral.
#[allow(missing_docs)]
pub struct LowPowerTimDiverged<T: LowPowerTimMap> {
    pub rcc_apb1enr_lptimen: T::SRccApb1EnrLptimen,
    pub rcc_apb1rstr_lptimrst: T::SRccApb1RstrLptimrst,
    pub rcc_apb1smenr_lptimsmen: T::SRccApb1SmenrLptimsmen,
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
            rcc_apb1enr_lptimen: self.rcc_apb1enr_lptimen,
            rcc_apb1rstr_lptimrst: self.rcc_apb1rstr_lptimrst,
            rcc_apb1smenr_lptimsmen: self.rcc_apb1smenr_lptimsmen,
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
    type RccApbenr = T::SRccApb1Enr;
    type RccApbenrTimen = T::SRccApb1EnrLptimen;
    type RccApbrstr = T::SRccApb1Rstr;
    type RccApbrstrTimrst = T::SRccApb1RstrLptimrst;
    type RccApbsmenr = T::SRccApb1Smenr;
    type RccApbsmenrTimsmen = T::SRccApb1SmenrLptimsmen;

    #[inline]
    fn rcc_apbenr_timen(&self) -> &Self::RccApbenrTimen {
        &self.rcc_apb1enr_lptimen
    }

    #[inline]
    fn rcc_apbrstr_timrst(&self) -> &Self::RccApbrstrTimrst {
        &self.rcc_apb1rstr_lptimrst
    }

    #[inline]
    fn rcc_apbsmenr_timsmen(&self) -> &Self::RccApbsmenrTimsmen {
        &self.rcc_apb1smenr_lptimsmen
    }

    #[inline]
    fn presc(&mut self, _value: u16) {
        unimplemented!();
    }

    #[inline]
    fn sleep<I: IntToken<Att>>(&mut self, _duration: usize, _int: I) -> TimerSleep<'_, Self> {
        unimplemented!()
    }

    #[inline]
    fn interval<I: IntToken<Att>>(
        &mut self,
        _duration: usize,
        _int: I,
    ) -> TimerInterval<'_, Self, Result<(), TimerOverflow>> {
        unimplemented!()
    }

    #[inline]
    fn interval_skip<I: IntToken<Att>>(
        &mut self,
        _duration: usize,
        _int: I,
    ) -> TimerInterval<'_, Self, ()> {
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
