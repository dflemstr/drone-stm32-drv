use super::{TimDiverged, TimPeriph};
use core::num::NonZeroUsize;
use drone_cortex_m::{
    drv::timer::{TimerInterval, TimerOverflow, TimerSleep, TimerStop},
    thr::prelude::*,
};
use drone_stm32_map::periph::tim::advanced::{AdvancedTimMap, AdvancedTimPeriph};

/// Advanced-control timer diverged peripheral.
#[allow(missing_docs)]
pub struct AdvancedTimDiverged<T: AdvancedTimMap> {
    pub rcc_busenr_timen: T::SRccBusenrTimen,
    pub rcc_busrstr_timrst: T::SRccBusrstrTimrst,
    pub rcc_bussmenr_timsmen: T::SRccBussmenrTimsmen,
    pub tim_cr1: T::STimCr1,
    pub tim_cr2: T::STimCr2,
    pub tim_smcr: T::STimSmcr,
    pub tim_dier: T::STimDier,
    pub tim_sr: T::STimSr,
    pub tim_egr: T::STimEgr,
    pub tim_ccmr1_output: T::STimCcmr1Output,
    pub tim_ccmr1_input: T::STimCcmr1Input,
    pub tim_ccmr2_output: T::STimCcmr2Output,
    pub tim_ccmr2_input: T::STimCcmr2Input,
    pub tim_ccer: T::STimCcer,
    pub tim_cnt: T::STimCnt,
    pub tim_psc: T::STimPsc,
    pub tim_arr: T::STimArr,
    pub tim_rcr: T::STimRcr,
    pub tim_ccr1: T::STimCcr1,
    pub tim_ccr2: T::STimCcr2,
    pub tim_ccr3: T::STimCcr3,
    pub tim_ccr4: T::STimCcr4,
    pub tim_bdtr: T::STimBdtr,
    pub tim_dcr: T::STimDcr,
    pub tim_dmar: T::STimDmar,
    pub tim_or1: T::STimOr1,
    pub tim_ccmr3_output: T::STimCcmr3Output,
    pub tim_ccr5: T::STimCcr5,
    pub tim_ccr6: T::STimCcr6,
    pub tim_or2: T::STimOr2,
    pub tim_or3: T::STimOr3,
}

impl<T: AdvancedTimMap> TimPeriph for AdvancedTimPeriph<T> {
    type Diverged = AdvancedTimDiverged<T>;

    #[inline]
    fn diverge(self) -> Self::Diverged {
        AdvancedTimDiverged {
            rcc_busenr_timen: self.rcc_busenr_timen,
            rcc_busrstr_timrst: self.rcc_busrstr_timrst,
            rcc_bussmenr_timsmen: self.rcc_bussmenr_timsmen,
            tim_cr1: self.tim_cr1,
            tim_cr2: self.tim_cr2,
            tim_smcr: self.tim_smcr,
            tim_dier: self.tim_dier,
            tim_sr: self.tim_sr,
            tim_egr: self.tim_egr,
            tim_ccmr1_output: self.tim_ccmr1_output,
            tim_ccmr1_input: self.tim_ccmr1_input,
            tim_ccmr2_output: self.tim_ccmr2_output,
            tim_ccmr2_input: self.tim_ccmr2_input,
            tim_ccer: self.tim_ccer,
            tim_cnt: self.tim_cnt,
            tim_psc: self.tim_psc,
            tim_arr: self.tim_arr,
            tim_rcr: self.tim_rcr,
            tim_ccr1: self.tim_ccr1,
            tim_ccr2: self.tim_ccr2,
            tim_ccr3: self.tim_ccr3,
            tim_ccr4: self.tim_ccr4,
            tim_bdtr: self.tim_bdtr,
            tim_dcr: self.tim_dcr,
            tim_dmar: self.tim_dmar,
            tim_or1: self.tim_or1,
            tim_ccmr3_output: self.tim_ccmr3_output,
            tim_ccr5: self.tim_ccr5,
            tim_ccr6: self.tim_ccr6,
            tim_or2: self.tim_or2,
            tim_or3: self.tim_or3,
        }
    }
}

impl<T: AdvancedTimMap> TimDiverged for AdvancedTimDiverged<T> {
    type RccBusenr = T::SRccBusenr;
    type RccBusenrTimen = T::SRccBusenrTimen;
    type RccBusrstr = T::SRccBusrstr;
    type RccBusrstrTimrst = T::SRccBusrstrTimrst;
    type RccBussmenr = T::SRccBussmenr;
    type RccBussmenrTimsmen = T::SRccBussmenrTimsmen;

    #[inline]
    fn rcc_busenr_timen(&self) -> &Self::RccBusenrTimen {
        &self.rcc_busenr_timen
    }

    #[inline]
    fn rcc_busrstr_timrst(&self) -> &Self::RccBusrstrTimrst {
        &self.rcc_busrstr_timrst
    }

    #[inline]
    fn rcc_bussmenr_timsmen(&self) -> &Self::RccBussmenrTimsmen {
        &self.rcc_bussmenr_timsmen
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

impl<T: AdvancedTimMap> TimerStop for AdvancedTimDiverged<T> {
    #[inline]
    fn stop(&mut self) {
        unimplemented!()
    }
}
