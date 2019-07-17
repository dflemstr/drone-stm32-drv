use super::{TimDiverged, TimPeriph};
use drone_cortex_m::{
    drv::timer::{TimerInterval, TimerOverflow, TimerSleep, TimerStop},
    thr::prelude::*,
};
use drone_stm32_map::periph::tim::advanced::{AdvancedTimMap, AdvancedTimPeriph};

/// Advanced-control timer diverged peripheral.
#[allow(missing_docs)]
pub struct AdvancedTimDiverged<T: AdvancedTimMap> {
    pub rcc_apb2enr_timen: T::SRccApb2EnrTimen,
    pub rcc_apb2rstr_timrst: T::SRccApb2RstrTimrst,
    pub rcc_apb2smenr_timsmen: T::SRccApb2SmenrTimsmen,
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
            rcc_apb2enr_timen: self.rcc_apb2enr_timen,
            rcc_apb2rstr_timrst: self.rcc_apb2rstr_timrst,
            rcc_apb2smenr_timsmen: self.rcc_apb2smenr_timsmen,
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
    type RccApbenr = T::SRccApb2Enr;
    type RccApbenrTimen = T::SRccApb2EnrTimen;
    type RccApbrstr = T::SRccApb2Rstr;
    type RccApbrstrTimrst = T::SRccApb2RstrTimrst;
    type RccApbsmenr = T::SRccApb2Smenr;
    type RccApbsmenrTimsmen = T::SRccApb2SmenrTimsmen;

    #[inline]
    fn rcc_apbenr_timen(&self) -> &Self::RccApbenrTimen {
        &self.rcc_apb2enr_timen
    }

    #[inline]
    fn rcc_apbrstr_timrst(&self) -> &Self::RccApbrstrTimrst {
        &self.rcc_apb2rstr_timrst
    }

    #[inline]
    fn rcc_apbsmenr_timsmen(&self) -> &Self::RccApbsmenrTimsmen {
        &self.rcc_apb2smenr_timsmen
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

impl<T: AdvancedTimMap> TimerStop for AdvancedTimDiverged<T> {
    #[inline]
    fn stop(&mut self) {
        unimplemented!()
    }
}
