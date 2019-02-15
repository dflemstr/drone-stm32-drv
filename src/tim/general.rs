use super::{TimDiverged, TimPeriph};
use drone_cortex_m::{
  drv::timer::{TimerInterval, TimerOverflow, TimerSleep, TimerStop},
  thr::prelude::*,
};
use drone_stm32_map::periph::tim::general::{GeneralTimMap, GeneralTimPeriph};

/// General-purpose timer diverged peripheral.
#[allow(missing_docs)]
pub struct GeneralTimDiverged<T: GeneralTimMap> {
  pub rcc_apbenr_timen: T::SRccApbenrTimen,
  pub rcc_apbrstr_timrst: T::SRccApbrstrTimrst,
  pub rcc_apbsmenr_timsmen: T::SRccApbsmenrTimsmen,
  pub tim_cr1: T::STimCr1,
  pub tim_cr2: T::STimCr2,
  pub tim_smcr: T::STimSmcrOpt,
  pub tim_dier: T::STimDier,
  pub tim_sr: T::STimSr,
  pub tim_egr: T::STimEgr,
  pub tim_ccmr1_output: T::STimCcmr1Output,
  pub tim_ccmr1_input: T::STimCcmr1Input,
  pub tim_ccmr2_output: T::STimCcmr2OutputOpt,
  pub tim_ccmr2_input: T::STimCcmr2InputOpt,
  pub tim_ccer: T::STimCcer,
  pub tim_cnt: T::STimCnt,
  pub tim_psc: T::STimPsc,
  pub tim_arr: T::STimArr,
  pub tim_rcr: T::STimRcrOpt,
  pub tim_ccr1: T::STimCcr1,
  pub tim_ccr2: T::STimCcr2Opt,
  pub tim_ccr3: T::STimCcr3Opt,
  pub tim_ccr4: T::STimCcr4Opt,
  pub tim_bdtr: T::STimBdtrOpt,
  pub tim_dcr: T::STimDcr,
  pub tim_dmar: T::STimDmar,
  pub tim_or1: T::STimOr1Opt,
  pub tim_or2: T::STimOr2Opt,
}

impl<T: GeneralTimMap> TimPeriph for GeneralTimPeriph<T> {
  type Diverged = GeneralTimDiverged<T>;

  #[inline]
  fn diverge(self) -> Self::Diverged {
    GeneralTimDiverged {
      rcc_apbenr_timen: self.rcc_apbenr_timen,
      rcc_apbrstr_timrst: self.rcc_apbrstr_timrst,
      rcc_apbsmenr_timsmen: self.rcc_apbsmenr_timsmen,
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
      tim_or2: self.tim_or2,
    }
  }
}

impl<T: GeneralTimMap> TimDiverged for GeneralTimDiverged<T> {
  type RccApbenr = T::SRccApbenr;
  type RccApbenrTimen = T::SRccApbenrTimen;
  type RccApbrstr = T::SRccApbrstr;
  type RccApbrstrTimrst = T::SRccApbrstrTimrst;
  type RccApbsmenr = T::SRccApbsmenr;
  type RccApbsmenrTimsmen = T::SRccApbsmenrTimsmen;

  #[inline]
  fn rcc_apbenr_timen(&self) -> &Self::RccApbenrTimen {
    &self.rcc_apbenr_timen
  }

  #[inline]
  fn rcc_apbrstr_timrst(&self) -> &Self::RccApbrstrTimrst {
    &self.rcc_apbrstr_timrst
  }

  #[inline]
  fn rcc_apbsmenr_timsmen(&self) -> &Self::RccApbsmenrTimsmen {
    &self.rcc_apbsmenr_timsmen
  }

  #[inline]
  fn presc(&mut self, _value: u16) {
    unimplemented!();
  }

  #[inline]
  fn sleep<I: IntToken<Att>>(
    &mut self,
    _duration: usize,
    _int: I,
  ) -> TimerSleep<'_, Self> {
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

impl<T: GeneralTimMap> TimerStop for GeneralTimDiverged<T> {
  #[inline]
  fn stop(&mut self) {
    unimplemented!()
  }
}
