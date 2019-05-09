use super::{TimDiverged, TimPeriph};
use core::convert::identity;
use drone_cortex_m::{
  drv::timer::{TimerInterval, TimerOverflow, TimerSleep, TimerStop},
  fib::{self, Fiber},
  reg::prelude::*,
  thr::prelude::*,
};
use drone_stm32_map::periph::tim::basic::{
  traits::*, BasicTimMap, BasicTimPeriph,
};

/// Basic timer diverged peripheral.
#[allow(missing_docs)]
pub struct BasicTimDiverged<T: BasicTimMap> {
  pub rcc_apb1enr1_timen: T::SRccApb1Enr1Timen,
  pub rcc_apb1rstr1_timrst: T::SRccApb1Rstr1Timrst,
  pub rcc_apb1smenr1_timsmen: T::SRccApb1Smenr1Timsmen,
  pub tim_cr1: T::STimCr1,
  pub tim_cr2: T::STimCr2,
  pub tim_dier: T::STimDier,
  pub tim_sr: T::CTimSr,
  pub tim_egr: T::STimEgr,
  pub tim_cnt: T::STimCnt,
  pub tim_psc: T::STimPsc,
  pub tim_arr: T::STimArr,
}

impl<T: BasicTimMap> TimPeriph for BasicTimPeriph<T> {
  type Diverged = BasicTimDiverged<T>;

  #[inline]
  fn diverge(self) -> Self::Diverged {
    BasicTimDiverged {
      rcc_apb1enr1_timen: self.rcc_apb1enr1_timen,
      rcc_apb1rstr1_timrst: self.rcc_apb1rstr1_timrst,
      rcc_apb1smenr1_timsmen: self.rcc_apb1smenr1_timsmen,
      tim_cr1: self.tim_cr1,
      tim_cr2: self.tim_cr2,
      tim_dier: self.tim_dier,
      tim_sr: self.tim_sr.into_copy(),
      tim_egr: self.tim_egr,
      tim_cnt: self.tim_cnt,
      tim_psc: self.tim_psc,
      tim_arr: self.tim_arr,
    }
  }
}

impl<T: BasicTimMap> TimDiverged for BasicTimDiverged<T> {
  type RccApbenr = T::SRccApb1Enr1;
  type RccApbenrTimen = T::SRccApb1Enr1Timen;
  type RccApbrstr = T::SRccApb1Rstr1;
  type RccApbrstrTimrst = T::SRccApb1Rstr1Timrst;
  type RccApbsmenr = T::SRccApb1Smenr1;
  type RccApbsmenrTimsmen = T::SRccApb1Smenr1Timsmen;

  #[inline]
  fn rcc_apbenr_timen(&self) -> &Self::RccApbenrTimen {
    &self.rcc_apb1enr1_timen
  }

  #[inline]
  fn rcc_apbrstr_timrst(&self) -> &Self::RccApbrstrTimrst {
    &self.rcc_apb1rstr1_timrst
  }

  #[inline]
  fn rcc_apbsmenr_timsmen(&self) -> &Self::RccApbsmenrTimsmen {
    &self.rcc_apb1smenr1_timsmen
  }

  #[inline]
  fn presc(&mut self, value: u16) {
    self.tim_psc.store_val({
      let mut val = self.tim_psc.default_val();
      self.tim_psc.psc().write(&mut val, u32::from(value));
      val
    });
    self.tim_dier.reset();
    self.tim_egr.store_val({
      let mut val = self.tim_egr.default_val();
      self.tim_egr.ug().set(&mut val);
      val
    });
    self.tim_sr.reset();
    self.tim_dier.store_val({
      let mut val = self.tim_dier.default_val();
      self.tim_dier.uie().set(&mut val);
      val
    });
  }

  #[inline]
  fn sleep<I: IntToken<Att>>(
    &mut self,
    duration: usize,
    int: I,
  ) -> TimerSleep<'_, Self> {
    let uif = *self.tim_sr.uif();
    let future = Box::pin(int.add_future(fib::new(move || {
      loop {
        if uif.read_bit() {
          uif.clear_bit();
          break;
        }
        yield;
      }
    })));
    self.schedule(duration, |mut val| {
      self.tim_cr1.opm().set(&mut val);
      val
    });
    TimerSleep::new(self, future)
  }

  #[inline]
  fn interval<I: IntToken<Att>>(
    &mut self,
    duration: usize,
    int: I,
  ) -> TimerInterval<'_, Self, Result<(), TimerOverflow>> {
    let stream = Box::pin(int.add_stream(
      || Err(TimerOverflow),
      Self::interval_fib(*self.tim_sr.uif()),
    ));
    self.schedule(duration, identity);
    TimerInterval::new(self, stream)
  }

  #[inline]
  fn interval_skip<I: IntToken<Att>>(
    &mut self,
    duration: usize,
    int: I,
  ) -> TimerInterval<'_, Self, ()> {
    let stream =
      Box::pin(int.add_stream_skip(Self::interval_fib(*self.tim_sr.uif())));
    self.schedule(duration, identity);
    TimerInterval::new(self, stream)
  }
}

impl<T: BasicTimMap> TimerStop for BasicTimDiverged<T> {
  #[inline]
  fn stop(&mut self) {
    self.tim_cr1.reset();
  }
}

impl<T: BasicTimMap> BasicTimDiverged<T> {
  fn interval_fib<R>(
    uif: T::CTimSrUif,
  ) -> impl Fiber<Input = (), Yield = Option<()>, Return = R> {
    fib::new(move || {
      loop {
        if uif.read_bit() {
          uif.set_bit();
          yield Some(());
        }
        yield None;
      }
    })
  }

  fn schedule(
    &self,
    duration: usize,
    f: impl FnOnce(T::TimCr1Val) -> T::TimCr1Val,
  ) {
    self.tim_cnt.reset();
    self.tim_arr.store_val({
      let mut val = self.tim_arr.default_val();
      self.tim_arr.arr().write(&mut val, duration as u32);
      val
    });
    self.tim_cr1.store_val({
      let mut val = self.tim_cr1.default_val();
      self.tim_cr1.cen().set(&mut val);
      self.tim_cr1.urs().set(&mut val);
      f(val)
    });
  }
}
