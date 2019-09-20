use super::{TimDiverged, TimPeriph};
use core::{convert::identity, num::NonZeroUsize};
use drone_cortex_m::{
    drv::timer::{TimerInterval, TimerOverflow, TimerSleep, TimerStop},
    fib::{self, Fiber},
    reg::prelude::*,
    thr::prelude::*,
};
use drone_stm32_map::periph::tim::basic::{traits::*, BasicTimMap, BasicTimPeriph};

/// Basic timer diverged peripheral.
#[allow(missing_docs)]
pub struct BasicTimDiverged<T: BasicTimMap> {
    pub rcc_busenr_timen: T::SRccBusenrTimen,
    pub rcc_busrstr_timrst: T::SRccBusrstrTimrst,
    pub rcc_bussmenr_timsmen: T::SRccBussmenrTimsmen,
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
            rcc_busenr_timen: self.rcc_busenr_timen,
            rcc_busrstr_timrst: self.rcc_busrstr_timrst,
            rcc_bussmenr_timsmen: self.rcc_bussmenr_timsmen,
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
    fn presc(&mut self, value: u32) {
        self.tim_psc.store_val({
            let mut val = self.tim_psc.default_val();
            self.tim_psc.psc().write(&mut val, value);
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
    fn sleep<I: IntToken>(&mut self, duration: u32, int: I) -> TimerSleep<'_, Self> {
        let uif = *self.tim_sr.uif();
        let future = Box::pin(int.add_future(fib::new_fn(move || {
            if uif.read_bit() {
                uif.clear_bit();
                fib::Complete(())
            } else {
                fib::Yielded(())
            }
        })));
        self.schedule(duration, |mut val| {
            self.tim_cr1.opm().set(&mut val);
            val
        });
        TimerSleep::new(self, future)
    }

    #[inline]
    fn interval<I: IntToken>(
        &mut self,
        duration: u32,
        int: I,
    ) -> TimerInterval<'_, Self, Result<NonZeroUsize, TimerOverflow>> {
        let stream = Box::pin(int.add_stream_pulse(
            || Err(TimerOverflow),
            Self::interval_fib(*self.tim_sr.uif()),
        ));
        self.schedule(duration, identity);
        TimerInterval::new(self, stream)
    }

    #[inline]
    fn interval_skip<I: IntToken>(
        &mut self,
        duration: u32,
        int: I,
    ) -> TimerInterval<'_, Self, NonZeroUsize> {
        let stream = Box::pin(int.add_stream_pulse_skip(Self::interval_fib(*self.tim_sr.uif())));
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
    ) -> impl Fiber<Input = (), Yield = Option<usize>, Return = R> {
        fib::new_fn(move || {
            if uif.read_bit() {
                uif.set_bit();
                fib::Yielded(Some(1))
            } else {
                fib::Yielded(None)
            }
        })
    }

    fn schedule(&self, duration: u32, f: impl FnOnce(T::TimCr1Val) -> T::TimCr1Val) {
        self.tim_cnt.reset();
        self.tim_arr.store_val({
            let mut val = self.tim_arr.default_val();
            self.tim_arr.arr().write(&mut val, duration);
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
