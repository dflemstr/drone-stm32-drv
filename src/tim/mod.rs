//! Timer.

mod advanced;
mod basic;
mod general;
mod low_power;

pub use self::{
    advanced::AdvancedTimDiverged, basic::BasicTimDiverged, general::GeneralTimDiverged,
    low_power::LowPowerTimDiverged,
};

use crate::common::{DrvClockSel, DrvRcc};
use core::num::NonZeroUsize;
use drone_core::{
    bitfield::Bitfield,
    inventory::{self, Inventory0, Inventory1},
};
use drone_cortex_m::{
    drv::timer::{Timer, TimerInterval, TimerOverflow, TimerSleep, TimerStop},
    reg::{marker::*, prelude::*},
    thr::prelude::*,
};

/// Timer driver.
pub struct Tim<T: TimPeriph, I: IntToken>(Inventory0<TimEn<T, I>>);

/// Timer enabled driver.
pub struct TimEn<T: TimPeriph, I: IntToken> {
    periph: T::Diverged,
    int: I,
}

/// Timer peripheral.
pub trait TimPeriph {
    /// Timer diverged peripheral.
    type Diverged: TimDiverged;

    /// Converts to the diverged peripheral.
    fn diverge(self) -> Self::Diverged;
}

#[doc(hidden)]
pub trait TimDiverged: TimerStop + Sized + Send + Sync + 'static {
    type RccBusenr: SRwRegBitBand;
    type RccBusenrTimen: SRwRwRegFieldBitBand<Reg = Self::RccBusenr>;
    type RccBusrstr: SRwRegBitBand;
    type RccBusrstrTimrst: SRwRwRegFieldBitBand<Reg = Self::RccBusrstr>;
    type RccBussmenr: SRwRegBitBand;
    type RccBussmenrTimsmen: SRwRwRegFieldBitBand<Reg = Self::RccBussmenr>;

    fn rcc_busenr_timen(&self) -> &Self::RccBusenrTimen;
    fn rcc_busrstr_timrst(&self) -> &Self::RccBusrstrTimrst;
    fn rcc_bussmenr_timsmen(&self) -> &Self::RccBussmenrTimsmen;

    fn presc(&mut self, value: u32);

    fn sleep<I: IntToken>(&mut self, duration: u32, int: I) -> TimerSleep<'_, Self>;

    fn interval<I: IntToken>(
        &mut self,
        duration: u32,
        int: I,
    ) -> TimerInterval<'_, Self, Result<NonZeroUsize, TimerOverflow>>;

    fn interval_skip<I: IntToken>(
        &mut self,
        duration: u32,
        int: I,
    ) -> TimerInterval<'_, Self, NonZeroUsize>;
}

#[doc(hidden)]
pub trait TimDivergedClockSel: TimDiverged {
    type RccCciprVal: Bitfield<Bits = u32>;
    type RccCcipr: SRwRegBitBand<Val = Self::RccCciprVal>;
    type RccCciprTimsel: SRwRwRegFieldBits<Reg = Self::RccCcipr>;

    fn rcc_ccipr_timsel(&self) -> &Self::RccCciprTimsel;
}

impl<T: TimPeriph, I: IntToken> Tim<T, I> {
    /// Creates a new [`Tim`].
    #[inline]
    pub fn new(periph: T, int: I) -> Self {
        Self(Inventory0::new(TimEn {
            periph: periph.diverge(),
            int,
        }))
    }

    /// Creates a new [`Tim`].
    ///
    /// # Safety
    ///
    /// Some of the `Crt` register tokens can be still in use.
    #[inline]
    pub unsafe fn from_diverged(periph: T::Diverged, int: I) -> Self {
        Self(Inventory0::new(TimEn { periph, int }))
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> T::Diverged {
        Inventory0::free(self.0).periph
    }

    /// Enables timer clock.
    pub fn enable(&mut self) -> inventory::Guard<'_, TimEn<T, I>> {
        self.setup();
        Inventory0::guard(&mut self.0)
    }

    /// Enables timer clock.
    pub fn into_enabled(self) -> Inventory1<TimEn<T, I>> {
        self.setup();
        let (enabled, token) = self.0.share1();
        // To be recreated in `from_enabled()`.
        drop(token);
        enabled
    }

    /// Disables timer clock.
    pub fn from_enabled(enabled: Inventory1<TimEn<T, I>>) -> Self {
        // Restoring the token dropped in `into_enabled()`.
        let token = unsafe { inventory::Token::new() };
        let mut enabled = enabled.merge1(token);
        Inventory0::teardown(&mut enabled);
        Self(enabled)
    }

    fn setup(&self) {
        let timen = &self.0.periph.rcc_busenr_timen();
        if timen.read_bit() {
            panic!("Timer wasn't turned off");
        }
        timen.set_bit();
    }
}

impl<T: TimPeriph, I: IntToken> TimEn<T, I> {
    /// Sets the counter clock prescaler.
    pub fn presc(&mut self, value: u32) {
        self.periph.presc(value);
    }
}

impl<T: TimPeriph, I: IntToken> inventory::Item for TimEn<T, I> {
    fn teardown(&mut self, _token: &mut inventory::GuardToken<Self>) {
        self.periph.rcc_busenr_timen().clear_bit()
    }
}

impl<T: TimPeriph, I: IntToken> Timer for TimEn<T, I> {
    type Stop = T::Diverged;

    fn sleep(&mut self, duration: u32) -> TimerSleep<'_, Self::Stop> {
        self.periph.sleep(duration, self.int)
    }

    fn interval(
        &mut self,
        duration: u32,
    ) -> TimerInterval<'_, Self::Stop, Result<NonZeroUsize, TimerOverflow>> {
        self.periph.interval(duration, self.int)
    }

    fn interval_skip(&mut self, duration: u32) -> TimerInterval<'_, Self::Stop, NonZeroUsize> {
        self.periph.interval_skip(duration, self.int)
    }
}

impl<T: TimPeriph, I: IntToken> DrvRcc for Tim<T, I> {
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

impl<T: TimPeriph, I: IntToken> DrvRcc for TimEn<T, I> {
    fn reset(&mut self) {
        self.periph.rcc_busrstr_timrst().set_bit();
    }

    fn disable_stop_mode(&self) {
        self.periph.rcc_bussmenr_timsmen().clear_bit();
    }

    fn enable_stop_mode(&self) {
        self.periph.rcc_bussmenr_timsmen().set_bit();
    }
}

impl<T: TimPeriph, I: IntToken> DrvClockSel for Tim<T, I>
where
    T::Diverged: TimDivergedClockSel,
{
    #[inline]
    fn clock_sel(&self, value: u32) {
        self.0.clock_sel(value);
    }
}

impl<T: TimPeriph, I: IntToken> DrvClockSel for TimEn<T, I>
where
    T::Diverged: TimDivergedClockSel,
{
    fn clock_sel(&self, value: u32) {
        self.periph.rcc_ccipr_timsel().write_bits(value);
    }
}
