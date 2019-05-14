//! Timer.

mod advanced;
mod basic;
mod general;
mod low_power;

pub use self::{
  advanced::AdvancedTimDiverged, basic::BasicTimDiverged,
  general::GeneralTimDiverged, low_power::LowPowerTimDiverged,
};

use crate::common::{DrvClockSel, DrvRcc};
use drone_core::{
  bitfield::Bitfield,
  inventory::{Inventory0, InventoryGuard, InventoryResource},
};
use drone_cortex_m::{
  drv::timer::{Timer, TimerInterval, TimerOverflow, TimerSleep, TimerStop},
  reg::{marker::*, prelude::*},
  thr::prelude::*,
};

/// Timer driver.
pub struct Tim<T: TimPeriph, I: IntToken<Att>>(Inventory0<TimEn<T, I>>);

/// Timer enabled driver.
pub struct TimEn<T: TimPeriph, I: IntToken<Att>> {
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
  type RccApbenr: SRwRegBitBand;
  type RccApbenrTimen: SRwRwRegFieldBitBand<Reg = Self::RccApbenr>;
  type RccApbrstr: SRwRegBitBand;
  type RccApbrstrTimrst: SRwRwRegFieldBitBand<Reg = Self::RccApbrstr>;
  type RccApbsmenr: SRwRegBitBand;
  type RccApbsmenrTimsmen: SRwRwRegFieldBitBand<Reg = Self::RccApbsmenr>;

  fn rcc_apbenr_timen(&self) -> &Self::RccApbenrTimen;
  fn rcc_apbrstr_timrst(&self) -> &Self::RccApbrstrTimrst;
  fn rcc_apbsmenr_timsmen(&self) -> &Self::RccApbsmenrTimsmen;

  fn presc(&mut self, value: u16);

  fn sleep<I: IntToken<Att>>(
    &mut self,
    duration: usize,
    int: I,
  ) -> TimerSleep<'_, Self>;

  fn interval<I: IntToken<Att>>(
    &mut self,
    duration: usize,
    int: I,
  ) -> TimerInterval<'_, Self, Result<(), TimerOverflow>>;

  fn interval_skip<I: IntToken<Att>>(
    &mut self,
    duration: usize,
    int: I,
  ) -> TimerInterval<'_, Self, ()>;
}

#[doc(hidden)]
pub trait TimDivergedClockSel: TimDiverged {
  type RccCciprVal: Bitfield<Bits = u32>;
  type RccCcipr: SRwRegBitBand<Val = Self::RccCciprVal>;
  type RccCciprTimsel: SRwRwRegFieldBits<Reg = Self::RccCcipr>;

  fn rcc_ccipr_timsel(&self) -> &Self::RccCciprTimsel;
}

impl<T: TimPeriph, I: IntToken<Att>> Tim<T, I> {
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
    self.0.free().periph
  }

  /// Enables timer clock.
  pub fn enable(&mut self) -> InventoryGuard<'_, TimEn<T, I>> {
    self.setup();
    self.0.guard()
  }

  /// Enables timer clock.
  pub fn into_enabled(self) -> Inventory0<TimEn<T, I>> {
    self.setup();
    self.0
  }

  /// Disables timer clock.
  pub fn from_enabled(mut enabled: Inventory0<TimEn<T, I>>) -> Self {
    enabled.teardown();
    Self(enabled)
  }

  fn setup(&self) {
    let timen = &self.0.periph.rcc_apbenr_timen();
    if timen.read_bit() {
      panic!("Timer wasn't turned off");
    }
    timen.set_bit();
  }
}

impl<T: TimPeriph, I: IntToken<Att>> TimEn<T, I> {
  /// Sets the counter clock prescaler.
  pub fn presc(&mut self, value: u16) {
    self.periph.presc(value);
  }
}

impl<T: TimPeriph, I: IntToken<Att>> InventoryResource for TimEn<T, I> {
  fn teardown(&mut self) {
    self.periph.rcc_apbenr_timen().clear_bit()
  }
}

impl<T: TimPeriph, I: IntToken<Att>> Timer for TimEn<T, I> {
  type Stop = T::Diverged;

  fn sleep(&mut self, duration: usize) -> TimerSleep<'_, Self::Stop> {
    self.periph.sleep(duration, self.int)
  }

  fn interval(
    &mut self,
    duration: usize,
  ) -> TimerInterval<'_, Self::Stop, Result<(), TimerOverflow>> {
    self.periph.interval(duration, self.int)
  }

  fn interval_skip(
    &mut self,
    duration: usize,
  ) -> TimerInterval<'_, Self::Stop, ()> {
    self.periph.interval_skip(duration, self.int)
  }
}

impl<T: TimPeriph, I: IntToken<Att>> DrvRcc for Tim<T, I> {
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

impl<T: TimPeriph, I: IntToken<Att>> DrvRcc for TimEn<T, I> {
  fn reset(&mut self) {
    self.periph.rcc_apbrstr_timrst().set_bit();
  }

  fn disable_stop_mode(&self) {
    self.periph.rcc_apbsmenr_timsmen().clear_bit();
  }

  fn enable_stop_mode(&self) {
    self.periph.rcc_apbsmenr_timsmen().set_bit();
  }
}

impl<T: TimPeriph, I: IntToken<Att>> DrvClockSel for Tim<T, I>
where
  T::Diverged: TimDivergedClockSel,
{
  #[inline]
  fn clock_sel(&self, value: u32) {
    self.0.clock_sel(value);
  }
}

impl<T: TimPeriph, I: IntToken<Att>> DrvClockSel for TimEn<T, I>
where
  T::Diverged: TimDivergedClockSel,
{
  fn clock_sel(&self, value: u32) {
    self.periph.rcc_ccipr_timsel().write_bits(value);
  }
}
