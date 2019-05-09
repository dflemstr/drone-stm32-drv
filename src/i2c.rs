//! Inter-Integrated Circuit.

use crate::{
  common::{DrvClockSel, DrvDmaRx, DrvDmaTx, DrvRcc},
  dma::{DmaChEn, DmaTransferError},
  select3::{Output3, Select3},
};
use core::marker::PhantomData;
use drone_core::{
  awt,
  shared_guard::{Guard, GuardHandler},
};
use drone_cortex_m::{fib, reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::{
  dma::ch::{traits::*, DmaChMap},
  i2c::{traits::*, I2CMap, I2CPeriph},
};
use failure::Fail;
use futures::prelude::*;

/// I2C DMA error.
#[derive(Debug, Fail)]
pub enum I2CDmaError {
  /// DMA error.
  #[fail(display = "DMA error: {}", _0)]
  Dma(DmaTransferError),
  /// I2C transfer failure.
  #[fail(display = "I2C failure: {}", _0)]
  I2CBreak(I2CBreak),
  /// I2C error.
  #[fail(display = "I2C error: {}", _0)]
  I2CError(I2CError),
}

/// I2C error.
#[derive(Debug, Fail)]
pub enum I2CError {
  /// Bus error.
  #[fail(display = "I2C bus error.")]
  Berr,
  /// Overrun/Underrun.
  #[fail(display = "I2C overrun.")]
  Ovr,
  /// Arbitration lost.
  #[fail(display = "I2C arbitration lost.")]
  Arlo,
  /// Timeout or t_low detection flag.
  #[fail(display = "I2C timeout.")]
  Timeout,
  /// SMBus alert.
  #[fail(display = "I2C SMBus alert.")]
  Alert,
  /// PEC error in reception.
  #[fail(display = "I2C PEC error.")]
  Pecerr,
}

/// I2C transfer failure event.
#[derive(Debug, Fail)]
pub enum I2CBreak {
  /// NACK reception.
  #[fail(display = "I2C NACK received.")]
  Nack,
  /// Stop reception.
  #[fail(display = "I2C STOP received.")]
  Stop,
}

/// I2C driver.
pub struct I2C<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>>(
  I2CEn<T, Ev, Er>,
);

/// I2C enabled driver.
pub struct I2CEn<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>> {
  periph: I2CDiverged<T>,
  int_ev: Ev,
  int_er: Er,
}

/// I2C enabled guard handler.
pub struct I2CEnGuard<T: I2CMap>(PhantomData<T>);

/// I2C diverged peripheral.
#[allow(missing_docs)]
pub struct I2CDiverged<T: I2CMap> {
  pub rcc_apb1enr_i2cen: T::SRccApb1EnrI2Cen,
  pub rcc_apb1rstr_i2crst: T::SRccApb1RstrI2Crst,
  pub rcc_apb1smenr_i2csmen: T::SRccApb1SmenrI2Csmen,
  pub rcc_ccipr_i2csel: T::SRccCciprI2Csel,
  pub i2c_cr1: T::SI2CCr1,
  pub i2c_cr2: T::SI2CCr2,
  pub i2c_oar1: T::SI2COar1,
  pub i2c_oar2: T::SI2COar2,
  pub i2c_timingr: T::SI2CTimingr,
  pub i2c_timeoutr: T::SI2CTimeoutr,
  pub i2c_isr: T::CI2CIsr,
  pub i2c_icr: T::CI2CIcr,
  pub i2c_pecr: T::SI2CPecr,
  pub i2c_rxdr: T::SI2CRxdr,
  pub i2c_txdr: T::SI2CTxdr,
}

impl<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>> I2C<T, Ev, Er> {
  /// Creates a new [`I2C`].
  #[inline]
  pub fn new(periph: I2CPeriph<T>, int_ev: Ev, int_er: Er) -> Self {
    let periph = I2CDiverged {
      rcc_apb1enr_i2cen: periph.rcc_apb1enr_i2cen,
      rcc_apb1rstr_i2crst: periph.rcc_apb1rstr_i2crst,
      rcc_apb1smenr_i2csmen: periph.rcc_apb1smenr_i2csmen,
      rcc_ccipr_i2csel: periph.rcc_ccipr_i2csel,
      i2c_cr1: periph.i2c_cr1,
      i2c_cr2: periph.i2c_cr2,
      i2c_oar1: periph.i2c_oar1,
      i2c_oar2: periph.i2c_oar2,
      i2c_timingr: periph.i2c_timingr,
      i2c_timeoutr: periph.i2c_timeoutr,
      i2c_isr: periph.i2c_isr.into_copy(),
      i2c_icr: periph.i2c_icr.into_copy(),
      i2c_pecr: periph.i2c_pecr,
      i2c_rxdr: periph.i2c_rxdr,
      i2c_txdr: periph.i2c_txdr,
    };
    Self(I2CEn {
      periph,
      int_ev,
      int_er,
    })
  }

  /// Creates a new [`I2C`].
  ///
  /// # Safety
  ///
  /// Some of the `Crt` register tokens can be still in use.
  #[inline]
  pub unsafe fn from_diverged(
    periph: I2CDiverged<T>,
    int_ev: Ev,
    int_er: Er,
  ) -> Self {
    Self(I2CEn {
      periph,
      int_ev,
      int_er,
    })
  }

  /// Releases the peripheral.
  #[inline]
  pub fn free(self) -> I2CDiverged<T> {
    self.0.periph
  }

  /// Enables I2C clock.
  pub fn enable(&mut self) -> Guard<'_, I2CEn<T, Ev, Er>, I2CEnGuard<T>> {
    let i2cen = &self.0.periph.rcc_apb1enr_i2cen;
    if i2cen.read_bit() {
      panic!("I2C wasn't turned off");
    }
    i2cen.set_bit();
    Guard::new(&mut self.0, I2CEnGuard(PhantomData))
  }

  /// Enables I2C clock.
  pub fn into_enabled(self) -> I2CEn<T, Ev, Er> {
    self.0.periph.rcc_apb1enr_i2cen.set_bit();
    self.0
  }
}

impl<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>> I2CEn<T, Ev, Er> {
  /// Disables I2C clock.
  pub fn into_disabled(self) -> I2C<T, Ev, Er> {
    self.periph.rcc_apb1enr_i2cen.clear_bit();
    I2C(self)
  }

  /// Reads bytes to `buf` from `slave_addr`. Leaves the session open.
  ///
  /// # Panics
  ///
  /// If length of `buf` is greater than 255.
  pub fn read<'a, Rx: DmaChMap>(
    &'a self,
    dma_rx: &'a DmaChEn<Rx, impl IntToken<Att>>,
    buf: &'a mut [u8],
    slave_addr: u8,
    i2c_cr1_val: T::I2CCr1Val,
    i2c_cr2_val: T::I2CCr2Val,
  ) -> impl Future<Output = Result<(), I2CDmaError>> + 'a {
    self.read_impl(dma_rx, buf, slave_addr, i2c_cr1_val, i2c_cr2_val, false)
  }

  /// Reads bytes to `buf` from `slave_addr`. Closes the session afterwards.
  ///
  /// # Panics
  ///
  /// If length of `buf` is greater than 255.
  pub fn read_and_stop<'a, Rx: DmaChMap>(
    &'a self,
    dma_rx: &'a DmaChEn<Rx, impl IntToken<Att>>,
    buf: &'a mut [u8],
    slave_addr: u8,
    i2c_cr1_val: T::I2CCr1Val,
    i2c_cr2_val: T::I2CCr2Val,
  ) -> impl Future<Output = Result<(), I2CDmaError>> + 'a {
    self.read_impl(dma_rx, buf, slave_addr, i2c_cr1_val, i2c_cr2_val, true)
  }

  /// Writes bytes from `buf` to `slave_addr`. Leaves the session open.
  ///
  /// # Panics
  ///
  /// If length of `buf` is greater than 255.
  pub fn write<'a, Tx: DmaChMap>(
    &'a self,
    dma_tx: &'a DmaChEn<Tx, impl IntToken<Att>>,
    buf: &'a [u8],
    slave_addr: u8,
    i2c_cr1_val: T::I2CCr1Val,
    i2c_cr2_val: T::I2CCr2Val,
  ) -> impl Future<Output = Result<(), I2CDmaError>> + 'a {
    self.write_impl(dma_tx, buf, slave_addr, i2c_cr1_val, i2c_cr2_val, false)
  }

  /// Writes bytes from `buf` to `slave_addr`. Closes the session afterwards.
  ///
  /// # Panics
  ///
  /// If length of `buf` is greater than 255.
  pub fn write_and_stop<'a, Tx: DmaChMap>(
    &'a self,
    dma_tx: &'a DmaChEn<Tx, impl IntToken<Att>>,
    buf: &'a [u8],
    slave_addr: u8,
    i2c_cr1_val: T::I2CCr1Val,
    i2c_cr2_val: T::I2CCr2Val,
  ) -> impl Future<Output = Result<(), I2CDmaError>> + 'a {
    self.write_impl(dma_tx, buf, slave_addr, i2c_cr1_val, i2c_cr2_val, true)
  }

  /// Returns a future, which resolves on I2C error event.
  pub fn transfer_error(&self) -> impl Future<Output = I2CError> {
    let berr = *self.periph.i2c_isr.berr();
    let ovr = *self.periph.i2c_isr.ovr();
    let arlo = *self.periph.i2c_isr.arlo();
    let timeout = *self.periph.i2c_isr.timeout();
    let alert = *self.periph.i2c_isr.alert();
    let pecerr = *self.periph.i2c_isr.pecerr();
    let berrcf = *self.periph.i2c_icr.berrcf();
    let ovrcf = *self.periph.i2c_icr.ovrcf();
    let arlocf = *self.periph.i2c_icr.arlocf();
    let timoutcf = *self.periph.i2c_icr.timoutcf();
    let alertcf = *self.periph.i2c_icr.alertcf();
    let peccf = *self.periph.i2c_icr.peccf();
    self.int_er.add_future(fib::new(move || {
      loop {
        if berr.read_bit_band() {
          berrcf.set_bit_band();
          break I2CError::Berr;
        }
        if ovr.read_bit_band() {
          ovrcf.set_bit_band();
          break I2CError::Ovr;
        }
        if arlo.read_bit_band() {
          arlocf.set_bit_band();
          break I2CError::Arlo;
        }
        if timeout.read_bit_band() {
          timoutcf.set_bit_band();
          break I2CError::Timeout;
        }
        if alert.read_bit_band() {
          alertcf.set_bit_band();
          break I2CError::Alert;
        }
        if pecerr.read_bit_band() {
          peccf.set_bit_band();
          break I2CError::Pecerr;
        }
        yield;
      }
    }))
  }

  /// Returns a future, which resolves on I2C transfer failure event.
  pub fn transfer_break(&self) -> impl Future<Output = I2CBreak> {
    let nackf = *self.periph.i2c_isr.nackf();
    let stopf = *self.periph.i2c_isr.stopf();
    let nackcf = *self.periph.i2c_icr.nackcf();
    let stopcf = *self.periph.i2c_icr.stopcf();
    self.int_ev.add_future(fib::new(move || {
      loop {
        if nackf.read_bit_band() {
          nackcf.set_bit_band();
          break I2CBreak::Nack;
        }
        if stopf.read_bit_band() {
          stopcf.set_bit_band();
          break I2CBreak::Stop;
        }
        yield;
      }
    }))
  }

  fn read_impl<'a, Rx: DmaChMap>(
    &'a self,
    dma_rx: &'a DmaChEn<Rx, impl IntToken<Att>>,
    buf: &'a mut [u8],
    slave_addr: u8,
    mut i2c_cr1_val: T::I2CCr1Val,
    mut i2c_cr2_val: T::I2CCr2Val,
    autoend: bool,
  ) -> impl Future<Output = Result<(), I2CDmaError>> + 'a {
    if buf.len() > 255 {
      panic!("I2C read overflow");
    }
    asnc(move || {
      unsafe { dma_rx.set_maddr(buf.as_mut_ptr()) };
      dma_rx.set_size(buf.len());
      dma_rx.ccr().store_val({
        let mut rx_ccr = self.init_dma_rx_ccr(dma_rx);
        dma_rx.ccr().en().set(&mut rx_ccr);
        rx_ccr
      });
      self.periph.i2c_cr1.store_val({
        self.periph.i2c_cr1.pe().set(&mut i2c_cr1_val);
        self.periph.i2c_cr1.errie().set(&mut i2c_cr1_val);
        self.periph.i2c_cr1.nackie().set(&mut i2c_cr1_val);
        self.periph.i2c_cr1.rxdmaen().set(&mut i2c_cr1_val);
        i2c_cr1_val
      });
      let dma_rx_complete = dma_rx.transfer_complete();
      let i2c_break = self.transfer_break();
      let i2c_error = self.transfer_error();
      self.set_i2c_cr2(&mut i2c_cr2_val, slave_addr, autoend, buf.len(), false);
      self.periph.i2c_cr2.store_val(i2c_cr2_val);
      match awt!(Select3::new(dma_rx_complete, i2c_break, i2c_error)) {
        Output3::A(Ok(()), i2c_break, i2c_error) => {
          drop(i2c_break);
          drop(i2c_error);
          dma_rx.ccr().store_val(self.init_dma_rx_ccr(dma_rx));
          self.int_ev.trigger();
          self.int_er.trigger();
          Ok(())
        }
        Output3::A(Err(dma_rx_err), i2c_break, i2c_error) => {
          drop(i2c_break);
          drop(i2c_error);
          dma_rx.ccr().store_val(self.init_dma_rx_ccr(dma_rx));
          self.int_ev.trigger();
          self.int_er.trigger();
          Err(dma_rx_err.into())
        }
        Output3::B(dma_rx_fut, i2c_break, i2c_error) => {
          drop(dma_rx_fut);
          drop(i2c_error);
          dma_rx.ccr().store_val(self.init_dma_rx_ccr(dma_rx));
          dma_rx.int().trigger();
          self.int_er.trigger();
          Err(i2c_break.into())
        }
        Output3::C(dma_rx_fut, i2c_break, i2c_error) => {
          drop(dma_rx_fut);
          drop(i2c_break);
          dma_rx.ccr().store_val(self.init_dma_rx_ccr(dma_rx));
          dma_rx.int().trigger();
          self.int_ev.trigger();
          Err(i2c_error.into())
        }
      }
    })
  }

  fn write_impl<'a, Tx: DmaChMap>(
    &'a self,
    dma_tx: &'a DmaChEn<Tx, impl IntToken<Att>>,
    buf: &'a [u8],
    slave_addr: u8,
    mut i2c_cr1_val: T::I2CCr1Val,
    mut i2c_cr2_val: T::I2CCr2Val,
    autoend: bool,
  ) -> impl Future<Output = Result<(), I2CDmaError>> + 'a {
    if buf.len() > 255 {
      panic!("I2C write overflow");
    }
    asnc(move || {
      unsafe { dma_tx.set_maddr(buf.as_ptr()) };
      dma_tx.set_size(buf.len());
      dma_tx.ccr().store_val({
        let mut tx_ccr = self.init_dma_tx_ccr(dma_tx);
        dma_tx.ccr().en().set(&mut tx_ccr);
        tx_ccr
      });
      self.periph.i2c_cr1.store_val({
        self.periph.i2c_cr1.pe().set(&mut i2c_cr1_val);
        self.periph.i2c_cr1.errie().set(&mut i2c_cr1_val);
        self.periph.i2c_cr1.nackie().set(&mut i2c_cr1_val);
        self.periph.i2c_cr1.txdmaen().set(&mut i2c_cr1_val);
        i2c_cr1_val
      });
      let dma_tx_complete = dma_tx.transfer_complete();
      let i2c_break = self.transfer_break();
      let i2c_error = self.transfer_error();
      self.set_i2c_cr2(&mut i2c_cr2_val, slave_addr, autoend, buf.len(), true);
      self.periph.i2c_cr2.store_val(i2c_cr2_val);
      match awt!(Select3::new(dma_tx_complete, i2c_break, i2c_error)) {
        Output3::A(Ok(()), i2c_break, i2c_error) => {
          drop(i2c_break);
          drop(i2c_error);
          dma_tx.ccr().store_val(self.init_dma_tx_ccr(dma_tx));
          self.int_ev.trigger();
          self.int_er.trigger();
          Ok(())
        }
        Output3::A(Err(dma_tx_err), i2c_break, i2c_error) => {
          drop(i2c_break);
          drop(i2c_error);
          dma_tx.ccr().store_val(self.init_dma_tx_ccr(dma_tx));
          self.int_ev.trigger();
          self.int_er.trigger();
          Err(dma_tx_err.into())
        }
        Output3::B(dma_tx_fut, i2c_break, i2c_error) => {
          drop(dma_tx_fut);
          drop(i2c_error);
          dma_tx.ccr().store_val(self.init_dma_tx_ccr(dma_tx));
          dma_tx.int().trigger();
          self.int_er.trigger();
          Err(i2c_break.into())
        }
        Output3::C(dma_tx_fut, i2c_break, i2c_error) => {
          drop(dma_tx_fut);
          drop(i2c_break);
          dma_tx.ccr().store_val(self.init_dma_tx_ccr(dma_tx));
          dma_tx.int().trigger();
          self.int_ev.trigger();
          Err(i2c_error.into())
        }
      }
    })
  }

  fn set_i2c_cr2(
    &self,
    val: &mut T::I2CCr2Val,
    slave_addr: u8,
    autoend: bool,
    nbytes: usize,
    write: bool,
  ) {
    self.periph.i2c_cr2.add10().clear(val);
    let slave_addr = u32::from(slave_addr << 1);
    self.periph.i2c_cr2.sadd().write(val, slave_addr);
    if write {
      self.periph.i2c_cr2.rd_wrn().clear(val);
    } else {
      self.periph.i2c_cr2.rd_wrn().set(val);
    }
    self.periph.i2c_cr2.nbytes().write(val, nbytes as u32);
    if autoend {
      self.periph.i2c_cr2.autoend().set(val);
    } else {
      self.periph.i2c_cr2.autoend().clear(val);
    }
    self.periph.i2c_cr2.start().set(val);
  }

  fn init_dma_rx_ccr<Rx: DmaChMap>(
    &self,
    dma_rx: &DmaChEn<Rx, impl IntToken<Att>>,
  ) -> Rx::DmaCcrVal {
    let mut val = dma_rx.ccr().default_val();
    dma_rx.ccr().mem2mem().clear(&mut val);
    dma_rx.ccr().msize().write(&mut val, 0b00);
    dma_rx.ccr().psize().write(&mut val, 0b00);
    dma_rx.ccr().minc().set(&mut val);
    dma_rx.ccr().pinc().clear(&mut val);
    dma_rx.ccr().circ().clear(&mut val);
    dma_rx.ccr().dir().clear(&mut val);
    dma_rx.ccr().teie().set(&mut val);
    dma_rx.ccr().htie().clear(&mut val);
    dma_rx.ccr().tcie().set(&mut val);
    dma_rx.ccr().en().clear(&mut val);
    val
  }

  fn init_dma_tx_ccr<Tx: DmaChMap>(
    &self,
    dma_tx: &DmaChEn<Tx, impl IntToken<Att>>,
  ) -> Tx::DmaCcrVal {
    let mut val = dma_tx.ccr().default_val();
    dma_tx.ccr().mem2mem().clear(&mut val);
    dma_tx.ccr().msize().write(&mut val, 0b00);
    dma_tx.ccr().psize().write(&mut val, 0b00);
    dma_tx.ccr().minc().set(&mut val);
    dma_tx.ccr().pinc().clear(&mut val);
    dma_tx.ccr().circ().clear(&mut val);
    dma_tx.ccr().dir().set(&mut val);
    dma_tx.ccr().teie().set(&mut val);
    dma_tx.ccr().htie().clear(&mut val);
    dma_tx.ccr().tcie().set(&mut val);
    dma_tx.ccr().en().clear(&mut val);
    val
  }
}

#[allow(missing_docs)]
impl<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>> I2CEn<T, Ev, Er> {
  #[inline]
  pub fn cr1(&self) -> &T::SI2CCr1 {
    &self.periph.i2c_cr1
  }

  #[inline]
  pub fn cr2(&self) -> &T::SI2CCr2 {
    &self.periph.i2c_cr2
  }

  #[inline]
  pub fn timingr(&self) -> &T::SI2CTimingr {
    &self.periph.i2c_timingr
  }
}

impl<T, Ev, Er, Rx> DrvDmaRx<Rx> for I2C<T, Ev, Er>
where
  T: I2CMap,
  Ev: IntToken<Att>,
  Er: IntToken<Att>,
  Rx: DmaChMap,
{
  #[inline]
  fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken<Att>>) {
    self.0.dma_rx_paddr_init(dma_rx);
  }
}

impl<T, Ev, Er, Tx> DrvDmaTx<Tx> for I2C<T, Ev, Er>
where
  T: I2CMap,
  Ev: IntToken<Att>,
  Er: IntToken<Att>,
  Tx: DmaChMap,
{
  #[inline]
  fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken<Att>>) {
    self.0.dma_tx_paddr_init(dma_tx);
  }
}

impl<T, Ev, Er, Rx> DrvDmaRx<Rx> for I2CEn<T, Ev, Er>
where
  T: I2CMap,
  Ev: IntToken<Att>,
  Er: IntToken<Att>,
  Rx: DmaChMap,
{
  fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken<Att>>) {
    unsafe { dma_rx.set_paddr(self.periph.i2c_rxdr.to_ptr()) };
  }
}

impl<T, Ev, Er, Tx> DrvDmaTx<Tx> for I2CEn<T, Ev, Er>
where
  T: I2CMap,
  Ev: IntToken<Att>,
  Er: IntToken<Att>,
  Tx: DmaChMap,
{
  fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken<Att>>) {
    unsafe { dma_tx.set_paddr(self.periph.i2c_txdr.to_mut_ptr()) };
  }
}

impl<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>> DrvRcc
  for I2C<T, Ev, Er>
{
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

impl<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>> DrvRcc
  for I2CEn<T, Ev, Er>
{
  fn reset(&mut self) {
    self.periph.rcc_apb1rstr_i2crst.set_bit();
  }

  fn disable_stop_mode(&self) {
    self.periph.rcc_apb1smenr_i2csmen.clear_bit();
  }

  fn enable_stop_mode(&self) {
    self.periph.rcc_apb1smenr_i2csmen.set_bit();
  }
}

impl<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>> DrvClockSel
  for I2C<T, Ev, Er>
{
  #[inline]
  fn clock_sel(&self, value: u32) {
    self.0.clock_sel(value);
  }
}

impl<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>> DrvClockSel
  for I2CEn<T, Ev, Er>
{
  fn clock_sel(&self, value: u32) {
    self.periph.rcc_ccipr_i2csel.write_bits(value);
  }
}

impl<T: I2CMap, Ev: IntToken<Att>, Er: IntToken<Att>>
  GuardHandler<I2CEn<T, Ev, Er>> for I2CEnGuard<T>
{
  fn teardown(&mut self, i2c: &mut I2CEn<T, Ev, Er>) {
    i2c.periph.rcc_apb1enr_i2cen.clear_bit()
  }
}

impl From<DmaTransferError> for I2CDmaError {
  fn from(err: DmaTransferError) -> Self {
    I2CDmaError::Dma(err)
  }
}

impl From<I2CBreak> for I2CDmaError {
  fn from(err: I2CBreak) -> Self {
    I2CDmaError::I2CBreak(err)
  }
}

impl From<I2CError> for I2CDmaError {
  fn from(err: I2CError) -> Self {
    I2CDmaError::I2CError(err)
  }
}
