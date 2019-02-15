//! Serial Peripheral Interface.

use crate::{
  common::{DrvDmaRx, DrvDmaTx, DrvRcc},
  dma::DmaChEn,
};
use core::{
  marker::PhantomData,
  ptr::{read_volatile, write_volatile},
};
use drone_core::shared_guard::{Guard, GuardHandler};
use drone_cortex_m::{reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::{
  dma::ch::DmaChMap,
  spi::{traits::*, SpiMap, SpiPeriph},
};
use failure::Fail;

/// Motorola SPI mode error.
#[derive(Debug, Fail)]
pub enum SpiError {
  /// CRC value received does not match the `SPIx_RXCRCR` value.
  #[fail(display = "SPI CRC mismatch.")]
  Crcerr,
  /// Overrun occurred.
  #[fail(display = "SPI queue overrun.")]
  Ovr,
  /// Mode fault occurred.
  #[fail(display = "SPI mode fault.")]
  Modf,
}

/// SPI driver.
pub struct Spi<T: SpiMap, I: IntToken<Att>>(SpiEn<T, I>);

/// SPI enabled driver.
pub struct SpiEn<T: SpiMap, I: IntToken<Att>> {
  periph: SpiDiverged<T>,
  int: I,
}

/// SPI enabled guard handler.
pub struct SpiEnGuard<T: SpiMap>(PhantomData<T>);

/// SPI diverged peripheral.
#[allow(missing_docs)]
pub struct SpiDiverged<T: SpiMap> {
  pub rcc_apbenr_spien: T::SRccApbenrSpien,
  pub rcc_apbrstr_spirst: T::SRccApbrstrSpirst,
  pub rcc_apbsmenr_spismen: T::SRccApbsmenrSpismen,
  pub spi_cr1: T::SSpiCr1,
  pub spi_cr2: T::SSpiCr2,
  pub spi_crcpr: T::SSpiCrcpr,
  pub spi_dr: T::CSpiDr,
  pub spi_rxcrcr: T::SSpiRxcrcr,
  pub spi_sr: T::SSpiSr,
  pub spi_txcrcr: T::SSpiTxcrcr,
}

impl<T: SpiMap, I: IntToken<Att>> Spi<T, I> {
  /// Creates a new [`Spi`].
  #[inline]
  pub fn new(periph: SpiPeriph<T>, int: I) -> Self {
    let periph = SpiDiverged {
      rcc_apbenr_spien: periph.rcc_apbenr_spien,
      rcc_apbrstr_spirst: periph.rcc_apbrstr_spirst,
      rcc_apbsmenr_spismen: periph.rcc_apbsmenr_spismen,
      spi_cr1: periph.spi_cr1,
      spi_cr2: periph.spi_cr2,
      spi_crcpr: periph.spi_crcpr,
      spi_dr: periph.spi_dr.into_copy(),
      spi_rxcrcr: periph.spi_rxcrcr,
      spi_sr: periph.spi_sr,
      spi_txcrcr: periph.spi_txcrcr,
    };
    Self(SpiEn { periph, int })
  }

  /// Creates a new [`Spi`].
  ///
  /// # Safety
  ///
  /// Some of the `Crt` register tokens can be still in use.
  #[inline]
  pub unsafe fn from_diverged(periph: SpiDiverged<T>, int: I) -> Self {
    Self(SpiEn { periph, int })
  }

  /// Releases the peripheral.
  #[inline]
  pub fn free(self) -> SpiDiverged<T> {
    self.0.periph
  }

  /// Enables SPI clock.
  pub fn enable(&mut self) -> Guard<'_, SpiEn<T, I>, SpiEnGuard<T>> {
    let spien = &self.0.periph.rcc_apbenr_spien;
    if spien.read_bit() {
      panic!("SPI wasn't turned off");
    }
    spien.set_bit();
    Guard::new(&mut self.0, SpiEnGuard(PhantomData))
  }

  /// Enables SPI clock.
  pub fn into_enabled(self) -> SpiEn<T, I> {
    self.0.periph.rcc_apbenr_spien.set_bit();
    self.0
  }
}

impl<T: SpiMap, I: IntToken<Att>> SpiEn<T, I> {
  /// Disables SPI clock.
  pub fn into_disabled(self) -> Spi<T, I> {
    self.periph.rcc_apbenr_spien.clear_bit();
    Spi(self)
  }

  /// Sets the size of a data frame to 8 bits.
  #[allow(unused_variables)]
  #[inline]
  pub fn set_frame_8(&self, cr2: &mut T::SpiCr2Val) {
    #[cfg(any(
      feature = "stm32l4x1",
      feature = "stm32l4x2",
      feature = "stm32l4x3",
      feature = "stm32l4x5",
      feature = "stm32l4x6",
      feature = "stm32l4r5",
      feature = "stm32l4r7",
      feature = "stm32l4r9",
      feature = "stm32l4s5",
      feature = "stm32l4s7",
      feature = "stm32l4s9"
    ))]
    {
      self.periph.spi_cr2.frxth().set(cr2);
      self.periph.spi_cr2.ds().write(cr2, 0b0111);
    }
  }

  /// Writes a byte to the data register.
  #[inline]
  pub fn send_byte(&self, value: u8) {
    Self::dr_send_byte(&self.periph.spi_dr, value);
  }

  /// Returns a closure, which writes a byte to the data register.
  #[inline]
  pub fn send_byte_fn(&self) -> impl Fn(u8) {
    let dr = self.periph.spi_dr;
    move |value| Self::dr_send_byte(&dr, value)
  }

  /// Writes a half word to the data register.
  #[inline]
  pub fn send_hword(&self, value: u16) {
    Self::dr_send_hword(&self.periph.spi_dr, value);
  }

  /// Returns a closure, which writes a half word to the data register.
  #[inline]
  pub fn send_hword_fn(&self) -> impl Fn(u16) {
    let dr = self.periph.spi_dr;
    move |value| Self::dr_send_hword(&dr, value)
  }

  /// Reads a byte from the data register.
  #[inline]
  pub fn recv_byte(&self) -> u8 {
    unsafe { read_volatile(self.periph.spi_dr.to_ptr() as *const _) }
  }

  /// Reads a half word from the data register.
  #[inline]
  pub fn recv_hword(&self) -> u16 {
    unsafe { read_volatile(self.periph.spi_dr.to_ptr() as *const _) }
  }

  /// Waits while SPI is busy in communication or Tx buffer is not empty.
  #[inline]
  pub fn busy_wait(&self) {
    while self.periph.spi_sr.bsy().read_bit_band() {}
  }

  /// Checks for SPI mode errors.
  #[inline]
  pub fn spi_errck(&self, sr: &T::SpiSrVal) -> Result<(), SpiError> {
    if self.periph.spi_sr.ovr().read(sr) {
      Err(SpiError::Ovr)
    } else if self.periph.spi_sr.modf().read(sr) {
      Err(SpiError::Modf)
    } else if self.periph.spi_sr.crcerr().read(sr) {
      Err(SpiError::Crcerr)
    } else {
      Ok(())
    }
  }

  #[inline]
  fn dr_send_byte(dr: &T::CSpiDr, value: u8) {
    unsafe { write_volatile(dr.to_mut_ptr() as *mut _, value) };
  }

  #[inline]
  fn dr_send_hword(dr: &T::CSpiDr, value: u16) {
    unsafe { write_volatile(dr.to_mut_ptr() as *mut _, value) };
  }
}

#[allow(missing_docs)]
impl<T: SpiMap, I: IntToken<Att>> SpiEn<T, I> {
  #[inline]
  pub fn int(&self) -> &I {
    &self.int
  }

  #[inline]
  pub fn cr1(&self) -> &T::SSpiCr1 {
    &self.periph.spi_cr1
  }

  #[inline]
  pub fn cr2(&self) -> &T::SSpiCr2 {
    &self.periph.spi_cr2
  }

  #[inline]
  pub fn sr(&self) -> &T::SSpiSr {
    &self.periph.spi_sr
  }
}

impl<T: SpiMap, I: IntToken<Att>, Rx: DmaChMap> DrvDmaRx<Rx> for Spi<T, I> {
  #[inline]
  fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken<Att>>) {
    self.0.dma_rx_paddr_init(dma_rx);
  }
}

impl<T: SpiMap, I: IntToken<Att>, Tx: DmaChMap> DrvDmaTx<Tx> for Spi<T, I> {
  #[inline]
  fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken<Att>>) {
    self.0.dma_tx_paddr_init(dma_tx);
  }
}

impl<T: SpiMap, I: IntToken<Att>, Rx: DmaChMap> DrvDmaRx<Rx> for SpiEn<T, I> {
  fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken<Att>>) {
    unsafe { dma_rx.set_paddr(self.periph.spi_dr.to_ptr()) };
  }
}

impl<T: SpiMap, I: IntToken<Att>, Tx: DmaChMap> DrvDmaTx<Tx> for SpiEn<T, I> {
  fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken<Att>>) {
    unsafe { dma_tx.set_paddr(self.periph.spi_dr.to_mut_ptr()) };
  }
}

impl<T: SpiMap, I: IntToken<Att>> DrvRcc for Spi<T, I> {
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

impl<T: SpiMap, I: IntToken<Att>> DrvRcc for SpiEn<T, I> {
  fn reset(&mut self) {
    self.periph.rcc_apbrstr_spirst.set_bit();
  }

  fn disable_stop_mode(&self) {
    self.periph.rcc_apbsmenr_spismen.clear_bit();
  }

  fn enable_stop_mode(&self) {
    self.periph.rcc_apbsmenr_spismen.set_bit();
  }
}

impl<T: SpiMap, I: IntToken<Att>> GuardHandler<SpiEn<T, I>> for SpiEnGuard<T> {
  fn teardown(&mut self, spi: &mut SpiEn<T, I>) {
    spi.periph.rcc_apbenr_spien.clear_bit()
  }
}
