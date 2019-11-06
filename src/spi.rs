//! Serial Peripheral Interface.

use crate::{
    common::{DrvDmaRx, DrvDmaTx, DrvRcc},
    dma::DmaChEn,
};
use core::{
    fmt,
    ptr::{read_volatile, write_volatile},
};
use drone_core::inventory::{self, Inventory0, Inventory1};
use drone_cortex_m::{reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::{
    dma::ch::DmaChMap,
    spi::{traits::*, SpiMap, SpiPeriph},
};

/// Motorola SPI mode error.
#[derive(Debug)]
pub enum SpiError {
    /// CRC value received does not match the `SPIx_RXCRCR` value.
    Crcerr,
    /// Overrun occurred.
    Ovr,
    /// Mode fault occurred.
    Modf,
}

/// SPI driver.
pub struct Spi<T: SpiMap, I: IntToken>(Inventory0<SpiEn<T, I>>);

/// SPI enabled driver.
pub struct SpiEn<T: SpiMap, I: IntToken> {
    periph: SpiDiverged<T>,
    int: I,
}

/// SPI diverged peripheral.
#[allow(missing_docs)]
pub struct SpiDiverged<T: SpiMap> {
    pub rcc_busenr_spien: T::SRccBusenrSpien,
    pub rcc_busrstr_spirst: T::SRccBusrstrSpirst,
    pub rcc_bussmenr_spismen: T::SRccBussmenrSpismen,
    pub spi_cr1: T::SSpiCr1,
    pub spi_cr2: T::SSpiCr2,
    pub spi_crcpr: T::SSpiCrcpr,
    pub spi_dr: T::CSpiDr,
    pub spi_rxcrcr: T::SSpiRxcrcr,
    pub spi_sr: T::SSpiSr,
    pub spi_txcrcr: T::SSpiTxcrcr,
}

impl<T: SpiMap, I: IntToken> Spi<T, I> {
    /// Creates a new [`Spi`].
    #[inline]
    pub fn new(periph: SpiPeriph<T>, int: I) -> Self {
        let periph = SpiDiverged {
            rcc_busenr_spien: periph.rcc_busenr_spien,
            rcc_busrstr_spirst: periph.rcc_busrstr_spirst,
            rcc_bussmenr_spismen: periph.rcc_bussmenr_spismen,
            spi_cr1: periph.spi_cr1,
            spi_cr2: periph.spi_cr2,
            spi_crcpr: periph.spi_crcpr,
            spi_dr: periph.spi_dr.into_copy(),
            spi_rxcrcr: periph.spi_rxcrcr,
            spi_sr: periph.spi_sr,
            spi_txcrcr: periph.spi_txcrcr,
        };
        Self(Inventory0::new(SpiEn { periph, int }))
    }

    /// Creates a new [`Spi`].
    ///
    /// # Safety
    ///
    /// Some of the `Crt` register tokens can be still in use.
    #[inline]
    pub unsafe fn from_diverged(periph: SpiDiverged<T>, int: I) -> Self {
        Self(Inventory0::new(SpiEn { periph, int }))
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> SpiDiverged<T> {
        Inventory0::free(self.0).periph
    }

    /// Enables SPI clock.
    pub fn enable(&mut self) -> inventory::Guard<'_, SpiEn<T, I>> {
        self.setup();
        Inventory0::guard(&mut self.0)
    }

    /// Enables SPI clock.
    pub fn into_enabled(self) -> Inventory1<SpiEn<T, I>> {
        self.setup();
        let (enabled, token) = self.0.share1();
        // To be recreated in `from_enabled()`.
        drop(token);
        enabled
    }

    /// Disables SPI clock.
    pub fn from_enabled(enabled: Inventory1<SpiEn<T, I>>) -> Self {
        // Restoring the token dropped in `into_enabled()`.
        let token = unsafe { inventory::Token::new() };
        let mut enabled = enabled.merge1(token);
        Inventory0::teardown(&mut enabled);
        Self(enabled)
    }

    fn setup(&self) {
        let spien = &self.0.periph.rcc_busenr_spien;
        if spien.read_bit() {
            panic!("SPI wasn't turned off");
        }
        spien.set_bit();
    }
}

impl<T: SpiMap, I: IntToken> SpiEn<T, I> {
    /// Sets the size of a data frame to 8 bits.
    #[allow(unused_variables)]
    #[inline]
    pub fn set_frame_8(&self, cr2: &mut T::SpiCr2Val) {
        #[cfg(any(
            stm32_mcu = "stm32l4x1",
            stm32_mcu = "stm32l4x2",
            stm32_mcu = "stm32l4x3",
            stm32_mcu = "stm32l4x5",
            stm32_mcu = "stm32l4x6",
            stm32_mcu = "stm32l4r5",
            stm32_mcu = "stm32l4r7",
            stm32_mcu = "stm32l4r9",
            stm32_mcu = "stm32l4s5",
            stm32_mcu = "stm32l4s7",
            stm32_mcu = "stm32l4s9"
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
        unsafe { read_volatile(self.periph.spi_dr.as_ptr() as *const _) }
    }

    /// Reads a half word from the data register.
    #[inline]
    pub fn recv_hword(&self) -> u16 {
        unsafe { read_volatile(self.periph.spi_dr.as_ptr() as *const _) }
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
        unsafe { write_volatile(dr.as_mut_ptr() as *mut _, value) };
    }

    #[inline]
    fn dr_send_hword(dr: &T::CSpiDr, value: u16) {
        unsafe { write_volatile(dr.as_mut_ptr() as *mut _, value) };
    }
}

#[allow(missing_docs)]
impl<T: SpiMap, I: IntToken> SpiEn<T, I> {
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

impl<T: SpiMap, I: IntToken> inventory::Item for SpiEn<T, I> {
    fn teardown(&mut self, _token: &mut inventory::GuardToken<Self>) {
        self.periph.rcc_busenr_spien.clear_bit()
    }
}

impl<T: SpiMap, I: IntToken, Rx: DmaChMap> DrvDmaRx<Rx> for Spi<T, I> {
    #[inline]
    fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) {
        self.0.dma_rx_paddr_init(dma_rx);
    }
}

impl<T: SpiMap, I: IntToken, Tx: DmaChMap> DrvDmaTx<Tx> for Spi<T, I> {
    #[inline]
    fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken>) {
        self.0.dma_tx_paddr_init(dma_tx);
    }
}

impl<T: SpiMap, I: IntToken, Rx: DmaChMap> DrvDmaRx<Rx> for SpiEn<T, I> {
    fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) {
        unsafe { dma_rx.set_paddr(self.periph.spi_dr.as_ptr()) };
    }
}

impl<T: SpiMap, I: IntToken, Tx: DmaChMap> DrvDmaTx<Tx> for SpiEn<T, I> {
    fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken>) {
        unsafe { dma_tx.set_paddr(self.periph.spi_dr.as_mut_ptr()) };
    }
}

impl<T: SpiMap, I: IntToken> DrvRcc for Spi<T, I> {
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

impl<T: SpiMap, I: IntToken> DrvRcc for SpiEn<T, I> {
    fn reset(&mut self) {
        self.periph.rcc_busrstr_spirst.set_bit();
    }

    fn disable_stop_mode(&self) {
        self.periph.rcc_bussmenr_spismen.clear_bit();
    }

    fn enable_stop_mode(&self) {
        self.periph.rcc_bussmenr_spismen.set_bit();
    }
}

impl fmt::Display for SpiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Crcerr => write!(f, "SPI CRC mismatch."),
            Self::Ovr => write!(f, "SPI queue overrun."),
            Self::Modf => write!(f, "SPI mode fault."),
        }
    }
}
