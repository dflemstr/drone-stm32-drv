//! Inter-Integrated Circuit.

use crate::{
    common::{DrvClockSel, DrvDmaRx, DrvDmaTx, DrvRcc},
    dma::{DmaChEn, DmaTransferError},
    select3::{Output3, Select3},
};
use core::fmt;
use drone_core::inventory::{self, Inventory0, Inventory1};
use drone_cortex_m::{fib, reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::{
    dma::ch::{traits::*, DmaChMap},
    i2c::{traits::*, I2CMap, I2CPeriph},
};
use futures::prelude::*;

/// I2C DMA error.
#[derive(Debug)]
pub enum I2CDmaError {
    /// DMA error.
    Dma(DmaTransferError),
    /// I2C transfer failure.
    I2CBreak(I2CBreak),
    /// I2C error.
    I2CError(I2CError),
}

/// I2C error.
#[derive(Debug)]
pub enum I2CError {
    /// Bus error.
    Berr,
    /// Overrun/Underrun.
    Ovr,
    /// Arbitration lost.
    Arlo,
    /// Timeout or t_low detection flag.
    Timeout,
    /// SMBus alert.
    Alert,
    /// PEC error in reception.
    Pecerr,
}

/// I2C transfer failure event.
#[derive(Debug)]
pub enum I2CBreak {
    /// NACK reception.
    Nack,
    /// Stop reception.
    Stop,
}

/// I2C driver.
pub struct I2C<T: I2CMap, Ev: IntToken, Er: IntToken>(Inventory0<I2CEn<T, Ev, Er>>);

/// I2C enabled driver.
pub struct I2CEn<T: I2CMap, Ev: IntToken, Er: IntToken> {
    periph: I2CDiverged<T>,
    int_ev: Ev,
    int_er: Er,
}

/// I2C diverged peripheral.
#[allow(missing_docs)]
pub struct I2CDiverged<T: I2CMap> {
    pub rcc_busenr_i2cen: T::SRccBusenrI2Cen,
    pub rcc_busrstr_i2crst: T::SRccBusrstrI2Crst,
    pub rcc_bussmenr_i2csmen: T::SRccBussmenrI2Csmen,
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

impl<T: I2CMap, Ev: IntToken, Er: IntToken> I2C<T, Ev, Er> {
    /// Creates a new [`I2C`].
    #[inline]
    pub fn new(periph: I2CPeriph<T>, int_ev: Ev, int_er: Er) -> Self {
        let periph = I2CDiverged {
            rcc_busenr_i2cen: periph.rcc_busenr_i2cen,
            rcc_busrstr_i2crst: periph.rcc_busrstr_i2crst,
            rcc_bussmenr_i2csmen: periph.rcc_bussmenr_i2csmen,
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
        Self(Inventory0::new(I2CEn {
            periph,
            int_ev,
            int_er,
        }))
    }

    /// Creates a new [`I2C`].
    ///
    /// # Safety
    ///
    /// Some of the `Crt` register tokens can be still in use.
    #[inline]
    pub unsafe fn from_diverged(periph: I2CDiverged<T>, int_ev: Ev, int_er: Er) -> Self {
        Self(Inventory0::new(I2CEn {
            periph,
            int_ev,
            int_er,
        }))
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> I2CDiverged<T> {
        Inventory0::free(self.0).periph
    }

    /// Enables I2C clock.
    pub fn enable(&mut self) -> inventory::Guard<'_, I2CEn<T, Ev, Er>> {
        self.setup();
        Inventory0::guard(&mut self.0)
    }

    /// Enables I2C clock.
    pub fn into_enabled(self) -> Inventory1<I2CEn<T, Ev, Er>> {
        self.setup();
        let (enabled, token) = self.0.share1();
        // To be recreated in `from_enabled()`.
        drop(token);
        enabled
    }

    /// Disables I2C clock.
    pub fn from_enabled(enabled: Inventory1<I2CEn<T, Ev, Er>>) -> Self {
        // Restoring the token dropped in `into_enabled()`.
        let token = unsafe { inventory::Token::new() };
        let mut enabled = enabled.merge1(token);
        Inventory0::teardown(&mut enabled);
        Self(enabled)
    }

    fn setup(&self) {
        let i2cen = &self.0.periph.rcc_busenr_i2cen;
        if i2cen.read_bit() {
            panic!("I2C wasn't turned off");
        }
        i2cen.set_bit();
    }
}

impl<T: I2CMap, Ev: IntToken, Er: IntToken> I2CEn<T, Ev, Er> {
    /// Reads bytes to `buf` from `slave_addr`. Leaves the session open.
    ///
    /// # Panics
    ///
    /// If length of `buf` is greater than 255.
    pub fn read<'a, Rx: DmaChMap>(
        &'a self,
        dma_rx: &'a DmaChEn<Rx, impl IntToken>,
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
        dma_rx: &'a DmaChEn<Rx, impl IntToken>,
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
        dma_tx: &'a DmaChEn<Tx, impl IntToken>,
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
        dma_tx: &'a DmaChEn<Tx, impl IntToken>,
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
        self.int_er.add_future(fib::new_fn(move || {
            if berr.read_bit_band() {
                berrcf.set_bit_band();
                fib::Complete(I2CError::Berr)
            } else if ovr.read_bit_band() {
                ovrcf.set_bit_band();
                fib::Complete(I2CError::Ovr)
            } else if arlo.read_bit_band() {
                arlocf.set_bit_band();
                fib::Complete(I2CError::Arlo)
            } else if timeout.read_bit_band() {
                timoutcf.set_bit_band();
                fib::Complete(I2CError::Timeout)
            } else if alert.read_bit_band() {
                alertcf.set_bit_band();
                fib::Complete(I2CError::Alert)
            } else if pecerr.read_bit_band() {
                peccf.set_bit_band();
                fib::Complete(I2CError::Pecerr)
            } else {
                fib::Yielded(())
            }
        }))
    }

    /// Returns a future, which resolves on I2C transfer failure event.
    pub fn transfer_break(&self) -> impl Future<Output = I2CBreak> {
        let nackf = *self.periph.i2c_isr.nackf();
        let stopf = *self.periph.i2c_isr.stopf();
        let nackcf = *self.periph.i2c_icr.nackcf();
        let stopcf = *self.periph.i2c_icr.stopcf();
        self.int_ev.add_future(fib::new_fn(move || {
            if nackf.read_bit_band() {
                nackcf.set_bit_band();
                fib::Complete(I2CBreak::Nack)
            } else if stopf.read_bit_band() {
                stopcf.set_bit_band();
                fib::Complete(I2CBreak::Stop)
            } else {
                fib::Yielded(())
            }
        }))
    }

    async fn read_impl<Rx: DmaChMap>(
        &self,
        dma_rx: &DmaChEn<Rx, impl IntToken>,
        buf: &mut [u8],
        slave_addr: u8,
        mut i2c_cr1_val: T::I2CCr1Val,
        mut i2c_cr2_val: T::I2CCr2Val,
        autoend: bool,
    ) -> Result<(), I2CDmaError> {
        if buf.len() > 255 {
            panic!("I2C read overflow");
        }
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
        match Select3::new(dma_rx_complete, i2c_break, i2c_error).await {
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
    }

    async fn write_impl<Tx: DmaChMap>(
        &self,
        dma_tx: &DmaChEn<Tx, impl IntToken>,
        buf: &[u8],
        slave_addr: u8,
        mut i2c_cr1_val: T::I2CCr1Val,
        mut i2c_cr2_val: T::I2CCr2Val,
        autoend: bool,
    ) -> Result<(), I2CDmaError> {
        if buf.len() > 255 {
            panic!("I2C write overflow");
        }
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
        match Select3::new(dma_tx_complete, i2c_break, i2c_error).await {
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

    fn init_dma_rx_ccr<Rx: DmaChMap>(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) -> Rx::DmaCcrVal {
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

    fn init_dma_tx_ccr<Tx: DmaChMap>(&self, dma_tx: &DmaChEn<Tx, impl IntToken>) -> Tx::DmaCcrVal {
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
impl<T: I2CMap, Ev: IntToken, Er: IntToken> I2CEn<T, Ev, Er> {
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

impl<T: I2CMap, Ev: IntToken, Er: IntToken> inventory::Item for I2CEn<T, Ev, Er> {
    fn teardown(&mut self, _token: &mut inventory::GuardToken<Self>) {
        self.periph.rcc_busenr_i2cen.clear_bit()
    }
}

impl<T, Ev, Er, Rx> DrvDmaRx<Rx> for I2C<T, Ev, Er>
where
    T: I2CMap,
    Ev: IntToken,
    Er: IntToken,
    Rx: DmaChMap,
{
    #[inline]
    fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) {
        self.0.dma_rx_paddr_init(dma_rx);
    }
}

impl<T, Ev, Er, Tx> DrvDmaTx<Tx> for I2C<T, Ev, Er>
where
    T: I2CMap,
    Ev: IntToken,
    Er: IntToken,
    Tx: DmaChMap,
{
    #[inline]
    fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken>) {
        self.0.dma_tx_paddr_init(dma_tx);
    }
}

impl<T, Ev, Er, Rx> DrvDmaRx<Rx> for I2CEn<T, Ev, Er>
where
    T: I2CMap,
    Ev: IntToken,
    Er: IntToken,
    Rx: DmaChMap,
{
    fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) {
        unsafe { dma_rx.set_paddr(self.periph.i2c_rxdr.to_ptr()) };
    }
}

impl<T, Ev, Er, Tx> DrvDmaTx<Tx> for I2CEn<T, Ev, Er>
where
    T: I2CMap,
    Ev: IntToken,
    Er: IntToken,
    Tx: DmaChMap,
{
    fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken>) {
        unsafe { dma_tx.set_paddr(self.periph.i2c_txdr.to_mut_ptr()) };
    }
}

impl<T: I2CMap, Ev: IntToken, Er: IntToken> DrvRcc for I2C<T, Ev, Er> {
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

impl<T: I2CMap, Ev: IntToken, Er: IntToken> DrvRcc for I2CEn<T, Ev, Er> {
    fn reset(&mut self) {
        self.periph.rcc_busrstr_i2crst.set_bit();
    }

    fn disable_stop_mode(&self) {
        self.periph.rcc_bussmenr_i2csmen.clear_bit();
    }

    fn enable_stop_mode(&self) {
        self.periph.rcc_bussmenr_i2csmen.set_bit();
    }
}

impl<T: I2CMap, Ev: IntToken, Er: IntToken> DrvClockSel for I2C<T, Ev, Er> {
    #[inline]
    fn clock_sel(&self, value: u32) {
        self.0.clock_sel(value);
    }
}

impl<T: I2CMap, Ev: IntToken, Er: IntToken> DrvClockSel for I2CEn<T, Ev, Er> {
    fn clock_sel(&self, value: u32) {
        self.periph.rcc_ccipr_i2csel.write_bits(value);
    }
}

impl From<DmaTransferError> for I2CDmaError {
    fn from(err: DmaTransferError) -> Self {
        Self::Dma(err)
    }
}

impl From<I2CBreak> for I2CDmaError {
    fn from(err: I2CBreak) -> Self {
        Self::I2CBreak(err)
    }
}

impl From<I2CError> for I2CDmaError {
    fn from(err: I2CError) -> Self {
        Self::I2CError(err)
    }
}

impl fmt::Display for I2CDmaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Dma(err) => write!(f, "DMA error: {}", err),
            Self::I2CBreak(err) => write!(f, "I2C failure: {}", err),
            Self::I2CError(err) => write!(f, "I2C error: {}", err),
        }
    }
}

impl fmt::Display for I2CError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Berr => write!(f, "I2C bus error."),
            Self::Ovr => write!(f, "I2C overrun."),
            Self::Arlo => write!(f, "I2C arbitration lost."),
            Self::Timeout => write!(f, "I2C timeout."),
            Self::Alert => write!(f, "I2C SMBus alert."),
            Self::Pecerr => write!(f, "I2C PEC error."),
        }
    }
}

impl fmt::Display for I2CBreak {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Nack => write!(f, "I2C NACK received."),
            Self::Stop => write!(f, "I2C STOP received."),
        }
    }
}
