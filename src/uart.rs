//! Universal Asynchronous Receiver/Transmitter.

use crate::{
    common::{DrvClockSel, DrvDmaRx, DrvDmaTx, DrvRcc},
    dma::DmaChEn,
};
use core::{fmt, ptr::read_volatile};
use drone_core::inventory::{self, Inventory0, Inventory1};
use drone_cortex_m::{
    fib::{self, Fiber},
    reg::prelude::*,
    thr::prelude::*,
};
use drone_stm32_map::periph::{
    dma::ch::DmaChMap,
    uart::{traits::*, UartMap, UartPeriph},
};
use futures::prelude::*;

/// UART receive stream overflow.
#[derive(Debug)]
pub struct UartRxOverflow;

/// UART driver.
pub struct Uart<T: UartMap, I: IntToken>(Inventory0<UartEn<T, I>>);

/// UART enabled driver.
pub struct UartEn<T: UartMap, I: IntToken> {
    periph: UartDiverged<T>,
    int: I,
}

/// UART diverged peripheral.
#[allow(missing_docs)]
pub struct UartDiverged<T: UartMap> {
    pub rcc_busenr_uarten: T::SRccBusenrUarten,
    pub rcc_busrstr_uartrst: T::SRccBusrstrUartrst,
    pub rcc_bussmenr_uartsmen: T::SRccBussmenrUartsmen,
    pub rcc_ccipr_uartsel: T::SRccCciprUartsel,
    pub uart_cr1: T::CUartCr1,
    pub uart_cr2: T::SUartCr2,
    pub uart_cr3: T::SUartCr3,
    pub uart_brr: T::SUartBrr,
    pub uart_gtpr: T::SUartGtprOpt,
    pub uart_rtor: T::SUartRtorOpt,
    pub uart_rqr: T::SUartRqr,
    pub uart_isr: T::CUartIsr,
    pub uart_icr: T::SUartIcr,
    pub uart_rdr: T::CUartRdr,
    pub uart_tdr: T::SUartTdr,
}

impl<T: UartMap, I: IntToken> Uart<T, I> {
    /// Creates a new [`Uart`].
    #[inline]
    pub fn new(periph: UartPeriph<T>, int: I) -> Self {
        let periph = UartDiverged {
            rcc_busenr_uarten: periph.rcc_busenr_uarten,
            rcc_busrstr_uartrst: periph.rcc_busrstr_uartrst,
            rcc_bussmenr_uartsmen: periph.rcc_bussmenr_uartsmen,
            rcc_ccipr_uartsel: periph.rcc_ccipr_uartsel,
            uart_cr1: periph.uart_cr1.into_copy(),
            uart_cr2: periph.uart_cr2,
            uart_cr3: periph.uart_cr3,
            uart_brr: periph.uart_brr,
            uart_gtpr: periph.uart_gtpr,
            uart_rtor: periph.uart_rtor,
            uart_rqr: periph.uart_rqr,
            uart_isr: periph.uart_isr.into_copy(),
            uart_icr: periph.uart_icr,
            uart_rdr: periph.uart_rdr.into_copy(),
            uart_tdr: periph.uart_tdr,
        };
        Self(Inventory0::new(UartEn { periph, int }))
    }

    /// Creates a new [`Uart`].
    ///
    /// # Safety
    ///
    /// Some of the `Crt` register tokens can be still in use.
    #[inline]
    pub unsafe fn from_diverged(periph: UartDiverged<T>, int: I) -> Self {
        Self(Inventory0::new(UartEn { periph, int }))
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> UartDiverged<T> {
        Inventory0::free(self.0).periph
    }

    /// Enables UART clock.
    pub fn enable(&mut self) -> inventory::Guard<'_, UartEn<T, I>> {
        self.setup();
        Inventory0::guard(&mut self.0)
    }

    /// Enables UART clock.
    pub fn into_enabled(self) -> Inventory1<UartEn<T, I>> {
        self.setup();
        let (enabled, token) = self.0.share1();
        // To be recreated in `from_enabled()`.
        drop(token);
        enabled
    }

    /// Disables UART clock.
    pub fn from_enabled(enabled: Inventory1<UartEn<T, I>>) -> Self {
        // Restoring the token dropped in `into_enabled()`.
        let token = unsafe { inventory::Token::new() };
        let mut enabled = enabled.merge1(token);
        Inventory0::teardown(&mut enabled);
        Self(enabled)
    }

    fn setup(&self) {
        let uarten = &self.0.periph.rcc_busenr_uarten;
        if uarten.read_bit() {
            panic!("UART wasn't turned off");
        }
        uarten.set_bit();
    }
}

impl<T: UartMap, I: IntToken> UartEn<T, I> {
    /// Returns a future, which resolves on transmission complete event.
    pub fn transmission_complete(&self) -> impl Future<Output = ()> {
        let tc = *self.periph.uart_isr.tc();
        let tcie = *self.periph.uart_cr1.tcie();
        self.int.add_future(fib::new_fn(move || {
            if tc.read_bit_band() {
                tcie.clear_bit();
                fib::Complete(())
            } else {
                fib::Yielded(())
            }
        }))
    }

    /// Returns a stream of bytes from the receiver.
    pub fn rx_stream(&self, capacity: usize) -> impl Stream<Item = Result<u8, UartRxOverflow>> {
        let overflow = |_| Err(UartRxOverflow);
        let fib = self.rx_stream_fib();
        self.int.add_stream_ring(capacity, overflow, fib)
    }

    /// Returns a stream of bytes from the receiver.
    pub fn rx_stream_skip(&self, capacity: usize) -> impl Stream<Item = u8> {
        let fib = self.rx_stream_fib();
        self.int.add_stream_ring_skip(capacity, fib)
    }

    /// Returns a stream of bytes from the receiver.
    pub fn rx_stream_overwrite(&self, capacity: usize) -> impl Stream<Item = u8> {
        let fib = self.rx_stream_fib();
        self.int.add_stream_ring_overwrite(capacity, fib)
    }

    fn rx_stream_fib<R>(&self) -> impl Fiber<Input = (), Yield = Option<u8>, Return = R> {
        let rxne = *self.periph.uart_isr.rxne();
        let rdr = self.periph.uart_rdr;
        fib::new_fn(move || {
            if rxne.read_bit_band() {
                let byte = unsafe { read_volatile(rdr.to_ptr() as *const _) };
                fib::Yielded(Some(byte))
            } else {
                fib::Yielded(None)
            }
        })
    }
}

impl<T: UartMap, I: IntToken> inventory::Item for UartEn<T, I> {
    fn teardown(&mut self, _token: &mut inventory::GuardToken<Self>) {
        self.periph.rcc_busenr_uarten.clear_bit()
    }
}

impl<T: UartMap, I: IntToken, Rx: DmaChMap> DrvDmaRx<Rx> for Uart<T, I> {
    #[inline]
    fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) {
        self.0.dma_rx_paddr_init(dma_rx);
    }
}

impl<T: UartMap, I: IntToken, Tx: DmaChMap> DrvDmaTx<Tx> for Uart<T, I> {
    #[inline]
    fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken>) {
        self.0.dma_tx_paddr_init(dma_tx);
    }
}

impl<T: UartMap, I: IntToken, Rx: DmaChMap> DrvDmaRx<Rx> for UartEn<T, I> {
    fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken>) {
        unsafe { dma_rx.set_paddr(self.periph.uart_rdr.to_ptr()) };
    }
}

impl<T: UartMap, I: IntToken, Tx: DmaChMap> DrvDmaTx<Tx> for UartEn<T, I> {
    fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken>) {
        unsafe { dma_tx.set_paddr(self.periph.uart_tdr.to_mut_ptr()) };
    }
}

#[allow(missing_docs)]
impl<T: UartMap, I: IntToken> UartEn<T, I> {
    #[inline]
    pub fn brr(&self) -> &T::SUartBrr {
        &self.periph.uart_brr
    }

    #[inline]
    pub fn cr1(&self) -> &T::CUartCr1 {
        &self.periph.uart_cr1
    }

    #[inline]
    pub fn cr3(&self) -> &T::SUartCr3 {
        &self.periph.uart_cr3
    }

    #[inline]
    pub fn icr(&self) -> &T::SUartIcr {
        &self.periph.uart_icr
    }
}

impl<T: UartMap, I: IntToken> DrvRcc for Uart<T, I> {
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

impl<T: UartMap, I: IntToken> DrvRcc for UartEn<T, I> {
    fn reset(&mut self) {
        self.periph.rcc_busrstr_uartrst.set_bit();
    }

    fn disable_stop_mode(&self) {
        self.periph.rcc_bussmenr_uartsmen.clear_bit();
    }

    fn enable_stop_mode(&self) {
        self.periph.rcc_bussmenr_uartsmen.set_bit();
    }
}

impl<T: UartMap, I: IntToken> DrvClockSel for Uart<T, I> {
    #[inline]
    fn clock_sel(&self, value: u32) {
        self.0.clock_sel(value);
    }
}

impl<T: UartMap, I: IntToken> DrvClockSel for UartEn<T, I> {
    fn clock_sel(&self, value: u32) {
        self.periph.rcc_ccipr_uartsel.write_bits(value);
    }
}

impl fmt::Display for UartRxOverflow {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "UART RX stream overflow.")
    }
}
