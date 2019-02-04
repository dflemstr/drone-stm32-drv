//! Universal Asynchronous Receiver/Transmitter.

use crate::{
  common::{DrvClockSel, DrvDmaRx, DrvDmaTx, DrvRcc},
  dma::DmaChEn,
};
use core::{marker::PhantomData, ptr::read_volatile};
use drone_core::shared_guard::{Guard, GuardHandler};
use drone_cortex_m::{
  fib::{self, Fiber},
  reg::prelude::*,
  thr::prelude::*,
};
use drone_stm32_map::periph::{
  dma::ch::DmaChMap,
  uart::{traits::*, UartMap, UartPeriph},
};
use failure::Fail;
use futures::prelude::*;

/// UART receive stream overflow.
#[derive(Debug, Fail)]
#[fail(display = "UART RX stream overflow.")]
pub struct UartRxOverflow;

/// UART driver.
pub struct Uart<T: UartMap, I: IntToken<Rtt>>(UartEn<T, I>);

/// UART enabled driver.
pub struct UartEn<T: UartMap, I: IntToken<Rtt>> {
  periph: UartDiverged<T>,
  int: I,
}

/// UART enabled guard handler.
pub struct UartEnGuard<T: UartMap>(PhantomData<T>);

/// UART diverged peripheral.
#[allow(missing_docs)]
pub struct UartDiverged<T: UartMap> {
  pub rcc_apbenr_uarten: T::SRccApbenrUarten,
  pub rcc_apbrstr_uartrst: T::SRccApbrstrUartrst,
  pub rcc_apbsmenr_uartsmen: T::SRccApbsmenrUartsmen,
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

impl<T: UartMap, I: IntToken<Rtt>> Uart<T, I> {
  /// Creates a new [`Uart`].
  #[inline(always)]
  pub fn new(periph: UartPeriph<T>, int: I) -> Self {
    let periph = UartDiverged {
      rcc_apbenr_uarten: periph.rcc_apbenr_uarten,
      rcc_apbrstr_uartrst: periph.rcc_apbrstr_uartrst,
      rcc_apbsmenr_uartsmen: periph.rcc_apbsmenr_uartsmen,
      rcc_ccipr_uartsel: periph.rcc_ccipr_uartsel,
      uart_cr1: periph.uart_cr1.to_copy(),
      uart_cr2: periph.uart_cr2,
      uart_cr3: periph.uart_cr3,
      uart_brr: periph.uart_brr,
      uart_gtpr: periph.uart_gtpr,
      uart_rtor: periph.uart_rtor,
      uart_rqr: periph.uart_rqr,
      uart_isr: periph.uart_isr.to_copy(),
      uart_icr: periph.uart_icr,
      uart_rdr: periph.uart_rdr.to_copy(),
      uart_tdr: periph.uart_tdr,
    };
    Self(UartEn { periph, int })
  }

  /// Creates a new [`Uart`].
  ///
  /// # Safety
  ///
  /// Some of the `Crt` register tokens can be still in use.
  #[inline(always)]
  pub unsafe fn from_diverged(periph: UartDiverged<T>, int: I) -> Self {
    Self(UartEn { periph, int })
  }

  /// Releases the peripheral.
  #[inline(always)]
  pub fn free(self) -> UartDiverged<T> {
    self.0.periph
  }

  /// Enables UART clock.
  pub fn enable(&mut self) -> Guard<'_, UartEn<T, I>, UartEnGuard<T>> {
    let uarten = &self.0.periph.rcc_apbenr_uarten;
    if uarten.read_bit() {
      panic!("UART wasn't turned off");
    }
    uarten.set_bit();
    Guard::new(&mut self.0, UartEnGuard(PhantomData))
  }
}

impl<T: UartMap, I: IntToken<Rtt>> UartEn<T, I> {
  /// Returns a future, which resolves on transmission complete event.
  pub fn transmission_complete(&self) -> impl Future<Output = ()> {
    let tc = *self.periph.uart_isr.tc();
    let tcie = *self.periph.uart_cr1.tcie();
    self.int.add_future(fib::new(move || loop {
      if tc.read_bit_band() {
        tcie.clear_bit();
        break;
      }
      yield;
    }))
  }

  /// Returns a stream of bytes from the receiver.
  pub fn rx_stream(
    &self,
    capacity: usize,
  ) -> impl Stream<Item = Result<u8, UartRxOverflow>> {
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

  fn rx_stream_fib<R>(
    &self,
  ) -> impl Fiber<Input = (), Yield = Option<u8>, Return = R> {
    let rxne = *self.periph.uart_isr.rxne();
    let rdr = self.periph.uart_rdr;
    fib::new(move || loop {
      if rxne.read_bit_band() {
        let byte = unsafe { read_volatile(rdr.to_ptr() as *const _) };
        yield Some(byte);
      }
      yield None;
    })
  }
}

impl<T: UartMap, I: IntToken<Rtt>, Rx: DmaChMap> DrvDmaRx<Rx> for Uart<T, I> {
  #[inline]
  fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken<Rtt>>) {
    self.0.dma_rx_paddr_init(dma_rx);
  }
}

impl<T: UartMap, I: IntToken<Rtt>, Tx: DmaChMap> DrvDmaTx<Tx> for Uart<T, I> {
  #[inline]
  fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken<Rtt>>) {
    self.0.dma_tx_paddr_init(dma_tx);
  }
}

impl<T: UartMap, I: IntToken<Rtt>, Rx: DmaChMap> DrvDmaRx<Rx> for UartEn<T, I> {
  fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken<Rtt>>) {
    unsafe { dma_rx.set_paddr(self.periph.uart_rdr.to_ptr()) };
  }
}

impl<T: UartMap, I: IntToken<Rtt>, Tx: DmaChMap> DrvDmaTx<Tx> for UartEn<T, I> {
  fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken<Rtt>>) {
    unsafe { dma_tx.set_paddr(self.periph.uart_tdr.to_mut_ptr()) };
  }
}

#[allow(missing_docs)]
impl<T: UartMap, I: IntToken<Rtt>> UartEn<T, I> {
  #[inline(always)]
  pub fn brr(&self) -> &T::SUartBrr {
    &self.periph.uart_brr
  }

  #[inline(always)]
  pub fn cr1(&self) -> &T::CUartCr1 {
    &self.periph.uart_cr1
  }

  #[inline(always)]
  pub fn cr3(&self) -> &T::SUartCr3 {
    &self.periph.uart_cr3
  }

  #[inline(always)]
  pub fn icr(&self) -> &T::SUartIcr {
    &self.periph.uart_icr
  }
}

impl<T: UartMap, I: IntToken<Rtt>> DrvRcc for Uart<T, I> {
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

impl<T: UartMap, I: IntToken<Rtt>> DrvRcc for UartEn<T, I> {
  fn reset(&mut self) {
    self.periph.rcc_apbrstr_uartrst.set_bit();
  }

  fn disable_stop_mode(&self) {
    self.periph.rcc_apbsmenr_uartsmen.clear_bit();
  }

  fn enable_stop_mode(&self) {
    self.periph.rcc_apbsmenr_uartsmen.set_bit();
  }
}

impl<T: UartMap, I: IntToken<Rtt>> DrvClockSel for Uart<T, I> {
  #[inline]
  fn clock_sel(&self, value: u32) {
    self.0.clock_sel(value);
  }
}

impl<T: UartMap, I: IntToken<Rtt>> DrvClockSel for UartEn<T, I> {
  fn clock_sel(&self, value: u32) {
    self.periph.rcc_ccipr_uartsel.write_bits(value);
  }
}

impl<T: UartMap, I: IntToken<Rtt>> GuardHandler<UartEn<T, I>>
  for UartEnGuard<T>
{
  fn teardown(&mut self, uart: &mut UartEn<T, I>) {
    uart.periph.rcc_apbenr_uarten.clear_bit()
  }
}
