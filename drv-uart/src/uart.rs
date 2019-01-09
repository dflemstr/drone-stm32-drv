//! Universal Asynchronous Receiver/Transmitter.

use core::ptr::read_volatile;
#[allow(unused_imports)]
use drone_core::res_impl;
use drone_core::{bitfield::Bitfield, res_decl};
use drone_cortex_m::{
  fib::{self, Fiber},
  reg::{marker::*, prelude::*, RegGuard, RegGuardCnt, RegGuardRes},
  thr::prelude::*,
};
#[cfg(any(
  feature = "stm32l4x1",
  feature = "stm32l4x2",
  feature = "stm32l4x3",
  feature = "stm32l4x5",
  feature = "stm32l4x6"
))]
use drone_stm32_drv_dma::dma::{
  Dma, Dma1Ch2Bond, Dma1Ch2Res, Dma1Ch3Bond, Dma1Ch3Res, Dma1Ch4Bond,
  Dma1Ch4Res, Dma1Ch5Bond, Dma1Ch5Res, Dma1Ch6Bond, Dma1Ch6Res, Dma1Ch7Bond,
  Dma1Ch7Res, Dma2Ch1Bond, Dma2Ch1Res, Dma2Ch2Bond, Dma2Ch2Res, Dma2Ch3Bond,
  Dma2Ch3Res, Dma2Ch5Bond, Dma2Ch5Res, Dma2Ch6Bond, Dma2Ch6Res, Dma2Ch7Bond,
  Dma2Ch7Res, DmaRes,
};
use drone_stm32_drv_dma::dma::{DmaBond, DmaBondOnRgc, DmaTxRes};
#[cfg(any(
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
))]
use drone_stm32_drv_dma::dmamux::{DmamuxCh, DmamuxChRes};
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
use drone_stm32_map::reg::{
  lpuart1, rcc, uart4, uart5, usart1, usart2, usart3,
};
#[cfg(any(
  feature = "stm32l4x1",
  feature = "stm32l4x2",
  feature = "stm32l4x3",
  feature = "stm32l4x5",
  feature = "stm32l4x6"
))]
use drone_stm32_map::thr::{
  IntDma1Ch2, IntDma1Ch3, IntDma1Ch4, IntDma1Ch5, IntDma1Ch6, IntDma1Ch7,
  IntDma2Ch1, IntDma2Ch2, IntDma2Ch3, IntDma2Ch5, IntDma2Ch6, IntDma2Ch7,
};
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
use drone_stm32_map::thr::{
  IntLpuart1, IntUart4, IntUart5, IntUsart1, IntUsart2, IntUsart3,
};
use failure::Fail;
use futures::prelude::*;

/// Error returned from [`Uart::rx_stream`](Uart::rx_stream) on overflow.
#[derive(Debug, Fail)]
#[fail(display = "UART RX stream overflow.")]
pub struct UartRxOverflow;

/// UART driver.
pub struct Uart<T, C>(T, C)
where
  T: UartRes,
  C: RegGuardCnt<UartOn<T>>;

/// UART resource.
#[allow(missing_docs)]
pub trait UartRes
where
  Self: Sized + Send + 'static,
  Self: UartResCr1 + UartResCr2 + UartResCr3 + UartResIsr + UartResIcr,
{
  type Int: IntToken<Rtt>;
  type Brr: SRwRegBitBand;
  type Rqr: SWoRegBitBand;
  type Rdr: CRoRegBitBand;
  type Tdr: SRwRegBitBand;
  type RccApbEnrVal: Bitfield<Bits = u32>;
  type RccApbEnr: CRwRegBitBand<Val = Self::RccApbEnrVal>;
  type RccApbEnrUartEn: CRwRwRegFieldBitBand<Reg = Self::RccApbEnr>;

  fn int(&self) -> Self::Int;

  res_decl!(Brr, brr);
  res_decl!(Rqr, rqr);
  res_decl!(Rdr, rdr);
  res_decl!(Tdr, tdr);
  res_decl!(RccApbEnrUartEn, rcc_en);
}

#[allow(missing_docs)]
pub trait UartResCr1 {
  type Cr1: CRwRegBitBand;
  type Cr1M1: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Deat4: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Deat3: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Deat2: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Deat1: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Deat0: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Dedt4: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Dedt3: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Dedt2: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Dedt1: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Dedt0: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Cmie: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Mme: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1M0: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Wake: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Pce: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Ps: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Peie: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Txeie: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Tcie: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Rxneie: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Idleie: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Te: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Re: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Uesm: CRwRwRegFieldBitBand<Reg = Self::Cr1>;
  type Cr1Ue: CRwRwRegFieldBitBand<Reg = Self::Cr1>;

  res_decl!(Cr1, cr1);
  res_decl!(Cr1M1, cr1_m1);
  res_decl!(Cr1Deat4, cr1_deat4);
  res_decl!(Cr1Deat3, cr1_deat3);
  res_decl!(Cr1Deat2, cr1_deat2);
  res_decl!(Cr1Deat1, cr1_deat1);
  res_decl!(Cr1Deat0, cr1_deat0);
  res_decl!(Cr1Dedt4, cr1_dedt4);
  res_decl!(Cr1Dedt3, cr1_dedt3);
  res_decl!(Cr1Dedt2, cr1_dedt2);
  res_decl!(Cr1Dedt1, cr1_dedt1);
  res_decl!(Cr1Dedt0, cr1_dedt0);
  res_decl!(Cr1Cmie, cr1_cmie);
  res_decl!(Cr1Mme, cr1_mme);
  res_decl!(Cr1M0, cr1_m0);
  res_decl!(Cr1Wake, cr1_wake);
  res_decl!(Cr1Pce, cr1_pce);
  res_decl!(Cr1Ps, cr1_ps);
  res_decl!(Cr1Peie, cr1_peie);
  res_decl!(Cr1Txeie, cr1_txeie);
  res_decl!(Cr1Tcie, cr1_tcie);
  res_decl!(Cr1Rxneie, cr1_rxneie);
  res_decl!(Cr1Idleie, cr1_idleie);
  res_decl!(Cr1Te, cr1_te);
  res_decl!(Cr1Re, cr1_re);
  res_decl!(Cr1Uesm, cr1_uesm);
  res_decl!(Cr1Ue, cr1_ue);
}

#[allow(missing_docs)]
pub trait UartResCr2 {
  type Cr2: SRwRegBitBand;
  type Cr2Add47: SRwRwRegFieldBits<Reg = Self::Cr2>;
  type Cr2Add03: SRwRwRegFieldBits<Reg = Self::Cr2>;
  type Cr2Msbfirst: SRwRwRegFieldBitBand<Reg = Self::Cr2>;
  type Cr2Tainv: SRwRwRegFieldBitBand<Reg = Self::Cr2>;
  type Cr2Txinv: SRwRwRegFieldBitBand<Reg = Self::Cr2>;
  type Cr2Rxinv: SRwRwRegFieldBitBand<Reg = Self::Cr2>;
  type Cr2Swap: SRwRwRegFieldBitBand<Reg = Self::Cr2>;
  type Cr2Stop: SRwRwRegFieldBits<Reg = Self::Cr2>;
  type Cr2Clken: SRwRwRegFieldBitBand<Reg = Self::Cr2>;
  type Cr2Addm7: SRwRwRegFieldBitBand<Reg = Self::Cr2>;

  res_decl!(Cr2, cr2);
  res_decl!(Cr2Add47, cr2_add4_7);
  res_decl!(Cr2Add03, cr2_add0_3);
  res_decl!(Cr2Msbfirst, cr2_msbfirst);
  res_decl!(Cr2Tainv, cr2_tainv);
  res_decl!(Cr2Txinv, cr2_txinv);
  res_decl!(Cr2Rxinv, cr2_rxinv);
  res_decl!(Cr2Swap, cr2_swap);
  res_decl!(Cr2Stop, cr2_stop);
  res_decl!(Cr2Clken, cr2_clken);
  res_decl!(Cr2Addm7, cr2_addm7);
}

#[allow(missing_docs)]
pub trait UartResCr3 {
  type Cr3: SRwRegBitBand;
  type Cr3Wufie: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Wus: SRwRwRegFieldBits<Reg = Self::Cr3>;
  type Cr3Dep: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Dem: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Ddre: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Ovrdis: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Ctsie: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Ctse: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Rtse: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Dmat: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Dmar: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Hdsel: SRwRwRegFieldBitBand<Reg = Self::Cr3>;
  type Cr3Eie: SRwRwRegFieldBitBand<Reg = Self::Cr3>;

  res_decl!(Cr3, cr3);
  res_decl!(Cr3Wufie, cr3_wufie);
  res_decl!(Cr3Wus, cr3_wus);
  res_decl!(Cr3Dep, cr3_dep);
  res_decl!(Cr3Dem, cr3_dem);
  res_decl!(Cr3Ddre, cr3_ddre);
  res_decl!(Cr3Ovrdis, cr3_ovrdis);
  res_decl!(Cr3Ctsie, cr3_ctsie);
  res_decl!(Cr3Ctse, cr3_ctse);
  res_decl!(Cr3Rtse, cr3_rtse);
  res_decl!(Cr3Dmat, cr3_dmat);
  res_decl!(Cr3Dmar, cr3_dmar);
  res_decl!(Cr3Hdsel, cr3_hdsel);
  res_decl!(Cr3Eie, cr3_eie);
}

#[allow(missing_docs)]
pub trait UartResIsr {
  type Isr: CRoRegBitBand;
  type IsrReack: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrTeack: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrWuf: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrRwu: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrSbkf: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrCmf: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrBusy: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrCts: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrCtsif: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrTxe: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrTc: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrRxne: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrIdle: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrOre: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrNf: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrFe: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrPe: CRoRoRegFieldBitBand<Reg = Self::Isr>;

  res_decl!(Isr, isr);
  res_decl!(IsrReack, isr_reack);
  res_decl!(IsrTeack, isr_teack);
  res_decl!(IsrWuf, isr_wuf);
  res_decl!(IsrRwu, isr_rwu);
  res_decl!(IsrSbkf, isr_sbkf);
  res_decl!(IsrCmf, isr_cmf);
  res_decl!(IsrBusy, isr_busy);
  res_decl!(IsrCts, isr_cts);
  res_decl!(IsrCtsif, isr_ctsif);
  res_decl!(IsrTxe, isr_txe);
  res_decl!(IsrTc, isr_tc);
  res_decl!(IsrRxne, isr_rxne);
  res_decl!(IsrIdle, isr_idle);
  res_decl!(IsrOre, isr_ore);
  res_decl!(IsrNf, isr_nf);
  res_decl!(IsrFe, isr_fe);
  res_decl!(IsrPe, isr_pe);
}

#[allow(missing_docs)]
pub trait UartResIcr {
  type Icr: CWoRegBitBand;
  type IcrWucf: CWoWoRegFieldBitBand<Reg = Self::Icr>;
  type IcrCmcf: CWoWoRegFieldBitBand<Reg = Self::Icr>;
  type IcrCtscf: CWoWoRegFieldBitBand<Reg = Self::Icr>;
  type IcrTccf: CWoWoRegFieldBitBand<Reg = Self::Icr>;
  type IcrIdlecf: CWoWoRegFieldBitBand<Reg = Self::Icr>;
  type IcrOrecf: CWoWoRegFieldBitBand<Reg = Self::Icr>;
  type IcrNcf: CWoWoRegFieldBitBand<Reg = Self::Icr>;
  type IcrFecf: CWoWoRegFieldBitBand<Reg = Self::Icr>;
  type IcrPecf: CWoWoRegFieldBitBand<Reg = Self::Icr>;

  res_decl!(Icr, icr);
  res_decl!(IcrWucf, icr_wucf);
  res_decl!(IcrCmcf, icr_cmcf);
  res_decl!(IcrCtscf, icr_ctscf);
  res_decl!(IcrTccf, icr_tccf);
  res_decl!(IcrIdlecf, icr_idlecf);
  res_decl!(IcrOrecf, icr_orecf);
  res_decl!(IcrNcf, icr_ncf);
  res_decl!(IcrFecf, icr_fecf);
  res_decl!(IcrPecf, icr_pecf);
}

/// DMA-receiver UART resource.
#[allow(missing_docs)]
pub trait UartDmaRxRes<T: DmaBond>: UartRes {
  #[cfg(any(
    feature = "stm32l4r5",
    feature = "stm32l4r7",
    feature = "stm32l4r9",
    feature = "stm32l4s5",
    feature = "stm32l4s7",
    feature = "stm32l4s9"
  ))]
  fn dmamux_rx_init(
    &self,
    cr_val: &mut <<T::DmamuxChRes as DmamuxChRes>::Cr as Reg<Srt>>::Val,
    dmamux: &DmamuxCh<T::DmamuxChRes>,
  );

  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  fn dma_rx_ch_init(
    &self,
    cs_val: &mut <<T::DmaRes as DmaRes>::Cselr as Reg<Srt>>::Val,
    dma: &Dma<T::DmaRes>,
  );
}

/// DMA-transmitter UART resource.
#[allow(missing_docs)]
pub trait UartDmaTxRes<T: DmaBond>: UartRes {
  #[cfg(any(
    feature = "stm32l4r5",
    feature = "stm32l4r7",
    feature = "stm32l4r9",
    feature = "stm32l4s5",
    feature = "stm32l4s7",
    feature = "stm32l4s9"
  ))]
  fn dmamux_tx_init(
    &self,
    cr_val: &mut <<T::DmamuxChRes as DmamuxChRes>::Cr as Reg<Srt>>::Val,
    dmamux: &DmamuxCh<T::DmamuxChRes>,
  );

  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  fn dma_tx_ch_init(
    &self,
    cs_val: &mut <<T::DmaRes as DmaRes>::Cselr as Reg<Srt>>::Val,
    dma: &Dma<T::DmaRes>,
  );
}

#[allow(missing_docs)]
impl<T, C> Uart<T, C>
where
  T: UartRes,
  C: RegGuardCnt<UartOn<T>>,
{
  #[inline(always)]
  pub fn int(&self) -> T::Int {
    self.0.int()
  }

  #[inline(always)]
  pub fn cr1(&self) -> &<T::Cr1 as Reg<Crt>>::SReg {
    self.0.cr1().as_sync()
  }

  #[inline(always)]
  pub fn cr2(&self) -> &T::Cr2 {
    self.0.cr2()
  }

  #[inline(always)]
  pub fn cr3(&self) -> &T::Cr3 {
    self.0.cr3()
  }

  #[inline(always)]
  pub fn brr(&self) -> &T::Brr {
    self.0.brr()
  }

  #[inline(always)]
  pub fn rqr(&self) -> &T::Rqr {
    self.0.rqr()
  }

  #[inline(always)]
  pub fn isr(&self) -> &<T::Isr as Reg<Crt>>::SReg {
    self.0.isr().as_sync()
  }

  #[inline(always)]
  pub fn icr(&self) -> &<T::Icr as Reg<Crt>>::SReg {
    self.0.icr().as_sync()
  }

  #[inline(always)]
  pub fn rdr(&self) -> &<T::Rdr as Reg<Crt>>::SReg {
    self.0.rdr().as_sync()
  }

  #[inline(always)]
  pub fn tdr(&self) -> &T::Tdr {
    self.0.tdr()
  }
}

impl<T, C> Uart<T, C>
where
  T: UartRes,
  C: RegGuardCnt<UartOn<T>>,
{
  /// Creates a new `Uart`.
  ///
  /// # Safety
  ///
  /// `res` must be the only owner of its contained resources.
  pub unsafe fn new(res: T, rgc: C) -> Self {
    Self(res, rgc)
  }

  /// Releases the underlying resources.
  pub fn free(self) -> T {
    self.0
  }

  /// Enables the clock.
  pub fn on(&self) -> RegGuard<UartOn<T>, C> {
    RegGuard::new(UartOn(*self.0.rcc_en()))
  }

  /// Initializes DMA for the UART as peripheral.
  pub fn dma_rx_init<Rx>(&self, rx: &Rx)
  where
    Rx: DmaBond,
    T: UartDmaRxRes<Rx>,
    C: DmaBondOnRgc<Rx::DmaRes>,
  {
    self.set_dma_rx_paddr(rx);
    self.dmamux_rx_init(rx);
    #[cfg(any(
      feature = "stm32l4x1",
      feature = "stm32l4x2",
      feature = "stm32l4x3",
      feature = "stm32l4x5",
      feature = "stm32l4x6"
    ))]
    rx.dma_ch().cselr_cs().modify(|r| {
      self.0.dma_rx_ch_init(r, rx.dma_ch());
    });
  }

  /// Initializes DMA for the UART as peripheral.
  pub fn dma_tx_init<Tx>(&self, tx: &Tx)
  where
    Tx: DmaBond,
    T: UartDmaTxRes<Tx>,
    C: DmaBondOnRgc<Tx::DmaRes>,
  {
    self.set_dma_tx_paddr(tx);
    self.dmamux_tx_init(tx);
    #[cfg(any(
      feature = "stm32l4x1",
      feature = "stm32l4x2",
      feature = "stm32l4x3",
      feature = "stm32l4x5",
      feature = "stm32l4x6"
    ))]
    tx.dma_ch().cselr_cs().modify(|r| {
      self.0.dma_tx_ch_init(r, tx.dma_ch());
    });
  }

  /// Initializes DMA for the UART as peripheral.
  pub fn dma_init<Rx, Tx>(&self, rx: &Rx, tx: &Tx)
  where
    Rx: DmaBond,
    Tx: DmaBond,
    Tx::DmaRes: DmaTxRes<Rx::DmaRes>,
    T: UartDmaRxRes<Rx> + UartDmaTxRes<Tx>,
    C: DmaBondOnRgc<Rx::DmaRes> + DmaBondOnRgc<Tx::DmaRes>,
  {
    self.set_dma_rx_paddr(rx);
    self.set_dma_tx_paddr(tx);
    self.dmamux_rx_init(rx);
    self.dmamux_tx_init(tx);
    #[cfg(any(
      feature = "stm32l4x1",
      feature = "stm32l4x2",
      feature = "stm32l4x3",
      feature = "stm32l4x5",
      feature = "stm32l4x6"
    ))]
    rx.dma_ch().cselr_cs().modify(|r| {
      self.0.dma_rx_ch_init(r, rx.dma_ch());
      self.0.dma_tx_ch_init(r, tx.dma_ch());
    });
  }

  #[inline]
  fn set_dma_rx_paddr<Rx: DmaBond>(&self, rx: &Rx) {
    unsafe { rx.dma_ch().set_paddr(self.0.rdr().to_ptr() as usize) };
  }

  #[inline]
  fn set_dma_tx_paddr<Tx: DmaBond>(&self, tx: &Tx) {
    unsafe { tx.dma_ch().set_paddr(self.0.tdr().to_mut_ptr() as usize) };
  }

  #[allow(unused_variables)]
  #[inline]
  fn dmamux_rx_init<Rx>(&self, rx: &Rx)
  where
    Rx: DmaBond,
    T: UartDmaRxRes<Rx>,
    C: DmaBondOnRgc<Rx::DmaRes>,
  {
    #[cfg(any(
      feature = "stm32l4r5",
      feature = "stm32l4r7",
      feature = "stm32l4r9",
      feature = "stm32l4s5",
      feature = "stm32l4s7",
      feature = "stm32l4s9"
    ))]
    rx.dmamux_ch().cr_dmareq_id().modify(|r| {
      self.0.dmamux_rx_init(r, rx.dmamux_ch());
    });
  }

  #[allow(unused_variables)]
  #[inline]
  fn dmamux_tx_init<Tx>(&self, tx: &Tx)
  where
    Tx: DmaBond,
    T: UartDmaTxRes<Tx>,
    C: DmaBondOnRgc<Tx::DmaRes>,
  {
    #[cfg(any(
      feature = "stm32l4r5",
      feature = "stm32l4r7",
      feature = "stm32l4r9",
      feature = "stm32l4s5",
      feature = "stm32l4s7",
      feature = "stm32l4s9"
    ))]
    tx.dmamux_ch().cr_dmareq_id().modify(|r| {
      self.0.dmamux_tx_init(r, tx.dmamux_ch());
    });
  }

  /// Returns a future, which resolves on transmission complete event.
  pub fn transmission_complete(&self) -> impl Future<Item = (), Error = !> {
    let tc = *self.0.isr_tc();
    let tcie = *self.0.cr1_tcie();
    self.0.int().add_future(fib::new(move || loop {
      if tc.read_bit_band() {
        tcie.clear_bit();
        break Ok(());
      }
      yield;
    }))
  }

  /// Returns a stream of bytes from the receiver.
  pub fn rx_stream(
    &self,
    capacity: usize,
  ) -> impl Stream<Item = u8, Error = UartRxOverflow> {
    self.0.int().add_stream_ring(
      capacity,
      |_| Err(UartRxOverflow),
      self.rx_stream_fib(),
    )
  }

  /// Returns a stream of bytes from the receiver.
  pub fn rx_stream_skip(
    &self,
    capacity: usize,
  ) -> impl Stream<Item = u8, Error = UartRxOverflow> {
    self
      .0
      .int()
      .add_stream_ring_skip(capacity, self.rx_stream_fib())
  }

  /// Returns a stream of bytes from the receiver.
  pub fn rx_stream_overwrite(
    &self,
    capacity: usize,
  ) -> impl Stream<Item = u8, Error = UartRxOverflow> {
    self
      .0
      .int()
      .add_stream_ring_overwrite(capacity, self.rx_stream_fib())
  }

  fn rx_stream_fib<E: Send>(
    &self,
  ) -> impl Fiber<Input = (), Yield = Option<u8>, Return = Result<Option<u8>, E>>
  {
    let rxne = *self.0.isr_rxne();
    let rdr = *self.0.rdr();
    fib::new(move || loop {
      if rxne.read_bit_band() {
        let byte = unsafe { read_volatile(rdr.to_ptr() as *const _) };
        yield Some(byte);
      }
      yield None;
    })
  }
}

/// UART clock on guard resource.
pub struct UartOn<T: UartRes>(T::RccApbEnrUartEn);

impl<T: UartRes> Clone for UartOn<T> {
  #[inline(always)]
  fn clone(&self) -> Self {
    Self(self.0)
  }
}

impl<T: UartRes> RegGuardRes for UartOn<T> {
  type Reg = T::RccApbEnr;
  type Field = T::RccApbEnrUartEn;

  #[inline(always)]
  fn field(&self) -> &Self::Field {
    &self.0
  }

  #[inline(always)]
  fn up(&self, val: &mut <Self::Reg as Reg<Crt>>::Val) {
    self.0.set(val)
  }

  #[inline(always)]
  fn down(&self, val: &mut <Self::Reg as Reg<Crt>>::Val) {
    self.0.clear(val)
  }
}

#[allow(unused_macros)]
macro_rules! uart {
  (
    $doc:expr,
    $name:ident,
    $name_macro:ident,
    $doc_res:expr,
    $name_res:ident,
    $doc_on:expr,
    $name_on:ident,
    $int_ty:ident,
    $uarten_ty:ident,
    $uart:ident,
    $uart_cr1:ident,
    $uart_cr2:ident,
    $uart_cr3:ident,
    $uart_brr:ident,
    $uart_rqr:ident,
    $uart_isr:ident,
    $uart_icr:ident,
    $uart_rdr:ident,
    $uart_tdr:ident,
    $apb_enr:ident,
    $rcc_apb_enr_uarten:ident,
    $rcc_apb_enr:ident,
    $uarten:ident,
    (
      $dma_rx_req_id:expr
      $(,(
        $dma_rx_bond:ident,
        $dma_rx_res:ident,
        $int_dma_rx:ident,
        $dma_rx_cs:expr
      ))*
    ),
    (
      $dma_tx_req_id:expr
      $(,(
        $dma_tx_bond:ident,
        $dma_tx_res:ident,
        $int_dma_tx:ident,
        $dma_tx_cs:expr
      ))*
    ),
  ) => {
    #[doc = $doc]
    pub type $name<I, C> = Uart<$name_res<I>, C>;

    #[doc = $doc_res]
    #[allow(missing_docs)]
    pub struct $name_res<I: $int_ty<Rtt>> {
      pub $uart: I,
      pub $uart_cr1: $uart::Cr1<Crt>,
      pub $uart_cr2: $uart::Cr2<Srt>,
      pub $uart_cr3: $uart::Cr3<Srt>,
      pub $uart_brr: $uart::Brr<Srt>,
      pub $uart_rqr: $uart::Rqr<Srt>,
      pub $uart_isr: $uart::Isr<Crt>,
      pub $uart_icr: $uart::Icr<Crt>,
      pub $uart_rdr: $uart::Rdr<Crt>,
      pub $uart_tdr: $uart::Tdr<Srt>,
      pub $rcc_apb_enr_uarten: rcc::$apb_enr::$uarten_ty<Crt>,
    }

    #[doc = $doc_on]
    pub type $name_on<I> = UartOn<$name_res<I>>;

    /// Creates a new `Uart`.
    #[macro_export]
    macro_rules! $name_macro {
      ($reg:ident, $thr:ident, $rgc:path) => {
        unsafe {
          $crate::uart::Uart::new(
            $crate::uart::$name_res {
              $uart: $thr.$uart.to_regular(),
              $uart_cr1: $reg.$uart_cr1.acquire_copy(),
              $uart_cr2: $reg.$uart_cr2,
              $uart_cr3: $reg.$uart_cr3,
              $uart_brr: $reg.$uart_brr,
              $uart_rqr: $reg.$uart_rqr,
              $uart_isr: $reg.$uart_isr.acquire_copy(),
              $uart_icr: $reg.$uart_icr.acquire_copy(),
              $uart_rdr: $reg.$uart_rdr.acquire_copy(),
              $uart_tdr: $reg.$uart_tdr,
              $rcc_apb_enr_uarten: $reg.$rcc_apb_enr.$uarten.acquire_copy(),
            },
            $rgc,
          )
        }
      };
    }

    impl<I: $int_ty<Rtt>> UartRes for $name_res<I> {
      type Int = I;
      type Brr = $uart::Brr<Srt>;
      type Rqr = $uart::Rqr<Srt>;
      type Rdr = $uart::Rdr<Crt>;
      type Tdr = $uart::Tdr<Srt>;
      type RccApbEnrVal = rcc::$apb_enr::Val;
      type RccApbEnr = rcc::$apb_enr::Reg<Crt>;
      type RccApbEnrUartEn = rcc::$apb_enr::$uarten_ty<Crt>;

      #[inline(always)]
      fn int(&self) -> Self::Int {
        self.$uart
      }

      res_impl!(Brr, brr, $uart_brr);
      res_impl!(Rqr, rqr, $uart_rqr);
      res_impl!(Rdr, rdr, $uart_rdr);
      res_impl!(Tdr, tdr, $uart_tdr);
      res_impl!(RccApbEnrUartEn, rcc_en, $rcc_apb_enr_uarten);
    }

    impl<I: $int_ty<Rtt>> UartResCr1 for $name_res<I> {
      type Cr1 = $uart::Cr1<Crt>;
      type Cr1M1 = $uart::cr1::M1<Crt>;
      type Cr1Deat4 = $uart::cr1::Deat4<Crt>;
      type Cr1Deat3 = $uart::cr1::Deat3<Crt>;
      type Cr1Deat2 = $uart::cr1::Deat2<Crt>;
      type Cr1Deat1 = $uart::cr1::Deat1<Crt>;
      type Cr1Deat0 = $uart::cr1::Deat0<Crt>;
      type Cr1Dedt4 = $uart::cr1::Dedt4<Crt>;
      type Cr1Dedt3 = $uart::cr1::Dedt3<Crt>;
      type Cr1Dedt2 = $uart::cr1::Dedt2<Crt>;
      type Cr1Dedt1 = $uart::cr1::Dedt1<Crt>;
      type Cr1Dedt0 = $uart::cr1::Dedt0<Crt>;
      type Cr1Cmie = $uart::cr1::Cmie<Crt>;
      type Cr1Mme = $uart::cr1::Mme<Crt>;
      type Cr1M0 = $uart::cr1::M0<Crt>;
      type Cr1Wake = $uart::cr1::Wake<Crt>;
      type Cr1Pce = $uart::cr1::Pce<Crt>;
      type Cr1Ps = $uart::cr1::Ps<Crt>;
      type Cr1Peie = $uart::cr1::Peie<Crt>;
      type Cr1Txeie = $uart::cr1::Txeie<Crt>;
      type Cr1Tcie = $uart::cr1::Tcie<Crt>;
      type Cr1Rxneie = $uart::cr1::Rxneie<Crt>;
      type Cr1Idleie = $uart::cr1::Idleie<Crt>;
      type Cr1Te = $uart::cr1::Te<Crt>;
      type Cr1Re = $uart::cr1::Re<Crt>;
      type Cr1Uesm = $uart::cr1::Uesm<Crt>;
      type Cr1Ue = $uart::cr1::Ue<Crt>;

      res_impl!(Cr1, cr1, $uart_cr1);
      res_impl!(Cr1M1, cr1_m1, $uart_cr1.m1);
      res_impl!(Cr1Deat4, cr1_deat4, $uart_cr1.deat4);
      res_impl!(Cr1Deat3, cr1_deat3, $uart_cr1.deat3);
      res_impl!(Cr1Deat2, cr1_deat2, $uart_cr1.deat2);
      res_impl!(Cr1Deat1, cr1_deat1, $uart_cr1.deat1);
      res_impl!(Cr1Deat0, cr1_deat0, $uart_cr1.deat0);
      res_impl!(Cr1Dedt4, cr1_dedt4, $uart_cr1.dedt4);
      res_impl!(Cr1Dedt3, cr1_dedt3, $uart_cr1.dedt3);
      res_impl!(Cr1Dedt2, cr1_dedt2, $uart_cr1.dedt2);
      res_impl!(Cr1Dedt1, cr1_dedt1, $uart_cr1.dedt1);
      res_impl!(Cr1Dedt0, cr1_dedt0, $uart_cr1.dedt0);
      res_impl!(Cr1Cmie, cr1_cmie, $uart_cr1.cmie);
      res_impl!(Cr1Mme, cr1_mme, $uart_cr1.mme);
      res_impl!(Cr1M0, cr1_m0, $uart_cr1.m0);
      res_impl!(Cr1Wake, cr1_wake, $uart_cr1.wake);
      res_impl!(Cr1Pce, cr1_pce, $uart_cr1.pce);
      res_impl!(Cr1Ps, cr1_ps, $uart_cr1.ps);
      res_impl!(Cr1Peie, cr1_peie, $uart_cr1.peie);
      res_impl!(Cr1Txeie, cr1_txeie, $uart_cr1.txeie);
      res_impl!(Cr1Tcie, cr1_tcie, $uart_cr1.tcie);
      res_impl!(Cr1Rxneie, cr1_rxneie, $uart_cr1.rxneie);
      res_impl!(Cr1Idleie, cr1_idleie, $uart_cr1.idleie);
      res_impl!(Cr1Te, cr1_te, $uart_cr1.te);
      res_impl!(Cr1Re, cr1_re, $uart_cr1.re);
      res_impl!(Cr1Uesm, cr1_uesm, $uart_cr1.uesm);
      res_impl!(Cr1Ue, cr1_ue, $uart_cr1.ue);
    }

    impl<I: $int_ty<Rtt>> UartResCr2 for $name_res<I> {
      type Cr2 = $uart::Cr2<Srt>;
      type Cr2Add47 = $uart::cr2::Add47<Srt>;
      type Cr2Add03 = $uart::cr2::Add03<Srt>;
      type Cr2Msbfirst = $uart::cr2::Msbfirst<Srt>;
      type Cr2Tainv = $uart::cr2::Tainv<Srt>;
      type Cr2Txinv = $uart::cr2::Txinv<Srt>;
      type Cr2Rxinv = $uart::cr2::Rxinv<Srt>;
      type Cr2Swap = $uart::cr2::Swap<Srt>;
      type Cr2Stop = $uart::cr2::Stop<Srt>;
      type Cr2Clken = $uart::cr2::Clken<Srt>;
      type Cr2Addm7 = $uart::cr2::Addm7<Srt>;

      res_impl!(Cr2, cr2, $uart_cr2);
      res_impl!(Cr2Add47, cr2_add4_7, $uart_cr2.add4_7);
      res_impl!(Cr2Add03, cr2_add0_3, $uart_cr2.add0_3);
      res_impl!(Cr2Msbfirst, cr2_msbfirst, $uart_cr2.msbfirst);
      res_impl!(Cr2Tainv, cr2_tainv, $uart_cr2.tainv);
      res_impl!(Cr2Txinv, cr2_txinv, $uart_cr2.txinv);
      res_impl!(Cr2Rxinv, cr2_rxinv, $uart_cr2.rxinv);
      res_impl!(Cr2Swap, cr2_swap, $uart_cr2.swap);
      res_impl!(Cr2Stop, cr2_stop, $uart_cr2.stop);
      res_impl!(Cr2Clken, cr2_clken, $uart_cr2.clken);
      res_impl!(Cr2Addm7, cr2_addm7, $uart_cr2.addm7);
    }

    impl<I: $int_ty<Rtt>> UartResCr3 for $name_res<I> {
      type Cr3 = $uart::Cr3<Srt>;
      type Cr3Wufie = $uart::cr3::Wufie<Srt>;
      type Cr3Wus = $uart::cr3::Wus<Srt>;
      type Cr3Dep = $uart::cr3::Dep<Srt>;
      type Cr3Dem = $uart::cr3::Dem<Srt>;
      type Cr3Ddre = $uart::cr3::Ddre<Srt>;
      type Cr3Ovrdis = $uart::cr3::Ovrdis<Srt>;
      type Cr3Ctsie = $uart::cr3::Ctsie<Srt>;
      type Cr3Ctse = $uart::cr3::Ctse<Srt>;
      type Cr3Rtse = $uart::cr3::Rtse<Srt>;
      type Cr3Dmat = $uart::cr3::Dmat<Srt>;
      type Cr3Dmar = $uart::cr3::Dmar<Srt>;
      type Cr3Hdsel = $uart::cr3::Hdsel<Srt>;
      type Cr3Eie = $uart::cr3::Eie<Srt>;

      res_impl!(Cr3, cr3, $uart_cr3);
      res_impl!(Cr3Wufie, cr3_wufie, $uart_cr3.wufie);
      res_impl!(Cr3Wus, cr3_wus, $uart_cr3.wus);
      res_impl!(Cr3Dep, cr3_dep, $uart_cr3.dep);
      res_impl!(Cr3Dem, cr3_dem, $uart_cr3.dem);
      res_impl!(Cr3Ddre, cr3_ddre, $uart_cr3.ddre);
      res_impl!(Cr3Ovrdis, cr3_ovrdis, $uart_cr3.ovrdis);
      res_impl!(Cr3Ctsie, cr3_ctsie, $uart_cr3.ctsie);
      res_impl!(Cr3Ctse, cr3_ctse, $uart_cr3.ctse);
      res_impl!(Cr3Rtse, cr3_rtse, $uart_cr3.rtse);
      res_impl!(Cr3Dmat, cr3_dmat, $uart_cr3.dmat);
      res_impl!(Cr3Dmar, cr3_dmar, $uart_cr3.dmar);
      res_impl!(Cr3Hdsel, cr3_hdsel, $uart_cr3.hdsel);
      res_impl!(Cr3Eie, cr3_eie, $uart_cr3.eie);
    }

    impl<I: $int_ty<Rtt>> UartResIsr for $name_res<I> {
      type Isr = $uart::Isr<Crt>;
      type IsrReack = $uart::isr::Reack<Crt>;
      type IsrTeack = $uart::isr::Teack<Crt>;
      type IsrWuf = $uart::isr::Wuf<Crt>;
      type IsrRwu = $uart::isr::Rwu<Crt>;
      type IsrSbkf = $uart::isr::Sbkf<Crt>;
      type IsrCmf = $uart::isr::Cmf<Crt>;
      type IsrBusy = $uart::isr::Busy<Crt>;
      type IsrCts = $uart::isr::Cts<Crt>;
      type IsrCtsif = $uart::isr::Ctsif<Crt>;
      type IsrTxe = $uart::isr::Txe<Crt>;
      type IsrTc = $uart::isr::Tc<Crt>;
      type IsrRxne = $uart::isr::Rxne<Crt>;
      type IsrIdle = $uart::isr::Idle<Crt>;
      type IsrOre = $uart::isr::Ore<Crt>;
      type IsrNf = $uart::isr::Nf<Crt>;
      type IsrFe = $uart::isr::Fe<Crt>;
      type IsrPe = $uart::isr::Pe<Crt>;

      res_impl!(Isr, isr, $uart_isr);
      res_impl!(IsrReack, isr_reack, $uart_isr.reack);
      res_impl!(IsrTeack, isr_teack, $uart_isr.teack);
      res_impl!(IsrWuf, isr_wuf, $uart_isr.wuf);
      res_impl!(IsrRwu, isr_rwu, $uart_isr.rwu);
      res_impl!(IsrSbkf, isr_sbkf, $uart_isr.sbkf);
      res_impl!(IsrCmf, isr_cmf, $uart_isr.cmf);
      res_impl!(IsrBusy, isr_busy, $uart_isr.busy);
      res_impl!(IsrCts, isr_cts, $uart_isr.cts);
      res_impl!(IsrCtsif, isr_ctsif, $uart_isr.ctsif);
      res_impl!(IsrTxe, isr_txe, $uart_isr.txe);
      res_impl!(IsrTc, isr_tc, $uart_isr.tc);
      res_impl!(IsrRxne, isr_rxne, $uart_isr.rxne);
      res_impl!(IsrIdle, isr_idle, $uart_isr.idle);
      res_impl!(IsrOre, isr_ore, $uart_isr.ore);
      res_impl!(IsrNf, isr_nf, $uart_isr.nf);
      res_impl!(IsrFe, isr_fe, $uart_isr.fe);
      res_impl!(IsrPe, isr_pe, $uart_isr.pe);
    }

    impl<I: $int_ty<Rtt>> UartResIcr for $name_res<I> {
      type Icr = $uart::Icr<Crt>;
      type IcrWucf = $uart::icr::Wucf<Crt>;
      type IcrCmcf = $uart::icr::Cmcf<Crt>;
      type IcrCtscf = $uart::icr::Ctscf<Crt>;
      type IcrTccf = $uart::icr::Tccf<Crt>;
      type IcrIdlecf = $uart::icr::Idlecf<Crt>;
      type IcrOrecf = $uart::icr::Orecf<Crt>;
      type IcrNcf = $uart::icr::Ncf<Crt>;
      type IcrFecf = $uart::icr::Fecf<Crt>;
      type IcrPecf = $uart::icr::Pecf<Crt>;

      res_impl!(Icr, icr, $uart_icr);
      res_impl!(IcrWucf, icr_wucf, $uart_icr.wucf);
      res_impl!(IcrCmcf, icr_cmcf, $uart_icr.cmcf);
      res_impl!(IcrCtscf, icr_ctscf, $uart_icr.ctscf);
      res_impl!(IcrTccf, icr_tccf, $uart_icr.tccf);
      res_impl!(IcrIdlecf, icr_idlecf, $uart_icr.idlecf);
      res_impl!(IcrOrecf, icr_orecf, $uart_icr.orecf);
      res_impl!(IcrNcf, icr_ncf, $uart_icr.ncf);
      res_impl!(IcrFecf, icr_fecf, $uart_icr.fecf);
      res_impl!(IcrPecf, icr_pecf, $uart_icr.pecf);
    }

    #[cfg(any(
      feature = "stm32l4r5",
      feature = "stm32l4r7",
      feature = "stm32l4r9",
      feature = "stm32l4s5",
      feature = "stm32l4s7",
      feature = "stm32l4s9"
    ))]
    impl<I, T> UartDmaRxRes<T> for $name_res<I>
    where
      T: DmaBond,
      I: $int_ty<Rtt>,
    {
      #[inline(always)]
      fn dmamux_rx_init(
        &self,
        cr_val: &mut <<T::DmamuxChRes as DmamuxChRes>::Cr as Reg<Srt>>::Val,
        dmamux: &DmamuxCh<T::DmamuxChRes>,
      ) {
        dmamux.cr_dmareq_id().write(cr_val, $dma_rx_req_id);
      }
    }

    $(
      #[cfg(not(any(
        feature = "stm32l4r5",
        feature = "stm32l4r7",
        feature = "stm32l4r9",
        feature = "stm32l4s5",
        feature = "stm32l4s7",
        feature = "stm32l4s9"
      )))]
      impl<I, Rx, C> UartDmaRxRes<$dma_rx_bond<Rx, C>> for $name_res<I>
      where
        Rx: $int_dma_rx<Rtt>,
        I: $int_ty<Rtt>,
        C: DmaBondOnRgc<$dma_rx_res<Rx>>,
      {
        #[cfg(any(
          feature = "stm32l4x1",
          feature = "stm32l4x2",
          feature = "stm32l4x3",
          feature = "stm32l4x5",
          feature = "stm32l4x6"
        ))]
        #[inline(always)]
        fn dma_rx_ch_init(
          &self,
          cs_val: &mut <<<$dma_rx_bond<Rx, C> as DmaBond>::DmaRes
            as DmaRes>::Cselr as Reg<Srt>>::Val,
          dma: &Dma<<$dma_rx_bond<Rx, C> as DmaBond>::DmaRes>,
        ) {
          dma.cselr_cs().write(cs_val, $dma_rx_cs);
        }
      }
    )*

    #[cfg(any(
      feature = "stm32l4r5",
      feature = "stm32l4r7",
      feature = "stm32l4r9",
      feature = "stm32l4s5",
      feature = "stm32l4s7",
      feature = "stm32l4s9"
    ))]
    impl<I, T> UartDmaTxRes<T> for $name_res<I>
    where
      T: DmaBond,
      I: $int_ty<Rtt>,
    {
      #[inline(always)]
      fn dmamux_tx_init(
        &self,
        cr_val: &mut <<T::DmamuxChRes as DmamuxChRes>::Cr as Reg<Srt>>::Val,
        dmamux: &DmamuxCh<T::DmamuxChRes>,
      ) {
        dmamux.cr_dmareq_id().write(cr_val, $dma_tx_req_id);
      }
    }

    $(
      #[cfg(not(any(
        feature = "stm32l4r5",
        feature = "stm32l4r7",
        feature = "stm32l4r9",
        feature = "stm32l4s5",
        feature = "stm32l4s7",
        feature = "stm32l4s9"
      )))]
      impl<I, Tx, C> UartDmaTxRes<$dma_tx_bond<Tx, C>> for $name_res<I>
      where
        Tx: $int_dma_tx<Rtt>,
        I: $int_ty<Rtt>,
        C: DmaBondOnRgc<$dma_tx_res<Tx>>,
      {
        #[cfg(any(
          feature = "stm32l4x1",
          feature = "stm32l4x2",
          feature = "stm32l4x3",
          feature = "stm32l4x5",
          feature = "stm32l4x6"
        ))]
        #[inline(always)]
        fn dma_tx_ch_init(
          &self,
          cs_val: &mut <<<$dma_tx_bond<Tx, C> as DmaBond>::DmaRes
            as DmaRes>::Cselr as Reg<Srt>>::Val,
          dma: &Dma<<$dma_tx_bond<Tx, C> as DmaBond>::DmaRes>,
        ) {
          dma.cselr_cs().write(cs_val, $dma_tx_cs);
        }
      }
    )*
  };
}

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
uart! {
  "USART1 driver.",
  Usart1,
  drv_usart1,
  "USART1 resource.",
  Usart1Res,
  "USART1 clock on guard resource.",
  Usart1On,
  IntUsart1,
  Usart1En,
  usart1,
  usart1_cr1,
  usart1_cr2,
  usart1_cr3,
  usart1_brr,
  usart1_rqr,
  usart1_isr,
  usart1_icr,
  usart1_rdr,
  usart1_tdr,
  apb2enr,
  rcc_apb2enr_usart1en,
  rcc_apb2enr,
  usart1en,
  (
    24,
    (Dma1Ch5Bond, Dma1Ch5Res, IntDma1Ch5, 0b0010),
    (Dma2Ch7Bond, Dma2Ch7Res, IntDma2Ch7, 0b0010)
  ),
  (
    25,
    (Dma1Ch4Bond, Dma1Ch4Res, IntDma1Ch4, 0b0010),
    (Dma2Ch6Bond, Dma2Ch6Res, IntDma2Ch6, 0b0010)
 ),
}

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
uart! {
  "USART2 driver.",
  Usart2,
  drv_usart2,
  "USART2 resource.",
  Usart2Res,
  "USART2 clock on guard resource.",
  Usart2On,
  IntUsart2,
  Usart2En,
  usart2,
  usart2_cr1,
  usart2_cr2,
  usart2_cr3,
  usart2_brr,
  usart2_rqr,
  usart2_isr,
  usart2_icr,
  usart2_rdr,
  usart2_tdr,
  apb1enr1,
  rcc_apb1enr1_usart2en,
  rcc_apb1enr1,
  usart2en,
  (26, (Dma1Ch6Bond, Dma1Ch6Res, IntDma1Ch6, 0b0010)),
  (27, (Dma1Ch7Bond, Dma1Ch7Res, IntDma1Ch7, 0b0010)),
}

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
uart! {
  "USART3 driver.",
  Usart3,
  drv_usart3,
  "USART3 resource.",
  Usart3Res,
  "USART3 clock on guard resource.",
  Usart3On,
  IntUsart3,
  Usart3En,
  usart3,
  usart3_cr1,
  usart3_cr2,
  usart3_cr3,
  usart3_brr,
  usart3_rqr,
  usart3_isr,
  usart3_icr,
  usart3_rdr,
  usart3_tdr,
  apb1enr1,
  rcc_apb1enr1_usart3en,
  rcc_apb1enr1,
  usart3en,
  (28, (Dma1Ch3Bond, Dma1Ch3Res, IntDma1Ch3, 0b0010)),
  (29, (Dma1Ch2Bond, Dma1Ch2Res, IntDma1Ch2, 0b0010)),
}

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
uart! {
  "UART4 driver.",
  Uart4,
  drv_uart4,
  "UART4 resource.",
  Uart4Res,
  "UART4 clock on guard resource.",
  Uart4On,
  IntUart4,
  Uart4En,
  uart4,
  uart4_cr1,
  uart4_cr2,
  uart4_cr3,
  uart4_brr,
  uart4_rqr,
  uart4_isr,
  uart4_icr,
  uart4_rdr,
  uart4_tdr,
  apb1enr1,
  rcc_apb1enr1_uart4en,
  rcc_apb1enr1,
  uart4en,
  (30, (Dma2Ch5Bond, Dma2Ch5Res, IntDma2Ch5, 0b0010)),
  (31, (Dma2Ch3Bond, Dma2Ch3Res, IntDma2Ch3, 0b0010)),
}

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
uart! {
  "UART5 driver.",
  Uart5,
  drv_uart5,
  "UART5 resource.",
  Uart5Res,
  "UART5 clock on guard resource.",
  Uart5On,
  IntUart5,
  Uart5En,
  uart5,
  uart5_cr1,
  uart5_cr2,
  uart5_cr3,
  uart5_brr,
  uart5_rqr,
  uart5_isr,
  uart5_icr,
  uart5_rdr,
  uart5_tdr,
  apb1enr1,
  rcc_apb1enr1_uart5en,
  rcc_apb1enr1,
  uart5en,
  (32, (Dma2Ch2Bond, Dma2Ch2Res, IntDma2Ch2, 0b0010)),
  (33, (Dma2Ch1Bond, Dma2Ch1Res, IntDma2Ch1, 0b0010)),
}

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
uart! {
  "LPUART1 driver.",
  Lpuart1,
  drv_lpuart1,
  "LPUART1 resource.",
  Lpuart1Res,
  "LPUART1 clock on guard resource.",
  Lpuart1On,
  IntLpuart1,
  Lpuart1En,
  lpuart1,
  lpuart1_cr1,
  lpuart1_cr2,
  lpuart1_cr3,
  lpuart1_brr,
  lpuart1_rqr,
  lpuart1_isr,
  lpuart1_icr,
  lpuart1_rdr,
  lpuart1_tdr,
  apb1enr2,
  rcc_apb1enr2_lpuart1en,
  rcc_apb1enr2,
  lpuart1en,
  (34, (Dma2Ch7Bond, Dma2Ch7Res, IntDma2Ch7, 0b0100)),
  (35, (Dma2Ch6Bond, Dma2Ch6Res, IntDma2Ch6, 0b0100)),
}
