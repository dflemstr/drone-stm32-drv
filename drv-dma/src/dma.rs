//! Direct memory access controller.

use core::marker::PhantomData;
#[cfg(any(
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
))]
use dmamux::{
  Dmamux1Ch0Res, Dmamux1Ch10Res, Dmamux1Ch11Res, Dmamux1Ch12Res,
  Dmamux1Ch13Res, Dmamux1Ch1Res, Dmamux1Ch2Res, Dmamux1Ch3Res, Dmamux1Ch4Res,
  Dmamux1Ch5Res, Dmamux1Ch6Res, Dmamux1Ch7Res, Dmamux1Ch8Res, Dmamux1Ch9Res,
  DmamuxCh, DmamuxChRes, DmamuxOn,
};
use drone_core::bitfield::Bitfield;
use drone_core::drv::Resource;
use drone_cortex_m::fib;
use drone_cortex_m::reg::marker::*;
use drone_cortex_m::reg::prelude::*;
use drone_cortex_m::reg::{RegGuard, RegGuardCnt, RegGuardRes};
use drone_cortex_m::thr::prelude::*;
#[cfg(any(
  feature = "stm32f100",
  feature = "stm32f101",
  feature = "stm32f102",
  feature = "stm32f103",
  feature = "stm32f107",
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
use drone_stm32_map::reg::{dma1, dma2, rcc};
#[cfg(any(
  feature = "stm32f100",
  feature = "stm32f101",
  feature = "stm32f102",
  feature = "stm32f103"
))]
use drone_stm32_map::thr::IntDma2Channel45 as IntDma2Ch4;
#[cfg(any(
  feature = "stm32f100",
  feature = "stm32f101",
  feature = "stm32f102",
  feature = "stm32f103"
))]
use drone_stm32_map::thr::IntDma2Channel45 as IntDma2Ch5;
#[cfg(any(
  feature = "stm32l4x1",
  feature = "stm32l4x2",
  feature = "stm32l4x6",
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
))]
use drone_stm32_map::thr::{
  IntDma1Ch1, IntDma1Ch2, IntDma1Ch3, IntDma1Ch4, IntDma1Ch5, IntDma1Ch6,
  IntDma1Ch7, IntDma2Ch1, IntDma2Ch2, IntDma2Ch3, IntDma2Ch4, IntDma2Ch5,
  IntDma2Ch6, IntDma2Ch7,
};
#[cfg(any(
  feature = "stm32f100",
  feature = "stm32f101",
  feature = "stm32f102",
  feature = "stm32f103",
  feature = "stm32f107",
  feature = "stm32l4x3",
  feature = "stm32l4x5"
))]
use drone_stm32_map::thr::{
  IntDma1Channel1 as IntDma1Ch1, IntDma1Channel2 as IntDma1Ch2,
  IntDma1Channel3 as IntDma1Ch3, IntDma1Channel4 as IntDma1Ch4,
  IntDma1Channel5 as IntDma1Ch5, IntDma1Channel6 as IntDma1Ch6,
  IntDma1Channel7 as IntDma1Ch7, IntDma2Channel1 as IntDma2Ch1,
  IntDma2Channel2 as IntDma2Ch2, IntDma2Channel3 as IntDma2Ch3,
};
#[cfg(any(
  feature = "stm32f107",
  feature = "stm32l4x3",
  feature = "stm32l4x5"
))]
use drone_stm32_map::thr::{
  IntDma2Channel4 as IntDma2Ch4, IntDma2Channel5 as IntDma2Ch5,
};
#[cfg(any(feature = "stm32l4x3", feature = "stm32l4x5"))]
use drone_stm32_map::thr::{
  IntDma2Channel6 as IntDma2Ch6, IntDma2Channel7 as IntDma2Ch7,
};
use futures::prelude::*;

/// Error returned when `DMA_ISR_TEIFx` flag in set.
#[derive(Debug, Fail)]
#[fail(display = "DMA transfer error.")]
pub struct DmaTransferError;

/// DMA driver.
#[derive(Driver)]
pub struct Dma<T: DmaRes>(T);

/// DMA resource.
#[allow(missing_docs)]
pub trait DmaRes: Resource + DmaResCcr + DmaResIfcr + DmaResIsr {
  type RccRes: DmaRccRes;
  type Int: IntToken<Rtt>;
  type CmarVal: Bitfield<Bits = u32>;
  type Cmar: SRwReg<Val = Self::CmarVal>;
  type CmarMa: SRwRwRegFieldBits<Reg = Self::Cmar>;
  type CndtrVal: Bitfield<Bits = u32>;
  type Cndtr: SRwReg<Val = Self::CndtrVal>;
  type CndtrNdt: SRwRwRegFieldBits<Reg = Self::Cndtr>;
  type CparVal: Bitfield<Bits = u32>;
  type Cpar: SRwReg<Val = Self::CparVal>;
  type CparPa: SRwRwRegFieldBits<Reg = Self::Cpar>;
  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  type Cselr: SRwReg;
  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  type CselrCs: SRwRwRegFieldBits<Reg = Self::Cselr>;

  #[cfg(any(
    feature = "stm32l4r5",
    feature = "stm32l4r7",
    feature = "stm32l4r9",
    feature = "stm32l4s5",
    feature = "stm32l4s7",
    feature = "stm32l4s9"
  ))]
  type DmamuxChRes: DmamuxChRes;

  fn int(&self) -> Self::Int;

  res_decl!(Cmar, cmar);
  res_decl!(CmarMa, cmar_ma);
  res_decl!(Cndtr, cndtr);
  res_decl!(CndtrNdt, cndtr_ndt);
  res_decl!(Cpar, cpar);
  res_decl!(CparPa, cpar_pa);
  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  res_decl!(CselrCs, cselr_cs);
}

#[allow(missing_docs)]
pub trait DmaResCcr {
  type CcrVal: Bitfield<Bits = u32>;
  type Ccr: SRwReg<Val = Self::CcrVal>;
  type CcrMem2Mem: SRwRwRegFieldBit<Reg = Self::Ccr>;
  type CcrMsize: SRwRwRegFieldBits<Reg = Self::Ccr>;
  type CcrPsize: SRwRwRegFieldBits<Reg = Self::Ccr>;
  type CcrMinc: SRwRwRegFieldBit<Reg = Self::Ccr>;
  type CcrPinc: SRwRwRegFieldBit<Reg = Self::Ccr>;
  type CcrCirc: SRwRwRegFieldBit<Reg = Self::Ccr>;
  type CcrDir: SRwRwRegFieldBit<Reg = Self::Ccr>;
  type CcrTeie: SRwRwRegFieldBit<Reg = Self::Ccr>;
  type CcrHtie: SRwRwRegFieldBit<Reg = Self::Ccr>;
  type CcrTcie: SRwRwRegFieldBit<Reg = Self::Ccr>;
  type CcrEn: SRwRwRegFieldBit<Reg = Self::Ccr>;

  res_decl!(Ccr, ccr);
  res_decl!(CcrMem2Mem, ccr_mem2mem);
  res_decl!(CcrMsize, ccr_msize);
  res_decl!(CcrPsize, ccr_psize);
  res_decl!(CcrMinc, ccr_minc);
  res_decl!(CcrPinc, ccr_pinc);
  res_decl!(CcrCirc, ccr_circ);
  res_decl!(CcrDir, ccr_dir);
  res_decl!(CcrTeie, ccr_teie);
  res_decl!(CcrHtie, ccr_htie);
  res_decl!(CcrTcie, ccr_tcie);
  res_decl!(CcrEn, ccr_en);
}

#[allow(missing_docs)]
pub trait DmaResIfcr {
  type Ifcr: CWoRegBitBand;
  type IfcrCgif: CWoWoRegFieldBitBand<Reg = Self::Ifcr>;
  type IfcrChtif: CWoWoRegFieldBitBand<Reg = Self::Ifcr>;
  type IfcrCtcif: CWoWoRegFieldBitBand<Reg = Self::Ifcr>;
  type IfcrCteif: CWoWoRegFieldBitBand<Reg = Self::Ifcr>;

  res_decl!(IfcrCgif, ifcr_cgif);
  res_decl!(IfcrChtif, ifcr_chtif);
  res_decl!(IfcrCtcif, ifcr_ctcif);
  res_decl!(IfcrCteif, ifcr_cteif);
}

#[allow(missing_docs)]
pub trait DmaResIsr {
  type Isr: CRoRegBitBand;
  type IsrGif: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrHtif: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrTcif: CRoRoRegFieldBitBand<Reg = Self::Isr>;
  type IsrTeif: CRoRoRegFieldBitBand<Reg = Self::Isr>;

  res_decl!(IsrGif, isr_gif);
  res_decl!(IsrHtif, isr_htif);
  res_decl!(IsrTcif, isr_tcif);
  res_decl!(IsrTeif, isr_teif);
}

/// DMA reset and clock control driver.
#[derive(Driver)]
pub struct DmaRcc<T, C>(T, PhantomData<C>)
where
  T: DmaRccRes,
  C: RegGuardCnt<DmaOn<T>>;

/// DMA reset and clock control resource.
#[allow(missing_docs)]
pub trait DmaRccRes: Resource {
  type RccAhb1EnrVal: Bitfield<Bits = u32>;
  type RccAhb1Enr: CRwRegBitBand<Val = Self::RccAhb1EnrVal>;
  type RccAhb1EnrDmaEn: CRwRwRegFieldBitBand<Reg = Self::RccAhb1Enr>;

  res_decl!(RccAhb1EnrDmaEn, en);
}

/// DMA clock on guard resource.
pub struct DmaOn<T: DmaRccRes>(T::RccAhb1EnrDmaEn);

/// DMA bond.
pub trait DmaBond: Sized + Send + 'static {
  /// Register guard counters.
  type Rgc: DmaBondOnRgc<Self::DmaRes>;

  /// DMA resource.
  type DmaRes: DmaRes;

  #[cfg(any(
    feature = "stm32l4r5",
    feature = "stm32l4r7",
    feature = "stm32l4r9",
    feature = "stm32l4s5",
    feature = "stm32l4s7",
    feature = "stm32l4s9"
  ))]
  /// DMAMUX resource.
  type DmamuxChRes: DmamuxChRes;

  /// Returns a reference to the DMA channel.
  fn dma_ch(&self) -> &Dma<Self::DmaRes>;

  #[cfg(any(
    feature = "stm32l4r5",
    feature = "stm32l4r7",
    feature = "stm32l4r9",
    feature = "stm32l4s5",
    feature = "stm32l4s7",
    feature = "stm32l4s9"
  ))]
  /// Returns a reference to the DMAMUX channel.
  fn dmamux_ch(&self) -> &DmamuxCh<Self::DmamuxChRes>;
}

#[cfg(any(
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
))]
/// DMA on register guard counters.
#[marker]
pub trait DmaBondOnRgc<T: DmaRes>
where
  Self: RegGuardCnt<DmaOn<T::RccRes>>,
  Self: RegGuardCnt<DmamuxOn<<T::DmamuxChRes as DmamuxChRes>::RccRes>>,
{
}

#[cfg(any(
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
))]
impl<T, U: DmaRes> DmaBondOnRgc<U> for T
where
  T: RegGuardCnt<DmaOn<U::RccRes>>,
  T: RegGuardCnt<DmamuxOn<<U::DmamuxChRes as DmamuxChRes>::RccRes>>,
{
}

#[cfg(not(any(
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
)))]
/// DMA on register guard counters.
#[marker]
pub trait DmaBondOnRgc<T: DmaRes>
where
  Self: RegGuardCnt<DmaOn<T::RccRes>>,
{
}

#[cfg(not(any(
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
)))]
impl<T, U: DmaRes> DmaBondOnRgc<U> for T where T: RegGuardCnt<DmaOn<U::RccRes>> {}

#[cfg(not(any(
  feature = "stm32l4x1",
  feature = "stm32l4x2",
  feature = "stm32l4x3",
  feature = "stm32l4x5",
  feature = "stm32l4x6"
)))]
/// DMA transmitter resource.
#[marker]
pub trait DmaTxRes<Rx>
where
  Rx: DmaRes,
  Self: DmaRes,
{
}

#[cfg(not(any(
  feature = "stm32l4x1",
  feature = "stm32l4x2",
  feature = "stm32l4x3",
  feature = "stm32l4x5",
  feature = "stm32l4x6"
)))]
impl<Rx, Tx> DmaTxRes<Rx> for Tx
where
  Rx: DmaRes,
  Tx: DmaRes,
{
}

#[cfg(any(
  feature = "stm32l4x1",
  feature = "stm32l4x2",
  feature = "stm32l4x3",
  feature = "stm32l4x5",
  feature = "stm32l4x6"
))]
/// DMA transmitter resource.
#[marker]
pub trait DmaTxRes<Rx>
where
  Rx: DmaRes,
  Self: DmaRes<Cselr = Rx::Cselr>,
{
}

#[cfg(any(
  feature = "stm32l4x1",
  feature = "stm32l4x2",
  feature = "stm32l4x3",
  feature = "stm32l4x5",
  feature = "stm32l4x6"
))]
impl<Rx, Tx> DmaTxRes<Rx> for Tx
where
  Rx: DmaRes,
  Tx: DmaRes<Cselr = Rx::Cselr>,
{
}

#[allow(missing_docs)]
impl<T: DmaRes> Dma<T> {
  #[inline(always)]
  pub fn int(&self) -> T::Int {
    self.0.int()
  }

  #[inline(always)]
  pub fn ccr(&self) -> &T::Ccr {
    self.0.ccr()
  }

  #[inline(always)]
  pub fn ccr_mem2mem(&self) -> &T::CcrMem2Mem {
    self.0.ccr_mem2mem()
  }

  #[inline(always)]
  pub fn ccr_msize(&self) -> &T::CcrMsize {
    self.0.ccr_msize()
  }

  #[inline(always)]
  pub fn ccr_psize(&self) -> &T::CcrPsize {
    self.0.ccr_psize()
  }

  #[inline(always)]
  pub fn ccr_minc(&self) -> &T::CcrMinc {
    self.0.ccr_minc()
  }

  #[inline(always)]
  pub fn ccr_pinc(&self) -> &T::CcrPinc {
    self.0.ccr_pinc()
  }

  #[inline(always)]
  pub fn ccr_circ(&self) -> &T::CcrCirc {
    self.0.ccr_circ()
  }

  #[inline(always)]
  pub fn ccr_dir(&self) -> &T::CcrDir {
    self.0.ccr_dir()
  }

  #[inline(always)]
  pub fn ccr_teie(&self) -> &T::CcrTeie {
    self.0.ccr_teie()
  }

  #[inline(always)]
  pub fn ccr_htie(&self) -> &T::CcrHtie {
    self.0.ccr_htie()
  }

  #[inline(always)]
  pub fn ccr_tcie(&self) -> &T::CcrTcie {
    self.0.ccr_tcie()
  }

  #[inline(always)]
  pub fn ccr_en(&self) -> &T::CcrEn {
    self.0.ccr_en()
  }

  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  #[inline(always)]
  pub fn cselr_cs(&self) -> &T::CselrCs {
    self.0.cselr_cs()
  }

  #[inline(always)]
  pub fn ifcr_cgif(&self) -> &<T::IfcrCgif as RegField<Crt>>::SRegField {
    self.0.ifcr_cgif().as_sync()
  }

  #[inline(always)]
  pub fn ifcr_chtif(&self) -> &<T::IfcrChtif as RegField<Crt>>::SRegField {
    self.0.ifcr_chtif().as_sync()
  }

  #[inline(always)]
  pub fn ifcr_ctcif(&self) -> &<T::IfcrCtcif as RegField<Crt>>::SRegField {
    self.0.ifcr_ctcif().as_sync()
  }

  #[inline(always)]
  pub fn ifcr_cteif(&self) -> &<T::IfcrCteif as RegField<Crt>>::SRegField {
    self.0.ifcr_cteif().as_sync()
  }

  #[inline(always)]
  pub fn isr_gif(&self) -> &<T::IsrGif as RegField<Crt>>::SRegField {
    self.0.isr_gif().as_sync()
  }

  #[inline(always)]
  pub fn isr_htif(&self) -> &<T::IsrHtif as RegField<Crt>>::SRegField {
    self.0.isr_htif().as_sync()
  }

  #[inline(always)]
  pub fn isr_tcif(&self) -> &<T::IsrTcif as RegField<Crt>>::SRegField {
    self.0.isr_tcif().as_sync()
  }

  #[inline(always)]
  pub fn isr_teif(&self) -> &<T::IsrTeif as RegField<Crt>>::SRegField {
    self.0.isr_teif().as_sync()
  }
}

impl<T: DmaRes> Dma<T> {
  /// Returns a number of data to transfer.
  #[inline]
  pub fn size(&self) -> usize {
    self.0.cndtr_ndt().read_bits() as usize
  }

  /// Sets the number of data to transfer.
  #[inline]
  pub fn set_size(&self, number: usize) {
    self.0.cndtr_ndt().write_bits(number as u32);
  }

  /// Returns a peripheral address.
  #[inline]
  pub fn paddr(&self) -> usize {
    self.0.cpar_pa().read_bits() as usize
  }

  /// Sets the peripheral address.
  ///
  /// # Safety
  ///
  /// `addr` must be a valid address, and must remain valid while DMA is
  /// enabled.
  #[inline]
  pub unsafe fn set_paddr(&self, addr: usize) {
    self.0.cpar_pa().write_bits(addr as u32);
  }

  /// Returns a memory address.
  #[inline]
  pub fn maddr(&self) -> usize {
    self.0.cmar_ma().read_bits() as usize
  }

  /// Sets the memory address.
  ///
  /// # Safety
  ///
  /// `addr` must be a valid address, and must remain valid while DMA is
  /// enabled.
  #[inline]
  pub unsafe fn set_maddr(&self, addr: usize) {
    self.0.cmar_ma().write_bits(addr as u32);
  }

  /// Returns a future, which resolves on DMA transfer complete event.
  pub fn transfer_complete(
    &self,
  ) -> impl Future<Item = (), Error = DmaTransferError> {
    let teif = *self.0.isr_teif();
    let tcif = *self.0.isr_tcif();
    let cgif = *self.0.ifcr_cgif();
    let ctcif = *self.0.ifcr_ctcif();
    self.0.int().add_future(fib::new(move || loop {
      if teif.read_bit_band() {
        cgif.set_bit_band();
        break Err(DmaTransferError);
      }
      if tcif.read_bit_band() {
        ctcif.set_bit_band();
        break Ok(());
      }
      yield;
    }))
  }

  /// Returns a future, which resolves on DMA half transfer event.
  pub fn half_transfer(
    &self,
  ) -> impl Future<Item = (), Error = DmaTransferError> {
    let teif = *self.0.isr_teif();
    let htif = *self.0.isr_htif();
    let cgif = *self.0.ifcr_cgif();
    let chtif = *self.0.ifcr_chtif();
    self.0.int().add_future(fib::new(move || loop {
      if teif.read_bit_band() {
        cgif.set_bit_band();
        break Err(DmaTransferError);
      }
      if htif.read_bit_band() {
        chtif.set_bit_band();
        break Ok(());
      }
      yield;
    }))
  }
}

impl<T, C> DmaRcc<T, C>
where
  T: DmaRccRes,
  C: RegGuardCnt<DmaOn<T>>,
{
  /// Enables the clock.
  pub fn on(&self) -> RegGuard<DmaOn<T>, C> {
    RegGuard::new(DmaOn(*self.0.en()))
  }
}

impl<T: DmaRccRes> Clone for DmaOn<T> {
  #[inline(always)]
  fn clone(&self) -> Self {
    DmaOn(self.0)
  }
}

impl<T: DmaRccRes> RegGuardRes for DmaOn<T> {
  type Reg = T::RccAhb1Enr;
  type Field = T::RccAhb1EnrDmaEn;

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
macro_rules! dma {
  (
    $doc:expr,
    $name:ident,
    $name_macro:ident,
    $doc_res:expr,
    $name_res:ident,
    $doc_on:expr,
    $name_on:ident,
    $dmaen_ty:ident,
    $dmaen:ident,
    $dma_on:ident,
    $dmamux_on:ident,
    $rcc_ahb1enr_dmaen:ident,
    $(
      $(#[$attr_ch:meta])*
      (
        $doc_ch:expr,
        $name_ch:ident,
        $name_ch_macro:ident,
        $doc_ch_res:expr,
        $name_ch_res:ident,
        $doc_ch_bond:expr,
        $name_ch_bond:ident,
        $dmamux_ch_res:ident,
        $int_ty:ident,
        $ccr_ty:ident,
        $cmar_ty:ident,
        $cndtr_ty:ident,
        $cpar_ty:ident,
        $cs_ty:ident,
        $cgif_ty:ident,
        $chtif_ty:ident,
        $ctcif_ty:ident,
        $cteif_ty:ident,
        $gif_ty:ident,
        $htif_ty:ident,
        $tcif_ty:ident,
        $teif_ty:ident,
        $dma_ch:ident,
        $dma:ident,
        $dma_ccr:ident,
        $dma_cmar:ident,
        $dma_cndtr:ident,
        $dma_cpar:ident,
        $dma_cselr:ident,
        $dma_ifcr:ident,
        $dma_isr:ident,
        $dma_cselr_cs:ident,
        $dma_ifcr_cgif:ident,
        $dma_ifcr_chtif:ident,
        $dma_ifcr_ctcif:ident,
        $dma_ifcr_cteif:ident,
        $dma_isr_gif:ident,
        $dma_isr_htif:ident,
        $dma_isr_tcif:ident,
        $dma_isr_teif:ident,
        $dmamux_ch:ident,
        $ccr_path:ident,
        $cmar_path:ident,
        $cndtr_path:ident,
        $cpar_path:ident,
        $cs:ident,
        $cgif:ident,
        $chtif:ident,
        $ctcif:ident,
        $cteif:ident,
        $gif:ident,
        $htif:ident,
        $tcif:ident,
        $teif:ident,
      ),
    )*
  ) => {
    #[doc = $doc]
    pub type $name<C> = DmaRcc<$name_res<Crt>, C>;

    #[doc = $doc_res]
    #[allow(missing_docs)]
    pub struct $name_res<Rt: RegTag> {
      pub $rcc_ahb1enr_dmaen: rcc::ahb1enr::$dmaen_ty<Rt>,
    }

    #[doc = $doc_on]
    pub type $name_on = DmaOn<$name_res<Crt>>;

    /// Creates a new `DmaRcc`.
    #[macro_export]
    macro_rules! $name_macro {
      ($reg:ident, $rgc:path) => {
        <$crate::dma::DmaRcc<_, $rgc> as ::drone_core::drv::Driver>::new(
          $crate::dma::$name_res {
            $rcc_ahb1enr_dmaen: $reg.rcc_ahb1enr.$dmaen,
          },
        )
      };
    }

    impl Resource for $name_res<Crt> {
      type Source = $name_res<Srt>;

      #[inline(always)]
      fn from_source(source: Self::Source) -> Self {
        Self {
          $rcc_ahb1enr_dmaen: source.$rcc_ahb1enr_dmaen.to_copy(),
        }
      }
    }

    impl DmaRccRes for $name_res<Crt> {
      type RccAhb1EnrVal = rcc::ahb1enr::Val;
      type RccAhb1Enr = rcc::ahb1enr::Reg<Crt>;
      type RccAhb1EnrDmaEn = rcc::ahb1enr::$dmaen_ty<Crt>;

      res_impl!(RccAhb1EnrDmaEn, en, $rcc_ahb1enr_dmaen);
    }

    $(
      $(#[$attr_ch])*
      #[doc = $doc_ch]
      pub type $name_ch<I> = Dma<$name_ch_res<I, Crt>>;

      $(#[$attr_ch])*
      #[doc = $doc_ch_res]
      #[allow(missing_docs)]
      pub struct $name_ch_res<I: $int_ty<Rtt>, Rt: RegTag> {
        pub $dma_ch: I,
        pub $dma_ccr: $dma::$ccr_ty<Srt>,
        pub $dma_cmar: $dma::$cmar_ty<Srt>,
        pub $dma_cndtr: $dma::$cndtr_ty<Srt>,
        pub $dma_cpar: $dma::$cpar_ty<Srt>,
        #[cfg(any(
          feature = "stm32l4x1",
          feature = "stm32l4x2",
          feature = "stm32l4x3",
          feature = "stm32l4x5",
          feature = "stm32l4x6"
        ))]
        pub $dma_cselr_cs: $dma::cselr::$cs_ty<Srt>,
        pub $dma_ifcr_cgif: $dma::ifcr::$cgif_ty<Rt>,
        pub $dma_ifcr_chtif: $dma::ifcr::$chtif_ty<Rt>,
        pub $dma_ifcr_ctcif: $dma::ifcr::$ctcif_ty<Rt>,
        pub $dma_ifcr_cteif: $dma::ifcr::$cteif_ty<Rt>,
        pub $dma_isr_gif: $dma::isr::$gif_ty<Rt>,
        pub $dma_isr_htif: $dma::isr::$htif_ty<Rt>,
        pub $dma_isr_tcif: $dma::isr::$tcif_ty<Rt>,
        pub $dma_isr_teif: $dma::isr::$teif_ty<Rt>,
      }

      $(#[$attr_ch])*
      #[doc = $doc_ch_bond]
      #[allow(missing_docs)]
      pub struct $name_ch_bond<I, C>
      where
        I: $int_ty<Rtt>,
        C: DmaBondOnRgc<$name_ch_res<I, Crt>>,
      {
        pub $dma_ch: $name_ch<I>,
        pub $dma_on: RegGuard<$name_on, C>,
        #[cfg(any(
          feature = "stm32l4r5",
          feature = "stm32l4r7",
          feature = "stm32l4r9",
          feature = "stm32l4s5",
          feature = "stm32l4s7",
          feature = "stm32l4s9"
        ))]
        pub $dmamux_ch: DmamuxCh<$dmamux_ch_res>,
        #[cfg(any(
          feature = "stm32l4r5",
          feature = "stm32l4r7",
          feature = "stm32l4r9",
          feature = "stm32l4s5",
          feature = "stm32l4s7",
          feature = "stm32l4s9"
        ))]
        pub $dmamux_on: RegGuard<
          DmamuxOn<<$dmamux_ch_res as DmamuxChRes>::RccRes>,
          C,
        >,
      }

      $(#[$attr_ch])*
      #[cfg(any(
        feature = "stm32l4x1",
        feature = "stm32l4x2",
        feature = "stm32l4x3",
        feature = "stm32l4x5",
        feature = "stm32l4x6"
      ))]
      /// Creates a new `Dma`.
      #[macro_export]
      macro_rules! $name_ch_macro {
        ($reg: ident, $thr: ident) => {
          <$crate::dma::Dma<_> as ::drone_core::drv::Driver>::new(
            $crate::dma::$name_ch_res {
              $dma_ch: $thr.$dma_ch.to_regular(),
              $dma_ccr: $reg.$dma_ccr,
              $dma_cmar: $reg.$dma_cmar,
              $dma_cndtr: $reg.$dma_cndtr,
              $dma_cpar: $reg.$dma_cpar,
              $dma_cselr_cs: $reg.$dma_cselr.$cs,
              $dma_ifcr_cgif: $reg.$dma_ifcr.$cgif,
              $dma_ifcr_chtif: $reg.$dma_ifcr.$chtif,
              $dma_ifcr_ctcif: $reg.$dma_ifcr.$ctcif,
              $dma_ifcr_cteif: $reg.$dma_ifcr.$cteif,
              $dma_isr_gif: $reg.$dma_isr.$gif,
              $dma_isr_htif: $reg.$dma_isr.$htif,
              $dma_isr_tcif: $reg.$dma_isr.$tcif,
              $dma_isr_teif: $reg.$dma_isr.$teif,
            },
          )
        };
      }

      $(#[$attr_ch])*
      #[cfg(not(any(
        feature = "stm32l4x1",
        feature = "stm32l4x2",
        feature = "stm32l4x3",
        feature = "stm32l4x5",
        feature = "stm32l4x6"
      )))]
      /// Creates a new `Dma`.
      #[macro_export]
      macro_rules! $name_ch_macro {
        ($reg: ident, $thr: ident) => {
          <$crate::dma::Dma<_> as ::drone_core::drv::Driver>::new(
            $crate::dma::$name_ch_res {
              $dma_ch: $thr.$dma_ch.to_regular(),
              $dma_ccr: $reg.$dma_ccr,
              $dma_cmar: $reg.$dma_cmar,
              $dma_cndtr: $reg.$dma_cndtr,
              $dma_cpar: $reg.$dma_cpar,
              $dma_ifcr_cgif: $reg.$dma_ifcr.$cgif,
              $dma_ifcr_chtif: $reg.$dma_ifcr.$chtif,
              $dma_ifcr_ctcif: $reg.$dma_ifcr.$ctcif,
              $dma_ifcr_cteif: $reg.$dma_ifcr.$cteif,
              $dma_isr_gif: $reg.$dma_isr.$gif,
              $dma_isr_htif: $reg.$dma_isr.$htif,
              $dma_isr_tcif: $reg.$dma_isr.$tcif,
              $dma_isr_teif: $reg.$dma_isr.$teif,
            },
          )
        };
      }

      $(#[$attr_ch])*
      impl<I: $int_ty<Rtt>> Resource for $name_ch_res<I, Crt> {
        type Source = $name_ch_res<I, Srt>;

        #[inline(always)]
        fn from_source(source: Self::Source) -> Self {
          Self {
            $dma_ch: source.$dma_ch,
            $dma_ccr: source.$dma_ccr,
            $dma_cmar: source.$dma_cmar,
            $dma_cndtr: source.$dma_cndtr,
            $dma_cpar: source.$dma_cpar,
            #[cfg(any(
              feature = "stm32l4x1",
              feature = "stm32l4x2",
              feature = "stm32l4x3",
              feature = "stm32l4x5",
              feature = "stm32l4x6"
            ))]
            $dma_cselr_cs: source.$dma_cselr_cs,
            $dma_ifcr_cgif: source.$dma_ifcr_cgif.to_copy(),
            $dma_ifcr_chtif: source.$dma_ifcr_chtif.to_copy(),
            $dma_ifcr_ctcif: source.$dma_ifcr_ctcif.to_copy(),
            $dma_ifcr_cteif: source.$dma_ifcr_cteif.to_copy(),
            $dma_isr_gif: source.$dma_isr_gif.to_copy(),
            $dma_isr_htif: source.$dma_isr_htif.to_copy(),
            $dma_isr_tcif: source.$dma_isr_tcif.to_copy(),
            $dma_isr_teif: source.$dma_isr_teif.to_copy(),
          }
        }
      }

      $(#[$attr_ch])*
      impl<I: $int_ty<Rtt>> DmaRes for $name_ch_res<I, Crt> {
        type RccRes = $name_res<Crt>;
        type Int = I;
        type CmarVal = $dma::$cmar_path::Val;
        type Cmar = $dma::$cmar_ty<Srt>;
        type CmarMa = $dma::$cmar_path::Ma<Srt>;
        type CndtrVal = $dma::$cndtr_path::Val;
        type Cndtr = $dma::$cndtr_ty<Srt>;
        type CndtrNdt = $dma::$cndtr_path::Ndt<Srt>;
        type CparVal = $dma::$cpar_path::Val;
        type Cpar = $dma::$cpar_ty<Srt>;
        type CparPa = $dma::$cpar_path::Pa<Srt>;
        #[cfg(any(
          feature = "stm32l4x1",
          feature = "stm32l4x2",
          feature = "stm32l4x3",
          feature = "stm32l4x5",
          feature = "stm32l4x6"
        ))]
        type Cselr = $dma::cselr::Reg<Srt>;
        #[cfg(any(
          feature = "stm32l4x1",
          feature = "stm32l4x2",
          feature = "stm32l4x3",
          feature = "stm32l4x5",
          feature = "stm32l4x6"
        ))]
        type CselrCs = $dma::cselr::$cs_ty<Srt>;

        #[cfg(any(
          feature = "stm32l4r5",
          feature = "stm32l4r7",
          feature = "stm32l4r9",
          feature = "stm32l4s5",
          feature = "stm32l4s7",
          feature = "stm32l4s9"
        ))]
        type DmamuxChRes = $dmamux_ch_res;

        #[inline(always)]
        fn int(&self) -> Self::Int {
          self.$dma_ch
        }

        res_impl!(Cmar, cmar, $dma_cmar);
        res_impl!(CmarMa, cmar_ma, $dma_cmar.ma);
        res_impl!(Cndtr, cndtr, $dma_cndtr);
        res_impl!(CndtrNdt, cndtr_ndt, $dma_cndtr.ndt);
        res_impl!(Cpar, cpar, $dma_cpar);
        res_impl!(CparPa, cpar_pa, $dma_cpar.pa);
        #[cfg(any(
          feature = "stm32l4x1",
          feature = "stm32l4x2",
          feature = "stm32l4x3",
          feature = "stm32l4x5",
          feature = "stm32l4x6"
        ))]
        res_impl!(CselrCs, cselr_cs, $dma_cselr_cs);
      }

      $(#[$attr_ch])*
      impl<I: $int_ty<Rtt>> DmaResCcr for $name_ch_res<I, Crt> {
        type CcrVal = $dma::$ccr_path::Val;
        type Ccr = $dma::$ccr_ty<Srt>;
        type CcrMem2Mem = $dma::$ccr_path::Mem2Mem<Srt>;
        type CcrMsize = $dma::$ccr_path::Msize<Srt>;
        type CcrPsize = $dma::$ccr_path::Psize<Srt>;
        type CcrMinc = $dma::$ccr_path::Minc<Srt>;
        type CcrPinc = $dma::$ccr_path::Pinc<Srt>;
        type CcrCirc = $dma::$ccr_path::Circ<Srt>;
        type CcrDir = $dma::$ccr_path::Dir<Srt>;
        type CcrTeie = $dma::$ccr_path::Teie<Srt>;
        type CcrHtie = $dma::$ccr_path::Htie<Srt>;
        type CcrTcie = $dma::$ccr_path::Tcie<Srt>;
        type CcrEn = $dma::$ccr_path::En<Srt>;

        res_impl!(Ccr, ccr, $dma_ccr);
        res_impl!(CcrMem2Mem, ccr_mem2mem, $dma_ccr.mem2mem);
        res_impl!(CcrMsize, ccr_msize, $dma_ccr.msize);
        res_impl!(CcrPsize, ccr_psize, $dma_ccr.psize);
        res_impl!(CcrMinc, ccr_minc, $dma_ccr.minc);
        res_impl!(CcrPinc, ccr_pinc, $dma_ccr.pinc);
        res_impl!(CcrCirc, ccr_circ, $dma_ccr.circ);
        res_impl!(CcrDir, ccr_dir, $dma_ccr.dir);
        res_impl!(CcrTeie, ccr_teie, $dma_ccr.teie);
        res_impl!(CcrHtie, ccr_htie, $dma_ccr.htie);
        res_impl!(CcrTcie, ccr_tcie, $dma_ccr.tcie);
        res_impl!(CcrEn, ccr_en, $dma_ccr.en);
      }

      $(#[$attr_ch])*
      impl<I: $int_ty<Rtt>> DmaResIfcr for $name_ch_res<I, Crt> {
        type Ifcr = $dma::Ifcr<Crt>;
        type IfcrCgif = $dma::ifcr::$cgif_ty<Crt>;
        type IfcrChtif = $dma::ifcr::$chtif_ty<Crt>;
        type IfcrCtcif = $dma::ifcr::$ctcif_ty<Crt>;
        type IfcrCteif = $dma::ifcr::$cteif_ty<Crt>;

        res_impl!(IfcrCgif, ifcr_cgif, $dma_ifcr_cgif);
        res_impl!(IfcrChtif, ifcr_chtif, $dma_ifcr_chtif);
        res_impl!(IfcrCtcif, ifcr_ctcif, $dma_ifcr_ctcif);
        res_impl!(IfcrCteif, ifcr_cteif, $dma_ifcr_cteif);
      }

      $(#[$attr_ch])*
      impl<I: $int_ty<Rtt>> DmaResIsr for $name_ch_res<I, Crt> {
        type Isr = $dma::Isr<Crt>;
        type IsrGif = $dma::isr::$gif_ty<Crt>;
        type IsrHtif = $dma::isr::$htif_ty<Crt>;
        type IsrTcif = $dma::isr::$tcif_ty<Crt>;
        type IsrTeif = $dma::isr::$teif_ty<Crt>;

        res_impl!(IsrGif, isr_gif, $dma_isr_gif);
        res_impl!(IsrHtif, isr_htif, $dma_isr_htif);
        res_impl!(IsrTcif, isr_tcif, $dma_isr_tcif);
        res_impl!(IsrTeif, isr_teif, $dma_isr_teif);
      }

      impl<I, C> DmaBond for $name_ch_bond<I, C>
      where
        I: $int_ty<Rtt>,
        C: DmaBondOnRgc<$name_ch_res<I, Crt>>,
      {
        type Rgc = C;

        type DmaRes = $name_ch_res<I, Crt>;

        #[cfg(any(
          feature = "stm32l4r5",
          feature = "stm32l4r7",
          feature = "stm32l4r9",
          feature = "stm32l4s5",
          feature = "stm32l4s7",
          feature = "stm32l4s9"
        ))]
        type DmamuxChRes = $dmamux_ch_res;

        #[inline(always)]
        fn dma_ch(&self) -> &Dma<Self::DmaRes> {
          &self.$dma_ch
        }

        #[cfg(any(
          feature = "stm32l4r5",
          feature = "stm32l4r7",
          feature = "stm32l4r9",
          feature = "stm32l4s5",
          feature = "stm32l4s7",
          feature = "stm32l4s9"
        ))]
        #[inline(always)]
        fn dmamux_ch(&self) -> &DmamuxCh<Self::DmamuxChRes> {
          &self.$dmamux_ch
        }
      }
    )*
  };
}

#[cfg(any(
  feature = "stm32f100",
  feature = "stm32f101",
  feature = "stm32f102",
  feature = "stm32f103",
  feature = "stm32f107",
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
dma! {
  "DMA1 reset and clock control driver.",
  Dma1Rcc,
  drv_dma1_rcc,
  "DMA1 reset and clock control resource.",
  Dma1RccRes,
  "DMA1 clock on guard resource.",
  Dma1On,
  Dma1En,
  dma1en,
  dma1_on,
  dmamux1_on,
  rcc_ahb1enr_dma1en,
  (
    "DMA1 Channel 1 driver.",
    Dma1Ch1,
    drv_dma1_ch1,
    "DMA1 Channel 1 resource.",
    Dma1Ch1Res,
    "DMA1 Channel 1 bond.",
    Dma1Ch1Bond,
    Dmamux1Ch0Res,
    IntDma1Ch1,
    Ccr1,
    Cmar1,
    Cndtr1,
    Cpar1,
    C1S,
    Cgif1,
    Chtif1,
    Ctcif1,
    Cteif1,
    Gif1,
    Htif1,
    Tcif1,
    Teif1,
    dma1_ch1,
    dma1,
    dma1_ccr1,
    dma1_cmar1,
    dma1_cndtr1,
    dma1_cpar1,
    dma1_cselr,
    dma1_ifcr,
    dma1_isr,
    dma1_cselr_c1s,
    dma1_ifcr_cgif1,
    dma1_ifcr_chtif1,
    dma1_ifcr_ctcif1,
    dma1_ifcr_cteif1,
    dma1_isr_gif1,
    dma1_isr_htif1,
    dma1_isr_tcif1,
    dma1_isr_teif1,
    dmamux1_ch0,
    ccr1,
    cmar1,
    cndtr1,
    cpar1,
    c1s,
    cgif1,
    chtif1,
    ctcif1,
    cteif1,
    gif1,
    htif1,
    tcif1,
    teif1,
  ),
  (
    "DMA1 Channel 2 driver.",
    Dma1Ch2,
    drv_dma1_ch2,
    "DMA1 Channel 2 resource.",
    Dma1Ch2Res,
    "DMA1 Channel 2 bond.",
    Dma1Ch2Bond,
    Dmamux1Ch1Res,
    IntDma1Ch2,
    Ccr2,
    Cmar2,
    Cndtr2,
    Cpar2,
    C2S,
    Cgif2,
    Chtif2,
    Ctcif2,
    Cteif2,
    Gif2,
    Htif2,
    Tcif2,
    Teif2,
    dma1_ch2,
    dma1,
    dma1_ccr2,
    dma1_cmar2,
    dma1_cndtr2,
    dma1_cpar2,
    dma1_cselr,
    dma1_ifcr,
    dma1_isr,
    dma1_cselr_c2s,
    dma1_ifcr_cgif2,
    dma1_ifcr_chtif2,
    dma1_ifcr_ctcif2,
    dma1_ifcr_cteif2,
    dma1_isr_gif2,
    dma1_isr_htif2,
    dma1_isr_tcif2,
    dma1_isr_teif2,
    dmamux1_ch1,
    ccr2,
    cmar2,
    cndtr2,
    cpar2,
    c2s,
    cgif2,
    chtif2,
    ctcif2,
    cteif2,
    gif2,
    htif2,
    tcif2,
    teif2,
  ),
  (
    "DMA1 Channel 3 driver.",
    Dma1Ch3,
    drv_dma1_ch3,
    "DMA1 Channel 3 resource.",
    Dma1Ch3Res,
    "DMA1 Channel 3 bond.",
    Dma1Ch3Bond,
    Dmamux1Ch2Res,
    IntDma1Ch3,
    Ccr3,
    Cmar3,
    Cndtr3,
    Cpar3,
    C3S,
    Cgif3,
    Chtif3,
    Ctcif3,
    Cteif3,
    Gif3,
    Htif3,
    Tcif3,
    Teif3,
    dma1_ch3,
    dma1,
    dma1_ccr3,
    dma1_cmar3,
    dma1_cndtr3,
    dma1_cpar3,
    dma1_cselr,
    dma1_ifcr,
    dma1_isr,
    dma1_cselr_c3s,
    dma1_ifcr_cgif3,
    dma1_ifcr_chtif3,
    dma1_ifcr_ctcif3,
    dma1_ifcr_cteif3,
    dma1_isr_gif3,
    dma1_isr_htif3,
    dma1_isr_tcif3,
    dma1_isr_teif3,
    dmamux1_ch2,
    ccr3,
    cmar3,
    cndtr3,
    cpar3,
    c3s,
    cgif3,
    chtif3,
    ctcif3,
    cteif3,
    gif3,
    htif3,
    tcif3,
    teif3,
  ),
  (
    "DMA1 Channel 4 driver.",
    Dma1Ch4,
    drv_dma1_ch4,
    "DMA1 Channel 4 resource.",
    Dma1Ch4Res,
    "DMA1 Channel 4 bond.",
    Dma1Ch4Bond,
    Dmamux1Ch3Res,
    IntDma1Ch4,
    Ccr4,
    Cmar4,
    Cndtr4,
    Cpar4,
    C4S,
    Cgif4,
    Chtif4,
    Ctcif4,
    Cteif4,
    Gif4,
    Htif4,
    Tcif4,
    Teif4,
    dma1_ch4,
    dma1,
    dma1_ccr4,
    dma1_cmar4,
    dma1_cndtr4,
    dma1_cpar4,
    dma1_cselr,
    dma1_ifcr,
    dma1_isr,
    dma1_cselr_c4s,
    dma1_ifcr_cgif4,
    dma1_ifcr_chtif4,
    dma1_ifcr_ctcif4,
    dma1_ifcr_cteif4,
    dma1_isr_gif4,
    dma1_isr_htif4,
    dma1_isr_tcif4,
    dma1_isr_teif4,
    dmamux1_ch3,
    ccr4,
    cmar4,
    cndtr4,
    cpar4,
    c4s,
    cgif4,
    chtif4,
    ctcif4,
    cteif4,
    gif4,
    htif4,
    tcif4,
    teif4,
  ),
  (
    "DMA1 Channel 5 driver.",
    Dma1Ch5,
    drv_dma1_ch5,
    "DMA1 Channel 5 resource.",
    Dma1Ch5Res,
    "DMA1 Channel 5 bond.",
    Dma1Ch5Bond,
    Dmamux1Ch4Res,
    IntDma1Ch5,
    Ccr5,
    Cmar5,
    Cndtr5,
    Cpar5,
    C5S,
    Cgif5,
    Chtif5,
    Ctcif5,
    Cteif5,
    Gif5,
    Htif5,
    Tcif5,
    Teif5,
    dma1_ch5,
    dma1,
    dma1_ccr5,
    dma1_cmar5,
    dma1_cndtr5,
    dma1_cpar5,
    dma1_cselr,
    dma1_ifcr,
    dma1_isr,
    dma1_cselr_c5s,
    dma1_ifcr_cgif5,
    dma1_ifcr_chtif5,
    dma1_ifcr_ctcif5,
    dma1_ifcr_cteif5,
    dma1_isr_gif5,
    dma1_isr_htif5,
    dma1_isr_tcif5,
    dma1_isr_teif5,
    dmamux1_ch4,
    ccr5,
    cmar5,
    cndtr5,
    cpar5,
    c5s,
    cgif5,
    chtif5,
    ctcif5,
    cteif5,
    gif5,
    htif5,
    tcif5,
    teif5,
  ),
  (
    "DMA1 Channel 6 driver.",
    Dma1Ch6,
    drv_dma1_ch6,
    "DMA1 Channel 6 resource.",
    Dma1Ch6Res,
    "DMA1 Channel 6 bond.",
    Dma1Ch6Bond,
    Dmamux1Ch5Res,
    IntDma1Ch6,
    Ccr6,
    Cmar6,
    Cndtr6,
    Cpar6,
    C6S,
    Cgif6,
    Chtif6,
    Ctcif6,
    Cteif6,
    Gif6,
    Htif6,
    Tcif6,
    Teif6,
    dma1_ch6,
    dma1,
    dma1_ccr6,
    dma1_cmar6,
    dma1_cndtr6,
    dma1_cpar6,
    dma1_cselr,
    dma1_ifcr,
    dma1_isr,
    dma1_cselr_c6s,
    dma1_ifcr_cgif6,
    dma1_ifcr_chtif6,
    dma1_ifcr_ctcif6,
    dma1_ifcr_cteif6,
    dma1_isr_gif6,
    dma1_isr_htif6,
    dma1_isr_tcif6,
    dma1_isr_teif6,
    dmamux1_ch5,
    ccr6,
    cmar6,
    cndtr6,
    cpar6,
    c6s,
    cgif6,
    chtif6,
    ctcif6,
    cteif6,
    gif6,
    htif6,
    tcif6,
    teif6,
  ),
  (
    "DMA1 Channel 7 driver.",
    Dma1Ch7,
    drv_dma1_ch7,
    "DMA1 Channel 7 resource.",
    Dma1Ch7Res,
    "DMA1 Channel 7 bond.",
    Dma1Ch7Bond,
    Dmamux1Ch6Res,
    IntDma1Ch7,
    Ccr7,
    Cmar7,
    Cndtr7,
    Cpar7,
    C7S,
    Cgif7,
    Chtif7,
    Ctcif7,
    Cteif7,
    Gif7,
    Htif7,
    Tcif7,
    Teif7,
    dma1_ch7,
    dma1,
    dma1_ccr7,
    dma1_cmar7,
    dma1_cndtr7,
    dma1_cpar7,
    dma1_cselr,
    dma1_ifcr,
    dma1_isr,
    dma1_cselr_c7s,
    dma1_ifcr_cgif7,
    dma1_ifcr_chtif7,
    dma1_ifcr_ctcif7,
    dma1_ifcr_cteif7,
    dma1_isr_gif7,
    dma1_isr_htif7,
    dma1_isr_tcif7,
    dma1_isr_teif7,
    dmamux1_ch6,
    ccr7,
    cmar7,
    cndtr7,
    cpar7,
    c7s,
    cgif7,
    chtif7,
    ctcif7,
    cteif7,
    gif7,
    htif7,
    tcif7,
    teif7,
  ),
}

#[cfg(any(
  feature = "stm32f100",
  feature = "stm32f101",
  feature = "stm32f102",
  feature = "stm32f103",
  feature = "stm32f107",
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
dma! {
  "DMA2 reset and clock control driver.",
  Dma2Rcc,
  drv_dma2_rcc,
  "DMA2 reset and clock control resource.",
  Dma2RccRes,
  "DMA2 clock on guard resource.",
  Dma2On,
  Dma2En,
  dma2en,
  dma2_on,
  dmamux1_on,
  rcc_ahb1enr_dma2en,
  (
    "DMA2 Channel 1 driver.",
    Dma2Ch1,
    drv_dma2_ch1,
    "DMA2 Channel 1 resource.",
    Dma2Ch1Res,
    "DMA2 Channel 1 bond.",
    Dma2Ch1Bond,
    Dmamux1Ch7Res,
    IntDma2Ch1,
    Ccr1,
    Cmar1,
    Cndtr1,
    Cpar1,
    C1S,
    Cgif1,
    Chtif1,
    Ctcif1,
    Cteif1,
    Gif1,
    Htif1,
    Tcif1,
    Teif1,
    dma2_ch1,
    dma2,
    dma2_ccr1,
    dma2_cmar1,
    dma2_cndtr1,
    dma2_cpar1,
    dma2_cselr,
    dma2_ifcr,
    dma2_isr,
    dma2_cselr_c1s,
    dma2_ifcr_cgif1,
    dma2_ifcr_chtif1,
    dma2_ifcr_ctcif1,
    dma2_ifcr_cteif1,
    dma2_isr_gif1,
    dma2_isr_htif1,
    dma2_isr_tcif1,
    dma2_isr_teif1,
    dmamux1_ch7,
    ccr1,
    cmar1,
    cndtr1,
    cpar1,
    c1s,
    cgif1,
    chtif1,
    ctcif1,
    cteif1,
    gif1,
    htif1,
    tcif1,
    teif1,
  ),
  (
    "DMA2 Channel 2 driver.",
    Dma2Ch2,
    drv_dma2_ch2,
    "DMA2 Channel 2 resource.",
    Dma2Ch2Res,
    "DMA2 Channel 2 bond.",
    Dma2Ch2Bond,
    Dmamux1Ch8Res,
    IntDma2Ch2,
    Ccr2,
    Cmar2,
    Cndtr2,
    Cpar2,
    C2S,
    Cgif2,
    Chtif2,
    Ctcif2,
    Cteif2,
    Gif2,
    Htif2,
    Tcif2,
    Teif2,
    dma2_ch2,
    dma2,
    dma2_ccr2,
    dma2_cmar2,
    dma2_cndtr2,
    dma2_cpar2,
    dma2_cselr,
    dma2_ifcr,
    dma2_isr,
    dma2_cselr_c2s,
    dma2_ifcr_cgif2,
    dma2_ifcr_chtif2,
    dma2_ifcr_ctcif2,
    dma2_ifcr_cteif2,
    dma2_isr_gif2,
    dma2_isr_htif2,
    dma2_isr_tcif2,
    dma2_isr_teif2,
    dmamux1_ch8,
    ccr2,
    cmar2,
    cndtr2,
    cpar2,
    c2s,
    cgif2,
    chtif2,
    ctcif2,
    cteif2,
    gif2,
    htif2,
    tcif2,
    teif2,
  ),
  (
    "DMA2 Channel 3 driver.",
    Dma2Ch3,
    drv_dma2_ch3,
    "DMA2 Channel 3 resource.",
    Dma2Ch3Res,
    "DMA2 Channel 3 bond.",
    Dma2Ch3Bond,
    Dmamux1Ch9Res,
    IntDma2Ch3,
    Ccr3,
    Cmar3,
    Cndtr3,
    Cpar3,
    C3S,
    Cgif3,
    Chtif3,
    Ctcif3,
    Cteif3,
    Gif3,
    Htif3,
    Tcif3,
    Teif3,
    dma2_ch3,
    dma2,
    dma2_ccr3,
    dma2_cmar3,
    dma2_cndtr3,
    dma2_cpar3,
    dma2_cselr,
    dma2_ifcr,
    dma2_isr,
    dma2_cselr_c3s,
    dma2_ifcr_cgif3,
    dma2_ifcr_chtif3,
    dma2_ifcr_ctcif3,
    dma2_ifcr_cteif3,
    dma2_isr_gif3,
    dma2_isr_htif3,
    dma2_isr_tcif3,
    dma2_isr_teif3,
    dmamux1_ch9,
    ccr3,
    cmar3,
    cndtr3,
    cpar3,
    c3s,
    cgif3,
    chtif3,
    ctcif3,
    cteif3,
    gif3,
    htif3,
    tcif3,
    teif3,
  ),
  (
    "DMA2 Channel 4 driver.",
    Dma2Ch4,
    drv_dma2_ch4,
    "DMA2 Channel 4 resource.",
    Dma2Ch4Res,
    "DMA2 Channel 4 bond.",
    Dma2Ch4Bond,
    Dmamux1Ch10Res,
    IntDma2Ch4,
    Ccr4,
    Cmar4,
    Cndtr4,
    Cpar4,
    C4S,
    Cgif4,
    Chtif4,
    Ctcif4,
    Cteif4,
    Gif4,
    Htif4,
    Tcif4,
    Teif4,
    dma2_ch4,
    dma2,
    dma2_ccr4,
    dma2_cmar4,
    dma2_cndtr4,
    dma2_cpar4,
    dma2_cselr,
    dma2_ifcr,
    dma2_isr,
    dma2_cselr_c4s,
    dma2_ifcr_cgif4,
    dma2_ifcr_chtif4,
    dma2_ifcr_ctcif4,
    dma2_ifcr_cteif4,
    dma2_isr_gif4,
    dma2_isr_htif4,
    dma2_isr_tcif4,
    dma2_isr_teif4,
    dmamux1_ch10,
    ccr4,
    cmar4,
    cndtr4,
    cpar4,
    c4s,
    cgif4,
    chtif4,
    ctcif4,
    cteif4,
    gif4,
    htif4,
    tcif4,
    teif4,
  ),
  (
    "DMA2 Channel 5 driver.",
    Dma2Ch5,
    drv_dma2_ch5,
    "DMA2 Channel 5 resource.",
    Dma2Ch5Res,
    "DMA2 Channel 5 bond.",
    Dma2Ch5Bond,
    Dmamux1Ch11Res,
    IntDma2Ch5,
    Ccr5,
    Cmar5,
    Cndtr5,
    Cpar5,
    C5S,
    Cgif5,
    Chtif5,
    Ctcif5,
    Cteif5,
    Gif5,
    Htif5,
    Tcif5,
    Teif5,
    dma2_ch5,
    dma2,
    dma2_ccr5,
    dma2_cmar5,
    dma2_cndtr5,
    dma2_cpar5,
    dma2_cselr,
    dma2_ifcr,
    dma2_isr,
    dma2_cselr_c5s,
    dma2_ifcr_cgif5,
    dma2_ifcr_chtif5,
    dma2_ifcr_ctcif5,
    dma2_ifcr_cteif5,
    dma2_isr_gif5,
    dma2_isr_htif5,
    dma2_isr_tcif5,
    dma2_isr_teif5,
    dmamux1_ch11,
    ccr5,
    cmar5,
    cndtr5,
    cpar5,
    c5s,
    cgif5,
    chtif5,
    ctcif5,
    cteif5,
    gif5,
    htif5,
    tcif5,
    teif5,
  ),
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
  (
    "DMA2 Channel 6 driver.",
    Dma2Ch6,
    drv_dma2_ch6,
    "DMA2 Channel 6 resource.",
    Dma2Ch6Res,
    "DMA2 Channel 6 bond.",
    Dma2Ch6Bond,
    Dmamux1Ch12Res,
    IntDma2Ch6,
    Ccr6,
    Cmar6,
    Cndtr6,
    Cpar6,
    C6S,
    Cgif6,
    Chtif6,
    Ctcif6,
    Cteif6,
    Gif6,
    Htif6,
    Tcif6,
    Teif6,
    dma2_ch6,
    dma2,
    dma2_ccr6,
    dma2_cmar6,
    dma2_cndtr6,
    dma2_cpar6,
    dma2_cselr,
    dma2_ifcr,
    dma2_isr,
    dma2_cselr_c6s,
    dma2_ifcr_cgif6,
    dma2_ifcr_chtif6,
    dma2_ifcr_ctcif6,
    dma2_ifcr_cteif6,
    dma2_isr_gif6,
    dma2_isr_htif6,
    dma2_isr_tcif6,
    dma2_isr_teif6,
    dmamux1_ch12,
    ccr6,
    cmar6,
    cndtr6,
    cpar6,
    c6s,
    cgif6,
    chtif6,
    ctcif6,
    cteif6,
    gif6,
    htif6,
    tcif6,
    teif6,
  ),
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
  (
    "DMA2 Channel 7 driver.",
    Dma2Ch7,
    drv_dma2_ch7,
    "DMA2 Channel 7 resource.",
    Dma2Ch7Res,
    "DMA2 Channel 7 bond.",
    Dma2Ch7Bond,
    Dmamux1Ch13Res,
    IntDma2Ch7,
    Ccr7,
    Cmar7,
    Cndtr7,
    Cpar7,
    C7S,
    Cgif7,
    Chtif7,
    Ctcif7,
    Cteif7,
    Gif7,
    Htif7,
    Tcif7,
    Teif7,
    dma2_ch7,
    dma2,
    dma2_ccr7,
    dma2_cmar7,
    dma2_cndtr7,
    dma2_cpar7,
    dma2_cselr,
    dma2_ifcr,
    dma2_isr,
    dma2_cselr_c7s,
    dma2_ifcr_cgif7,
    dma2_ifcr_chtif7,
    dma2_ifcr_ctcif7,
    dma2_ifcr_cteif7,
    dma2_isr_gif7,
    dma2_isr_htif7,
    dma2_isr_tcif7,
    dma2_isr_teif7,
    dmamux1_ch13,
    ccr7,
    cmar7,
    cndtr7,
    cpar7,
    c7s,
    cgif7,
    chtif7,
    ctcif7,
    cteif7,
    gif7,
    htif7,
    tcif7,
    teif7,
  ),
}
