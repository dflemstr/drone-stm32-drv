use super::DmaEnGuard;
use drone_core::shared_guard::GuardToken;
use drone_cortex_m::{fib, reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::dma::ch::{traits::*, DmaChMap, DmaChPeriph};
use failure::Fail;
use futures::prelude::*;

/// Error returned when `DMA_ISR_TEIFx` flag in set.
#[derive(Debug, Fail)]
#[fail(display = "DMA transfer error.")]
pub struct DmaTransferError;

/// DMA channel driver.
pub struct DmaCh<T: DmaChMap, I: IntToken<Rtt>>(DmaChEn<T, I>);

/// DMA channel enabled driver.
pub struct DmaChEn<T: DmaChMap, I: IntToken<Rtt>> {
  periph: DmaChDiverged<T>,
  int: I,
}

/// DMA channel diverged peripheral.
#[allow(missing_docs)]
pub struct DmaChDiverged<T: DmaChMap> {
  pub dma_ccr: T::SDmaCcr,
  pub dma_cmar: T::SDmaCmar,
  pub dma_cndtr: T::SDmaCndtr,
  pub dma_cpar: T::SDmaCpar,
  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  pub dma_cselr_cs: T::SDmaCselrCs,
  pub dma_ifcr_cgif: T::CDmaIfcrCgif,
  pub dma_ifcr_chtif: T::CDmaIfcrChtif,
  pub dma_ifcr_ctcif: T::CDmaIfcrCtcif,
  pub dma_ifcr_cteif: T::SDmaIfcrCteif,
  pub dma_isr_gif: T::SDmaIsrGif,
  pub dma_isr_htif: T::CDmaIsrHtif,
  pub dma_isr_tcif: T::CDmaIsrTcif,
  pub dma_isr_teif: T::CDmaIsrTeif,
}

impl<T: DmaChMap, I: IntToken<Rtt>> DmaCh<T, I> {
  /// Creates a new [`Dma`].
  #[inline(always)]
  pub fn new(periph: DmaChPeriph<T>, int: I) -> Self {
    let periph = DmaChDiverged {
      dma_ccr: periph.dma_ccr,
      dma_cmar: periph.dma_cmar,
      dma_cndtr: periph.dma_cndtr,
      dma_cpar: periph.dma_cpar,
      #[cfg(any(
        feature = "stm32l4x1",
        feature = "stm32l4x2",
        feature = "stm32l4x3",
        feature = "stm32l4x5",
        feature = "stm32l4x6"
      ))]
      dma_cselr_cs: periph.dma_cselr_cs,
      dma_ifcr_cgif: periph.dma_ifcr_cgif.to_copy(),
      dma_ifcr_chtif: periph.dma_ifcr_chtif.to_copy(),
      dma_ifcr_ctcif: periph.dma_ifcr_ctcif.to_copy(),
      dma_ifcr_cteif: periph.dma_ifcr_cteif,
      dma_isr_gif: periph.dma_isr_gif,
      dma_isr_htif: periph.dma_isr_htif.to_copy(),
      dma_isr_tcif: periph.dma_isr_tcif.to_copy(),
      dma_isr_teif: periph.dma_isr_teif.to_copy(),
    };
    Self(DmaChEn { periph, int })
  }

  /// Creates a new [`Dma`].
  ///
  /// # Safety
  ///
  /// Some of the `Crt` register tokens can be still in use.
  #[inline(always)]
  pub unsafe fn from_diverged(periph: DmaChDiverged<T>, int: I) -> Self {
    Self(DmaChEn { periph, int })
  }

  /// Releases the peripheral.
  #[inline(always)]
  pub fn free(self) -> DmaChDiverged<T> {
    self.0.periph
  }

  /// Acquires the enabled state.
  #[inline]
  pub fn as_enabled(
    &self,
    _token: &GuardToken<DmaEnGuard<T::DmaMap>>,
  ) -> &DmaChEn<T, I> {
    &self.0
  }

  /// Acquires the enabled state.
  #[inline]
  pub fn as_enabled_mut(
    &mut self,
    _token: &GuardToken<DmaEnGuard<T::DmaMap>>,
  ) -> &mut DmaChEn<T, I> {
    &mut self.0
  }

  /// Acquires the enabled state.
  #[inline]
  pub fn into_enabled(
    self,
    token: GuardToken<DmaEnGuard<T::DmaMap>>,
  ) -> DmaChEn<T, I> {
    // To be recreated in DmaChEn::into_disabled.
    drop(token);
    self.0
  }
}

impl<T: DmaChMap, I: IntToken<Rtt>> DmaChEn<T, I> {
  /// Releases the enabled state.
  #[inline]
  pub fn into_disabled(
    self,
  ) -> (DmaCh<T, I>, GuardToken<DmaEnGuard<T::DmaMap>>) {
    // An owned DmaChEn can come only from DmaCh::into_enabled.
    let token = unsafe { GuardToken::new() };
    (DmaCh(self), token)
  }

  /// Returns a number of data to transfer.
  pub fn size(&self) -> usize {
    self.periph.dma_cndtr.ndt().read_bits() as usize
  }

  /// Sets the number of data to transfer.
  pub fn set_size(&self, number: usize) {
    self.periph.dma_cndtr.ndt().write_bits(number as u32);
  }

  /// Returns the peripheral address.
  pub fn paddr<P>(&self) -> *mut P {
    self.periph.dma_cpar.pa().read_bits() as _
  }

  /// Sets the peripheral address.
  ///
  /// # Safety
  ///
  /// The method works with raw pointer.
  pub unsafe fn set_paddr<P>(&self, addr: *const P) {
    self.periph.dma_cpar.pa().write_bits(addr as _);
  }

  /// Returns he memory address.
  pub fn maddr<M>(&self) -> *mut M {
    self.periph.dma_cmar.ma().read_bits() as _
  }

  /// Sets the memory address.
  ///
  /// # Safety
  ///
  /// The method works with raw pointer.
  pub unsafe fn set_maddr<M>(&self, addr: *const M) {
    self.periph.dma_cmar.ma().write_bits(addr as _);
  }

  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  /// Selects DMA channel.
  pub fn ch_select(&self, ch: u32) {
    self.periph.dma_cselr_cs.modify(|r| {
      self.periph.dma_cselr_cs.write(r, ch);
    });
  }

  /// Returns a future, which resolves on DMA transfer complete event.
  pub fn transfer_complete(
    &self,
  ) -> impl Future<Output = Result<(), DmaTransferError>> {
    let teif = self.periph.dma_isr_teif;
    let tcif = self.periph.dma_isr_tcif;
    let cgif = self.periph.dma_ifcr_cgif;
    let ctcif = self.periph.dma_ifcr_ctcif;
    self.int.add_try_future(fib::new(move || loop {
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
  ) -> impl Future<Output = Result<(), DmaTransferError>> {
    let teif = self.periph.dma_isr_teif;
    let htif = self.periph.dma_isr_htif;
    let cgif = self.periph.dma_ifcr_cgif;
    let chtif = self.periph.dma_ifcr_chtif;
    self.int.add_try_future(fib::new(move || loop {
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

#[allow(missing_docs)]
impl<T: DmaChMap, I: IntToken<Rtt>> DmaChEn<T, I> {
  #[inline(always)]
  pub fn int(&self) -> &I {
    &self.int
  }

  #[inline(always)]
  pub fn ccr(&self) -> &T::SDmaCcr {
    &self.periph.dma_ccr
  }
}