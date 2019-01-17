//! I2C session.

use crate::i2c::{I2CBreak, I2CDmaRxRes, I2CDmaTxRes, I2CError, I2COn, I2C};
use core::{
  pin::Pin,
  task::{LocalWaker, Poll},
};
use drone_core::awt;
use drone_cortex_m::{
  reg::{prelude::*, RegGuard, RegGuardCnt},
  thr::prelude::*,
};
use drone_stm32_drv_dma::dma::{
  DmaBond, DmaBondOnRgc, DmaResCcr, DmaTransferError, DmaTxRes,
};
use failure::Fail;
use futures::prelude::*;

/// I2C session error.
#[derive(Debug, Fail)]
pub enum I2CSessError {
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

/// I2C session driver.
pub struct I2CSess<I2CRes, DmaRxBond, DmaTxBond, C>(
  I2CSessRes<I2CRes, DmaRxBond, DmaTxBond, C>,
)
where
  I2CRes: I2CDmaRxRes<DmaRxBond> + I2CDmaTxRes<DmaTxBond>,
  DmaRxBond: DmaBond,
  DmaTxBond: DmaBond,
  DmaTxBond::DmaRes: DmaTxRes<DmaRxBond::DmaRes>,
  C: RegGuardCnt<I2COn<I2CRes>>
    + DmaBondOnRgc<DmaRxBond::DmaRes>
    + DmaBondOnRgc<DmaTxBond::DmaRes>;

/// I2C session resource.
#[allow(missing_docs)]
pub struct I2CSessRes<I2CRes, DmaRxBond, DmaTxBond, C>
where
  I2CRes: I2CDmaRxRes<DmaRxBond> + I2CDmaTxRes<DmaTxBond>,
  DmaRxBond: DmaBond,
  DmaTxBond: DmaBond,
  DmaTxBond::DmaRes: DmaTxRes<DmaRxBond::DmaRes>,
  C: RegGuardCnt<I2COn<I2CRes>>
    + DmaBondOnRgc<DmaRxBond::DmaRes>
    + DmaBondOnRgc<DmaTxBond::DmaRes>,
{
  pub i2c: I2C<I2CRes, C>,
  pub i2c_on: RegGuard<I2COn<I2CRes>, C>,
  pub dma_rx: DmaRxBond,
  pub dma_tx: DmaTxBond,
}

enum Output3<A, B, C>
where
  A: Future,
  B: Future,
  C: Future,
{
  A(A::Output, B, C),
  B(A, B::Output, C),
  C(A, B, C::Output),
}

struct Select3<A, B, C>(Option<(A, B, C)>)
where
  A: Future + Unpin,
  B: Future + Unpin,
  C: Future + Unpin;

impl<A, B, C> Select3<A, B, C>
where
  A: Future + Unpin,
  B: Future + Unpin,
  C: Future + Unpin,
{
  fn new(a: A, b: B, c: C) -> Self {
    Self(Some((a, b, c)))
  }
}

impl<A, B, C> Future for Select3<A, B, C>
where
  A: Future + Unpin,
  B: Future + Unpin,
  C: Future + Unpin,
{
  type Output = Output3<A, B, C>;

  fn poll(self: Pin<&mut Self>, lw: &LocalWaker) -> Poll<Self::Output> {
    let select3 = self.get_mut();
    let (mut a, mut b, mut c) =
      select3.0.take().expect("cannot poll Select3 twice");
    match Pin::new(&mut a).poll(lw) {
      Poll::Ready(a) => Poll::Ready(Output3::A(a, b, c)),
      Poll::Pending => match Pin::new(&mut b).poll(lw) {
        Poll::Ready(b) => Poll::Ready(Output3::B(a, b, c)),
        Poll::Pending => match Pin::new(&mut c).poll(lw) {
          Poll::Ready(c) => Poll::Ready(Output3::C(a, b, c)),
          Poll::Pending => {
            select3.0 = Some((a, b, c));
            Poll::Pending
          }
        },
      },
    }
  }
}

#[allow(missing_docs)]
impl<I2CRes, DmaRxBond, DmaTxBond, C> I2CSess<I2CRes, DmaRxBond, DmaTxBond, C>
where
  I2CRes: I2CDmaRxRes<DmaRxBond> + I2CDmaTxRes<DmaTxBond>,
  DmaRxBond: DmaBond,
  DmaTxBond: DmaBond,
  DmaTxBond::DmaRes: DmaTxRes<DmaRxBond::DmaRes>,
  C: RegGuardCnt<I2COn<I2CRes>>
    + DmaBondOnRgc<DmaRxBond::DmaRes>
    + DmaBondOnRgc<DmaTxBond::DmaRes>,
{
  #[inline(always)]
  pub fn i2c(&self) -> &I2C<I2CRes, C> {
    &self.0.i2c
  }

  #[inline(always)]
  pub fn dma_rx(&self) -> &DmaRxBond {
    &self.0.dma_rx
  }

  #[inline(always)]
  pub fn dma_tx(&self) -> &DmaTxBond {
    &self.0.dma_tx
  }
}

impl<I2CRes, DmaRxBond, DmaTxBond, C> I2CSess<I2CRes, DmaRxBond, DmaTxBond, C>
where
  I2CRes: I2CDmaRxRes<DmaRxBond> + I2CDmaTxRes<DmaTxBond>,
  DmaRxBond: DmaBond,
  DmaTxBond: DmaBond,
  DmaTxBond::DmaRes: DmaTxRes<DmaRxBond::DmaRes>,
  C: RegGuardCnt<I2COn<I2CRes>>
    + DmaBondOnRgc<DmaRxBond::DmaRes>
    + DmaBondOnRgc<DmaTxBond::DmaRes>,
{
  /// Creates a new `I2CSess`.
  #[inline(always)]
  pub fn new(res: I2CSessRes<I2CRes, DmaRxBond, DmaTxBond, C>) -> Self {
    Self(res)
  }

  /// Releases the underlying resources.
  #[inline(always)]
  pub fn free(self) -> I2CSessRes<I2CRes, DmaRxBond, DmaTxBond, C> {
    self.0
  }

  /// Initializes DMA for the I2C as peripheral.
  #[inline]
  pub fn dma_init(&self) {
    self.0.i2c.dma_init(&self.0.dma_rx, &self.0.dma_tx);
  }

  /// Reads bytes to `buf` from `slave_addr`. Leaves the session open.
  ///
  /// # Panics
  ///
  /// If length of `buf` is greater than 255.
  pub fn read<'sess>(
    &'sess self,
    buf: &'sess mut [u8],
    slave_addr: u8,
    i2c_cr1_val: I2CRes::Cr1Val,
    i2c_cr2_val: I2CRes::Cr2Val,
  ) -> impl Future<Output = Result<(), I2CSessError>> + 'sess {
    self.read_impl(buf, slave_addr, i2c_cr1_val, i2c_cr2_val, false)
  }

  /// Reads bytes to `buf` from `slave_addr`. Closes the session afterwards.
  ///
  /// # Panics
  ///
  /// If length of `buf` is greater than 255.
  pub fn read_and_stop<'sess>(
    &'sess self,
    buf: &'sess mut [u8],
    slave_addr: u8,
    i2c_cr1_val: I2CRes::Cr1Val,
    i2c_cr2_val: I2CRes::Cr2Val,
  ) -> impl Future<Output = Result<(), I2CSessError>> + 'sess {
    self.read_impl(buf, slave_addr, i2c_cr1_val, i2c_cr2_val, true)
  }

  /// Writes bytes from `buf` to `slave_addr`. Leaves the session open.
  ///
  /// # Panics
  ///
  /// If length of `buf` is greater than 255.
  pub fn write<'sess>(
    &'sess self,
    buf: &'sess [u8],
    slave_addr: u8,
    i2c_cr1_val: I2CRes::Cr1Val,
    i2c_cr2_val: I2CRes::Cr2Val,
  ) -> impl Future<Output = Result<(), I2CSessError>> + 'sess {
    self.write_impl(buf, slave_addr, i2c_cr1_val, i2c_cr2_val, false)
  }

  /// Writes bytes from `buf` to `slave_addr`. Closes the session afterwards.
  ///
  /// # Panics
  ///
  /// If length of `buf` is greater than 255.
  pub fn write_and_stop<'sess>(
    &'sess self,
    buf: &'sess [u8],
    slave_addr: u8,
    i2c_cr1_val: I2CRes::Cr1Val,
    i2c_cr2_val: I2CRes::Cr2Val,
  ) -> impl Future<Output = Result<(), I2CSessError>> + 'sess {
    self.write_impl(buf, slave_addr, i2c_cr1_val, i2c_cr2_val, true)
  }

  fn read_impl<'sess>(
    &'sess self,
    buf: &'sess mut [u8],
    slave_addr: u8,
    mut i2c_cr1_val: I2CRes::Cr1Val,
    mut i2c_cr2_val: I2CRes::Cr2Val,
    autoend: bool,
  ) -> impl Future<Output = Result<(), I2CSessError>> + 'sess {
    if buf.len() > 255 {
      panic!("I2C session overflow");
    }
    asnc(move || {
      unsafe { self.0.dma_rx.dma_ch().set_maddr(buf.as_mut_ptr() as usize) };
      self.0.dma_rx.dma_ch().set_size(buf.len());
      self.0.dma_rx.dma_ch().ccr().store_val({
        let mut rx_ccr = self.init_dma_rx_ccr();
        self.0.dma_rx.dma_ch().ccr_en().set(&mut rx_ccr);
        rx_ccr
      });
      self.0.i2c.cr1().store_val({
        self.0.i2c.cr1_pe().set(&mut i2c_cr1_val);
        self.0.i2c.cr1_errie().set(&mut i2c_cr1_val);
        self.0.i2c.cr1_nackie().set(&mut i2c_cr1_val);
        self.0.i2c.cr1_rxdmaen().set(&mut i2c_cr1_val);
        i2c_cr1_val
      });
      let dma_rx = self.0.dma_rx.dma_ch().transfer_complete();
      let i2c_break = self.0.i2c.transfer_break();
      let i2c_error = self.0.i2c.transfer_error();
      self.set_i2c_cr2(&mut i2c_cr2_val, slave_addr, autoend, buf.len(), false);
      self.0.i2c.cr2().store_val(i2c_cr2_val);
      match awt!(Select3::new(dma_rx, i2c_break, i2c_error)) {
        Output3::A(Ok(()), i2c_break, i2c_error) => {
          drop(i2c_break);
          drop(i2c_error);
          self
            .0
            .dma_rx
            .dma_ch()
            .ccr()
            .store_val(self.init_dma_rx_ccr());
          self.0.i2c.int_ev().trigger();
          self.0.i2c.int_er().trigger();
          Ok(())
        }
        Output3::A(Err(dma_rx), i2c_break, i2c_error) => {
          drop(i2c_break);
          drop(i2c_error);
          self
            .0
            .dma_rx
            .dma_ch()
            .ccr()
            .store_val(self.init_dma_rx_ccr());
          self.0.i2c.int_ev().trigger();
          self.0.i2c.int_er().trigger();
          Err(dma_rx.into())
        }
        Output3::B(dma_rx, i2c_break, i2c_error) => {
          drop(dma_rx);
          drop(i2c_error);
          self
            .0
            .dma_rx
            .dma_ch()
            .ccr()
            .store_val(self.init_dma_rx_ccr());
          self.0.dma_rx.dma_ch().int().trigger();
          self.0.i2c.int_er().trigger();
          Err(i2c_break.into())
        }
        Output3::C(dma_rx, i2c_break, i2c_error) => {
          drop(dma_rx);
          drop(i2c_break);
          self
            .0
            .dma_rx
            .dma_ch()
            .ccr()
            .store_val(self.init_dma_rx_ccr());
          self.0.dma_rx.dma_ch().int().trigger();
          self.0.i2c.int_ev().trigger();
          Err(i2c_error.into())
        }
      }
    })
  }

  fn write_impl<'sess>(
    &'sess self,
    buf: &'sess [u8],
    slave_addr: u8,
    mut i2c_cr1_val: I2CRes::Cr1Val,
    mut i2c_cr2_val: I2CRes::Cr2Val,
    autoend: bool,
  ) -> impl Future<Output = Result<(), I2CSessError>> + 'sess {
    if buf.len() > 255 {
      panic!("I2C session overflow");
    }
    asnc(move || {
      unsafe { self.0.dma_tx.dma_ch().set_maddr(buf.as_ptr() as usize) };
      self.0.dma_tx.dma_ch().set_size(buf.len());
      self.0.dma_tx.dma_ch().ccr().store_val({
        let mut tx_ccr = self.init_dma_tx_ccr();
        self.0.dma_tx.dma_ch().ccr_en().set(&mut tx_ccr);
        tx_ccr
      });
      self.0.i2c.cr1().store_val({
        self.0.i2c.cr1_pe().set(&mut i2c_cr1_val);
        self.0.i2c.cr1_errie().set(&mut i2c_cr1_val);
        self.0.i2c.cr1_nackie().set(&mut i2c_cr1_val);
        self.0.i2c.cr1_txdmaen().set(&mut i2c_cr1_val);
        i2c_cr1_val
      });
      let dma_tx = self.0.dma_tx.dma_ch().transfer_complete();
      let i2c_break = self.0.i2c.transfer_break();
      let i2c_error = self.0.i2c.transfer_error();
      self.set_i2c_cr2(&mut i2c_cr2_val, slave_addr, autoend, buf.len(), true);
      self.0.i2c.cr2().store_val(i2c_cr2_val);
      match awt!(Select3::new(dma_tx, i2c_break, i2c_error)) {
        Output3::A(Ok(()), i2c_break, i2c_error) => {
          drop(i2c_break);
          drop(i2c_error);
          self
            .0
            .dma_tx
            .dma_ch()
            .ccr()
            .store_val(self.init_dma_tx_ccr());
          self.0.i2c.int_ev().trigger();
          self.0.i2c.int_er().trigger();
          Ok(())
        }
        Output3::A(Err(dma_tx), i2c_break, i2c_error) => {
          drop(i2c_break);
          drop(i2c_error);
          self
            .0
            .dma_tx
            .dma_ch()
            .ccr()
            .store_val(self.init_dma_tx_ccr());
          self.0.i2c.int_ev().trigger();
          self.0.i2c.int_er().trigger();
          Err(dma_tx.into())
        }
        Output3::B(dma_tx, i2c_break, i2c_error) => {
          drop(dma_tx);
          drop(i2c_error);
          self
            .0
            .dma_tx
            .dma_ch()
            .ccr()
            .store_val(self.init_dma_tx_ccr());
          self.0.dma_tx.dma_ch().int().trigger();
          self.0.i2c.int_er().trigger();
          Err(i2c_break.into())
        }
        Output3::C(dma_tx, i2c_break, i2c_error) => {
          drop(dma_tx);
          drop(i2c_break);
          self
            .0
            .dma_tx
            .dma_ch()
            .ccr()
            .store_val(self.init_dma_tx_ccr());
          self.0.dma_tx.dma_ch().int().trigger();
          self.0.i2c.int_ev().trigger();
          Err(i2c_error.into())
        }
      }
    })
  }

  fn set_i2c_cr2(
    &self,
    val: &mut I2CRes::Cr2Val,
    slave_addr: u8,
    autoend: bool,
    nbytes: usize,
    write: bool,
  ) {
    self.0.i2c.cr2_add10().clear(val);
    let slave_addr = u32::from(slave_addr << 1);
    self.0.i2c.cr2_sadd().write(val, slave_addr);
    if write {
      self.0.i2c.cr2_rd_wrn().clear(val);
    } else {
      self.0.i2c.cr2_rd_wrn().set(val);
    }
    self.0.i2c.cr2_nbytes().write(val, nbytes as u32);
    if autoend {
      self.0.i2c.cr2_autoend().set(val);
    } else {
      self.0.i2c.cr2_autoend().clear(val);
    }
    self.0.i2c.cr2_start().set(val);
  }

  fn init_dma_rx_ccr(&self) -> <DmaRxBond::DmaRes as DmaResCcr>::CcrVal {
    let mut val = self.0.dma_rx.dma_ch().ccr().default_val();
    self.0.dma_rx.dma_ch().ccr_mem2mem().clear(&mut val);
    self.0.dma_rx.dma_ch().ccr_msize().write(&mut val, 0b00);
    self.0.dma_rx.dma_ch().ccr_psize().write(&mut val, 0b00);
    self.0.dma_rx.dma_ch().ccr_minc().set(&mut val);
    self.0.dma_rx.dma_ch().ccr_pinc().clear(&mut val);
    self.0.dma_rx.dma_ch().ccr_circ().clear(&mut val);
    self.0.dma_rx.dma_ch().ccr_dir().clear(&mut val);
    self.0.dma_rx.dma_ch().ccr_teie().set(&mut val);
    self.0.dma_rx.dma_ch().ccr_htie().clear(&mut val);
    self.0.dma_rx.dma_ch().ccr_tcie().set(&mut val);
    self.0.dma_rx.dma_ch().ccr_en().clear(&mut val);
    val
  }

  fn init_dma_tx_ccr(&self) -> <DmaTxBond::DmaRes as DmaResCcr>::CcrVal {
    let mut val = self.0.dma_tx.dma_ch().ccr().default_val();
    self.0.dma_tx.dma_ch().ccr_mem2mem().clear(&mut val);
    self.0.dma_tx.dma_ch().ccr_msize().write(&mut val, 0b00);
    self.0.dma_tx.dma_ch().ccr_psize().write(&mut val, 0b00);
    self.0.dma_tx.dma_ch().ccr_minc().set(&mut val);
    self.0.dma_tx.dma_ch().ccr_pinc().clear(&mut val);
    self.0.dma_tx.dma_ch().ccr_circ().clear(&mut val);
    self.0.dma_tx.dma_ch().ccr_dir().set(&mut val);
    self.0.dma_tx.dma_ch().ccr_teie().set(&mut val);
    self.0.dma_tx.dma_ch().ccr_htie().clear(&mut val);
    self.0.dma_tx.dma_ch().ccr_tcie().set(&mut val);
    self.0.dma_tx.dma_ch().ccr_en().clear(&mut val);
    val
  }
}

impl From<DmaTransferError> for I2CSessError {
  fn from(err: DmaTransferError) -> Self {
    I2CSessError::Dma(err)
  }
}

impl From<I2CBreak> for I2CSessError {
  fn from(err: I2CBreak) -> Self {
    I2CSessError::I2CBreak(err)
  }
}

impl From<I2CError> for I2CSessError {
  fn from(err: I2CError) -> Self {
    I2CSessError::I2CError(err)
  }
}
