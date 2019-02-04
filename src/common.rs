//! Drivers common traits.

#[cfg(any(
  feature = "stm32l4r5",
  feature = "stm32l4r7",
  feature = "stm32l4r9",
  feature = "stm32l4s5",
  feature = "stm32l4s7",
  feature = "stm32l4s9"
))]
use crate::dma::mux::DmamuxChEn;
use crate::dma::DmaChEn;
use drone_cortex_m::thr::prelude::*;
use drone_stm32_map::periph::dma::ch::DmaChMap;

/// Driver reset and clock control.
pub trait DrvRcc {
  /// Resets the peripheral.
  fn reset(&mut self);

  /// Disables the peripheral clocks by the clock gating during Sleep and Stop
  /// modes.
  fn disable_stop_mode(&self);

  /// Enables the peripheral clocks by the clock gating during Sleep and Stop
  /// modes.
  fn enable_stop_mode(&self);
}

/// Driver clock source selection.
pub trait DrvClockSel {
  /// Selects a clock source for the peripheral.
  fn clock_sel(&self, value: u32);
}

/// Driver DMA receiver.
pub trait DrvDmaRx<Rx: DmaChMap> {
  /// Initializes peripheral address of the DMA channel to the receiver.
  fn dma_rx_paddr_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken<Rtt>>);

  #[cfg(any(
    feature = "stm32l4r5",
    feature = "stm32l4r7",
    feature = "stm32l4r9",
    feature = "stm32l4s5",
    feature = "stm32l4s7",
    feature = "stm32l4s9"
  ))]
  /// Initializes the DMA channel as a receiver.
  fn dma_rx_init(
    &self,
    dma_rx: &DmaChEn<Rx, impl IntToken<Rtt>>,
    dmamux_rx: &DmamuxChEn<Rx::DmamuxChMap>,
    rx_dma_req_id: u32,
  ) {
    self.dma_rx_paddr_init(dma_rx);
    dmamux_rx.set_dma_req_id(rx_dma_req_id);
  }

  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  /// Initializes the DMA channel as a receiver.
  fn dma_rx_init(
    &self,
    dma_rx: &DmaChEn<Rx, impl IntToken<Rtt>>,
    dma_rx_ch: u32,
  ) {
    self.dma_rx_paddr_init(dma_rx);
    dma_rx.ch_select(dma_rx_ch);
  }

  #[cfg(not(any(
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
  )))]
  /// Initializes the DMA channel as a receiver.
  fn dma_rx_init(&self, dma_rx: &DmaChEn<Rx, impl IntToken<Rtt>>) {
    self.dma_rx_paddr_init(dma_rx);
  }
}

/// Driver DMA transmitter.
pub trait DrvDmaTx<Tx: DmaChMap> {
  /// Initializes peripheral address of the DMA channel to the transmitter.
  fn dma_tx_paddr_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken<Rtt>>);

  #[cfg(any(
    feature = "stm32l4r5",
    feature = "stm32l4r7",
    feature = "stm32l4r9",
    feature = "stm32l4s5",
    feature = "stm32l4s7",
    feature = "stm32l4s9"
  ))]
  /// Initializes the DMA channel as a transmitter.
  fn dma_tx_init(
    &self,
    dma_tx: &DmaChEn<Tx, impl IntToken<Rtt>>,
    dmamux_tx: &DmamuxChEn<Tx::DmamuxChMap>,
    tx_dma_req_id: u32,
  ) {
    self.dma_tx_paddr_init(dma_tx);
    dmamux_tx.set_dma_req_id(tx_dma_req_id);
  }

  #[cfg(any(
    feature = "stm32l4x1",
    feature = "stm32l4x2",
    feature = "stm32l4x3",
    feature = "stm32l4x5",
    feature = "stm32l4x6"
  ))]
  /// Initializes the DMA channel as a transmitter.
  fn dma_tx_init(
    &self,
    dma_tx: &DmaChEn<Tx, impl IntToken<Rtt>>,
    dma_tx_ch: u32,
  ) {
    self.dma_tx_paddr_init(dma_tx);
    dma_tx.ch_select(dma_tx_ch);
  }

  #[cfg(not(any(
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
  )))]
  /// Initializes the DMA channel as a transmitter.
  fn dma_tx_init(&self, dma_tx: &DmaChEn<Tx, impl IntToken<Rtt>>) {
    self.dma_tx_paddr_init(dma_tx);
  }
}
