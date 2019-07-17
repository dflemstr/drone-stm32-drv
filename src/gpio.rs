//! General-purpose I/O.

use crate::common::DrvRcc;
use drone_core::inventory::{Inventory0, InventoryGuard, InventoryResource};
use drone_cortex_m::reg::prelude::*;
use drone_stm32_map::periph::gpio::head::{GpioHeadMap, GpioHeadPeriph};

/// GPIO port head driver.
pub struct GpioHead<T: GpioHeadMap>(Inventory0<GpioHeadEn<T>>);

/// GPIO port head enabled driver.
pub struct GpioHeadEn<T: GpioHeadMap> {
    periph: GpioHeadPeriph<T>,
}

impl<T: GpioHeadMap> GpioHead<T> {
    /// Creates a new [`GpioHead`].
    #[inline]
    pub fn new(periph: GpioHeadPeriph<T>) -> Self {
        Self(Inventory0::new(GpioHeadEn { periph }))
    }

    /// Releases the peripheral.
    #[inline]
    pub fn free(self) -> GpioHeadPeriph<T> {
        self.0.free().periph
    }

    /// Enables the port clock.
    pub fn enable(&mut self) -> InventoryGuard<'_, GpioHeadEn<T>> {
        self.setup();
        self.0.guard()
    }

    /// Enables the port clock.
    pub fn into_enabled(self) -> Inventory0<GpioHeadEn<T>> {
        self.setup();
        self.0
    }

    /// Disables the port clock.
    pub fn from_enabled(mut enabled: Inventory0<GpioHeadEn<T>>) -> Self {
        enabled.teardown();
        Self(enabled)
    }

    fn setup(&self) {
        let gpioen = &self.0.periph.rcc_ahb2enr_gpioen;
        if gpioen.read_bit() {
            panic!("GPIO wasn't turned off");
        }
        gpioen.set_bit();
    }
}

impl<T: GpioHeadMap> DrvRcc for GpioHead<T> {
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

impl<T: GpioHeadMap> InventoryResource for GpioHeadEn<T> {
    fn teardown(&mut self) {
        self.periph.rcc_ahb2enr_gpioen.clear_bit()
    }
}

impl<T: GpioHeadMap> DrvRcc for GpioHeadEn<T> {
    fn reset(&mut self) {
        self.periph.rcc_ahb2rstr_gpiorst.set_bit();
    }

    fn disable_stop_mode(&self) {
        self.periph.rcc_ahb2smenr_gpiosmen.clear_bit();
    }

    fn enable_stop_mode(&self) {
        self.periph.rcc_ahb2smenr_gpiosmen.set_bit();
    }
}
