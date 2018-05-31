//! Prelude

pub use flash::FlashExt as _stm32f30x_hal_flash_FlashExt;
pub use adc::adc12::Adc12Ext as _stm32f30x_hal_adc_adc12_Adc12Ext;
pub use adc::adc34::Adc34Ext as _stm32f30x_hal_adc_adc34_Adc34Ext;
pub use dac::DacExt as _stm32f30x_hal_dac_DacExt;
pub use dac::DacPendingChannel1 as _stm32f30x_hal_dac_DacPendingChannel1;
pub use dma::DmaChannel as _stm32f30x_hal_dma_DmaChannel;
pub use dma::DmaExt as _stm32f30x_hal_dma_DmaExt;
pub use gpio::GpioExt as _stm32f30x_hal_gpio_GpioExt;
pub use hal::prelude::*;
pub use rcc::RccExt as _stm32f30x_hal_rcc_RccExt;
pub use time::U32Ext as _stm32f30x_hal_time_U32Ext;
