//! System configuration controller (SYSCFG)

use core::marker::PhantomData;
use stm32f30x;

/// The bit is in the "remapped" state.
pub struct Remapped;
/// The bit is in the "not remapped" state.
pub struct NotRemapped;

/// Extension trait that constrains the system configuration controller
/// (SYSCFG).
pub trait SysCfgExt {
    /// Constrains the system configuration controller so that it plays nicely
    /// with the other abstractions.
    fn constrain(self) -> (SysCfg, Tokens);
}

impl SysCfgExt for stm32f30x::SYSCFG {
    fn constrain(self) -> (SysCfg, Tokens) {
        (
            SysCfg { reg: self },
            Tokens {
                adc24_dma_rmp: Adc24DmaRemap { state: PhantomData },
                tim16_dma_rmp: Tim16DmaRemap { state: PhantomData },
                tim17_dma_rmp: Tim17DmaRemap { state: PhantomData },
                tim6_dac1_dma_rmp: Tim6Dac1DmaRemap { state: PhantomData },
                tim7_dac2_dma_rmp: Tim7Dac2DmaRemap { state: PhantomData },
            },
        )
    }
}

/// Constrained SYSCFG controller.
pub struct SysCfg {
    reg: stm32f30x::SYSCFG,
}

/// Tokens that represent some of the system configuration at the type level.
pub struct Tokens {
    /// ADC2/4 DMA request remapping bit.
    pub adc24_dma_rmp: Adc24DmaRemap<NotRemapped>,
    /// TIM16 DMA request remapping bit.
    pub tim16_dma_rmp: Tim16DmaRemap<NotRemapped>,
    /// TIM17 DMA request remapping bit.
    pub tim17_dma_rmp: Tim17DmaRemap<NotRemapped>,
    /// TIM6 and DAC channel 1 DMA request remapping bit.
    pub tim6_dac1_dma_rmp: Tim6Dac1DmaRemap<NotRemapped>,
    /// TIM7 and DAC channel 2 DMA request remapping bit.
    pub tim7_dac2_dma_rmp: Tim7Dac2DmaRemap<NotRemapped>,
}

macro_rules! dma_rmp {
    ($name:ident, $Name:ident, $reg:ident, $doc:expr, $into_remapped_doc:expr, $into_not_remapped_doc:expr,) => {
        #[doc=$doc]
        pub struct $Name<State> {
            state: PhantomData<State>,
        }

        impl<State> $Name<State> {
            #[doc=$into_remapped_doc]
            pub fn into_remapped(self, syscfg: &mut SysCfg) -> $Name<Remapped> {
                syscfg.reg.$reg.modify(|_, w| w.$name().set_bit());
                $Name { state: PhantomData }
            }

            #[doc=$into_not_remapped_doc]
            pub fn into_not_remapped(self, syscfg: &mut SysCfg) -> $Name<NotRemapped> {
                syscfg.reg.$reg.modify(|_, w| w.$name().clear_bit());
                $Name { state: PhantomData }
            }
        }
    };
}
dma_rmp!(
    adc24_dma_rmp,
    Adc24DmaRemap,
    cfgr1,
    "ADC2/4 DMA request remapping bit.",
    "Remaps ADC2/4 DMA requests to DMA2 channels 3/4.",
    "Undoes remap. (Maps ADC2/4 DMA requests to DMA2 channels 1/2.)",
);
dma_rmp!(
    tim16_dma_rmp,
    Tim16DmaRemap,
    cfgr1,
    "TIM16 DMA request remapping bit.",
    "Remaps TIM16_CH1 and TIM16_UP DMA requests to DMA1 channel 6.",
    "Undoes remap. (Maps TIM16_CH1 and TIM16_UP DMA requests to DMA1 channel 3.)",
);
dma_rmp!(
    tim17_dma_rmp,
    Tim17DmaRemap,
    cfgr1,
    "TIM17 DMA request remapping bit.",
    "Remaps TIM17_CH1 and TIM17_UP DMA requests to DMA1 channel 7.",
    "Undoes remap. (Maps TIM17_CH1 and TIM17_UP DMA requests to DMA1 channel 1.)",
);
dma_rmp!(
    tim6_dac1_dma_rmp,
    Tim6Dac1DmaRemap,
    cfgr1,
    "TIM6 and DAC channel 1 DMA request remapping bit.",
    "Remaps TIM6_UP and DAC_CH1 DMA requests to DMA1 channel 3.",
    "Undoes remap. (Maps TIM6_UP and DAC_CH1 DMA requests to DMA2 channel 3.)",
);
dma_rmp!(
    tim7_dac2_dma_rmp,
    Tim7Dac2DmaRemap,
    cfgr1,
    "TIM7 and DAC channel 2 DMA request remapping bit.",
    "Remaps TIM7_UP and DAC_CH2 DMA requests to DMA1 channel 4.",
    "Undoes remap. (Maps TIM7_UP and DAC_CH2 DMA requests to DMA2 channel 4.)",
);
