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
}

/// ADC2/4 DMA request remapping bit.
pub struct Adc24DmaRemap<State> {
    state: PhantomData<State>,
}

impl<State> Adc24DmaRemap<State> {
    /// Remaps ADC2/4 DMA requests to DMA2 channels 3/4.
    pub fn into_remapped(self, syscfg: &mut SysCfg) -> Adc24DmaRemap<Remapped> {
        syscfg.reg.cfgr1.modify(|_, w| w.adc24_dma_rmp().set_bit());
        Adc24DmaRemap { state: PhantomData }
    }

    /// Undoes remap. (Remaps ADC2/4 DMA requests to DMA2 channels 1/2.)
    pub fn into_not_remapped(self, syscfg: &mut SysCfg) -> Adc24DmaRemap<NotRemapped> {
        syscfg
            .reg
            .cfgr1
            .modify(|_, w| w.adc24_dma_rmp().clear_bit());
        Adc24DmaRemap { state: PhantomData }
    }
}
