//! Wrappers for ADC3 and ADC4.

pub use self::channels::{Adc34Channels, Adc3Channel, Adc3ChannelId, Adc4Channel, Adc4ChannelId};

use super::*;
use prelude::*;
use rcc::{Clocks, AHB};
use stm32f30x;
use time::Hertz;

/// Extension trait to split the ADC3 and ADC4 peripherals into a wrapper
/// and channels.
pub trait Adc34Ext {
    /// Enables the ADC3/4 clock and returns a wrapper and channels.
    fn split(
        self,
        adc3: stm32f30x::ADC3,
        adc4: stm32f30x::ADC4,
        ahb: &mut AHB,
        clocks: Clocks,
    ) -> (Adc34<Independent, Unpowered, Unpowered>, Adc34Channels);
}

/// ADC3 wrapper.
///
/// `P` is the state of the ADC pair, and `S` is the state of this ADC.
pub struct Adc3<P, S> {
    clock_freq: Hertz,
    reg: stm32f30x::ADC3,
    pair_state: PhantomData<P>,
    state: S,
}

/// ADC4 wrapper.
///
/// `P` is the state of the ADC pair, and `S` is the state of this ADC.
pub struct Adc4<P, S> {
    clock_freq: Hertz,
    reg: stm32f30x::ADC4,
    pair_state: PhantomData<P>,
    state: S,
}

/// ADC3 and ADC4 pair.
///
/// `P` is the state of the ADC pair, `S3` is the state of ADC3, and `S4`
/// is the state of ADC4.
pub struct Adc34<P, S3, S4> {
    /// ADC3.
    pub adc3: Adc3<P, S3>,
    /// ADC4.
    pub adc4: Adc4<P, S4>,
}

impl<P, S3, S4> Adc34<P, S3, S4> {
    // /// Returns the CSR register.
    // fn csr(&self) -> &stm32f30x::adc1_2::CSR {
    //     // The pair of ADCs has exclusive access to its common registers.
    //     unsafe { &(*stm32f30x::ADC3_4::ptr()).csr }
    // }

    /// Returns the CCR register.
    fn ccr_mut(&mut self) -> &stm32f30x::adc1_2::CCR {
        // The pair of ADCs has exclusive access to its common registers.
        unsafe { &(*stm32f30x::ADC3_4::ptr()).ccr }
    }

    /// Returns the CDR register.
    fn cdr(&self) -> &stm32f30x::adc1_2::CDR {
        // The pair of ADCs has exclusive access to its common registers.
        unsafe { &(*stm32f30x::ADC3_4::ptr()).cdr }
    }
}

impl<P, S3, S4> AdcPair for Adc34<P, S3, S4> {
    type Master = Adc3<P, S3>;
    type Slave = Adc4<P, S4>;
    #[inline]
    fn master(&self) -> &Self::Master {
        &self.adc3
    }
    #[inline]
    fn slave(&self) -> &Self::Slave {
        &self.adc4
    }
    #[inline]
    fn master_mut(&mut self) -> &mut Self::Master {
        &mut self.adc3
    }
    #[inline]
    fn slave_mut(&mut self) -> &mut Self::Slave {
        &mut self.adc4
    }
    #[inline]
    fn split(self) -> (Self::Master, Self::Slave) {
        (self.adc3, self.adc4)
    }
    #[inline]
    fn join(master: Self::Master, slave: Self::Slave) -> Self {
        Adc34 {
            adc3: master,
            adc4: slave,
        }
    }
}

/// Continuous iterator over regular conversions in dual simultaneous mode.
pub struct Adc34ContIter<'p, 'm: 'p, 's: 'p> {
    pair: &'p mut Adc34<Dual, WithSequence<'m>, WithSequence<'s>>,
}

impl_iterator_for_paircontiter!(Adc34ContIter, adc3);

impl_pair_unpowered_unpowered!(Adc34, Adc3, Adc4, adc3, adc4);

impl_pair_disabled_disabled!(Adc34, Adc3, Adc4, adc3, adc4);

impl_pair_dual_enabled_enabled!(Adc34, adc3, adc4, Adc3Channel, Adc4Channel);

impl_pair_withsequence_withsequence!(
    Adc34,
    Adc34ContIter,
    Adc3,
    Adc4,
    adc3,
    adc4,
    stm32f30x::ADC3_4::ptr(),
    stm32f30x::ADC3::ptr(),
);

impl_pair_dual_runningdma_runningdma!(Adc34, Adc3, Adc4, adc3, adc4);

impl_single_any!(Adc3);
impl_single_any!(Adc4);

impl_single_unpowered!(Adc3);
impl_single_unpowered!(Adc4);

impl_single_disabled!(Adc3);
impl_single_disabled!(Adc4);

impl_single_enabled!(Adc3, Adc3Channel);
impl_single_enabled!(Adc4, Adc4Channel);

impl_single_with_sequence!(Adc3, Adc3ChannelId);
impl_single_with_sequence!(Adc4, Adc4ChannelId);

impl_single_independent_with_sequence!(Adc3, ADC3);
impl_single_independent_with_sequence!(Adc4, ADC4);

impl_single_independent_running_dma!(Adc3, dma::dma1::Channel1);
impl_single_independent_running_dma!(Adc4, dma::dma2::Channel1);

unsafe impl<'scope, P, S> AdcDmaTokens<'scope, Adc3<P, S>> for dma::dma2::Channel5 {
    type Channel = dma::dma2::Channel5;
    fn channel(self) -> Self::Channel {
        self
    }
}

unsafe impl<'scope, P, S> AdcDmaTokens<'scope, Adc4<P, S>>
    for (
        dma::dma2::Channel2,
        &'scope syscfg::Adc24DmaRemap<syscfg::NotRemapped>,
    )
{
    type Channel = dma::dma2::Channel2;
    fn channel(self) -> Self::Channel {
        self.0
    }
}

unsafe impl<'scope, P, S> AdcDmaTokens<'scope, Adc4<P, S>>
    for (
        dma::dma2::Channel4,
        &'scope syscfg::Adc24DmaRemap<syscfg::Remapped>,
    )
{
    type Channel = dma::dma2::Channel4;
    fn channel(self) -> Self::Channel {
        self.0
    }
}

/// ADC3 and ADC4 channels.
pub mod channels {
    use super::*;
    use gpio::gpiob::{PB0, PB1, PB12, PB13, PB14, PB15};
    use gpio::gpiod::{PD10, PD11, PD12, PD13, PD14, PD8, PD9};
    use gpio::gpioe::{PE10, PE11, PE12, PE13, PE14, PE15, PE7, PE8, PE9};

    define_impl_internalref!(Adc34);

    /// An ADC3 channel that has borrowed the necessary pins to be able to
    /// perform an ADC conversion.
    ///
    /// The only way to produce an `Adc3Channel` is through the `From`
    /// trait implementations; this ensures that while you have an
    /// `Adc3Channel` instance, the necessary pins are borrowed. It is not
    /// possible to copy/clone an `Adc3Channel` because the borrows could
    /// have been mutable.
    pub struct Adc3Channel<'a> {
        id: Adc3ChannelId,
        life: PhantomData<&'a ()>,
    }

    impl<'a> Adc3Channel<'a> {
        /// Returns the channel ID.
        pub fn id(&self) -> Adc3ChannelId {
            self.id
        }
    }

    /// An ADC4 channel that has borrowed the necessary pins to be able to
    /// perform an ADC conversion.
    ///
    /// The only way to produce an `Adc4Channel` is through the `From`
    /// trait implementations; this ensures that while you have an
    /// `Adc4Channel` instance, the necessary pins are borrowed. It is not
    /// possible to copy/clone an `Adc4Channel` because the borrows could
    /// have been mutable.
    pub struct Adc4Channel<'a> {
        id: Adc4ChannelId,
        life: PhantomData<&'a ()>,
    }

    impl<'a> Adc4Channel<'a> {
        /// Returns the channel ID.
        pub fn id(&self) -> Adc4ChannelId {
            self.id
        }
    }

    /// The numeric ID of an ADC3 channel.
    #[derive(Clone, Copy, Eq, PartialEq)]
    #[repr(u8)]
    pub enum Adc3ChannelId {
        /// ADC3_IN1
        Adc3In1 = 1,
        /// ADC3_IN2
        Adc3In2 = 2,
        /// ADC3_IN3
        Adc3In3 = 3,
        /// ADC3_IN4; the positive side is not bonded and is connected to ground (Vss)
        Adc3In4 = 4,
        /// ADC3_IN5
        Adc3In5 = 5,
        /// ADC34_IN6
        Adc34In6 = 6,
        /// ADC34_IN7
        Adc34In7 = 7,
        /// ADC34_IN8
        Adc34In8 = 8,
        /// ADC34_IN9
        Adc34In9 = 9,
        /// ADC34_IN10
        Adc34In10 = 10,
        /// ADC34_IN11
        Adc34In11 = 11,
        /// ADC3_IN12
        Adc3In12 = 12,
        /// ADC3_IN13
        Adc3In13 = 13,
        /// ADC3_IN14
        Adc3In14 = 14,
        /// ADC3_IN15
        Adc3In15 = 15,
        /// ADC3_IN16
        Adc3In16 = 16,
        /// V_OPAMP3
        OpAmp3 = 17,
        /// Internal reference voltage (V_REFINT)
        InternalRef = 18,
    }

    impl Adc3ChannelId {
        /// Returns the channel number for the sequence registers.
        pub fn number(self) -> u8 {
            self as u8
        }

        /// Returns `true` if the channels must not be converted at the same time.
        pub fn conflicts_with(self, other: Adc4ChannelId) -> bool {
            match (self, other) {
                (Adc3ChannelId::Adc34In6, Adc4ChannelId::Adc34In6)
                | (Adc3ChannelId::Adc34In7, Adc4ChannelId::Adc34In7)
                | (Adc3ChannelId::Adc34In8, Adc4ChannelId::Adc34In8)
                | (Adc3ChannelId::Adc34In9, Adc4ChannelId::Adc34In9)
                | (Adc3ChannelId::Adc34In10, Adc4ChannelId::Adc34In10)
                | (Adc3ChannelId::Adc34In11, Adc4ChannelId::Adc34In11)
                | (Adc3ChannelId::InternalRef, Adc4ChannelId::InternalRef) => true,
                _ => false,
            }
        }
    }

    /// The numeric ID of an ADC4 channel.
    #[derive(Clone, Copy, Eq, PartialEq)]
    #[repr(u8)]
    pub enum Adc4ChannelId {
        /// ADC4_IN1
        Adc4In1 = 1,
        /// ADC4_IN2
        Adc4In2 = 2,
        /// ADC4_IN3
        Adc4In3 = 3,
        /// ADC4_IN4
        Adc4In4 = 4,
        /// ADC4_IN5
        Adc4In5 = 5,
        /// ADC34_IN6
        Adc34In6 = 6,
        /// ADC34_IN7
        Adc34In7 = 7,
        /// ADC34_IN8
        Adc34In8 = 8,
        /// ADC34_IN9
        Adc34In9 = 9,
        /// ADC34_IN10
        Adc34In10 = 10,
        /// ADC34_IN11
        Adc34In11 = 11,
        /// ADC4_IN12
        Adc4In12 = 12,
        /// ADC4_IN13
        Adc4In13 = 13,
        // Channels 14, 15, and 16 are reserved.
        /// V_OPAMP4
        OpAmp4 = 17,
        /// Internal reference voltage (V_REFINT)
        InternalRef = 18,
    }

    impl Adc4ChannelId {
        /// Returns the channel number for the sequence registers.
        pub fn number(self) -> u8 {
            self as u8
        }

        /// Returns `true` if the channels must not be converted at the same time.
        pub fn conflicts_with(self, other: Adc3ChannelId) -> bool {
            other.conflicts_with(self)
        }
    }

    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc3In1, PB1, PE9);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc3In2, PE9, PE13);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc3In3, PE13);

    // The negative side of ADC3_IN3 is connected to ADC3_IN4, which is
    // connected to ground. (There is no negative pin to borrow.)
    impl<'a> From<(&'a Adc3In3<Differential>, &'a PE13<Analog>)> for Adc3Channel<'a> {
        fn from(_: (&'a Adc3In3<Differential>, &'a PE13<Analog>)) -> Adc3Channel<'a> {
            Adc3Channel {
                id: Adc3ChannelId::Adc3In3,
                life: PhantomData,
            }
        }
    }

    // The positive side of ADC3_IN4 is connected to ground. (There is no
    // positive pin to borrow.)
    impl<'a> From<&'a Adc3In4<SingleEnded>> for Adc3Channel<'a> {
        fn from(_: &'a Adc3In4<SingleEnded>) -> Adc3Channel<'a> {
            Adc3Channel {
                id: Adc3ChannelId::Adc3In4,
                life: PhantomData,
            }
        }
    }

    // The positive side of ADC3_IN4 is connected to ground. (There is no
    // positive pin to borrow.)
    impl<'a> From<(&'a Adc3In4<Differential>, &'a PB13<Analog>)> for Adc3Channel<'a> {
        fn from(_: (&'a Adc3In4<Differential>, &'a PB13<Analog>)) -> Adc3Channel<'a> {
            Adc3Channel {
                id: Adc3ChannelId::Adc3In4,
                life: PhantomData,
            }
        }
    }

    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc3In5, PB13, PE8);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc34In6, PE8, PD10);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc34In7, PD10, PD11);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc34In8, PD11, PD12);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc34In9, PD12, PD13);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc34In10, PD13, PD14);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc34In11, PD14, PB0);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc3In12, PB0, PE7);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc3In13, PE7, PE10);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc3In14, PE10, PE11);
    impl_channel_from_pins!(Adc3Channel, Adc3ChannelId, Adc3In15, PE11, PE12);

    impl<'a> From<(&'a Adc3In16, &'a PE12<Analog>)> for Adc3Channel<'a> {
        fn from(_: (&'a Adc3In16, &'a PE12<Analog>)) -> Adc3Channel<'a> {
            Adc3Channel {
                id: Adc3ChannelId::Adc3In16,
                life: PhantomData,
            }
        }
    }

    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc4In1, PE14, PE15);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc4In2, PE15, PB12);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc4In3, PB12, PB14);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc4In4, PB14, PB15);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc4In5, PB15, PE8);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc34In6, PE8, PD10);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc34In7, PD10, PD11);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc34In8, PD11, PD12);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc34In9, PD12, PD13);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc34In10, PD13, PD14);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc34In11, PD14, PD8);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc4In12, PD8, PD9);
    impl_channel_from_pins!(Adc4Channel, Adc4ChannelId, Adc4In13, PD9);

    // The negative side of ADC4_IN13 is connected to V_REF-. (There is no
    // negative pin to borrow.)
    impl<'a> From<(&'a Adc4In13<Differential>, &'a PD9<Analog>)> for Adc4Channel<'a> {
        fn from(_: (&'a Adc4In13<Differential>, &'a PD9<Analog>)) -> Adc4Channel<'a> {
            Adc4Channel {
                id: Adc4ChannelId::Adc4In13,
                life: PhantomData,
            }
        }
    }

    impl<'a> From<&'a mut InternalRef> for Adc3Channel<'a> {
        fn from(_: &'a mut InternalRef) -> Adc3Channel<'a> {
            Adc3Channel {
                id: Adc3ChannelId::InternalRef,
                life: PhantomData,
            }
        }
    }

    impl<'a> From<&'a mut InternalRef> for Adc4Channel<'a> {
        fn from(_: &'a mut InternalRef) -> Adc4Channel<'a> {
            Adc4Channel {
                id: Adc4ChannelId::InternalRef,
                life: PhantomData,
            }
        }
    }

    impl Adc34Ext for stm32f30x::ADC3_4 {
        fn split(
            self,
            adc3: stm32f30x::ADC3,
            adc4: stm32f30x::ADC4,
            ahb: &mut AHB,
            clocks: Clocks,
        ) -> (Adc34<Independent, Unpowered, Unpowered>, Adc34Channels) {
            // Enable and reset ADC.
            ahb.enr().modify(|_, w| w.adc34en().enabled());
            ahb.rstr().modify(|_, w| w.adc34rst().set_bit());
            ahb.rstr().modify(|_, w| w.adc34rst().clear_bit());

            // Use HCLK/2 as ADC clock.
            self.ccr.modify(|_, w| unsafe { w.ckmode().bits(0b10) });

            let clock_freq = clocks.hclk() / 2;
            (
                Adc34 {
                    adc3: Adc3 {
                        clock_freq,
                        reg: adc3,
                        pair_state: PhantomData,
                        state: Unpowered {},
                    },
                    adc4: Adc4 {
                        clock_freq,
                        reg: adc4,
                        pair_state: PhantomData,
                        state: Unpowered {},
                    },
                },
                Adc34Channels {
                    adc3_in1: Adc3In1 {
                        _state: PhantomData,
                    },
                    adc3_in2: Adc3In2 {
                        _state: PhantomData,
                    },
                    adc3_in3: Adc3In3 {
                        _state: PhantomData,
                    },
                    adc3_in4: Adc3In4 {
                        _state: PhantomData,
                    },
                    adc3_in5: Adc3In5 {
                        _state: PhantomData,
                    },
                    adc3_in12: Adc3In12 {
                        _state: PhantomData,
                    },
                    adc3_in13: Adc3In13 {
                        _state: PhantomData,
                    },
                    adc3_in14: Adc3In14 {
                        _state: PhantomData,
                    },
                    adc3_in15: Adc3In15 {
                        _state: PhantomData,
                    },
                    adc3_in16: Adc3In16 {
                        _state: PhantomData,
                    },
                    adc4_in1: Adc4In1 {
                        _state: PhantomData,
                    },
                    adc4_in2: Adc4In2 {
                        _state: PhantomData,
                    },
                    adc4_in3: Adc4In3 {
                        _state: PhantomData,
                    },
                    adc4_in4: Adc4In4 {
                        _state: PhantomData,
                    },
                    adc4_in5: Adc4In5 {
                        _state: PhantomData,
                    },
                    adc4_in12: Adc4In12 {
                        _state: PhantomData,
                    },
                    adc4_in13: Adc4In13 {
                        _state: PhantomData,
                    },
                    adc34_in6: Adc34In6 {
                        _state: PhantomData,
                    },
                    adc34_in7: Adc34In7 {
                        _state: PhantomData,
                    },
                    adc34_in8: Adc34In8 {
                        _state: PhantomData,
                    },
                    adc34_in9: Adc34In9 {
                        _state: PhantomData,
                    },
                    adc34_in10: Adc34In10 {
                        _state: PhantomData,
                    },
                    adc34_in11: Adc34In11 {
                        _state: PhantomData,
                    },
                },
            )
        }
    }

    /// ADC3 and ADC4 channel singletons.
    ///
    /// Note that this does not include the special internal voltage reference
    /// channel, which must be explicitly enabled to be used.
    pub struct Adc34Channels {
        /// ADC3_IN1
        pub adc3_in1: Adc3In1<SingleEnded>,
        /// ADC3_IN2
        pub adc3_in2: Adc3In2<SingleEnded>,
        /// ADC3_IN3
        pub adc3_in3: Adc3In3<SingleEnded>,
        /// ADC3_IN4; the positive side is not bonded and is connected to ground (Vss)
        pub adc3_in4: Adc3In4<SingleEnded>,
        /// ADC3_IN5
        pub adc3_in5: Adc3In5<SingleEnded>,
        /// ADC3_IN12
        pub adc3_in12: Adc3In12<SingleEnded>,
        /// ADC3_IN13
        pub adc3_in13: Adc3In13<SingleEnded>,
        /// ADC3_IN14
        pub adc3_in14: Adc3In14<SingleEnded>,
        /// ADC3_IN15
        pub adc3_in15: Adc3In15<SingleEnded>,
        /// ADC3_IN16
        pub adc3_in16: Adc3In16,
        /// ADC4_IN1
        pub adc4_in1: Adc4In1<SingleEnded>,
        /// ADC4_IN2
        pub adc4_in2: Adc4In2<SingleEnded>,
        /// ADC4_IN3
        pub adc4_in3: Adc4In3<SingleEnded>,
        /// ADC4_IN4
        pub adc4_in4: Adc4In4<SingleEnded>,
        /// ADC4_IN5
        pub adc4_in5: Adc4In5<SingleEnded>,
        /// ADC4_IN12
        pub adc4_in12: Adc4In12<SingleEnded>,
        /// ADC4_IN13
        pub adc4_in13: Adc4In13<SingleEnded>,
        /// ADC34_IN6
        pub adc34_in6: Adc34In6<SingleEnded>,
        /// ADC34_IN7
        pub adc34_in7: Adc34In7<SingleEnded>,
        /// ADC34_IN8
        pub adc34_in8: Adc34In8<SingleEnded>,
        /// ADC34_IN9
        pub adc34_in9: Adc34In9<SingleEnded>,
        /// ADC34_IN10
        pub adc34_in10: Adc34In10<SingleEnded>,
        /// ADC34_IN11
        pub adc34_in11: Adc34In11<SingleEnded>,
    }

    /// ADC3_IN1
    pub struct Adc3In1<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN2
    pub struct Adc3In2<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN3
    pub struct Adc3In3<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN4; the positive side is not bonded and is connected to ground (Vss)
    pub struct Adc3In4<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN5
    pub struct Adc3In5<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN12
    pub struct Adc3In12<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN13
    pub struct Adc3In13<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN14
    pub struct Adc3In14<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN15
    pub struct Adc3In15<S> {
        _state: PhantomData<S>,
    }

    /// ADC3_IN16 (fixed as single-ended mode)
    pub struct Adc3In16 {
        // The reference manual indicates that ADC3_IN16 is fixed as
        // single-ended mode in some places, but suggests othewise in other
        // places. The negative side is connected to V_REF-, though, so there's
        // no point in configuring it into differential mode anyway.
        _state: PhantomData<SingleEnded>,
    }

    /// ADC4_IN1
    pub struct Adc4In1<S> {
        _state: PhantomData<S>,
    }

    /// ADC4_IN2
    pub struct Adc4In2<S> {
        _state: PhantomData<S>,
    }

    /// ADC4_IN3
    pub struct Adc4In3<S> {
        _state: PhantomData<S>,
    }

    /// ADC4_IN4
    pub struct Adc4In4<S> {
        _state: PhantomData<S>,
    }

    /// ADC4_IN5
    pub struct Adc4In5<S> {
        _state: PhantomData<S>,
    }

    /// ADC4_IN12
    pub struct Adc4In12<S> {
        _state: PhantomData<S>,
    }

    /// ADC4_IN13
    pub struct Adc4In13<S> {
        _state: PhantomData<S>,
    }

    /// ADC34_IN6
    pub struct Adc34In6<S> {
        _state: PhantomData<S>,
    }

    /// ADC34_IN7
    pub struct Adc34In7<S> {
        _state: PhantomData<S>,
    }

    /// ADC34_IN8
    pub struct Adc34In8<S> {
        _state: PhantomData<S>,
    }

    /// ADC34_IN9
    pub struct Adc34In9<S> {
        _state: PhantomData<S>,
    }

    /// ADC34_IN10
    pub struct Adc34In10<S> {
        _state: PhantomData<S>,
    }

    /// ADC34_IN11
    pub struct Adc34In11<S> {
        _state: PhantomData<S>,
    }

    impl_channel_conversions!(Adc34, Adc3In1, Adc3In2, [(adc3, 1)]);
    impl_channel_conversions!(Adc34, Adc3In2, Adc3In3, [(adc3, 2)]);
    impl_channel_conversions!(Adc34, Adc3In3, Adc3In4, [(adc3, 3)]);
    impl_channel_conversions!(Adc34, Adc3In4, Adc3In5, [(adc3, 4)]);
    impl_channel_conversions_shared_neg!((Adc34, adc3, adc4), adc3, Adc3In5, Adc4In5, Adc34In6, 5);
    impl_channel_conversions!(Adc34, Adc3In12, Adc3In13, [(adc3, 12)]);
    impl_channel_conversions!(Adc34, Adc3In13, Adc3In14, [(adc3, 13)]);
    impl_channel_conversions!(Adc34, Adc3In14, Adc3In15, [(adc3, 14)]);

    impl Adc3In15<SingleEnded> {
        /// Changes the channel to differential mode, where `self` is the
        /// positive input and `_neg` is the negative input.
        pub fn into_differential<P>(
            self,
            _neg: Adc3In16,
            pair: &mut Adc34<P, Disabled, Disabled>,
        ) -> Adc3In15<Differential> {
            const CHANNEL_NUM: u8 = 15;
            pair.adc3.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() | (1 << CHANNEL_NUM))
            });
            Adc3In15 {
                _state: PhantomData,
            }
        }
    }

    impl Adc3In15<Differential> {
        /// Changes the channel to single-ended mode.
        pub fn into_single_ended<P>(
            self,
            pair: &mut Adc34<P, Disabled, Disabled>,
        ) -> (Adc3In15<SingleEnded>, Adc3In16) {
            const CHANNEL_NUM: u8 = 15;
            pair.adc3.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() & !(1 << CHANNEL_NUM))
            });
            (
                Adc3In15 {
                    _state: PhantomData,
                },
                Adc3In16 {
                    _state: PhantomData,
                },
            )
        }
    }

    impl_channel_conversions!(Adc34, Adc4In1, Adc4In2, [(adc4, 1)]);
    impl_channel_conversions!(Adc34, Adc4In2, Adc4In3, [(adc4, 2)]);
    impl_channel_conversions!(Adc34, Adc4In3, Adc4In4, [(adc4, 3)]);
    impl_channel_conversions!(Adc34, Adc4In4, Adc4In5, [(adc4, 4)]);
    impl_channel_conversions_shared_neg!((Adc34, adc3, adc4), adc4, Adc4In5, Adc3In5, Adc34In6, 5);
    impl_channel_conversions!(Adc34, Adc4In12, Adc4In13, [(adc4, 12)]);

    impl Adc4In13<SingleEnded> {
        /// Changes the channel to differential mode, where `self` is the positive input.
        ///
        /// The negative input for this channel is connected to `V_REF-`, so there isn't much point
        /// to changing the channel to differential mode, but doing so is allowed by the hardware.
        pub fn into_differential<P>(
            self,
            pair: &mut Adc34<P, Disabled, Disabled>,
        ) -> Adc4In13<Differential> {
            const CHANNEL_NUM: u8 = 13;
            pair.adc3.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() | (1 << CHANNEL_NUM))
            });
            pair.adc4.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() | (1 << CHANNEL_NUM))
            });
            Adc4In13 {
                _state: PhantomData,
            }
        }
    }

    impl Adc4In13<Differential> {
        /// Changes the channel to single-ended mode.
        pub fn into_single_ended<P>(
            self,
            pair: &mut Adc34<P, Disabled, Disabled>,
        ) -> Adc4In13<SingleEnded> {
            const CHANNEL_NUM: u8 = 13;
            pair.adc3.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() & !(1 << CHANNEL_NUM))
            });
            pair.adc4.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() & !(1 << CHANNEL_NUM))
            });
            Adc4In13 {
                _state: PhantomData,
            }
        }
    }

    impl_channel_conversions!(Adc34, Adc34In6, Adc34In7, [(adc3, 6), (adc4, 6)]);
    impl_channel_conversions!(Adc34, Adc34In7, Adc34In8, [(adc3, 7), (adc4, 7)]);
    impl_channel_conversions!(Adc34, Adc34In8, Adc34In9, [(adc3, 8), (adc4, 8)]);
    impl_channel_conversions!(Adc34, Adc34In9, Adc34In10, [(adc3, 9), (adc4, 9)]);
    impl_channel_conversions!(Adc34, Adc34In10, Adc34In11, [(adc3, 10), (adc4, 10)]);

    impl Adc34In11<SingleEnded> {
        /// Changes the channel to differential mode, where `self` is the
        /// positive input and `_neg` are the negative inputs.
        pub fn into_differential<P>(
            self,
            _neg: (Adc3In12<SingleEnded>, Adc4In12<SingleEnded>),
            pair: &mut Adc34<P, Disabled, Disabled>,
        ) -> Adc34In11<Differential> {
            const CHANNEL_NUM: u8 = 11;
            pair.adc3.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() | (1 << CHANNEL_NUM))
            });
            pair.adc4.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() | (1 << CHANNEL_NUM))
            });
            Adc34In11 {
                _state: PhantomData,
            }
        }
    }

    impl Adc34In11<Differential> {
        /// Changes the channel to single-ended mode.
        pub fn into_single_ended<P>(
            self,
            pair: &mut Adc34<P, Disabled, Disabled>,
        ) -> (
            Adc34In11<SingleEnded>,
            (Adc3In12<SingleEnded>, Adc4In12<SingleEnded>),
        ) {
            const CHANNEL_NUM: u8 = 11;
            pair.adc3.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() & !(1 << CHANNEL_NUM))
            });
            pair.adc4.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() & !(1 << CHANNEL_NUM))
            });
            (
                Adc34In11 {
                    _state: PhantomData,
                },
                (
                    Adc3In12 {
                        _state: PhantomData,
                    },
                    Adc4In12 {
                        _state: PhantomData,
                    },
                ),
            )
        }
    }
}
