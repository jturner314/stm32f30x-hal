//! Wrappers for ADC1 and ADC2.

pub use self::channels::{Adc12Channels, Adc1Channel, Adc1ChannelId, Adc2Channel, Adc2ChannelId};

use super::*;
use prelude::*;
use rcc::{Clocks, AHB};
use stm32f30x;
use time::Hertz;

/// Extension trait to split the ADC1 and ADC2 peripherals into a wrapper
/// and channels.
pub trait Adc12Ext {
    /// Enables the ADC1/2 clock and returns a wrapper and channels.
    fn split(
        self,
        adc1: stm32f30x::ADC1,
        adc2: stm32f30x::ADC2,
        ahb: &mut AHB,
        clocks: Clocks,
    ) -> (Adc12<Independent, Unpowered, Unpowered>, Adc12Channels);
}

/// ADC1 wrapper.
///
/// `P` is the state of the ADC pair, and `S` is the state of this ADC.
pub struct Adc1<P, S> {
    clock_freq: Hertz,
    reg: stm32f30x::ADC1,
    pair_state: PhantomData<P>,
    state: S,
}

/// ADC2 wrapper.
///
/// `P` is the state of the ADC pair, and `S` is the state of this ADC.
pub struct Adc2<P, S> {
    clock_freq: Hertz,
    reg: stm32f30x::ADC2,
    pair_state: PhantomData<P>,
    state: S,
}

/// ADC1 and ADC2 pair.
///
/// `P` is the state of the ADC pair, `S1` is the state of ADC1, and `S2`
/// is the state of ADC2.
pub struct Adc12<P, S1, S2> {
    /// ADC1.
    pub adc1: Adc1<P, S1>,
    /// ADC2.
    pub adc2: Adc2<P, S2>,
}

impl<P, S1, S2> Adc12<P, S1, S2> {
    // /// Returns the CSR register.
    // fn csr(&self) -> &stm32f30x::adc1_2::CSR {
    //     // The pair of ADCs has exclusive access to its common registers.
    //     unsafe { &(*stm32f30x::ADC1_2::ptr()).csr }
    // }

    /// Returns the CCR register.
    fn ccr_mut(&mut self) -> &stm32f30x::adc1_2::CCR {
        // The pair of ADCs has exclusive access to its common registers.
        unsafe { &(*stm32f30x::ADC1_2::ptr()).ccr }
    }

    /// Returns the CDR register.
    fn cdr(&self) -> &stm32f30x::adc1_2::CDR {
        // The pair of ADCs has exclusive access to its common registers.
        unsafe { &(*stm32f30x::ADC1_2::ptr()).cdr }
    }
}

impl<P, S1, S2> AdcPair for Adc12<P, S1, S2> {
    type Master = Adc1<P, S1>;
    type Slave = Adc2<P, S2>;
    #[inline]
    fn master(&self) -> &Self::Master {
        &self.adc1
    }
    #[inline]
    fn slave(&self) -> &Self::Slave {
        &self.adc2
    }
    #[inline]
    fn master_mut(&mut self) -> &mut Self::Master {
        &mut self.adc1
    }
    #[inline]
    fn slave_mut(&mut self) -> &mut Self::Slave {
        &mut self.adc2
    }
    #[inline]
    fn split(self) -> (Self::Master, Self::Slave) {
        (self.adc1, self.adc2)
    }
    #[inline]
    fn join(master: Self::Master, slave: Self::Slave) -> Self {
        Adc12 {
            adc1: master,
            adc2: slave,
        }
    }
}

/// Continuous iterator over regular conversions in dual simultaneous mode.
pub struct Adc12ContIter<'p, 'm: 'p, 's: 'p> {
    pair: &'p mut Adc12<Dual, WithSequence<'m>, WithSequence<'s>>,
}

impl_iterator_for_paircontiter!(Adc12ContIter, adc1);

impl_pair_unpowered_unpowered!(Adc12, Adc1, Adc2, adc1, adc2);

impl_pair_disabled_disabled!(Adc12, Adc1, Adc2, adc1, adc2);

impl_pair_dual_enabled_enabled!(Adc12, adc1, adc2, Adc1Channel, Adc2Channel);

impl_pair_withsequence_withsequence!(
    Adc12,
    Adc12ContIter,
    Adc1,
    Adc2,
    adc1,
    adc2,
    stm32f30x::ADC1_2::ptr(),
    stm32f30x::ADC1::ptr(),
);

impl_pair_dual_runningdma_runningdma!(Adc12, Adc1, Adc2, adc1, adc2);

impl_single_any!(Adc1, Adc1ChannelId);
impl_single_any!(Adc2, Adc2ChannelId);

impl_single_unpowered!(Adc1);
impl_single_unpowered!(Adc2);

impl_single_disabled!(Adc1);
impl_single_disabled!(Adc2);

impl_single_enabled!(Adc1, Adc1Channel);
impl_single_enabled!(Adc2, Adc2Channel);

impl_single_with_sequence!(Adc1, Adc1ChannelId);
impl_single_with_sequence!(Adc2, Adc2ChannelId);

impl_single_independent_with_sequence!(Adc1, ADC1);
impl_single_independent_with_sequence!(Adc2, ADC2);

impl_single_independent_running_dma!(Adc1, dma::dma1::Channel1);
impl_single_independent_running_dma!(Adc2, dma::dma2::Channel1);

unsafe impl<'scope, P, S> AdcDmaTokens<'scope, Adc1<P, S>> for dma::dma1::Channel1 {
    type Channel = dma::dma1::Channel1;
    fn channel(self) -> Self::Channel {
        self
    }
}

unsafe impl<'scope, P, S> AdcDmaTokens<'scope, Adc2<P, S>>
    for (
        dma::dma2::Channel1,
        &'scope syscfg::Adc24DmaRemap<syscfg::NotRemapped>,
    )
{
    type Channel = dma::dma2::Channel1;
    fn channel(self) -> Self::Channel {
        self.0
    }
}

unsafe impl<'scope, P, S> AdcDmaTokens<'scope, Adc2<P, S>>
    for (
        dma::dma2::Channel3,
        &'scope syscfg::Adc24DmaRemap<syscfg::Remapped>,
    )
{
    type Channel = dma::dma2::Channel3;
    fn channel(self) -> Self::Channel {
        self.0
    }
}

/// ADC1 and ADC2 channels.
pub mod channels {
    use super::*;
    use gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
    use gpio::gpiob::PB2;
    use gpio::gpioc::{PC0, PC1, PC2, PC3, PC4, PC5};
    use gpio::gpiof::{PF2, PF4};

    // TODO
    /// Op-amp 1
    pub struct OpAmp1 {
        _0: (),
    }

    /// Temperature sensor.
    ///
    /// An instance of this type can be obtained from the
    /// `.enable_temperature_sensor()` method on `Adc12`.
    pub struct TemperatureSensor {
        _0: (),
    }

    impl<P> Adc12<P, Disabled, Disabled> {
        /// Enables the temperature sensor and returns a handle to it.
        ///
        /// Returns `None` if the temperature sensor is already enabled.
        pub fn enable_temperature_sensor(&mut self) -> Option<TemperatureSensor> {
            let mut already_enabled = false;
            self.ccr_mut().modify(|r, w| {
                if r.tsen().bit_is_set() {
                    already_enabled = true;
                } else {
                    w.tsen().set_bit();
                }
                w
            });
            if already_enabled {
                None
            } else {
                Some(TemperatureSensor { _0: () })
            }
        }

        /// Disables the temperature sensor.
        pub fn disable_temperature_sensor(&mut self, _: TemperatureSensor) {
            self.ccr_mut().modify(|_, w| w.tsen().clear_bit());
        }
    }

    /// Battery voltage sensor (V_BAT/2).
    ///
    /// An instance of this type can be obtained from the
    /// `.enable_battery()` method on `Adc12`.
    pub struct HalfBattery {
        _0: (),
    }

    impl<P> Adc12<P, Disabled, Disabled> {
        /// Enables the battery voltage sensor and returns a handle to it.
        ///
        /// Returns `None` if the battery voltage sensor is already enabled.
        pub fn enable_battery_sensor(&mut self) -> Option<HalfBattery> {
            let mut already_enabled = false;
            self.ccr_mut().modify(|r, w| {
                if r.vbaten().bit_is_set() {
                    already_enabled = true;
                } else {
                    w.vbaten().set_bit();
                }
                w
            });
            if already_enabled {
                None
            } else {
                Some(HalfBattery { _0: () })
            }
        }

        /// Disables the battery voltage sensor.
        pub fn disable_battery_sensor(&mut self, _: HalfBattery) {
            self.ccr_mut().modify(|_, w| w.vbaten().clear_bit());
        }
    }

    define_impl_internalref!(Adc12);

    /// An ADC1 channel that has borrowed the necessary pins to be able to
    /// perform an ADC conversion.
    ///
    /// The only way to produce an `Adc1Channel` is through the `From`
    /// trait implementations; this ensures that while you have an
    /// `Adc1Channel` instance, the necessary pins are borrowed. It is not
    /// possible to copy/clone an `Adc1Channel` because the borrows could
    /// have been mutable.
    pub struct Adc1Channel<'a> {
        id: Adc1ChannelId,
        life: PhantomData<&'a ()>,
    }

    impl<'a> Adc1Channel<'a> {
        /// Returns the channel ID.
        pub fn id(&self) -> Adc1ChannelId {
            self.id
        }
    }

    /// An ADC2 channel that has borrowed the necessary pins to be able to
    /// perform an ADC conversion.
    ///
    /// The only way to produce an `Adc2Channel` is through the `From`
    /// trait implementations; this ensures that while you have an
    /// `Adc2Channel` instance, the necessary pins are borrowed. It is not
    /// possible to copy/clone an `Adc2Channel` because the borrows could
    /// have been mutable.
    pub struct Adc2Channel<'a> {
        id: Adc2ChannelId,
        life: PhantomData<&'a ()>,
    }

    impl<'a> Adc2Channel<'a> {
        /// Returns the channel ID.
        pub fn id(&self) -> Adc2ChannelId {
            self.id
        }
    }

    /// The numeric ID of an ADC1 channel.
    #[derive(Clone, Copy, Eq, PartialEq)]
    #[repr(u8)]
    pub enum Adc1ChannelId {
        /// ADC1_IN1
        Adc1In1 = 1,
        /// ADC1_IN2
        Adc1In2 = 2,
        /// ADC1_IN3
        Adc1In3 = 3,
        /// ADC1_IN4
        Adc1In4 = 4,
        /// ADC1_IN5
        Adc1In5 = 5,
        /// ADC12_IN6
        Adc12In6 = 6,
        /// ADC12_IN7
        Adc12In7 = 7,
        /// ADC12_IN8
        Adc12In8 = 8,
        /// ADC12_IN9
        Adc12In9 = 9,
        /// ADC12_IN10
        Adc12In10 = 10,
        /// ADC1_IN11
        Adc1In11 = 11,
        /// ADC1_IN12
        Adc1In12 = 12,
        /// ADC1_IN13
        Adc1In13 = 13,
        // Channel 14 is reserved.
        /// V_OPAMP1
        OpAmp1 = 15,
        /// Temperature sensor (V_TS)
        TemperatureSensor = 16,
        /// Battery voltage divided by two (V_BAT/2)
        HalfBattery = 17,
        /// Internal reference voltage (V_REFINT)
        InternalRef = 18,
    }

    impl Adc1ChannelId {
        /// Returns the channel number for the sequence registers.
        pub fn number(self) -> u8 {
            self as u8
        }

        /// Returns `true` if the channels must not be converted at the same time.
        pub fn conflicts_with(self, other: Adc2ChannelId) -> bool {
            match (self, other) {
                (Adc1ChannelId::Adc12In6, Adc2ChannelId::Adc12In6)
                | (Adc1ChannelId::Adc12In7, Adc2ChannelId::Adc12In7)
                | (Adc1ChannelId::Adc12In8, Adc2ChannelId::Adc12In8)
                | (Adc1ChannelId::Adc12In9, Adc2ChannelId::Adc12In9)
                | (Adc1ChannelId::Adc12In10, Adc2ChannelId::Adc12In10)
                | (Adc1ChannelId::InternalRef, Adc2ChannelId::InternalRef) => true,
                _ => false,
            }
        }
    }

    /// The numeric ID of an ADC2 channel.
    #[derive(Clone, Copy, Eq, PartialEq)]
    #[repr(u8)]
    pub enum Adc2ChannelId {
        /// ADC2_IN1
        Adc2In1 = 1,
        /// ADC2_IN2
        Adc2In2 = 2,
        /// ADC2_IN3
        Adc2In3 = 3,
        /// ADC2_IN4
        Adc2In4 = 4,
        /// ADC2_IN5
        Adc2In5 = 5,
        /// ADC12_IN6
        Adc12In6 = 6,
        /// ADC12_IN7
        Adc12In7 = 7,
        /// ADC12_IN8
        Adc12In8 = 8,
        /// ADC12_IN9
        Adc12In9 = 9,
        /// ADC12_IN10
        Adc12In10 = 10,
        /// ADC2_IN11
        Adc2In11 = 11,
        /// ADC2_IN12
        Adc2In12 = 12,
        /// ADC2_IN13
        Adc2In13 = 13,
        /// ADC2_IN14
        Adc2In14 = 14,
        /// ADC2_IN15
        Adc2In15 = 15,
        // Channel 16 is reserved.
        /// V_OPAMP2
        OpAmp2 = 17,
        /// Internal reference voltage (V_REFINT)
        InternalRef = 18,
    }

    impl Adc2ChannelId {
        /// Returns the channel number for the sequence registers.
        pub fn number(self) -> u8 {
            self as u8
        }

        /// Returns `true` if the channels must not be converted at the same time.
        pub fn conflicts_with(self, other: Adc1ChannelId) -> bool {
            other.conflicts_with(self)
        }
    }

    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In1, PA0, PA1);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In2, PA1, PA2);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In3, PA2, PA3);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In4, PA3, PF4);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In5, PF4, PC0);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In6, PC0, PC1);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In7, PC1, PC2);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In8, PC2, PC3);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In9, PC3, PF2);
    impl_channel_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In10, PF2);

    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In1, PA4, PA5);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In2, PA5, PA6);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In3, PA6, PA7);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In4, PA7, PC4);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In5, PC4, PC0);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In6, PC0, PC1);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In7, PC1, PC2);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In8, PC2, PC3);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In9, PC3, PF2);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In10, PF2, PC5);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In11, PC5, PB2);
    impl_channel_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In12, PB2);

    impl<'a> From<&'a TemperatureSensor> for Adc1Channel<'a> {
        fn from(_: &'a TemperatureSensor) -> Adc1Channel<'a> {
            Adc1Channel {
                id: Adc1ChannelId::TemperatureSensor,
                life: PhantomData,
            }
        }
    }

    impl<'a> From<&'a HalfBattery> for Adc1Channel<'a> {
        fn from(_: &'a HalfBattery) -> Adc1Channel<'a> {
            Adc1Channel {
                id: Adc1ChannelId::HalfBattery,
                life: PhantomData,
            }
        }
    }

    impl<'a> From<&'a mut InternalRef> for Adc1Channel<'a> {
        fn from(_: &'a mut InternalRef) -> Adc1Channel<'a> {
            Adc1Channel {
                id: Adc1ChannelId::InternalRef,
                life: PhantomData,
            }
        }
    }

    impl<'a> From<&'a mut InternalRef> for Adc2Channel<'a> {
        fn from(_: &'a mut InternalRef) -> Adc2Channel<'a> {
            Adc2Channel {
                id: Adc2ChannelId::InternalRef,
                life: PhantomData,
            }
        }
    }

    impl Adc12Ext for stm32f30x::ADC1_2 {
        fn split(
            self,
            adc1: stm32f30x::ADC1,
            adc2: stm32f30x::ADC2,
            ahb: &mut AHB,
            clocks: Clocks,
        ) -> (Adc12<Independent, Unpowered, Unpowered>, Adc12Channels) {
            // Enable and reset ADC.
            ahb.enr().modify(|_, w| w.adc12en().enabled());
            ahb.rstr().modify(|_, w| w.adc12rst().set_bit());
            ahb.rstr().modify(|_, w| w.adc12rst().clear_bit());

            // Use HCLK/2 as ADC clock.
            self.ccr.modify(|_, w| unsafe { w.ckmode().bits(0b10) });

            let clock_freq = clocks.hclk() / 2;
            (
                Adc12 {
                    adc1: Adc1 {
                        clock_freq,
                        reg: adc1,
                        pair_state: PhantomData,
                        state: Unpowered {},
                    },
                    adc2: Adc2 {
                        clock_freq,
                        reg: adc2,
                        pair_state: PhantomData,
                        state: Unpowered {},
                    },
                },
                Adc12Channels {
                    adc1_in1: Adc1In1 {
                        _state: PhantomData,
                    },
                    adc1_in2: Adc1In2 {
                        _state: PhantomData,
                    },
                    adc1_in3: Adc1In3 {
                        _state: PhantomData,
                    },
                    adc1_in4: Adc1In4 {
                        _state: PhantomData,
                    },
                    adc1_in5: Adc1In5 {
                        _state: PhantomData,
                    },
                    adc1_in11: Adc1In11 {
                        _state: PhantomData,
                    },
                    adc1_in12: Adc1In12 {
                        _state: PhantomData,
                    },
                    adc1_in13: Adc1In13 {
                        _state: PhantomData,
                    },
                    adc2_in1: Adc2In1 {
                        _state: PhantomData,
                    },
                    adc2_in2: Adc2In2 {
                        _state: PhantomData,
                    },
                    adc2_in3: Adc2In3 {
                        _state: PhantomData,
                    },
                    adc2_in4: Adc2In4 {
                        _state: PhantomData,
                    },
                    adc2_in5: Adc2In5 {
                        _state: PhantomData,
                    },
                    adc2_in11: Adc2In11 {
                        _state: PhantomData,
                    },
                    adc2_in12: Adc2In12 {
                        _state: PhantomData,
                    },
                    adc2_in13: Adc2In13 {
                        _state: PhantomData,
                    },
                    adc2_in14: Adc2In14 {
                        _state: PhantomData,
                    },
                    adc2_in15: Adc2In15 {
                        _state: PhantomData,
                    },
                    adc12_in6: Adc12In6 {
                        _state: PhantomData,
                    },
                    adc12_in7: Adc12In7 {
                        _state: PhantomData,
                    },
                    adc12_in8: Adc12In8 {
                        _state: PhantomData,
                    },
                    adc12_in9: Adc12In9 {
                        _state: PhantomData,
                    },
                    adc12_in10: Adc12In10 {
                        _state: PhantomData,
                    },
                },
            )
        }
    }

    /// ADC1 and ADC2 channel singletons.
    ///
    /// Note that this does not include the special channels (e.g. the
    /// temperature sensor and internal voltage reference), which must
    /// be explicitly enabled to be used.
    pub struct Adc12Channels {
        /// ADC1_IN1
        pub adc1_in1: Adc1In1<SingleEnded>,
        /// ADC1_IN2
        pub adc1_in2: Adc1In2<SingleEnded>,
        /// ADC1_IN3
        pub adc1_in3: Adc1In3<SingleEnded>,
        /// ADC1_IN4
        pub adc1_in4: Adc1In4<SingleEnded>,
        /// ADC1_IN5 (not available on all devices)
        pub adc1_in5: Adc1In5<SingleEnded>,
        /// ADC1_IN11 (not available on all devices)
        pub adc1_in11: Adc1In11<SingleEnded>,
        /// ADC1_IN12 (not available on all devices)
        pub adc1_in12: Adc1In12<SingleEnded>,
        /// ADC1_IN13 (not available on all devices)
        pub adc1_in13: Adc1In13<SingleEnded>,
        /// ADC2_IN1
        pub adc2_in1: Adc2In1<SingleEnded>,
        /// ADC2_IN2
        pub adc2_in2: Adc2In2<SingleEnded>,
        /// ADC2_IN3
        pub adc2_in3: Adc2In3<SingleEnded>,
        /// ADC2_IN4
        pub adc2_in4: Adc2In4<SingleEnded>,
        /// ADC2_IN5
        pub adc2_in5: Adc2In5<SingleEnded>,
        /// ADC2_IN11
        pub adc2_in11: Adc2In11<SingleEnded>,
        /// ADC2_IN12
        pub adc2_in12: Adc2In12<SingleEnded>,
        /// ADC2_IN13 (not available on all devices)
        pub adc2_in13: Adc2In13<SingleEnded>,
        /// ADC2_IN14 (not available on all devices)
        pub adc2_in14: Adc2In14<SingleEnded>,
        /// ADC2_IN15 (not available on all devices)
        pub adc2_in15: Adc2In15<SingleEnded>,
        /// ADC12_IN6
        pub adc12_in6: Adc12In6<SingleEnded>,
        /// ADC12_IN7
        pub adc12_in7: Adc12In7<SingleEnded>,
        /// ADC12_IN8
        pub adc12_in8: Adc12In8<SingleEnded>,
        /// ADC12_IN9
        pub adc12_in9: Adc12In9<SingleEnded>,
        /// ADC12_IN10 (not available on all devices)
        pub adc12_in10: Adc12In10<SingleEnded>,
    }

    /// ADC1_IN1
    pub struct Adc1In1<S> {
        _state: PhantomData<S>,
    }

    /// ADC1_IN2
    pub struct Adc1In2<S> {
        _state: PhantomData<S>,
    }

    /// ADC1_IN3
    pub struct Adc1In3<S> {
        _state: PhantomData<S>,
    }

    /// ADC1_IN4
    pub struct Adc1In4<S> {
        _state: PhantomData<S>,
    }

    /// ADC1_IN5 (not available on all devices)
    pub struct Adc1In5<S> {
        _state: PhantomData<S>,
    }

    /// ADC1_IN11 (not available on all devices)
    pub struct Adc1In11<S> {
        _state: PhantomData<S>,
    }

    /// ADC1_IN12 (not available on all devices)
    pub struct Adc1In12<S> {
        _state: PhantomData<S>,
    }

    /// ADC1_IN13 (not available on all devices)
    pub struct Adc1In13<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN1
    pub struct Adc2In1<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN2
    pub struct Adc2In2<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN3
    pub struct Adc2In3<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN4
    pub struct Adc2In4<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN5
    pub struct Adc2In5<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN11
    pub struct Adc2In11<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN12
    pub struct Adc2In12<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN13 (not available on all devices)
    pub struct Adc2In13<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN14 (not available on all devices)
    pub struct Adc2In14<S> {
        _state: PhantomData<S>,
    }

    /// ADC2_IN15 (not available on all devices)
    pub struct Adc2In15<S> {
        _state: PhantomData<S>,
    }

    /// ADC12_IN6
    pub struct Adc12In6<S> {
        _state: PhantomData<S>,
    }

    /// ADC12_IN7
    pub struct Adc12In7<S> {
        _state: PhantomData<S>,
    }

    /// ADC12_IN8
    pub struct Adc12In8<S> {
        _state: PhantomData<S>,
    }

    /// ADC12_IN9
    pub struct Adc12In9<S> {
        _state: PhantomData<S>,
    }

    /// ADC12_IN10 (not available on all devices)
    pub struct Adc12In10<S> {
        _state: PhantomData<S>,
    }

    impl_channel_conversions!(Adc12, Adc1In1, Adc1In2, [(adc1, Adc1ChannelId)]);
    impl_channel_conversions!(Adc12, Adc1In2, Adc1In3, [(adc1, Adc1ChannelId)]);
    impl_channel_conversions!(Adc12, Adc1In3, Adc1In4, [(adc1, Adc1ChannelId)]);
    impl_channel_conversions!(Adc12, Adc1In4, Adc1In5, [(adc1, Adc1ChannelId)]);
    impl_channel_conversions_shared_neg!(
        Adc12,
        (adc1, Adc1ChannelId, Adc1In5),
        (adc2, Adc2ChannelId, Adc2In5),
        Adc12In6
    );
    impl_channel_conversions!(Adc12, Adc1In11, Adc1In12, [(adc1, Adc1ChannelId)]);
    impl_channel_conversions!(Adc12, Adc1In12, Adc1In13, [(adc1, Adc1ChannelId)]);

    impl_channel_conversions!(Adc12, Adc2In1, Adc2In2, [(adc2, Adc2ChannelId)]);
    impl_channel_conversions!(Adc12, Adc2In2, Adc2In3, [(adc2, Adc2ChannelId)]);
    impl_channel_conversions!(Adc12, Adc2In3, Adc2In4, [(adc2, Adc2ChannelId)]);
    impl_channel_conversions!(Adc12, Adc2In4, Adc2In5, [(adc2, Adc2ChannelId)]);
    impl_channel_conversions_shared_neg!(
        Adc12,
        (adc2, Adc2ChannelId, Adc2In5),
        (adc1, Adc1ChannelId, Adc1In5),
        Adc12In6
    );
    impl_channel_conversions!(Adc12, Adc2In11, Adc2In12, [(adc2, Adc2ChannelId)]);
    impl_channel_conversions!(Adc12, Adc2In12, Adc2In13, [(adc2, Adc2ChannelId)]);
    impl_channel_conversions!(Adc12, Adc2In13, Adc2In14, [(adc2, Adc2ChannelId)]);
    impl_channel_conversions!(Adc12, Adc2In14, Adc2In15, [(adc2, Adc2ChannelId)]);

    impl_channel_conversions!(
        Adc12,
        Adc12In6,
        Adc12In7,
        [(adc1, Adc1ChannelId), (adc2, Adc2ChannelId)]
    );
    impl_channel_conversions!(
        Adc12,
        Adc12In7,
        Adc12In8,
        [(adc1, Adc1ChannelId), (adc2, Adc2ChannelId)]
    );
    impl_channel_conversions!(
        Adc12,
        Adc12In8,
        Adc12In9,
        [(adc1, Adc1ChannelId), (adc2, Adc2ChannelId)]
    );
    impl_channel_conversions!(
        Adc12,
        Adc12In9,
        Adc12In10,
        [(adc1, Adc1ChannelId), (adc2, Adc2ChannelId)]
    );

    impl Adc12In10<SingleEnded> {
        /// Changes the channel to differential mode, where `self` is the
        /// positive input and `_neg` are the negative inputs.
        pub fn into_differential<P>(
            self,
            _neg: (Adc1In11<SingleEnded>, Adc2In11<SingleEnded>),
            pair: &mut Adc12<P, Disabled, Disabled>,
        ) -> Adc12In10<Differential> {
            unsafe {
                pair
                    .adc1
                    .set_differential_unchecked(Adc1ChannelId::Adc12In10);
                pair
                    .adc2
                    .set_differential_unchecked(Adc2ChannelId::Adc12In10);
            }
            Adc12In10 {
                _state: PhantomData,
            }
        }
    }

    impl Adc12In10<Differential> {
        /// Changes the channel to single-ended mode.
        pub fn into_single_ended<P>(
            self,
            pair: &mut Adc12<P, Disabled, Disabled>,
        ) -> (
            Adc12In10<SingleEnded>,
            (Adc1In11<SingleEnded>, Adc2In11<SingleEnded>),
        ) {
            unsafe {
                pair
                    .adc1
                    .set_single_ended_unchecked(Adc1ChannelId::Adc12In10);
                pair
                    .adc2
                    .set_single_ended_unchecked(Adc2ChannelId::Adc12In10);
            }
            (
                Adc12In10 {
                    _state: PhantomData,
                },
                (
                    Adc1In11 {
                        _state: PhantomData,
                    },
                    Adc2In11 {
                        _state: PhantomData,
                    },
                ),
            )
        }
    }
}
