//! Wrappers for ADC1 and ADC2.

pub use self::channels::{Adc12Channels, Adc1Channel, Adc1ChannelId, Adc2Channel, Adc2ChannelId};

use super::*;
use core::cmp;
use delay::Delay;
use dma::DmaChannelPriv;
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

/// Continuous iterator over regular conversions in dual simultaneous mode.
pub struct Adc12ContIter<'adc12, 'seq1: 'adc12, 'seq2: 'adc12> {
    adc12: &'adc12 mut Adc12<Dual, WithSequence<'seq1>, WithSequence<'seq2>>,
}

impl<'adc12, 'seq1, 'seq2> Iterator for Adc12ContIter<'adc12, 'seq1, 'seq2> {
    type Item = (u16, u16);

    fn next(&mut self) -> Option<(u16, u16)> {
        while self.adc12.adc1.reg.isr.read().eoc().bit_is_clear() {}
        let common_data = self.adc12.cdr().read();
        // Manually clear EOC flag since the hardware doesn't
        // automatically do so when reading CDR.
        self.adc12.adc1.reg.isr.write(|w| w.eoc().set_bit());
        Some((
            common_data.rdata_mst().bits(),
            common_data.rdata_slv().bits(),
        ))
    }
}

impl<P, S1, S2> Adc12<P, S1, S2> {
    /// Returns the ADC clock frequency.
    ///
    /// This value is shared for both ADCs in the pair.
    pub fn clock_freq(&self) -> Hertz {
        self.adc1.clock_freq
    }
}

impl<P> Adc12<P, Unpowered, Unpowered> {
    /// Enable voltage regulator for both ADCs.
    ///
    /// This avoids having to wait 10 us for each ADC separately.
    pub fn power_on(self, delay: &mut Delay) -> Adc12<P, Disabled, Disabled> {
        self.adc1
            .reg
            .cr
            .write(|w| w.deeppwd().clear_bit().advregen().clear_bit());
        self.adc1
            .reg
            .cr
            .write(|w| w.deeppwd().clear_bit().advregen().set_bit());
        self.adc2
            .reg
            .cr
            .write(|w| w.deeppwd().clear_bit().advregen().clear_bit());
        self.adc2
            .reg
            .cr
            .write(|w| w.deeppwd().clear_bit().advregen().set_bit());
        delay.delay_us(10u8);
        Adc12 {
            adc1: Adc1 {
                clock_freq: self.adc1.clock_freq,
                reg: self.adc1.reg,
                pair_state: self.adc1.pair_state,
                state: Disabled {},
            },
            adc2: Adc2 {
                clock_freq: self.adc2.clock_freq,
                reg: self.adc2.reg,
                pair_state: self.adc2.pair_state,
                state: Disabled {},
            },
        }
    }
}

impl<P> Adc12<P, Disabled, Disabled> {
    /// Enables independent mode.
    pub fn into_independent(mut self) -> Adc12<Independent, Disabled, Disabled> {
        const INDEPENDENT: u8 = 0b00000;
        self.ccr_mut()
            .modify(|_, w| unsafe { w.mult().bits(INDEPENDENT) });
        Adc12 {
            adc1: Adc1 {
                clock_freq: self.adc1.clock_freq,
                reg: self.adc1.reg,
                pair_state: PhantomData,
                state: self.adc1.state,
            },
            adc2: Adc2 {
                clock_freq: self.adc2.clock_freq,
                reg: self.adc2.reg,
                pair_state: PhantomData,
                state: self.adc2.state,
            },
        }
    }

    /// Enables dual mode.
    pub fn into_dual(mut self) -> Adc12<Dual, Disabled, Disabled> {
        const REG_SIMUL_INJ_SIMUL: u8 = 0b00001;
        self.ccr_mut()
            .modify(|_, w| unsafe { w.mult().bits(REG_SIMUL_INJ_SIMUL) });
        Adc12 {
            adc1: Adc1 {
                clock_freq: self.adc1.clock_freq,
                reg: self.adc1.reg,
                pair_state: PhantomData,
                state: self.adc1.state,
            },
            adc2: Adc2 {
                clock_freq: self.adc2.clock_freq,
                reg: self.adc2.reg,
                pair_state: PhantomData,
                state: self.adc2.state,
            },
        }
    }

    /// Enables both ADCs. They should be calibrated first.
    ///
    /// This avoids waiting separately for each ADC to finish its
    /// initialization sequence.
    pub fn enable(self) -> Adc12<P, Enabled, Enabled> {
        self.adc1.reg.cr.modify(|_, w| w.aden().set_bit());
        self.adc2.reg.cr.modify(|_, w| w.aden().set_bit());
        while self.adc1.reg.isr.read().adrdy().bit_is_clear()
            && self.adc2.reg.isr.read().adrdy().bit_is_clear()
        {}
        Adc12 {
            adc1: Adc1 {
                clock_freq: self.adc1.clock_freq,
                reg: self.adc1.reg,
                pair_state: self.adc1.pair_state,
                state: Enabled {},
            },
            adc2: Adc2 {
                clock_freq: self.adc2.clock_freq,
                reg: self.adc2.reg,
                pair_state: self.adc2.pair_state,
                state: Enabled {},
            },
        }
    }
}

impl Adc12<Dual, Enabled, Enabled> {
    /// Sets the sequences for regular conversion.
    ///
    /// Returns `Err` if any of the following are true:
    ///
    /// * There are conflicting channels (i.e. electrically connected
    ///   channels that would be converted at the same time in the
    ///   sequence).
    ///
    /// * The sequences have different lengths.
    ///
    /// * The length of a sequence is less than 1 or greater than 16.
    ///
    /// In the case of `Err`, `self` is returned unchanged.
    pub fn with_sequences<'seq1, 'seq2>(
        self,
        adc1_sequence: &[Adc1Channel<'seq1>],
        adc2_sequence: &[Adc2Channel<'seq2>],
    ) -> Result<
        Adc12<Dual, WithSequence<'seq1>, WithSequence<'seq2>>,
        (Adc12<Dual, Enabled, Enabled>, DualSequenceError),
    > {
        if adc1_sequence.len() != adc2_sequence.len() {
            return Err((self, DualSequenceError::UnequalLen));
        }
        if adc1_sequence.len() < 1 || adc1_sequence.len() > 16 {
            return Err((self, DualSequenceError::BadLen));
        }
        for (chan1, chan2) in adc1_sequence.iter().zip(adc2_sequence) {
            if chan1.id().conflicts_with(chan2.id()) {
                return Err((self, DualSequenceError::Conflict));
            }
        }
        Ok(Adc12 {
            adc1: unsafe { self.adc1.with_sequence_unchecked(adc1_sequence) },
            adc2: unsafe { self.adc2.with_sequence_unchecked(adc2_sequence) },
        })
    }
}

impl<'seq1, 'seq2> Adc12<Dual, WithSequence<'seq1>, WithSequence<'seq2>> {
    /// Sets the ADCs to single conversion mode; runs the regular sequence;
    /// and for each conversion in the sequence, calls `f` with the
    /// (zero-based) index within the sequence and the pair of converted
    /// values.
    ///
    /// This method uses auto-delayed conversion mode (AUTDLY) to avoid
    /// overruns.
    pub fn run_sequence_once<F>(&mut self, mut f: F)
    where
        F: FnMut(u8, (u16, u16)),
    {
        debug_assert!(self.adc1.reg.cr.read().aden().bit_is_set());
        debug_assert!(self.adc1.reg.cr.read().addis().bit_is_clear());

        // Set the master ADC to single conversion mode with auto-delay.
        self.adc1.reg.cfgr.modify(|_, w| {
            w.cont().clear_bit();
            w.autdly().set_bit()
        });
        // Clear overrun, end-of-conversion, and end-of-sequence flags on
        // master ADC.
        self.adc1.reg.isr.write(|w| {
            w.ovr().set_bit();
            w.eoc().set_bit();
            w.eos().set_bit()
        });

        // Run sequence.
        let seq_len = self.adc1.sequence_len();
        self.adc1.reg.cr.modify(|_, w| w.adstart().set_bit());
        for i in 0..seq_len {
            while self.adc1.reg.isr.read().eoc().bit_is_clear() {}
            let common_data = self.cdr().read();
            // Manually clear EOC flag since the hardware doesn't
            // automatically do so when reading CDR.
            self.adc1.reg.isr.write(|w| w.eoc().set_bit());
            f(
                i,
                (
                    common_data.rdata_mst().bits(),
                    common_data.rdata_slv().bits(),
                ),
            );
        }
        let isr = self.adc1.reg.isr.read();
        debug_assert!(isr.eos().bit_is_set());
        // There should never be an overrun in auto-delayed mode.
        debug_assert!(isr.ovr().bit_is_clear());
    }

    /// Sets the ADCs to continuous conversion mode; starts the regular
    /// sequence; calls `f` with an (infinite) iterator over the converted
    /// values; and then stops the ADCs.
    ///
    /// This method uses auto-delayed conversion mode (AUTDLY) to avoid
    /// overruns.
    ///
    /// Note that the first conversion is performed immediately, and each
    /// subsequent conversion is performed immediately *after* reading each
    /// value by calling `.next()` on the iterator. This makes it possible
    /// for the ADC to perform conversions in parallel with user code, but
    /// it also means that if there's a significant amount of time between
    /// calls to `.next()`, the time between conversion and reading the
    /// converted value is large.
    pub fn with_running_cont<F, O>(&mut self, f: F) -> O
    where
        F: for<'a> FnOnce(Adc12ContIter<'a, 'seq1, 'seq2>) -> O,
    {
        debug_assert!(self.adc1.reg.cr.read().aden().bit_is_set());
        debug_assert!(self.adc1.reg.cr.read().addis().bit_is_clear());

        // Set the master ADC to continuous conversion mode with auto-delay.
        self.adc1.reg.cfgr.modify(|_, w| {
            w.cont().set_bit();
            w.autdly().set_bit()
        });
        // Clear overrun, end-of-conversion, and end-of-sequence flags on
        // master ADC.
        self.adc1.reg.isr.write(|w| {
            w.ovr().set_bit();
            w.eoc().set_bit();
            w.eos().set_bit()
        });

        // Start sequence.
        self.adc1.reg.cr.modify(|_, w| w.adstart().set_bit());
        // Run closure.
        let out = f(Adc12ContIter { adc12: self });
        // There should never be an overrun in auto-delayed mode.
        debug_assert!(self.adc1.reg.isr.read().ovr().bit_is_clear());
        // Stop the ADCs and wait for them to be stopped.
        self.adc1.reg.cr.modify(|_, w| w.adstp().set_bit());
        loop {
            let cr = self.adc1.reg.cr.read();
            if cr.adstp().bit_is_clear() && cr.adstart().bit_is_clear() {
                break;
            }
        }

        out
    }

    /// Sets the ADCs to continuous conversion mode with auto-delay;
    /// configures and enables the DMA channel for writing into the
    /// buffer in one shot mode; and starts the regular sequence.
    ///
    /// This method uses dual-DMA mode (`MDMA = 0b10`) so that only a
    /// single DMA channel is necessary.
    ///
    /// This method uses auto-delayed conversion mode (AUTDLY) to avoid
    /// overruns.
    pub fn start_dma<'body, 'scope, D>(
        mut self,
        buf: &'scope mut [[u16; 2]],
        mut guard: ScopeGuard<'body, 'scope, dma::WaitHandler<Option<fn()>>>,
        dma_tok: D,
    ) -> Adc12<
        Dual,
        RunningDmaMaster<'seq1, 'body, 'scope, D::Channel>,
        RunningDmaSlave<'seq2, 'body, 'scope>,
    >
    where
        'seq1: 'scope,
        'seq2: 'scope,
        D: AdcDmaTokens<'scope, Adc1<Dual, WithSequence<'seq1>>>,
    {
        debug_assert!(self.adc1.reg.cr.read().aden().bit_is_set());
        debug_assert!(self.adc1.reg.cr.read().addis().bit_is_clear());
        debug_assert!(self.adc1.reg.cr.read().adstart().bit_is_clear());
        debug_assert!(self.adc1.reg.cfgr.read().dmaen().bit_is_clear());
        debug_assert!(self.adc2.reg.cfgr.read().dmaen().bit_is_clear());

        // Set the master ADC to continuous conversion mode with auto-delay.
        self.adc1.reg.cfgr.modify(|_, w| {
            w.cont().set_bit();
            w.autdly().set_bit()
        });
        // Clear the MDMA bits to reset any pending DMA requests from this
        // ADC pair.
        self.ccr_mut().modify(|_, w| unsafe { w.mdma().bits(0b00) });
        // Clear overrun, end-of-conversion, and end-of-sequence flags.
        self.adc1.reg.isr.write(|w| {
            w.ovr().set_bit();
            w.eoc().set_bit();
            w.eos().set_bit()
        });

        // Set up the guard to wait for the ADCs to finish and disable generation of DMA
        // requests after the DMA transfer has completed.
        if let Some(handler) = guard.handler_mut() {
            handler.set_periph(Some(|| {
                // This is safe and we don't have to worry about concurrent access to the
                // register because:
                //
                // 1. The ADC has exclusive access to its register.
                //
                // 2. Since the running ADC has the `'body` lifetime from the `ScopeGuard`,
                //    the closure cannot not called while the running ADC is alive, so the
                //    running ADC cannot access the register concurrently with this
                //    closure.
                //
                // 3. The closure is disabled before returning the non-running ADC, so the
                //    non-running ADC cannot access the register with this
                //    closure.
                let adc1_reg = unsafe { &(*stm32f30x::ADC1::ptr()) };
                let adc12_reg = unsafe { &(*stm32f30x::ADC1_2::ptr()) };
                // The hardware should automatically stop the ADC after the DMA
                // transfer has completed. This loop is in case it takes the hardware a
                // little while to stop the ADC. (The reference manual does not specify
                // whether this is necessary.)
                while adc1_reg.cr.read().adstart().bit_is_set() {}
                // Disable generation of DMA requests.
                adc12_reg.ccr.modify(|_, w| unsafe { w.mdma().bits(0b00) });
            }));
        }

        // Configure and enable the DMA transfer.
        self.ccr_mut().modify(|_, w| {
            unsafe { w.mdma().bits(0b10) };
            w.dmacfg().clear_bit() // one-shot mode
        });
        let data_reg = self.cdr() as *const _ as *const [u16; 2];
        let transfer = unsafe { dma_tok.channel().enable_periph_to_mem(guard, data_reg, buf) };

        self.adc1.reg.cr.modify(|_, w| w.adstart().set_bit());
        Adc12 {
            adc1: Adc1 {
                clock_freq: self.adc1.clock_freq,
                reg: self.adc1.reg,
                pair_state: self.adc1.pair_state,
                state: RunningDmaMaster {
                    seq: self.adc1.state.life,
                    transfer,
                },
            },
            adc2: Adc2 {
                clock_freq: self.adc2.clock_freq,
                reg: self.adc2.reg,
                pair_state: self.adc2.pair_state,
                state: RunningDmaSlave {
                    seq: self.adc2.state.life,
                    body: PhantomData,
                    scope: PhantomData,
                },
            },
        }
    }
}

impl<'seq1, 'seq2, 'body, 'scope, Channel>
    Adc12<
        Dual,
        RunningDmaMaster<'seq1, 'body, 'scope, Channel>,
        RunningDmaSlave<'seq2, 'body, 'scope>,
    > where
    Channel: dma::DmaChannel,
{
    /// Waits for the DMA transfer to finish.
    pub fn wait(
        self,
    ) -> (
        Adc12<Dual, WithSequence<'seq1>, WithSequence<'seq2>>,
        Channel,
        ScopeGuard<'body, 'scope, dma::WaitHandler<Option<fn()>>>,
        &'scope mut [[u16; 2]],
    ) {
        let Adc12 { adc1, adc2 } = self;
        // Wait for DMA transfer to finish.
        let (chan, mut guard, buf) = adc1.state.transfer.wait();
        // The hardware should automatically stop the ADCs. This loop is in
        // case it takes a little while to stop the ADCs after the DMA
        // transfer has completed. (The reference manual does not specify
        // whether this is necessary.)
        while adc1.reg.cr.read().adstart().bit_is_set() {}
        // Put the ADCs back together so that we can access `.ccr_mut()`.
        let mut adc12 = Adc12 {
            adc1: Adc1 {
                clock_freq: adc1.clock_freq,
                reg: adc1.reg,
                pair_state: adc1.pair_state,
                state: WithSequence {
                    life: adc1.state.seq,
                },
            },
            adc2: Adc2 {
                clock_freq: adc2.clock_freq,
                reg: adc2.reg,
                pair_state: adc2.pair_state,
                state: WithSequence {
                    life: adc2.state.seq,
                },
            },
        };
        // Disable generation of DMA requests.
        adc12
            .ccr_mut()
            .modify(|_, w| unsafe { w.mdma().bits(0b00) });
        // Remove guard handler.
        if let Some(handler) = guard.handler_mut() {
            handler.set_periph(None);
        }
        // There should never be an overrun in auto-delayed mode.
        debug_assert!(adc12.adc1.reg.isr.read().ovr().bit_is_clear());

        (adc12, chan, guard, buf)
    }
}

impl_single_any!(Adc1);
impl_single_any!(Adc2);

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

    /// Channel is in single-ended mode.
    pub struct SingleEnded;
    /// Channel is in differential mode.
    pub struct Differential;

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

    /// Battery voltage sensor (V_BAT/2).
    ///
    /// An instance of this type can be obtained from the
    /// `.enable_battery()` method on `Adc12`.
    pub struct HalfBattery {
        _0: (),
    }

    /// Internal reference voltage (V_REFINT).
    ///
    /// Note that there are different `InternalRef` types for ADC1/2 and
    /// ADC3/4, but it's not possible to get more than one `InternalRef`
    /// instance at a time. (For example, once you get an `InternalRef`
    /// instance for ADC1/2, you cannot get one for ADC3/4 until the one
    /// for ADC1/2 is dropped.)
    pub struct InternalRef {
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

        /// Enables the internal reference voltage and returns a handle to
        /// it.
        ///
        /// Returns `None` if the internal reference voltage is already
        /// enabled on ADC1/2 or ADC3/4.
        pub fn enable_internal_ref(&mut self) -> Option<InternalRef> {
            cortex_m::interrupt::free(|_| {
                if unsafe { INTERNAL_REF_ENABLED } {
                    None
                } else {
                    self.ccr_mut().modify(|_, w| w.vrefen().set_bit());
                    unsafe { INTERNAL_REF_ENABLED = true };
                    Some(InternalRef { _0: () })
                }
            })
        }

        /// Disables the internal reference voltage.
        pub fn disable_internal_ref(&mut self, _: InternalRef) {
            cortex_m::interrupt::free(|_| {
                self.ccr_mut().modify(|_, w| w.vrefen().clear_bit());
                unsafe { INTERNAL_REF_ENABLED = false };
            });
        }
    }

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

    macro_rules! impl_from_pins {
        ($borrowed:ident, $id:ident, $channel:ident, $pos:ident) => {
            impl<'a> From<(&'a $channel<SingleEnded>, &'a $pos<Analog>)> for $borrowed<'a> {
                fn from(_: (&'a $channel<SingleEnded>, &'a $pos<Analog>)) -> $borrowed<'a> {
                    $borrowed {
                        id: $id::$channel,
                        life: PhantomData,
                    }
                }
            }
        };
        ($borrowed:ident, $id:ident, $channel:ident, $pos:ident, $neg:ident) => {
            impl_from_pins!($borrowed, $id, $channel, $pos);
            impl<'a>
                From<(
                    &'a $channel<Differential>,
                    &'a $pos<Analog>,
                    &'a $neg<Analog>,
                )> for $borrowed<'a>
            {
                fn from(
                    _: (
                        &'a $channel<Differential>,
                        &'a $pos<Analog>,
                        &'a $neg<Analog>,
                    ),
                ) -> $borrowed<'a> {
                    $borrowed {
                        id: $id::$channel,
                        life: PhantomData,
                    }
                }
            }
        };
    }

    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In1, PA0, PA1);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In2, PA1, PA2);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In3, PA2, PA3);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In4, PA3, PF4);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc1In5, PF4, PC0);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In6, PC0, PC1);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In7, PC1, PC2);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In8, PC2, PC3);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In9, PC3, PF2);
    impl_from_pins!(Adc1Channel, Adc1ChannelId, Adc12In10, PF2);

    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In1, PA4, PA5);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In2, PA5, PA6);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In3, PA6, PA7);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In4, PA7, PC4);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In5, PC4, PC0);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In6, PC0, PC1);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In7, PC1, PC2);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In8, PC2, PC3);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In9, PC3, PF2);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc12In10, PF2, PC5);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In11, PC5, PB2);
    impl_from_pins!(Adc2Channel, Adc2ChannelId, Adc2In12, PB2);

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
    /// temperature sensor and internal voltage reference).
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

    macro_rules! impl_channel_conversions {
            ($pos:ident, $neg:ident, [$(($adc:ident, $channel_num:expr)),*]) => {
                impl $pos<SingleEnded> {
                    /// Changes the channel to differential mode, where `self` is the
                    /// positive input and `_neg` is the negative input.
                    pub fn into_differential<P>(
                        self,
                        _neg: $neg<SingleEnded>,
                        adc12: &mut Adc12<P, Disabled, Disabled>,
                    ) -> $pos<Differential> {
                        $(
                            adc12.$adc.reg.difsel.modify(|r, w| unsafe {
                                w.difsel_1_15()
                                    .bits(r.difsel_1_15().bits() | (1 << $channel_num))
                            });
                        )*
                            $pos {
                                _state: PhantomData,
                            }
                    }
                }

                impl $pos<Differential> {
                    /// Changes the channel to single-ended mode.
                    pub fn into_single_ended<P>(
                        self,
                        adc12: &mut Adc12<P, Disabled, Disabled>,
                    ) -> ($pos<SingleEnded>, $neg<SingleEnded>) {
                        $(
                            adc12.$adc.reg.difsel.modify(|r, w| unsafe {
                                w.difsel_1_15()
                                    .bits(r.difsel_1_15().bits() & !(1 << $channel_num))
                            });
                        )*
                            (
                                $pos {
                                    _state: PhantomData,
                                },
                                $neg {
                                    _state: PhantomData,
                                },
                            )
                    }
                }
            };
        }

    impl_channel_conversions!(Adc1In1, Adc1In2, [(adc1, 1)]);
    impl_channel_conversions!(Adc1In2, Adc1In3, [(adc1, 2)]);
    impl_channel_conversions!(Adc1In3, Adc1In4, [(adc1, 3)]);
    impl_channel_conversions!(Adc1In4, Adc1In5, [(adc1, 4)]);
    impl_channel_conversions!(Adc1In5, Adc12In6, [(adc1, 5)]);
    impl_channel_conversions!(Adc1In11, Adc1In12, [(adc1, 11)]);
    impl_channel_conversions!(Adc1In12, Adc1In13, [(adc1, 12)]);

    impl_channel_conversions!(Adc2In1, Adc2In2, [(adc2, 1)]);
    impl_channel_conversions!(Adc2In2, Adc2In3, [(adc2, 2)]);
    impl_channel_conversions!(Adc2In3, Adc2In4, [(adc2, 3)]);
    impl_channel_conversions!(Adc2In4, Adc2In5, [(adc2, 4)]);
    impl_channel_conversions!(Adc2In5, Adc12In6, [(adc2, 5)]);
    impl_channel_conversions!(Adc2In11, Adc2In12, [(adc2, 11)]);
    impl_channel_conversions!(Adc2In12, Adc2In13, [(adc2, 12)]);
    impl_channel_conversions!(Adc2In13, Adc2In14, [(adc2, 13)]);
    impl_channel_conversions!(Adc2In14, Adc2In15, [(adc2, 14)]);

    impl_channel_conversions!(Adc12In6, Adc12In7, [(adc1, 6), (adc2, 6)]);
    impl_channel_conversions!(Adc12In7, Adc12In8, [(adc1, 7), (adc2, 7)]);
    impl_channel_conversions!(Adc12In8, Adc12In9, [(adc1, 8), (adc2, 8)]);
    impl_channel_conversions!(Adc12In9, Adc12In10, [(adc1, 9), (adc2, 9)]);

    impl Adc12In10<SingleEnded> {
        /// Changes the channel to differential mode, where `self` is the
        /// positive input and `_neg` are the negative inputs.
        pub fn into_differential<P>(
            self,
            _neg: (Adc1In11<SingleEnded>, Adc2In11<SingleEnded>),
            adc12: &mut Adc12<P, Disabled, Disabled>,
        ) -> Adc12In10<Differential> {
            const CHANNEL_NUM: u8 = 10;
            adc12.adc1.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() | (1 << CHANNEL_NUM))
            });
            adc12.adc2.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() | (1 << CHANNEL_NUM))
            });
            Adc12In10 {
                _state: PhantomData,
            }
        }
    }

    impl Adc12In10<Differential> {
        /// Changes the channel to single-ended mode.
        pub fn into_single_ended<P>(
            self,
            adc12: &mut Adc12<P, Disabled, Disabled>,
        ) -> (
            Adc12In10<SingleEnded>,
            (Adc1In11<SingleEnded>, Adc2In11<SingleEnded>),
        ) {
            const CHANNEL_NUM: u8 = 10;
            adc12.adc1.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() & !(1 << CHANNEL_NUM))
            });
            adc12.adc2.reg.difsel.modify(|r, w| unsafe {
                w.difsel_1_15()
                    .bits(r.difsel_1_15().bits() & !(1 << CHANNEL_NUM))
            });
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

    macro_rules! impl_channel_conversion_both {
        ($this_pos:ident, $other_pos:ident, $neg:ident, $channel_num:expr) => {
            impl $this_pos<SingleEnded> {
                /// Changes the channels to differential mode, where `self` and
                /// `_other_pos` are the positive inputs and `_neg` is the negative
                /// input.
                pub fn both_into_differential<P>(
                    self,
                    _other_pos: $other_pos<SingleEnded>,
                    _neg: $neg<SingleEnded>,
                    adc12: &mut Adc12<P, Disabled, Disabled>,
                ) -> ($this_pos<Differential>, $other_pos<Differential>) {
                    adc12.adc1.reg.difsel.modify(|r, w| unsafe {
                        w.difsel_1_15()
                            .bits(r.difsel_1_15().bits() | (1 << $channel_num))
                    });
                    adc12.adc2.reg.difsel.modify(|r, w| unsafe {
                        w.difsel_1_15()
                            .bits(r.difsel_1_15().bits() | (1 << $channel_num))
                    });
                    (
                        $this_pos {
                            _state: PhantomData,
                        },
                        $other_pos {
                            _state: PhantomData,
                        },
                    )
                }
            }

            impl $this_pos<Differential> {
                /// Changes the channels to single-ended mode.
                pub fn both_into_single_ended<P>(
                    self,
                    _other: $other_pos<Differential>,
                    adc12: &mut Adc12<P, Disabled, Disabled>,
                ) -> (
                    ($this_pos<SingleEnded>, $other_pos<SingleEnded>),
                    $neg<SingleEnded>,
                ) {
                    adc12.adc1.reg.difsel.modify(|r, w| unsafe {
                        w.difsel_1_15()
                            .bits(r.difsel_1_15().bits() & !(1 << $channel_num))
                    });
                    adc12.adc2.reg.difsel.modify(|r, w| unsafe {
                        w.difsel_1_15()
                            .bits(r.difsel_1_15().bits() & !(1 << $channel_num))
                    });
                    (
                        (
                            $this_pos {
                                _state: PhantomData,
                            },
                            $other_pos {
                                _state: PhantomData,
                            },
                        ),
                        $neg {
                            _state: PhantomData,
                        },
                    )
                }
            }
        };
    }
    impl_channel_conversion_both!(Adc1In5, Adc2In5, Adc12In6, 5);
    impl_channel_conversion_both!(Adc2In5, Adc1In5, Adc12In6, 5);
}
