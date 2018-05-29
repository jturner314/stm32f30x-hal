//! Analog-to-digital converters (ADCs)
//!
//! Some features are not supported, such as offsets and continuous conversion mode.

// TODO: allow converting channel to differential mode when only the corresponding ADC is disabled

use core::marker::PhantomData;
use cortex_m;
use dma;
use gpio::Analog;
use heapless::Vec;
use strong_scope_guard::ScopeGuard;
use syscfg;

/// Whether the internal reference voltage channel has been enabled on ADC12 or
/// ADC34.
///
/// Once this is `true`, the only safe way to make it `false` again is if an
/// `InternalRef` singleton is dropped (to ensure that nothing still has a
/// reference to it).
static mut INTERNAL_REF_ENABLED: bool = false;

/// ADC voltage regulator has not been powered on yet.
#[derive(Debug)]
pub struct Unpowered {}
/// ADC voltage regulator is powered, but the ADC is disabled.
#[derive(Debug)]
pub struct Disabled {}
/// ADC is enabled but not started.
#[derive(Debug)]
pub struct Enabled {}
/// The ADC is enabled, and the sequence and data type have been selected.
///
/// The lifetime `'a` is the lifetime of the borrows of the channels, GPIO
/// pins, and internal sensors in the sequence.
#[derive(Debug)]
pub struct WithSequence<'a> {
    life: PhantomData<&'a ()>,
}
/// The ADC is running with DMA.
#[derive(Debug)]
pub struct RunningDma<'seq: 'scope, 'body, 'scope: 'body, Channel> {
    seq: PhantomData<&'seq ()>,
    transfer:
        dma::Transfer<'body, 'scope, Channel, &'scope mut [u16], dma::WaitHandler<Option<fn()>>>,
}
/// The master ADC is running with DMA.
#[derive(Debug)]
pub struct RunningDmaMaster<'seq: 'scope, 'body, 'scope: 'body, Channel> {
    seq: PhantomData<&'seq ()>,
    transfer: dma::Transfer<
        'body,
        'scope,
        Channel,
        &'scope mut [[u16; 2]],
        dma::WaitHandler<Option<fn()>>,
    >,
}
/// The slave ADC is running with DMA.
#[derive(Debug)]
pub struct RunningDmaSlave<'seq: 'scope, 'body, 'scope: 'body> {
    seq: PhantomData<&'seq ()>,
    body: PhantomData<&'body ()>,
    scope: PhantomData<&'scope ()>,
}

/// ADCs in pair are in independent mode.
#[derive(Debug)]
pub struct Independent {}
/// ADCs in pair are in dual mode (combined regular simultaneous + injected simultaneous).
#[derive(Debug)]
pub struct Dual {}

/// Error when configuring ADC sequences in in dual mode.
#[derive(Debug)]
pub enum DualSequenceError {
    /// Tried to configure the same channel to be converted at the same time on
    /// two different ADCs.
    Conflict,
    /// The sequences are different lengths.
    UnequalLen,
    /// The sequences were too short or too long.
    BadLen,
}

/// Data alignment.
#[derive(Copy, Clone)]
pub enum Alignment {
    /// Right-aligned data.
    Right,
    /// Left-aligned data.
    Left,
}

impl Alignment {
    /// Convert the alignment to the corresponding register bit value.
    fn to_bit(self) -> bool {
        match self {
            Alignment::Right => false,
            Alignment::Left => true,
        }
    }

    /// Get alignment from the register bit value.
    fn from_bit(bit: bool) -> Self {
        match bit {
            false => Alignment::Right,
            true => Alignment::Left,
        }
    }
}

impl Default for Alignment {
    fn default() -> Self {
        Alignment::Right
    }
}

/// Data resolution.
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Resolution {
    /// 12-bit resolution.
    Bit12 = 12,
    /// 10-bit resolution.
    Bit10 = 10,
    /// 8-bit resolution.
    Bit8 = 8,
    /// 6-bit resolution.
    Bit6 = 6,
}

impl Resolution {
    /// Convert the resolution to the corresponding register bit values.
    fn to_bits(self) -> u8 {
        match self {
            Resolution::Bit12 => 0b00,
            Resolution::Bit10 => 0b01,
            Resolution::Bit8 => 0b10,
            Resolution::Bit6 => 0b11,
        }
    }

    /// Get resolution from the register bit values.
    fn from_bits(bits: u8) -> Self {
        match bits {
            0b00 => Resolution::Bit12,
            0b01 => Resolution::Bit10,
            0b10 => Resolution::Bit8,
            0b11 => Resolution::Bit6,
            _ => unreachable!(),
        }
    }
}

impl Default for Resolution {
    fn default() -> Self {
        Resolution::Bit12
    }
}

/// Overrun error when reading data with software.
#[derive(Clone, Debug)]
pub struct OverrunError {}

/// Converts the bits to data, performing the necessary shifts to get a standard `u16` value.
pub fn convert_data(bits: u16, align: Alignment, res: Resolution) -> u16 {
    use self::Alignment::*;
    use self::Resolution::*;
    match (align, res) {
        (Right, _) => bits,
        (Left, Bit12) => bits >> 4,
        (Left, Bit10) => bits >> 6,
        (Left, Bit8) => bits >> 8,
        (Left, Bit6) => bits >> 2,
    }
}

/// Tokens necessary to enable DMA for `Adc`.
pub unsafe trait AdcDmaTokens<'scope, Adc> {
    /// The DMA channel for this ADC.
    type Channel: dma::DmaChannel;
    /// Returns the DMA channel.
    fn channel(self) -> Self::Channel;
}

/// Wrappers for ADC1 and ADC2.
pub mod adc12 {
    pub use self::channels::{
        Adc12Channels, Adc1Channel, Adc1ChannelId, Adc2Channel, Adc2ChannelId,
    };

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

    /// ADC1 and ADC2 pair.
    pub struct Adc12<PairState, Adc1State, Adc2State> {
        reg: stm32f30x::ADC1_2,
        /// ADC1.
        pub adc1: Adc1<PairState, Adc1State>,
        /// ADC2.
        pub adc2: Adc2<PairState, Adc2State>,
    }

    /// ADC1 wrapper.
    pub struct Adc1<PairState, State> {
        clock_freq: Hertz,
        reg: stm32f30x::ADC1,
        pair_state: PhantomData<PairState>,
        state: State,
    }

    /// ADC2 wrapper.
    pub struct Adc2<PairState, State> {
        clock_freq: Hertz,
        reg: stm32f30x::ADC2,
        pair_state: PhantomData<PairState>,
        state: State,
    }

    /// Continuous iterator over regular conversions in dual simultaneous mode.
    pub struct Adc12ContIter<'adc12, 'seq1: 'adc12, 'seq2: 'adc12> {
        adc12: &'adc12 mut Adc12<Dual, WithSequence<'seq1>, WithSequence<'seq2>>,
    }

    impl<'adc12, 'seq1, 'seq2> Iterator for Adc12ContIter<'adc12, 'seq1, 'seq2> {
        type Item = (u16, u16);

        fn next(&mut self) -> Option<(u16, u16)> {
            while self.adc12.adc1.reg.isr.read().eoc().bit_is_clear() {}
            let common_data = self.adc12.reg.cdr.read();
            // Manually clear EOC flag since the hardware doesn't
            // automatically do so when reading CDR.
            self.adc12.adc1.reg.isr.write(|w| w.eoc().set_bit());
            Some((
                common_data.rdata_mst().bits(),
                common_data.rdata_slv().bits(),
            ))
        }
    }

    impl<PairState, Adc1State, Adc2State> Adc12<PairState, Adc1State, Adc2State> {
        /// Returns the ADC clock frequency.
        ///
        /// This value is shared for both ADCs in the pair.
        pub fn clock_freq(&self) -> Hertz {
            self.adc1.clock_freq
        }

        /// Map ADC1. (This allows changing its state.)
        pub fn map_adc1<F, S>(self, f: F) -> Adc12<PairState, S, Adc2State>
        where
            F: FnOnce(Adc1<PairState, Adc1State>) -> Adc1<PairState, S>,
        {
            Adc12 {
                reg: self.reg,
                adc1: f(self.adc1),
                adc2: self.adc2,
            }
        }

        /// Map ADC2. (This allows changing its state.)
        pub fn map_adc2<F, S>(self, f: F) -> Adc12<PairState, Adc1State, S>
        where
            F: FnOnce(Adc2<PairState, Adc2State>) -> Adc2<PairState, S>,
        {
            Adc12 {
                reg: self.reg,
                adc1: self.adc1,
                adc2: f(self.adc2),
            }
        }
    }

    impl<PairState> Adc12<PairState, Unpowered, Unpowered> {
        /// Enable voltage regulator for both ADCs.
        ///
        /// This avoids having to wait 10 us for each ADC separately.
        pub fn power_both_on(self, delay: &mut Delay) -> Adc12<PairState, Disabled, Disabled> {
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
                reg: self.reg,
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

    impl<PairState> Adc12<PairState, Disabled, Disabled> {
        /// Enables independent mode.
        pub fn into_independent(self) -> Adc12<Independent, Disabled, Disabled> {
            const INDEPENDENT: u8 = 0b00000;
            self.reg
                .ccr
                .modify(|_, w| unsafe { w.mult().bits(INDEPENDENT) });
            Adc12 {
                reg: self.reg,
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
        pub fn into_dual(self) -> Adc12<Dual, Disabled, Disabled> {
            const REG_SIMUL_INJ_SIMUL: u8 = 0b00001;
            self.reg
                .ccr
                .modify(|_, w| unsafe { w.mult().bits(REG_SIMUL_INJ_SIMUL) });
            Adc12 {
                reg: self.reg,
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
        pub fn enable_both(self) -> Adc12<PairState, Enabled, Enabled> {
            self.adc1.reg.cr.modify(|_, w| w.aden().set_bit());
            self.adc2.reg.cr.modify(|_, w| w.aden().set_bit());
            while self.adc1.reg.isr.read().adrdy().bit_is_clear()
                && self.adc2.reg.isr.read().adrdy().bit_is_clear()
            {}
            Adc12 {
                reg: self.reg,
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
                reg: self.reg,
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
                let common_data = self.reg.cdr.read();
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
            self,
            buf: &'scope mut [[u16; 2]],
            mut guard: ScopeGuard<'body, 'scope, dma::WaitHandler<Option<fn()>>>,
            dma: D,
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
            self.reg.ccr.modify(|_, w| unsafe { w.mdma().bits(0b00) });
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
            self.reg.ccr.modify(|_, w| {
                unsafe { w.mdma().bits(0b10) };
                w.dmacfg().clear_bit() // one-shot mode
            });
            let data_reg = &(*self.reg).cdr as *const _ as *const [u16; 2];
            let transfer = unsafe { dma.channel().enable_periph_to_mem(guard, data_reg, buf) };

            self.adc1.reg.cr.modify(|_, w| w.adstart().set_bit());
            Adc12 {
                reg: self.reg,
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
            // Wait for DMA transfer to finish.
            let (chan, mut guard, buf) = self.adc1.state.transfer.wait();
            // The hardware should automatically stop the ADCs. This loop is in
            // case it takes a little while to stop the ADCs after the DMA
            // transfer has completed. (The reference manual does not specify
            // whether this is necessary.)
            while self.adc1.reg.cr.read().adstart().bit_is_set() {}
            // Disable generation of DMA requests.
            self.reg.ccr.modify(|_, w| unsafe { w.mdma().bits(0b00) });
            // Remove guard handler.
            if let Some(handler) = guard.handler_mut() {
                handler.set_periph(None);
            }
            // There should never be an overrun in auto-delayed mode.
            debug_assert!(self.adc1.reg.isr.read().ovr().bit_is_clear());
            (
                Adc12 {
                    reg: self.reg,
                    adc1: Adc1 {
                        clock_freq: self.adc1.clock_freq,
                        reg: self.adc1.reg,
                        pair_state: self.adc1.pair_state,
                        state: WithSequence {
                            life: self.adc1.state.seq,
                        },
                    },
                    adc2: Adc2 {
                        clock_freq: self.adc2.clock_freq,
                        reg: self.adc2.reg,
                        pair_state: self.adc2.pair_state,
                        state: WithSequence {
                            life: self.adc2.state.seq,
                        },
                    },
                },
                chan,
                guard,
                buf,
            )
        }
    }

    macro_rules! impl_single_unpowered {
        ($adc:ident) => {
            impl<PairState> $adc<PairState, Unpowered> {
                /// Enable ADC voltage regulator.
                pub fn power_on(self, delay: &mut Delay) -> $adc<PairState, Disabled> {
                    // Enable ADC voltage regulator.
                    self.reg
                        .cr
                        .write(|w| w.deeppwd().clear_bit().advregen().clear_bit());
                    self.reg
                        .cr
                        .write(|w| w.deeppwd().clear_bit().advregen().set_bit());
                    delay.delay_us(10u8);

                    $adc {
                        clock_freq: self.clock_freq,
                        reg: self.reg,
                        pair_state: PhantomData,
                        state: Disabled {},
                    }
                }
            }
        };
    }
    impl_single_unpowered!(Adc1);
    impl_single_unpowered!(Adc2);

    macro_rules! impl_single_disabled {
        ($adc:ident) => {
            impl<PairState> $adc<PairState, Disabled> {
                /// Calibrates the ADC for single-ended inputs.
                pub fn calibrate_single_ended(&mut self, delay: &mut Delay) {
                    debug_assert!(self.reg.cr.read().aden().bit_is_clear());
                    // Select single-ended calibration.
                    self.reg.cr.modify(|_, w| w.adcaldif().clear_bit());
                    // Perform calibration.
                    self.reg.cr.modify(|_, w| w.adcal().set_bit());
                    while self.reg.cr.read().adcal().bit_is_set() {}
                    // Wait for at least 4 ADC clock cycles.
                    // (See note in sec 15.3.9 of reference manual.)
                    delay.delay_us(cmp::max(4_000_000 / self.clock_freq.0, 1))
                }

                /// Calibrates the ADC for differential inputs.
                pub fn calibrate_differential(&mut self, delay: &mut Delay) {
                    debug_assert!(self.reg.cr.read().aden().bit_is_clear());
                    // Select differential calibration.
                    self.reg.cr.modify(|_, w| w.adcaldif().set_bit());
                    // Perform calibration.
                    self.reg.cr.modify(|_, w| w.adcal().set_bit());
                    while self.reg.cr.read().adcal().bit_is_set() {}
                    // Wait for at least 4 ADC clock cycles.
                    // (See note in sec 15.3.9 of reference manual.)
                    delay.delay_us(cmp::max(4_000_000 / self.clock_freq.0, 1))
                }

                /// Enables the ADC. It should be calibrated first.
                pub fn enable(self) -> $adc<PairState, Enabled> {
                    self.reg.cr.modify(|_, w| w.aden().set_bit());
                    while self.reg.isr.read().adrdy().bit_is_clear() {}
                    $adc {
                        clock_freq: self.clock_freq,
                        reg: self.reg,
                        pair_state: PhantomData,
                        state: Enabled {},
                    }
                }
            }
        };
    }
    impl_single_disabled!(Adc1);
    impl_single_disabled!(Adc2);

    macro_rules! impl_single_enabled {
        ($adc:ident, $channel:ident) => {
            impl<PairState> $adc<PairState, Enabled> {
                /// Sets the data alignment.
                pub fn set_alignment(&mut self, align: Alignment) {
                    debug_assert!(self.reg.cr.read().adstart().bit_is_clear());
                    debug_assert!(self.reg.cr.read().jadstart().bit_is_clear());
                    self.reg.cfgr.modify(|_, w| w.align().bit(align.to_bit()));
                }

                /// Gets the data alignment.
                // TODO: allow this method in other states
                pub fn alignment(&self) -> Alignment {
                    Alignment::from_bit(self.reg.cfgr.read().align().bit())
                }

                /// Sets the data resolution.
                pub fn set_resolution(&mut self, res: Resolution) {
                    debug_assert!(self.reg.cr.read().adstart().bit_is_clear());
                    debug_assert!(self.reg.cr.read().jadstart().bit_is_clear());
                    self.reg
                        .cfgr
                        .modify(|_, w| unsafe { w.res().bits(res.to_bits()) });
                }

                /// Gets the data resolution.
                pub fn resolution(&self) -> Resolution {
                    Resolution::from_bits(self.reg.cfgr.read().res().bits())
                }

                /// Sets the sequence for regular conversion.
                ///
                /// For this to be safe, you must ensure all of the following:
                ///
                /// * Two different ADCs must not convert the same channel at
                ///   the same time. (This is a concern for channels that are
                ///   shared between ADCs, such as ADC12_IN6.)
                ///
                /// * If the ADCs are in dual simultaneous mode, their
                ///   sequences must be the same length.
                ///
                /// **Panics** if `sequence.len() < 1 || sequence.len() > 16`.
                pub unsafe fn with_sequence_unchecked<'a>(
                    self,
                    sequence: &[$channel<'a>],
                ) -> $adc<PairState, WithSequence<'a>> {
                    debug_assert!(self.reg.cr.read().aden().bit_is_set());
                    debug_assert!(self.reg.cr.read().adstart().bit_is_clear());
                    assert!(sequence.len() > 0);
                    assert!(sequence.len() <= 16);
                    let mut chan_iter = sequence.iter().enumerate();
                    self.reg.sqr1.write(|w| {
                        // Set the number of channels.
                        w.l3().bits(sequence.len() as u8 - 1);
                        // Set the first few channels.
                        for (index, chan) in chan_iter.by_ref().take(4) {
                            match index {
                                0 => {
                                    w.sq1().bits(chan.id() as u8);
                                }
                                1 => {
                                    w.sq2().bits(chan.id() as u8);
                                }
                                2 => {
                                    w.sq3().bits(chan.id() as u8);
                                }
                                3 => {
                                    w.sq4().bits(chan.id() as u8);
                                }
                                _ => unreachable!(),
                            }
                        }
                        w
                    });
                    self.reg.sqr2.write(|w| {
                        for (index, chan) in chan_iter.by_ref().take(5) {
                            match index {
                                4 => {
                                    w.sq5().bits(chan.id() as u8);
                                }
                                5 => {
                                    w.sq6().bits(chan.id() as u8);
                                }
                                6 => {
                                    w.sq7().bits(chan.id() as u8);
                                }
                                7 => {
                                    w.sq8().bits(chan.id() as u8);
                                }
                                8 => {
                                    w.sq9().bits(chan.id() as u8);
                                }
                                _ => unreachable!(),
                            }
                        }
                        w
                    });
                    self.reg.sqr3.write(|w| {
                        for (index, chan) in chan_iter.by_ref().take(5) {
                            match index {
                                9 => {
                                    w.sq10().bits(chan.id() as u8);
                                }
                                10 => {
                                    w.sq11().bits(chan.id() as u8);
                                }
                                11 => {
                                    w.sq12().bits(chan.id() as u8);
                                }
                                12 => {
                                    w.sq13().bits(chan.id() as u8);
                                }
                                13 => {
                                    w.sq14().bits(chan.id() as u8);
                                }
                                _ => unreachable!(),
                            }
                        }
                        w
                    });
                    self.reg.sqr4.write(|w| {
                        for (index, chan) in chan_iter.take(2) {
                            match index {
                                14 => {
                                    w.sq15().bits(chan.id() as u8);
                                }
                                15 => {
                                    w.sq16().bits(chan.id() as u8);
                                }
                                _ => unreachable!(),
                            }
                        }
                        w
                    });
                    $adc {
                        clock_freq: self.clock_freq,
                        reg: self.reg,
                        pair_state: PhantomData,
                        state: WithSequence { life: PhantomData },
                    }
                }

                /// Disables the ADC.
                // TODO: Enable this in the `WithSequence` state.
                pub fn disable(self) -> $adc<PairState, Disabled> {
                    debug_assert!(self.reg.cr.read().adstart().bit_is_clear());
                    debug_assert!(self.reg.cr.read().jadstart().bit_is_clear());
                    self.reg.cr.modify(|_, w| w.addis().set_bit());
                    while self.reg.cr.read().aden().bit_is_set() {}
                    debug_assert!(self.reg.cr.read().addis().bit_is_clear());
                    $adc {
                        clock_freq: self.clock_freq,
                        reg: self.reg,
                        pair_state: PhantomData,
                        state: Disabled {},
                    }
                }
            }
        };
    }
    impl_single_enabled!(Adc1, Adc1Channel);
    impl_single_enabled!(Adc2, Adc2Channel);

    macro_rules! impl_single_with_sequence {
        ($adc:ident, $id:ident) => {
            impl<'a, PairState> $adc<PairState, WithSequence<'a>> {
                /// Returns the number of channels in the regular conversion sequence.
                pub fn sequence_len(&self) -> u8 {
                    self.reg.sqr1.read().l3().bits() + 1
                }

                /// Returns the channel IDs in the regular sequence.
                pub fn sequence_ids(&self) -> Vec<$id, [$id; 16]> {
                    let sqr1 = self.reg.sqr1.read();
                    let num_channels = sqr1.l3().bits() + 1;
                    let mut ids = Vec::new();
                    for index in 0..num_channels {
                        match index {
                            0 => ids.push(unsafe {
                                *(&self.reg.sqr1.read().sq1().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            1 => ids.push(unsafe {
                                *(&self.reg.sqr1.read().sq2().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            2 => ids.push(unsafe {
                                *(&self.reg.sqr1.read().sq3().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            3 => ids.push(unsafe {
                                *(&self.reg.sqr1.read().sq4().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            4 => ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq5().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            5 => ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq6().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            6 => ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq7().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            7 => ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq8().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            8 => ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq9().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            9 => ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq10().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            10 => ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq11().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            11 => ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq12().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            12 => ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq13().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            13 => ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq14().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            14 => ids.push(unsafe {
                                *(&self.reg.sqr4.read().sq15().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            15 => ids.push(unsafe {
                                *(&self.reg.sqr4.read().sq16().bits() as *const u8 as *const $id)
                            }).unwrap(),
                            _ => unreachable!(),
                        }
                    }
                    ids
                }

                /// Drops the borrow of the sequence pins.
                pub fn drop_sequence(self) -> $adc<PairState, Enabled> {
                    $adc {
                        clock_freq: self.clock_freq,
                        reg: self.reg,
                        pair_state: self.pair_state,
                        state: Enabled {},
                    }
                }
            }
        };
    }
    impl_single_with_sequence!(Adc1, Adc1ChannelId);
    impl_single_with_sequence!(Adc2, Adc2ChannelId);

    macro_rules! impl_single_independent_with_sequence {
        ($Adc:ident, $ADC:ident) => {
            impl<'seq> $Adc<Independent, WithSequence<'seq>> {
                /// Sets the ADC to single conversion mode; runs the regular
                /// sequence; and for each conversion in the sequence, calls
                /// `f` with the (zero-based) index within the sequence and the
                /// converted value.
                ///
                /// This method uses auto-delayed conversion mode (AUTDLY) to avoid
                /// overruns.
                pub fn run_sequence_once<F>(&mut self, mut f: F)
                where
                    F: FnMut(u8, u16),
                {
                    debug_assert!(self.reg.cr.read().aden().bit_is_set());
                    debug_assert!(self.reg.cr.read().addis().bit_is_clear());

                    // Set the ADC to single conversion mode with auto-delay.
                    self.reg.cfgr.modify(|_, w| {
                        w.cont().clear_bit();
                        w.autdly().set_bit()
                    });
                    // Clear overrun, end-of-conversion, and end-of-sequence flags.
                    self.reg.isr.write(|w| {
                        w.ovr().set_bit();
                        w.eoc().set_bit();
                        w.eos().set_bit()
                    });

                    //  Start conversion.
                    let seq_len = self.sequence_len();
                    self.reg.cr.modify(|_, w| w.adstart().set_bit());
                    for i in 0..seq_len {
                        while self.reg.isr.read().eoc().bit_is_clear() {}
                        f(i, self.reg.dr.read().regular_data().bits());
                    }
                    let isr = self.reg.isr.read();
                    debug_assert!(isr.eos().bit_is_set());
                    // There should never be an overrun in auto-delayed mode.
                    debug_assert!(isr.ovr().bit_is_clear());
                }

                /// Sets the ADC to continuous conversion mode with auto-delay;
                /// configures and enables the DMA channel for writing into the
                /// buffer in one shot mode; and starts the regular sequence.
                pub fn start_dma<'body, 'scope, D>(
                    self,
                    buf: &'scope mut [u16],
                    mut guard: ScopeGuard<'body, 'scope, dma::WaitHandler<Option<fn()>>>,
                    dma: D,
                ) -> $Adc<Independent, RunningDma<'seq, 'body, 'scope, D::Channel>>
                where
                    'seq: 'scope,
                    D: AdcDmaTokens<'scope, Self>,
                {
                    debug_assert!(self.reg.cr.read().aden().bit_is_set());
                    debug_assert!(self.reg.cr.read().addis().bit_is_clear());
                    debug_assert!(self.reg.cr.read().adstart().bit_is_clear());

                    // Set the ADC to continuous conversion mode with
                    // auto-delay and clear the DMA enable bit to reset any
                    // pending DMA requests from this ADC.
                    self.reg.cfgr.modify(|_, w| {
                        w.cont().set_bit();
                        w.autdly().set_bit();
                        w.dmaen().clear_bit()
                    });
                    // Clear overrun, end-of-conversion, and end-of-sequence flags.
                    self.reg.isr.write(|w| {
                        w.ovr().set_bit();
                        w.eoc().set_bit();
                        w.eos().set_bit()
                    });

                    // Set up the guard to wait for the ADC to finish and disable generation of DMA
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
                            //    non-running ADC cannot access the register concurrently with this
                            //    closure.
                            let reg = unsafe { &(*stm32f30x::$ADC::ptr()) };
                            // The hardware should automatically stop the ADC after the DMA
                            // transfer has completed. This loop is in case it takes the hardware a
                            // little while to stop the ADC. (The reference manual does not specify
                            // whether this is necessary.)
                            while reg.cr.read().adstart().bit_is_set() {}
                            // Disable generation of DMA requests.
                            reg.cfgr.modify(|_, w| w.dmaen().clear_bit());
                        }));
                    }

                    // Configure and enable the DMA transfer.
                    self.reg.cfgr.modify(|_, w| {
                        w.dmaen().set_bit();
                        w.dmacfg().clear_bit() // one-shot mode
                    });
                    let data_reg = &(*self.reg).dr as *const _ as *const u32;
                    let transfer = unsafe { dma.channel().enable_periph_to_mem(guard, data_reg, buf) };

                    self.reg.cr.modify(|_, w| w.adstart().set_bit());
                    $Adc {
                        clock_freq: self.clock_freq,
                        reg: self.reg,
                        pair_state: self.pair_state,
                        state: RunningDma {
                            seq: self.state.life,
                            transfer: transfer,
                        },
                    }
                }
            }
        };
    }
    impl_single_independent_with_sequence!(Adc1, ADC1);
    impl_single_independent_with_sequence!(Adc2, ADC2);

    macro_rules! impl_single_independent_running_dma {
        ($Adc:ident, $dma_chan:ty) => {
            impl<'seq, 'body, 'scope>
                $Adc<Independent, RunningDma<'seq, 'body, 'scope, $dma_chan>>
            {
                /// Waits for the DMA transfer to finish.
                pub fn wait(
                    self,
                ) -> (
                    $Adc<Independent, WithSequence<'seq>>,
                    $dma_chan,
                    ScopeGuard<'body, 'scope, dma::WaitHandler<Option<fn()>>>,
                    &'scope mut [u16],
                ) {
                    // Wait for DMA transfer to finish.
                    let (chan, mut guard, buf) = self.state.transfer.wait();
                    // The hardware should automatically stop the ADC. This loop is in case it
                    // takes a little while to stop the ADC after the DMA transfer has completed.
                    // (The reference manual does not specify whether this is necessary.)
                    while self.reg.cr.read().adstart().bit_is_set() {}
                    // Disable generation of DMA requests.
                    self.reg.cfgr.modify(|_, w| w.dmaen().clear_bit());
                    // Remove guard handler.
                    if let Some(handler) = guard.handler_mut() {
                        handler.set_periph(None);
                    }
                    // There should never be an overrun in auto-delayed mode.
                    debug_assert!(self.reg.isr.read().ovr().bit_is_clear());
                    (
                        $Adc {
                            clock_freq: self.clock_freq,
                            reg: self.reg,
                            pair_state: self.pair_state,
                            state: WithSequence { life: PhantomData },
                        },
                        chan,
                        guard,
                        buf,
                    )
                }
            }
        };
    }
    impl_single_independent_running_dma!(Adc1, dma::dma1::Channel1);
    impl_single_independent_running_dma!(Adc2, dma::dma2::Channel1);

    unsafe impl<'scope, PairState, State> AdcDmaTokens<'scope, Adc1<PairState, State>>
        for dma::dma1::Channel1
    {
        type Channel = dma::dma1::Channel1;
        fn channel(self) -> Self::Channel {
            self
        }
    }

    unsafe impl<'scope, PairState, State> AdcDmaTokens<'scope, Adc2<PairState, State>>
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

    unsafe impl<'scope, PairState, State> AdcDmaTokens<'scope, Adc2<PairState, State>>
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

        impl<PairState> Adc12<PairState, Disabled, Disabled> {
            /// Enables the temperature sensor and returns a handle to it.
            ///
            /// Returns `None` if the temperature sensor is already enabled.
            pub fn enable_temperature_sensor(&mut self) -> Option<TemperatureSensor> {
                let mut already_enabled = false;
                self.reg.ccr.modify(|r, w| {
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
                self.reg.ccr.modify(|_, w| w.tsen().clear_bit());
            }

            /// Enables the battery voltage sensor and returns a handle to it.
            ///
            /// Returns `None` if the battery voltage sensor is already enabled.
            pub fn enable_battery_sensor(&mut self) -> Option<HalfBattery> {
                let mut already_enabled = false;
                self.reg.ccr.modify(|r, w| {
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
                self.reg.ccr.modify(|_, w| w.vbaten().clear_bit());
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
                        self.reg.ccr.modify(|_, w| w.vrefen().set_bit());
                        unsafe { INTERNAL_REF_ENABLED = true };
                        Some(InternalRef { _0: () })
                    }
                })
            }

            /// Disables the internal reference voltage.
            pub fn disable_internal_ref(&mut self, _: InternalRef) {
                cortex_m::interrupt::free(|_| {
                    self.reg.ccr.modify(|_, w| w.vrefen().clear_bit());
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
                        reg: self,
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
        pub struct Adc1In1<State> {
            _state: PhantomData<State>,
        }

        /// ADC1_IN2
        pub struct Adc1In2<State> {
            _state: PhantomData<State>,
        }

        /// ADC1_IN3
        pub struct Adc1In3<State> {
            _state: PhantomData<State>,
        }

        /// ADC1_IN4
        pub struct Adc1In4<State> {
            _state: PhantomData<State>,
        }

        /// ADC1_IN5 (not available on all devices)
        pub struct Adc1In5<State> {
            _state: PhantomData<State>,
        }

        /// ADC1_IN11 (not available on all devices)
        pub struct Adc1In11<State> {
            _state: PhantomData<State>,
        }

        /// ADC1_IN12 (not available on all devices)
        pub struct Adc1In12<State> {
            _state: PhantomData<State>,
        }

        /// ADC1_IN13 (not available on all devices)
        pub struct Adc1In13<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN1
        pub struct Adc2In1<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN2
        pub struct Adc2In2<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN3
        pub struct Adc2In3<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN4
        pub struct Adc2In4<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN5
        pub struct Adc2In5<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN11
        pub struct Adc2In11<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN12
        pub struct Adc2In12<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN13 (not available on all devices)
        pub struct Adc2In13<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN14 (not available on all devices)
        pub struct Adc2In14<State> {
            _state: PhantomData<State>,
        }

        /// ADC2_IN15 (not available on all devices)
        pub struct Adc2In15<State> {
            _state: PhantomData<State>,
        }

        /// ADC12_IN6
        pub struct Adc12In6<State> {
            _state: PhantomData<State>,
        }

        /// ADC12_IN7
        pub struct Adc12In7<State> {
            _state: PhantomData<State>,
        }

        /// ADC12_IN8
        pub struct Adc12In8<State> {
            _state: PhantomData<State>,
        }

        /// ADC12_IN9
        pub struct Adc12In9<State> {
            _state: PhantomData<State>,
        }

        /// ADC12_IN10 (not available on all devices)
        pub struct Adc12In10<State> {
            _state: PhantomData<State>,
        }

        macro_rules! impl_channel_conversions {
            ($pos:ident, $neg:ident, [$(($adc:ident, $channel_num:expr)),*]) => {
                impl $pos<SingleEnded> {
                    /// Changes the channel to differential mode, where `self` is the
                    /// positive input and `_neg` is the negative input.
                    pub fn into_differential<PairState>(
                        self,
                        _neg: $neg<SingleEnded>,
                        adc12: &mut Adc12<PairState, Disabled, Disabled>,
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
                    pub fn into_single_ended<PairState>(
                        self,
                        adc12: &mut Adc12<PairState, Disabled, Disabled>,
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
            pub fn into_differential<PairState>(
                self,
                _neg: (Adc1In11<SingleEnded>, Adc2In11<SingleEnded>),
                adc12: &mut Adc12<PairState, Disabled, Disabled>,
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
            pub fn into_single_ended<PairState>(
                self,
                adc12: &mut Adc12<PairState, Disabled, Disabled>,
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
                    pub fn both_into_differential<PairState>(
                        self,
                        _other_pos: $other_pos<SingleEnded>,
                        _neg: $neg<SingleEnded>,
                        adc12: &mut Adc12<PairState, Disabled, Disabled>,
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
                    pub fn both_into_single_ended<PairState>(
                        self,
                        _other: $other_pos<Differential>,
                        adc12: &mut Adc12<PairState, Disabled, Disabled>,
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
}
