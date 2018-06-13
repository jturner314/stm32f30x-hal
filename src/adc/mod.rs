//! Analog-to-digital converters (ADCs)
//!
//! Some features are not supported, such as offsets and continuous conversion mode.

// TODO: allow converting channel to differential mode when only the corresponding ADC is disabled

use core::marker::PhantomData;
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

/// Channel is in single-ended mode.
pub struct SingleEnded;
/// Channel is in differential mode.
pub struct Differential;

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

/// Regular conversions are not running (ADSTART = 0).
pub unsafe trait NotRunning {}
unsafe impl NotRunning for Unpowered {}
unsafe impl NotRunning for Disabled {}
unsafe impl NotRunning for Enabled {}
unsafe impl<'a> NotRunning for WithSequence<'a> {}

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

/// Trigger polarity and, if external, which external trigger.
#[derive(Clone, Copy, Debug)]
pub enum Trigger<External> {
    /// Software trigger.
    Software,
    /// Hardware trigger with detection on the rising edge.
    Rising(External),
    /// Hardware trigger with detection on the falling edge.
    Falling(External),
    /// Hardware trigger with detection on both the rising and falling edges.
    RisingAndFalling(External),
}

impl<External> Trigger<External> {
    /// Returns the bits for the EXTEN field in the CFGR register.
    fn exten_bits(self) -> u8 {
        match self {
            Trigger::Software => 0b00,
            Trigger::Rising(_) => 0b01,
            Trigger::Falling(_) => 0b10,
            Trigger::RisingAndFalling(_) => 0b11,
        }
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

/// Master/slave ADC pair.
pub trait AdcPair {
    /// The type of the master ADC in the pair.
    type Master;
    /// The type of the slave ADC in the pair.
    type Slave;
    /// Returns a reference to the master ADC.
    fn master(&self) -> &Self::Master;
    /// Returns a reference to the slave ADC.
    fn slave(&self) -> &Self::Slave;
    /// Returns a mutable reference to the master ADC.
    fn master_mut(&mut self) -> &mut Self::Master;
    /// Returns a mutable reference to the slave ADC.
    fn slave_mut(&mut self) -> &mut Self::Slave;
    /// Splits the pair into the master and slave ADCs.
    fn split(self) -> (Self::Master, Self::Slave);
    /// Joins the ADCs into a pair.
    fn join(master: Self::Master, slave: Self::Slave) -> Self;
}

macro_rules! impl_iterator_for_paircontiter {
    ($PairContIter:ident, $master:ident) => {
        impl<'p, 'm, 's> Iterator for $PairContIter<'p, 'm, 's> {
            type Item = (u16, u16);

            fn next(&mut self) -> Option<(u16, u16)> {
                while self.pair.$master.reg.isr.read().eoc().bit_is_clear() {}
                let common_data = self.pair.cdr().read();
                // Manually clear EOC flag since the hardware doesn't
                // automatically do so when reading CDR.
                self.pair.$master.reg.isr.write(|w| w.eoc().set_bit());
                Some((
                    common_data.rdata_mst().bits(),
                    common_data.rdata_slv().bits(),
                ))
            }
        }
    };
}

macro_rules! impl_pair_unpowered_unpowered {
    ($Pair:ident, $Master:ident, $Slave:ident, $master:ident, $slave:ident) => {
        impl<P> $Pair<P, Unpowered, Unpowered> {
            /// Enable voltage regulator for both ADCs.
            ///
            /// This avoids having to wait 10 us for each ADC separately.
            pub fn power_on(self, delay: &mut ::delay::Delay) -> $Pair<P, Disabled, Disabled> {
                self.$master
                    .reg
                    .cr
                    .write(|w| w.deeppwd().clear_bit().advregen().clear_bit());
                self.$master
                    .reg
                    .cr
                    .write(|w| w.deeppwd().clear_bit().advregen().set_bit());
                self.$slave
                    .reg
                    .cr
                    .write(|w| w.deeppwd().clear_bit().advregen().clear_bit());
                self.$slave
                    .reg
                    .cr
                    .write(|w| w.deeppwd().clear_bit().advregen().set_bit());
                delay.delay_us(10u8);
                $Pair {
                    $master: $Master {
                        clock_freq: self.$master.clock_freq,
                        reg: self.$master.reg,
                        pair_state: self.$master.pair_state,
                        state: Disabled {},
                    },
                    $slave: $Slave {
                        clock_freq: self.$slave.clock_freq,
                        reg: self.$slave.reg,
                        pair_state: self.$slave.pair_state,
                        state: Disabled {},
                    },
                }
            }
        }
    };
}

macro_rules! impl_pair_disabled_disabled {
    ($Pair:ident, $Master:ident, $Slave:ident, $master:ident, $slave:ident) => {
        impl<P> $Pair<P, Disabled, Disabled> {
            /// Enables independent mode.
            pub fn into_independent(mut self) -> $Pair<Independent, Disabled, Disabled> {
                const INDEPENDENT: u8 = 0b00000;
                self.ccr_mut()
                    .modify(|_, w| unsafe { w.mult().bits(INDEPENDENT) });
                $Pair {
                    $master: $Master {
                        clock_freq: self.$master.clock_freq,
                        reg: self.$master.reg,
                        pair_state: PhantomData,
                        state: self.$master.state,
                    },
                    $slave: $Slave {
                        clock_freq: self.$slave.clock_freq,
                        reg: self.$slave.reg,
                        pair_state: PhantomData,
                        state: self.$slave.state,
                    },
                }
            }

            /// Enables dual mode.
            pub fn into_dual(mut self) -> $Pair<Dual, Disabled, Disabled> {
                const REG_SIMUL_INJ_SIMUL: u8 = 0b00001;
                self.ccr_mut()
                    .modify(|_, w| unsafe { w.mult().bits(REG_SIMUL_INJ_SIMUL) });
                $Pair {
                    $master: $Master {
                        clock_freq: self.$master.clock_freq,
                        reg: self.$master.reg,
                        pair_state: PhantomData,
                        state: self.$master.state,
                    },
                    $slave: $Slave {
                        clock_freq: self.$slave.clock_freq,
                        reg: self.$slave.reg,
                        pair_state: PhantomData,
                        state: self.$slave.state,
                    },
                }
            }

            /// Enables both ADCs. They should be calibrated first.
            ///
            /// This avoids waiting separately for each ADC to finish its
            /// initialization sequence.
            pub fn enable(self) -> $Pair<P, Enabled, Enabled> {
                self.$master.reg.cr.modify(|_, w| w.aden().set_bit());
                self.$slave.reg.cr.modify(|_, w| w.aden().set_bit());
                while self.$master.reg.isr.read().adrdy().bit_is_clear()
                    && self.$slave.reg.isr.read().adrdy().bit_is_clear()
                {}
                $Pair {
                    $master: $Master {
                        clock_freq: self.$master.clock_freq,
                        reg: self.$master.reg,
                        pair_state: self.$master.pair_state,
                        state: Enabled {},
                    },
                    $slave: $Slave {
                        clock_freq: self.$slave.clock_freq,
                        reg: self.$slave.reg,
                        pair_state: self.$slave.pair_state,
                        state: Enabled {},
                    },
                }
            }
        }
    };
}

macro_rules! impl_pair_dual_enabled_enabled {
    ($Pair:ident, $master:ident, $slave:ident, $MasterChannelRef:ident, $SlaveChannelRef:ident) => {
        impl $Pair<Dual, Enabled, Enabled> {
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
            pub fn with_sequences<'m, 's>(
                self,
                master_sequence: &[$MasterChannelRef<'m>],
                slave_sequence: &[$SlaveChannelRef<'s>],
            ) -> Result<
                $Pair<Dual, WithSequence<'m>, WithSequence<'s>>,
                ($Pair<Dual, Enabled, Enabled>, DualSequenceError),
            > {
                if master_sequence.len() != slave_sequence.len() {
                    return Err((self, DualSequenceError::UnequalLen));
                }
                if master_sequence.len() < 1 || master_sequence.len() > 16 {
                    return Err((self, DualSequenceError::BadLen));
                }
                for (master_chan, slave_chan) in master_sequence.iter().zip(slave_sequence) {
                    if master_chan.id().conflicts_with(slave_chan.id()) {
                        return Err((self, DualSequenceError::Conflict));
                    }
                }
                Ok($Pair {
                    $master: unsafe { self.$master.with_sequence_unchecked(master_sequence) },
                    $slave: unsafe { self.$slave.with_sequence_unchecked(slave_sequence) },
                })
            }
        }
    };
}

macro_rules! impl_pair_withsequence_withsequence {
    (
        $Pair:ident,
        $PairContIter:ident,
        $Master:ident,
        $Slave:ident,
        $master:ident,
        $slave:ident,
        $pair_reg_ptr:expr,
        $master_reg_ptr:expr,
    ) => {
        impl<'m, 's> $Pair<Dual, WithSequence<'m>, WithSequence<'s>> {
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
                debug_assert!(self.$master.reg.cr.read().aden().bit_is_set());
                debug_assert!(self.$master.reg.cr.read().addis().bit_is_clear());

                // Set the master ADC to single conversion mode with auto-delay.
                self.$master.reg.cfgr.modify(|_, w| {
                    w.cont().clear_bit();
                    w.autdly().set_bit()
                });
                // Clear overrun, end-of-conversion, and end-of-sequence flags on
                // master ADC.
                self.$master.reg.isr.write(|w| {
                    w.ovr().set_bit();
                    w.eoc().set_bit();
                    w.eos().set_bit()
                });

                // Run sequence.
                let seq_len = self.$master.sequence_len();
                self.$master.reg.cr.modify(|_, w| w.adstart().set_bit());
                for i in 0..seq_len {
                    while self.$master.reg.isr.read().eoc().bit_is_clear() {}
                    let common_data = self.cdr().read();
                    // Manually clear EOC flag since the hardware doesn't
                    // automatically do so when reading CDR.
                    self.$master.reg.isr.write(|w| w.eoc().set_bit());
                    f(
                        i,
                        (
                            common_data.rdata_mst().bits(),
                            common_data.rdata_slv().bits(),
                        ),
                    );
                }
                let isr = self.$master.reg.isr.read();
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
                F: for<'a> FnOnce($PairContIter<'a, 'm, 's>) -> O,
            {
                debug_assert!(self.$master.reg.cr.read().aden().bit_is_set());
                debug_assert!(self.$master.reg.cr.read().addis().bit_is_clear());

                // Set the master ADC to continuous conversion mode with auto-delay.
                self.$master.reg.cfgr.modify(|_, w| {
                    w.cont().set_bit();
                    w.autdly().set_bit()
                });
                // Clear overrun, end-of-conversion, and end-of-sequence flags on
                // master ADC.
                self.$master.reg.isr.write(|w| {
                    w.ovr().set_bit();
                    w.eoc().set_bit();
                    w.eos().set_bit()
                });

                // Start sequence.
                self.$master.reg.cr.modify(|_, w| w.adstart().set_bit());
                // Run closure.
                let out = f($PairContIter { pair: self });
                // There should never be an overrun in auto-delayed mode.
                debug_assert!(self.$master.reg.isr.read().ovr().bit_is_clear());
                // Stop the ADCs and wait for them to be stopped.
                self.$master.reg.cr.modify(|_, w| w.adstp().set_bit());
                loop {
                    let cr = self.$master.reg.cr.read();
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
            ) -> $Pair<
                Dual,
                RunningDmaMaster<'m, 'body, 'scope, D::Channel>,
                RunningDmaSlave<'s, 'body, 'scope>,
            >
            where
                'm: 'scope,
                's: 'scope,
                D: AdcDmaTokens<'scope, <Self as AdcPair>::Master>,
            {
                use dma::DmaChannelPriv;

                debug_assert!(self.$master.reg.cr.read().aden().bit_is_set());
                debug_assert!(self.$master.reg.cr.read().addis().bit_is_clear());
                debug_assert!(self.$master.reg.cr.read().adstart().bit_is_clear());
                debug_assert!(self.$master.reg.cfgr.read().dmaen().bit_is_clear());
                debug_assert!(self.$slave.reg.cfgr.read().dmaen().bit_is_clear());

                // Set the master ADC to continuous conversion mode with auto-delay.
                self.$master.reg.cfgr.modify(|_, w| {
                    w.cont().set_bit();
                    w.autdly().set_bit()
                });
                // Clear the MDMA bits to reset any pending DMA requests from this
                // ADC pair.
                self.ccr_mut().modify(|_, w| unsafe { w.mdma().bits(0b00) });
                // Clear overrun, end-of-conversion, and end-of-sequence flags.
                self.$master.reg.isr.write(|w| {
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
                        let master_reg = unsafe { &(*$master_reg_ptr) };
                        let pair_reg = unsafe { &(*$pair_reg_ptr) };
                        // The hardware should automatically stop the ADC after the DMA
                        // transfer has completed. This loop is in case it takes the hardware a
                        // little while to stop the ADC. (The reference manual does not specify
                        // whether this is necessary.)
                        while master_reg.cr.read().adstart().bit_is_set() {}
                        // Disable generation of DMA requests.
                        pair_reg.ccr.modify(|_, w| unsafe { w.mdma().bits(0b00) });
                    }));
                }

                // Configure and enable the DMA transfer.
                self.ccr_mut().modify(|_, w| {
                    unsafe { w.mdma().bits(0b10) };
                    w.dmacfg().clear_bit() // one-shot mode
                });
                let data_reg = self.cdr() as *const _ as *const [u16; 2];
                let transfer =
                    unsafe { dma_tok.channel().enable_periph_to_mem(guard, data_reg, buf) };

                self.$master.reg.cr.modify(|_, w| w.adstart().set_bit());
                $Pair {
                    $master: $Master {
                        clock_freq: self.$master.clock_freq,
                        reg: self.$master.reg,
                        pair_state: self.$master.pair_state,
                        state: RunningDmaMaster {
                            seq: self.$master.state.life,
                            transfer,
                        },
                    },
                    $slave: $Slave {
                        clock_freq: self.$slave.clock_freq,
                        reg: self.$slave.reg,
                        pair_state: self.$slave.pair_state,
                        state: RunningDmaSlave {
                            seq: self.$slave.state.life,
                            body: PhantomData,
                            scope: PhantomData,
                        },
                    },
                }
            }

            /// Fills the buffer with ADC measurements, using DMA.
            ///
            /// Sets the ADCs to continuous conversion mode with auto-delay;
            /// configures and enables the DMA channel for writing into the
            /// buffer in one shot mode; starts the regular sequence; and waits
            /// for the DMA transfer to complete.
            ///
            /// This method uses dual-DMA mode (`MDMA = 0b10`) so that only a
            /// single DMA channel is necessary.
            // TODO: take self and the DMA channel by mutable reference
            pub fn fill_buf_dma<'a, D>(
                self,
                buf: &'a mut [[u16; 2]],
                dma_tok: D,
            ) -> (Self, D::Channel)
            where
                'm: 'a,
                's: 'a,
                D: AdcDmaTokens<'a, <Self as AdcPair>::Master>,
            {
                let (pair, dma_chan) = scope!(move |guard| {
                    let running = self.start_dma(buf, guard, dma_tok);
                    let (pair, dma_chan, _, _) = running.wait();
                    (pair, dma_chan)
                });
                (pair, dma_chan)
            }
        }
    };
}

macro_rules! impl_pair_dual_runningdma_runningdma {
    ($Pair:ident, $Master:ident, $Slave:ident, $master:ident, $slave:ident) => {
        impl<'m, 's, 'body, 'scope, C>
            $Pair<Dual, RunningDmaMaster<'m, 'body, 'scope, C>, RunningDmaSlave<'s, 'body, 'scope>>
        where
            C: dma::DmaChannel,
        {
            /// Waits for the DMA transfer to finish.
            pub fn wait(
                self,
            ) -> (
                $Pair<Dual, WithSequence<'m>, WithSequence<'s>>,
                C,
                ScopeGuard<'body, 'scope, dma::WaitHandler<Option<fn()>>>,
                &'scope mut [[u16; 2]],
            ) {
                let $Pair { $master, $slave } = self;
                // Wait for DMA transfer to finish.
                let (chan, mut guard, buf) = $master.state.transfer.wait();
                // The hardware should automatically stop the ADCs. This loop is in
                // case it takes a little while to stop the ADCs after the DMA
                // transfer has completed. (The reference manual does not specify
                // whether this is necessary.)
                while $master.reg.cr.read().adstart().bit_is_set() {}
                // Put the ADCs back together so that we can access `.ccr_mut()`.
                let mut pair = $Pair {
                    $master: $Master {
                        clock_freq: $master.clock_freq,
                        reg: $master.reg,
                        pair_state: $master.pair_state,
                        state: WithSequence {
                            life: $master.state.seq,
                        },
                    },
                    $slave: $Slave {
                        clock_freq: $slave.clock_freq,
                        reg: $slave.reg,
                        pair_state: $slave.pair_state,
                        state: WithSequence {
                            life: $slave.state.seq,
                        },
                    },
                };
                // Disable generation of DMA requests.
                pair.ccr_mut().modify(|_, w| unsafe { w.mdma().bits(0b00) });
                // Remove guard handler.
                if let Some(handler) = guard.handler_mut() {
                    handler.set_periph(None);
                }
                // There should never be an overrun in auto-delayed mode.
                debug_assert!(pair.$master.reg.isr.read().ovr().bit_is_clear());

                (pair, chan, guard, buf)
            }
        }
    };
}

macro_rules! impl_single_any {
    ($Adci:ident, $AdciChannel:ident) => {
        impl<P, S> $Adci<P, S> {
            /// Returns the ADC clock frequency.
            ///
            /// This value is shared for both ADCs in the pair.
            pub fn clock_freq(&self) -> Hertz {
                self.clock_freq
            }

            /// Sets the channel with the given type into single-ended mode.
            unsafe fn set_single_ended_unchecked<T: $AdciChannel>(&mut self, _: &T) {
                let channel_num = T::ID as u8;
                debug_assert!(1 <= channel_num && channel_num <= 15);
                self.reg.difsel.modify(|r, w| {
                    w.difsel_1_15()
                        .bits(r.difsel_1_15().bits() & !(1 << channel_num))
                });
            }

            /// Sets the channel with the given type into differential mode.
            unsafe fn set_differential_unchecked<T: $AdciChannel>(&mut self, _: &T) {
                let channel_num = T::ID as u8;
                debug_assert!(1 <= channel_num && channel_num <= 15);
                self.reg.difsel.modify(|r, w| {
                    w.difsel_1_15()
                        .bits(r.difsel_1_15().bits() | (1 << channel_num))
                });
            }
        }
    };
}

macro_rules! impl_single_unpowered {
    ($Adci:ident) => {
        impl<P> $Adci<P, Unpowered> {
            /// Enable ADC voltage regulator.
            pub fn power_on(self, delay: &mut ::delay::Delay) -> $Adci<P, Disabled> {
                // Enable ADC voltage regulator.
                self.reg
                    .cr
                    .write(|w| w.deeppwd().clear_bit().advregen().clear_bit());
                self.reg
                    .cr
                    .write(|w| w.deeppwd().clear_bit().advregen().set_bit());
                delay.delay_us(10u8);

                $Adci {
                    clock_freq: self.clock_freq,
                    reg: self.reg,
                    pair_state: PhantomData,
                    state: Disabled {},
                }
            }
        }
    };
}

macro_rules! impl_single_disabled {
    ($Adci:ident) => {
        impl<P> $Adci<P, Disabled> {
            /// Calibrates the ADC for single-ended inputs.
            pub fn calibrate_single_ended(&mut self, delay: &mut ::delay::Delay) {
                debug_assert!(self.reg.cr.read().aden().bit_is_clear());
                // Select single-ended calibration.
                self.reg.cr.modify(|_, w| w.adcaldif().clear_bit());
                // Perform calibration.
                self.reg.cr.modify(|_, w| w.adcal().set_bit());
                while self.reg.cr.read().adcal().bit_is_set() {}
                // Wait for at least 4 ADC clock cycles.
                // (See note in sec 15.3.9 of reference manual.)
                delay.delay_us(::core::cmp::max(4_000_000 / self.clock_freq.0, 1))
            }

            /// Calibrates the ADC for differential inputs.
            pub fn calibrate_differential(&mut self, delay: &mut ::delay::Delay) {
                debug_assert!(self.reg.cr.read().aden().bit_is_clear());
                // Select differential calibration.
                self.reg.cr.modify(|_, w| w.adcaldif().set_bit());
                // Perform calibration.
                self.reg.cr.modify(|_, w| w.adcal().set_bit());
                while self.reg.cr.read().adcal().bit_is_set() {}
                // Wait for at least 4 ADC clock cycles.
                // (See note in sec 15.3.9 of reference manual.)
                delay.delay_us(::core::cmp::max(4_000_000 / self.clock_freq.0, 1))
            }

            /// Enables the ADC. It should be calibrated first.
            pub fn enable(self) -> $Adci<P, Enabled> {
                self.reg.cr.modify(|_, w| w.aden().set_bit());
                while self.reg.isr.read().adrdy().bit_is_clear() {}
                $Adci {
                    clock_freq: self.clock_freq,
                    reg: self.reg,
                    pair_state: PhantomData,
                    state: Enabled {},
                }
            }
        }
    };
}

macro_rules! impl_single_enabled {
    ($Adci:ident, $AdciChannelRef:ident) => {
        impl<P> $Adci<P, Enabled> {
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
            // TODO: allow this method in other states
            pub fn set_resolution(&mut self, res: Resolution) {
                debug_assert!(self.reg.cr.read().adstart().bit_is_clear());
                debug_assert!(self.reg.cr.read().jadstart().bit_is_clear());
                self.reg
                    .cfgr
                    .modify(|_, w| unsafe { w.res().bits(res.to_bits()) });
            }

            /// Gets the data resolution.
            // TODO: allow this method in other states
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
                sequence: &[$AdciChannelRef<'a>],
            ) -> $Adci<P, WithSequence<'a>> {
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
                $Adci {
                    clock_freq: self.clock_freq,
                    reg: self.reg,
                    pair_state: PhantomData,
                    state: WithSequence { life: PhantomData },
                }
            }

            /// Disables the ADC.
            // TODO: Enable this in the `WithSequence` state.
            pub fn disable(self) -> $Adci<P, Disabled> {
                debug_assert!(self.reg.cr.read().adstart().bit_is_clear());
                debug_assert!(self.reg.cr.read().jadstart().bit_is_clear());
                self.reg.cr.modify(|_, w| w.addis().set_bit());
                while self.reg.cr.read().aden().bit_is_set() {}
                debug_assert!(self.reg.cr.read().addis().bit_is_clear());
                $Adci {
                    clock_freq: self.clock_freq,
                    reg: self.reg,
                    pair_state: PhantomData,
                    state: Disabled {},
                }
            }
        }
    };
}

macro_rules! impl_single_with_sequence {
    ($Adci:ident, $AdciChannelId:ident) => {
        impl<'a, P> $Adci<P, WithSequence<'a>> {
            /// Returns the number of channels in the regular conversion sequence.
            pub fn sequence_len(&self) -> u8 {
                self.reg.sqr1.read().l3().bits() + 1
            }

            /// Returns the channel IDs in the regular sequence.
            pub fn sequence_ids(&self) -> Vec<$AdciChannelId, [$AdciChannelId; 16]> {
                let sqr1 = self.reg.sqr1.read();
                let num_channels = sqr1.l3().bits() + 1;
                let mut ids = Vec::new();
                for index in 0..num_channels {
                    match index {
                        0 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr1.read().sq1().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        1 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr1.read().sq2().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        2 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr1.read().sq3().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        3 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr1.read().sq4().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        4 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq5().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        5 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq6().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        6 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq7().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        7 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq8().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        8 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr2.read().sq9().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        9 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq10().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        10 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq11().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        11 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq12().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        12 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq13().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        13 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr3.read().sq14().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        14 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr4.read().sq15().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        15 => {
                            ids.push(unsafe {
                                *(&self.reg.sqr4.read().sq16().bits() as *const u8
                                    as *const $AdciChannelId)
                            }).unwrap()
                        }
                        _ => unreachable!(),
                    }
                }
                ids
            }

            /// Drops the borrow of the sequence pins.
            pub fn drop_sequence(self) -> $Adci<P, Enabled> {
                $Adci {
                    clock_freq: self.clock_freq,
                    reg: self.reg,
                    pair_state: self.pair_state,
                    state: Enabled {},
                }
            }
        }
    };
}

macro_rules! impl_single_not_running {
    ($Adci:ident, $AdcijExternalTrigger:ident) => {
        impl<P, S> $Adci<P, S>
        where
            S: NotRunning,
        {
            /// Selects the trigger for regular conversions.
            ///
            /// Note that in dual ADC modes, the trigger of the slave ADC is ignored.
            pub fn set_trigger(&mut self, trigger: Trigger<$AdcijExternalTrigger>) {
                match trigger {
                    Trigger::Software => self
                        .reg
                        .cfgr
                        .modify(|_, w| unsafe { w.exten().bits(trigger.exten_bits()) }),
                    Trigger::Rising(ext)
                    | Trigger::Falling(ext)
                    | Trigger::RisingAndFalling(ext) => self.reg.cfgr.modify(|_, w| unsafe {
                        w.exten().bits(trigger.exten_bits());
                        w.extsel().bits(ext as u8)
                    }),
                }
            }
        }
    };
}

macro_rules! impl_single_independent_with_sequence {
    ($Adci:ident, $ADCi:ident) => {
        impl<'seq> $Adci<Independent, WithSequence<'seq>> {
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
                dma_tok: D,
            ) -> $Adci<Independent, RunningDma<'seq, 'body, 'scope, D::Channel>>
            where
                'seq: 'scope,
                D: AdcDmaTokens<'scope, Self>,
            {
                use dma::DmaChannelPriv;

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
                        let reg = unsafe { &(*stm32f30x::$ADCi::ptr()) };
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
                let transfer =
                    unsafe { dma_tok.channel().enable_periph_to_mem(guard, data_reg, buf) };

                self.reg.cr.modify(|_, w| w.adstart().set_bit());
                $Adci {
                    clock_freq: self.clock_freq,
                    reg: self.reg,
                    pair_state: self.pair_state,
                    state: RunningDma {
                        seq: self.state.life,
                        transfer: transfer,
                    },
                }
            }

            /// Fills the buffer with ADC measurements, using DMA.
            ///
            /// Sets the ADC to continuous conversion mode with auto-delay;
            /// configures and enables the DMA channel for writing into the
            /// buffer in one shot mode; starts the regular sequence; and waits
            /// for the DMA transfer to complete.
            // TODO: take self and the DMA channel by mutable reference
            pub fn fill_buf_dma<'a, D>(self, buf: &'a mut [u16], dma_tok: D) -> (Self, D::Channel)
            where
                'seq: 'a,
                D: AdcDmaTokens<'a, Self>,
            {
                let (adc, dma_chan) = scope!(move |guard| {
                    let running = self.start_dma(buf, guard, dma_tok);
                    let (adc, dma_chan, _, _) = running.wait();
                    (adc, dma_chan)
                });
                (adc, dma_chan)
            }
        }
    };
}

macro_rules! impl_single_independent_running_dma {
    ($Adci:ident) => {
        impl<'seq, 'body, 'scope, C> $Adci<Independent, RunningDma<'seq, 'body, 'scope, C>>
        where
            C: dma::DmaChannel,
        {
            /// Waits for the DMA transfer to finish.
            pub fn wait(
                self,
            ) -> (
                $Adci<Independent, WithSequence<'seq>>,
                C,
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
                    $Adci {
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

macro_rules! define_impl_internalref {
    ($Pair:ident) => {
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

        impl<P> $Pair<P, Disabled, Disabled> {
            /// Enables the internal reference voltage and returns a handle to
            /// it.
            ///
            /// Returns `None` if the internal reference voltage is already
            /// enabled on ADC1/2 or ADC3/4.
            pub fn enable_internal_ref(&mut self) -> Option<InternalRef> {
                ::cortex_m::interrupt::free(|_| {
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
                ::cortex_m::interrupt::free(|_| {
                    self.ccr_mut().modify(|_, w| w.vrefen().clear_bit());
                    unsafe { INTERNAL_REF_ENABLED = false };
                });
            }
        }
    };
}

macro_rules! impl_channel_from_pins {
    ($borrowed:ident, $chan_trait:ident, $channel:ident, $pos:ident) => {
        impl<'a> From<(&'a $channel<SingleEnded>, &'a $pos<Analog>)> for $borrowed<'a> {
            fn from(_: (&'a $channel<SingleEnded>, &'a $pos<Analog>)) -> $borrowed<'a> {
                $borrowed {
                    id: <$channel<SingleEnded> as $chan_trait>::ID,
                    life: PhantomData,
                }
            }
        }
    };
    ($borrowed:ident, $chan_trait:ident, $channel:ident, $pos:ident, $neg:ident) => {
        impl_channel_from_pins!($borrowed, $chan_trait, $channel, $pos);
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
                    id: <$channel<SingleEnded> as $chan_trait>::ID,
                    life: PhantomData,
                }
            }
        }
    };
}

macro_rules! impl_channel_conversions {
    ($Pair:ident, $pos:ident, $neg:ident, [$($adc:ident),*]) => {
        impl $pos<SingleEnded> {
            /// Changes the channel to differential mode, where `self` is the
            /// positive input and `_neg` is the negative input.
            pub fn into_differential<P>(
                self,
                _neg: $neg<SingleEnded>,
                pair: &mut $Pair<P, Disabled, Disabled>,
            ) -> $pos<Differential> {
                unsafe {
                    $(
                        pair.$adc.set_differential_unchecked(&self);
                    )*
                }
                $pos {
                    _state: PhantomData,
                }
            }
        }

        impl $pos<Differential> {
            /// Changes the channel to single-ended mode.
            pub fn into_single_ended<P>(
                self,
                pair: &mut $Pair<P, Disabled, Disabled>,
            ) -> ($pos<SingleEnded>, $neg<SingleEnded>) {
                unsafe {
                    $(
                        pair.$adc.set_single_ended_unchecked(&self);
                    )*
                }
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

macro_rules! impl_channel_conversions_shared_neg {
    (
        $Pair:ident,
        ($this_adc:ident, $this_pos:ident),
        ($other_adc:ident, $other_pos:ident),
        $neg:ident
    ) => {
        impl $this_pos<SingleEnded> {
            /// Changes the channel to differential mode, where `self` is the
            /// positive input and `_neg` is the negative input.
            pub fn into_differential<P>(
                self,
                _neg: $neg<SingleEnded>,
                pair: &mut $Pair<P, Disabled, Disabled>,
            ) -> $this_pos<Differential> {
                unsafe { pair.$this_adc.set_differential_unchecked(&self) }
                $this_pos {
                    _state: PhantomData,
                }
            }
        }

        impl $this_pos<Differential> {
            /// Changes the channel to single-ended mode.
            ///
            /// It is necessary to pass a reference to the other channel in single-ended mode
            /// that shares this channel's negative input. Otherwise, it would be possible to
            /// convert both into differential mode with `.both_into_differential()`, and then
            /// individually convert them back to single-ended mode with `.into_single_ended()`
            /// to get two copies of the negative input.
            pub fn into_single_ended<P>(
                self,
                _other_pos: &$other_pos<SingleEnded>,
                pair: &mut $Pair<P, Disabled, Disabled>,
            ) -> ($this_pos<SingleEnded>, $neg<SingleEnded>) {
                unsafe { pair.$this_adc.set_single_ended_unchecked(&self) }
                (
                    $this_pos {
                        _state: PhantomData,
                    },
                    $neg {
                        _state: PhantomData,
                    },
                )
            }
        }

        impl $this_pos<SingleEnded> {
            /// Changes the channels to differential mode, where `self` and
            /// `_other_pos` are the positive inputs and `_neg` is the negative
            /// input.
            pub fn both_into_differential<P>(
                self,
                other_pos: $other_pos<SingleEnded>,
                _neg: $neg<SingleEnded>,
                pair: &mut $Pair<P, Disabled, Disabled>,
            ) -> ($this_pos<Differential>, $other_pos<Differential>) {
                unsafe {
                    pair.$this_adc.set_differential_unchecked(&self);
                    pair.$other_adc.set_differential_unchecked(&other_pos);
                }
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
                other: $other_pos<Differential>,
                pair: &mut $Pair<P, Disabled, Disabled>,
            ) -> (
                ($this_pos<SingleEnded>, $other_pos<SingleEnded>),
                $neg<SingleEnded>,
            ) {
                unsafe {
                    pair.$this_adc.set_single_ended_unchecked(&self);
                    pair.$other_adc.set_single_ended_unchecked(&other);
                }
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

pub mod adc12;
pub mod adc34;
