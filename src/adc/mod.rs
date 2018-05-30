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

macro_rules! impl_single_any {
    ($Adci:ident) => {
        impl<P, S> $Adci<P, S> {
            /// Returns the ADC clock frequency.
            ///
            /// This value is shared for both ADCs in the pair.
            pub fn clock_freq(&self) -> Hertz {
                self.clock_freq
            }
        }
    };
}

macro_rules! impl_single_unpowered {
    ($Adci:ident) => {
        impl<P> $Adci<P, Unpowered> {
            /// Enable ADC voltage regulator.
            pub fn power_on(self, delay: &mut Delay) -> $Adci<P, Disabled> {
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
    ($Adci:ident, $AdciChannel:ident) => {
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
                sequence: &[$AdciChannel<'a>],
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
                        0 => ids.push(unsafe {
                            *(&self.reg.sqr1.read().sq1().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        1 => ids.push(unsafe {
                            *(&self.reg.sqr1.read().sq2().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        2 => ids.push(unsafe {
                            *(&self.reg.sqr1.read().sq3().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        3 => ids.push(unsafe {
                            *(&self.reg.sqr1.read().sq4().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        4 => ids.push(unsafe {
                            *(&self.reg.sqr2.read().sq5().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        5 => ids.push(unsafe {
                            *(&self.reg.sqr2.read().sq6().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        6 => ids.push(unsafe {
                            *(&self.reg.sqr2.read().sq7().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        7 => ids.push(unsafe {
                            *(&self.reg.sqr2.read().sq8().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        8 => ids.push(unsafe {
                            *(&self.reg.sqr2.read().sq9().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        9 => ids.push(unsafe {
                            *(&self.reg.sqr3.read().sq10().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        10 => ids.push(unsafe {
                            *(&self.reg.sqr3.read().sq11().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        11 => ids.push(unsafe {
                            *(&self.reg.sqr3.read().sq12().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        12 => ids.push(unsafe {
                            *(&self.reg.sqr3.read().sq13().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        13 => ids.push(unsafe {
                            *(&self.reg.sqr3.read().sq14().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        14 => ids.push(unsafe {
                            *(&self.reg.sqr4.read().sq15().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
                        15 => ids.push(unsafe {
                            *(&self.reg.sqr4.read().sq16().bits() as *const u8
                                as *const $AdciChannelId)
                        }).unwrap(),
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
        }
    };
}

macro_rules! impl_single_independent_running_dma {
    ($Adci:ident, $dma_chan:ty) => {
        impl<'seq, 'body, 'scope> $Adci<Independent, RunningDma<'seq, 'body, 'scope, $dma_chan>> {
            /// Waits for the DMA transfer to finish.
            pub fn wait(
                self,
            ) -> (
                $Adci<Independent, WithSequence<'seq>>,
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

pub mod adc12;
// pub mod adc34;
