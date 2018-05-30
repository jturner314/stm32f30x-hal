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

pub mod adc12;
