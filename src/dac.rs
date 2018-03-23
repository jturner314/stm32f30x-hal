//! Digital-to-analog converters (DACs)
//!
//! TODO: Is it reallly allowed to set output values before enabling the channels?

use core::marker::PhantomData;
use gpio::Analog;
use gpio::gpioa::{PA4, PA5};
use rcc::APB1;
use stm32f30x;

/// Extension trait to provide a safe wrapper for a DAC.
pub trait DacExt {
    /// The safe DAC wrapper.
    type Dac;
    /// Creates a safe wrapper for the DAC register block.
    fn wrap(self, apb1: &mut APB1) -> Self::Dac;
}

/// Disabled DAC.
pub struct Disabled {}
/// Single-channel mode.
///
/// `'chan1` is the lifetime of the borrow of the GPIO pin.
pub struct SingleEnabled<'chan1> {
    chan1: PhantomData<&'chan1 Analog>,
}
/// Dual-channel mode.
///
/// `'chan1` and `'chan2` are the lifetimes of the borrows of the GPIO pins.
pub struct DualEnabled<'chan1, 'chan2> {
    chan1: PhantomData<&'chan1 Analog>,
    chan2: PhantomData<&'chan2 Analog>,
}

/// A state that is not "enabled single mode".
pub trait NotSingleEnabled {}
impl NotSingleEnabled for Disabled {}
impl<'chan1, 'chan2> NotSingleEnabled for DualEnabled<'chan1, 'chan2> {}

impl DacExt for stm32f30x::DAC {
    type Dac = Dac1<Disabled>;

    fn wrap(self, apb1: &mut APB1) -> Dac1<Disabled> {
        // Enable and reset DAC.
        apb1.enr().modify(|_, w| w.dacen().enabled());
        apb1.rstr().modify(|_, w| w.dacrst().set_bit());
        apb1.rstr().modify(|_, w| w.dacrst().clear_bit());

        Dac1 {
            _state: Disabled {},
        }
    }
}

/// Safe wrapper for DAC1.
///
/// This represents exclusive access to DAC1.
pub struct Dac1<State> {
    _state: State,
}

impl<State> Dac1<State> {
    /// Returns a reference to the DAC's register block.
    fn reg(&self) -> &stm32f30x::dac::RegisterBlock {
        // This is safe because an instance of `Dac1` represents exclusive
        // ownership over the DAC.
        unsafe { &*(stm32f30x::DAC::ptr()) }
    }

    /// Returns the value that is the output for channel 1 when it's enabled.
    pub fn channel1_output(&self) -> u16 {
        self.reg().dor1.read().dacc1dor().bits()
    }

    /// Returns the value that is the output for channel 2 when it's enabled.
    pub fn channel2_output(&self) -> u16 {
        self.reg().dor2.read().dacc2dor().bits()
    }

    /// Enable/disable the output buffer/switch for channel 1.
    pub fn buffer_channel1(&mut self, buffer: bool) {
        self.reg().cr.modify(|_, w| w.boff1().bit(buffer));
    }
}

impl<State> Dac1<State>
where
    State: NotSingleEnabled,
{
    /// Enable/disable the output buffer/switch for channel 2.
    pub fn buffer_channel2(&mut self, buffer: bool) {
        self.reg().cr.modify(|_, w| w.boff2().bit(buffer));
    }
}

impl Dac1<Disabled> {
    /// Enables single-channel mode for the duration of `f`.
    ///
    /// This enables channel 1 and disables channel 2, calls `f` with the
    /// enabled DAC, and then disables the channels.
    ///
    /// **Note that this resets the channel 2 configuration and channel 2
    /// underrun status since these bits are reserved in single mode.** The
    /// only exceptions are the channel 2 pending and output data values, which
    /// are preserved.
    pub fn with_single_enabled<'chan1, F, O>(&mut self, _chan1: &'chan1 PA4<Analog>, f: F) -> O
    where
        F: FnOnce(&mut Dac1<SingleEnabled<'chan1>>) -> O,
    {
        // Clear DAC_SR bits that are reserved in single mode.
        self.reg().sr.modify(|_, w| w.dmaudr2().clear_bit());
        // Ideally, we'd also clear the DAC_SWTRIGR bits that are reserved in
        // single mode (SWTRIG2), but the register is write-only, so we can't
        // do so without modifying the other bits too (SWTRIG1). This shouldn't
        // be an issue in practice because the hardware clears the bits in
        // DAC_SWTRIGR one APB1 clock cycle after they're written, and, in
        // theory, one APB1 clock cycle will take place before channel 1 is
        // actually enabled.
        self.reg().cr.modify(|r, w| {
            // Clear DAC_CR bits that are reserved in single mode.
            unsafe { w.bits(r.bits() & 0x0000_FFFF) };
            // Enable channel 1.
            w.en1().set_bit()
        });
        let out = f(&mut Dac1 {
            _state: SingleEnabled { chan1: PhantomData },
        });
        self.reg().cr.modify(|_, w| w.en1().clear_bit());
        out
    }

    /// Enables dual-channel mode for the duration of `f`.
    ///
    /// This enables both channels, calls `f` with the enabled DAC, and then
    /// disables the channels.
    pub fn with_dual_enabled<'chan1, 'chan2, F, O>(
        self,
        _chan1: &'chan1 PA4<Analog>,
        _chan2: &'chan2 PA5<Analog>,
        f: F,
    ) -> O
    where
        F: FnOnce(&mut Dac1<DualEnabled<'chan1, 'chan2>>) -> O,
    {
        self.reg()
            .cr
            .modify(|_, w| w.en1().set_bit().en2().set_bit());
        let out = f(&mut Dac1 {
            _state: DualEnabled {
                chan1: PhantomData,
                chan2: PhantomData,
            },
        });
        self.reg()
            .cr
            .modify(|_, w| w.en1().clear_bit().en2().clear_bit());
        out
    }
}

/// Trait for setting/reading the pending output value for DAC channel 1.
pub trait DacPendingChannel1<T> {
    /// Sets the pending output value for channel 1.
    ///
    /// **For types `T` with more than 12 bits, only the least-significant 12
    /// bits are used.**
    ///
    /// If a trigger is configured, this value isn't actually output until the
    /// next trigger.
    fn set_pending_channel1(&mut self, value: T);

    /// Returns the pending output value for channel 1.
    ///
    /// (For types `T` less than 12 bits, returns the most-significant bits.)
    fn pending_channel1(&self) -> T;
}

macro_rules! impl_dac_pending_channel1 {
    ($value:ty, $reg:ident) => {
        impl<State> DacPendingChannel1<$value> for Dac1<State> {
            fn set_pending_channel1(&mut self, value: $value) {
                self.reg()
                    .$reg
                    .write(|w| unsafe { w.dacc1dhr().bits(value) })
            }
            fn pending_channel1(&self) -> $value {
                self.reg().$reg.read().dacc1dhr().bits()
            }
        }
    };
}
impl_dac_pending_channel1!(u8, dhr8r1);
impl_dac_pending_channel1!(u16, dhr12r1);

/// Trait for setting/reading the pending output value for DAC channel 2.
pub trait DacPendingChannel2<T> {
    /// Sets the pending output value for channel 2.
    ///
    /// **For types `T` with more than 12 bits, only the least-significant 12
    /// bits are used.**
    ///
    /// If a trigger is configured, this value isn't actually output until the
    /// next trigger.
    fn set_pending_channel2(&mut self, value: T);

    /// Returns the pending output value for channel 2.
    ///
    /// (For types `T` less than 12 bits, returns the most-significant bits.)
    fn pending_channel2(&self) -> T;
}

macro_rules! impl_dac_pending_channel2 {
    ($value:ty, $reg:ident) => {
        impl<State> DacPendingChannel2<$value> for Dac1<State> {
            fn set_pending_channel2(&mut self, value: $value) {
                self.reg()
                    .$reg
                    .write(|w| unsafe { w.dacc2dhr().bits(value) })
            }
            fn pending_channel2(&self) -> $value {
                self.reg().$reg.read().dacc2dhr().bits()
            }
        }
    };
}
impl_dac_pending_channel2!(u8, dhr8r2);
impl_dac_pending_channel2!(u16, dhr12r2);

/// Trait for setting/reading the pending output values for both DAC channels.
pub trait DacPendingChannels<T> {
    /// Sets the pending output values for both channels.
    ///
    /// **For types `T` with more than 12 bits, only the least-significant 12
    /// bits of each value are used.**
    ///
    /// If trigger(s) are configured, the corresponding value(s) aren't
    /// actually output until the next trigger(s).
    fn set_pending_channels(&mut self, values: [T; 2]);

    /// Returns the pending output values for both channels.
    ///
    /// (For types `T` less than 12 bits, returns the most-significant bits of
    /// each value.)
    fn pending_channels(&self) -> [T; 2];
}

macro_rules! impl_dac_pending_channels {
    ($value:ty, $reg:ident) => {
        impl<State> DacPendingChannels<$value> for Dac1<State> {
            fn set_pending_channels(&mut self, value: [$value; 2]) {
                self.reg()
                    .$reg
                    .write(|w| unsafe { w.dacc1dhr().bits(value[0]).dacc2dhr().bits(value[1]) })
            }
            fn pending_channels(&self) -> [$value; 2] {
                let dhr = self.reg().$reg.read();
                [dhr.dacc1dhr().bits(), dhr.dacc2dhr().bits()]
            }
        }
    };
}
impl_dac_pending_channels!(u8, dhr8rd);
impl_dac_pending_channels!(u16, dhr12rd);
