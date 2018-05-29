//! Direct memory access controllers (DMA)

// This should be true for all STM32 devices.
#![cfg(target_pointer_width = "32")]

pub(crate) use self::private::DmaChannelPriv;

use core::fmt::Debug;
use core::sync::atomic;
use rcc::AHB;
use strong_scope_guard::{ScopeEndHandler, ScopeGuard};

/// An ongoing DMA transfer.
///
/// The lifetimes `'body` and `'data` are from the `ScopeGuard` protecting the
/// source and destination data buffers.
#[derive(Debug)]
pub struct Transfer<'body, 'data, Channel, OutBuf, Handler>
where
    'data: 'body,
    Handler: ScopeEndHandler + 'body,
{
    guard: ScopeGuard<'body, 'data, Handler>,
    channel: Channel,
    out_buf: OutBuf,
}

/// Private module for preventing access to traits that appear in the public API.
mod private {
    use super::*;

    /// Crate-local trait implemented by all DMA channels.
    pub trait DmaChannelPriv: Sized {
        /// Enables a DMA transfer from a peripheral register to a memory buffer with
        /// circular mode disabled.
        ///
        /// The number of data to be transferred is `mem.len()`.
        ///
        /// The peripheral should add its handler to the `ScopeGuard` before passing
        /// the guard to this method.
        ///
        /// This method is unsafe because it causes the DMA to read from the memory
        /// located at the `periph` raw pointer. The caller must ensure that the DMA
        /// can read from this location until the `'data` lifetime ends or until the
        /// DMA transfer is disabled, whichever is shorter.
        ///
        /// **Panics** if `mem.len() > ::core::u16::MAX as usize`.
        ///
        /// **Warning**: If `periph` is not actually a peripheral register, then this
        /// transfer will never finish because the DMA will never receive any transfer
        /// requests.
        unsafe fn enable_periph_to_mem<'body, 'data, P, M, H>(
            self,
            guard: ScopeGuard<'body, 'data, WaitHandler<H>>,
            periph: *const P,
            mem: &'data mut [M],
        ) -> Transfer<'body, 'data, Self, &'data mut [M], WaitHandler<H>>
        where
            P: DataElem,
            M: DataElem + FromBits<P>,
            H: ScopeEndHandler;

        /// Enables a DMA transfer from a memory buffer to a peripheral register with
        /// circular mode disabled.
        ///
        /// The number of data to be transferred is `mem.len()`.
        ///
        /// The peripheral should add its handler to the `ScopeGuard` before passing
        /// the guard to this method.
        ///
        /// This method is unsafe because it causes the DMA to write to the memory
        /// located at the `periph` raw pointer. The caller must ensure that the DMA
        /// can write to this location until the `'data` lifetime ends or until the DMA
        /// transfer is disabled, whichever is shorter.
        ///
        /// **Panics** if `mem.len() > ::core::u16::MAX as usize`.
        ///
        /// **Warning**: If `periph` is not actually a peripheral register, then this
        /// transfer will never finish because the DMA will never receive any transfer
        /// requests.
        unsafe fn enable_mem_to_periph<'body, 'data, M, P, H>(
            self,
            guard: ScopeGuard<'body, 'data, WaitHandler<H>>,
            mem: &'data [M],
            periph: *mut P,
        ) -> Transfer<'body, 'data, Self, (), WaitHandler<H>>
        where
            P: DataElem,
            M: DataElem + FromBits<P>,
            H: ScopeEndHandler;

        /// Returns `true` iff the channel is enabled.
        #[doc(hidden)]
        fn is_enabled(&self) -> bool;

        /// Clears the enabled bit for this channel.
        // We can safely expose this method because the `Transfer` instance
        // contains the channel when it's enabled.
        #[doc(hidden)]
        fn clear_enabled(&mut self);
    }
}

/// Trait implemented by all DMA channels.
pub trait DmaChannel: Sized + Debug + DmaChannelPriv {
    /// Starts a DMA transfer from `src` to `dst`. (The number of data to be
    /// transferred is `src.len()`, which must be equal to `dst.len()`.)
    ///
    /// **Panics** if `src.len() > ::core::u16::MAX as usize` or if `src.len() !=
    /// dst.len()`.
    fn start_mem_to_mem<'body, 'data, S, D>(
        self,
        guard: ScopeGuard<'body, 'data, WaitHandler<()>>,
        src: &'data [S],
        dst: &'data mut [D],
    ) -> Transfer<'body, 'data, Self, &'data mut [D], WaitHandler<()>>
    where
        S: DataElem,
        D: DataElem + FromBits<S>;

    /// Returns the channel priority level.
    fn priority(&self) -> Priority;

    /// Sets the channel priority level.
    fn set_priority(&mut self, priority: Priority);

    /// Returns the global interrupt flag for this channel.
    fn global_interrupt(&self) -> bool;

    /// Clears the global interrupt flag for this channel.
    fn clear_global_interrupt(&mut self);

    /// Returns the transfer complete flag for this channel.
    fn transfer_complete(&self) -> bool;

    /// Clears the transfer complete flag for this channel.
    fn clear_transfer_complete(&mut self);

    /// Returns the half transfer complete flag for this channel.
    fn half_transfer_complete(&self) -> bool;

    /// Clears the half transfer complete flag for this channel.
    fn clear_half_transfer_complete(&mut self);

    /// Returns the transfer error flag for this channel.
    fn transfer_error(&self) -> bool;

    /// Clears the transfer error flag for this channel.
    fn clear_transfer_error(&mut self);

    /// Clears all of the flags at the same time.
    fn clear_all_flags(&mut self);
}

/// `ScopeGuard` handler for a DMA transfer.
///
/// `H` is the peripheral's handler if the transfer is to/from a peripheral.
///
/// When this handler is called, it executes the following two steps in order:
///
/// 1. Wait until the end of the DMA transfer, then disable the DMA channel.
/// 2. Call the peripheral handler.
#[derive(Debug)]
pub struct WaitHandler<H: ScopeEndHandler> {
    periph: H,
    dma: Option<unsafe fn()>,
}

impl<H: ScopeEndHandler> WaitHandler<H> {
    /// Sets the peripheral handler.
    pub(crate) fn set_periph(&mut self, handler: H) {
        self.periph = handler
    }
}

impl<H: ScopeEndHandler> ScopeEndHandler for WaitHandler<H> {
    fn none() -> Self {
        WaitHandler {
            periph: H::none(),
            dma: None,
        }
    }

    fn call(self) {
        if let Some(f) = self.dma {
            unsafe { (f)() }
        }
        self.periph.call();
    }
}

/// A scope guard handler that contains a DMA handler.
pub trait DmaHandler: ScopeEndHandler {
    /// Disables the DMA-related portion of the handler.
    #[doc(hidden)]
    fn disable_dma_handler(&mut self);
}

impl<H: ScopeEndHandler> DmaHandler for WaitHandler<H> {
    fn disable_dma_handler(&mut self) {
        self.dma = None;
    }
}

/// Extension trait to split the DMA block into independent channels.
pub trait DmaExt {
    /// DMA parts.
    type Parts;

    /// Splits the DMA block into independent channels.
    fn split(self, ahb: &mut AHB) -> Self::Parts;
}

/// DMA channel priority level.
#[derive(Clone, Copy)]
pub enum Priority {
    /// Low priority.
    Low,
    /// Medium priority.
    Medium,
    /// High priority.
    High,
    /// Very high priority.
    VeryHigh,
}

impl Default for Priority {
    fn default() -> Priority {
        Priority::Low
    }
}

impl Priority {
    /// Returns the bitwise representation for the CCR register.
    fn to_bits(self) -> u8 {
        match self {
            Priority::Low => 0b00,
            Priority::Medium => 0b01,
            Priority::High => 0b10,
            Priority::VeryHigh => 0b11,
        }
    }

    /// Returns the priority corresponding to the bit pattern.
    ///
    /// **Panics** if the bit battern is invalid.
    fn from_bits(bits: u8) -> Priority {
        match bits {
            0b00 => Priority::Low,
            0b01 => Priority::Medium,
            0b10 => Priority::High,
            0b11 => Priority::VeryHigh,
            _ => panic!("Invalid bit pattern for Priority: {:b}", bits),
        }
    }
}

/// A trait that DMA-transferrable element types must implement.
///
/// To implement this trait for a type `T`, `::core::mem::size_of::<T>()` must
/// be one of 8, 16, or 32. (Otherwise, transfers for that type will panic.)
pub trait DataElem: Copy {}

impl DataElem for u8 {}
impl DataElem for i8 {}
impl DataElem for u16 {}
impl DataElem for i16 {}
impl DataElem for u32 {}
impl DataElem for i32 {}
impl DataElem for usize {}
impl DataElem for isize {}
impl DataElem for [u8; 2] {}
impl DataElem for [i8; 2] {}
impl DataElem for [u16; 2] {}
impl DataElem for [i16; 2] {}
impl DataElem for [u8; 4] {}
impl DataElem for [i8; 4] {}

/// A type that can be safely be produced from any possible bit pattern of the
/// `Source` type (accounting for size and alignment).
///
/// For example, all bit patterns are valid for `u8`, but not necessarily for
/// enum types that are `#[repr(u8)]`.
///
/// If the destination type is larger than the source type, then the source is
/// copied to least-significant bits (right-aligned) of the destination with
/// zeros filling the most-significant bits.
///
/// If the source type is larger than the destination type, then only the
/// least-significant bits are copied to the destination.
///
/// Ideally, there would also be an `unsafe impl<T: DataElem> FromBits<T> for T
/// {}` implementation, but until Rust has specialization, this conflicts with
/// the implementations for the integers.
pub unsafe trait FromBits<Source: DataElem>: DataElem {}

unsafe impl<T: DataElem> FromBits<T> for u8 {}
unsafe impl<T: DataElem> FromBits<T> for i8 {}
unsafe impl<T: DataElem> FromBits<T> for u16 {}
unsafe impl<T: DataElem> FromBits<T> for i16 {}
unsafe impl<T: DataElem> FromBits<T> for u32 {}
unsafe impl<T: DataElem> FromBits<T> for i32 {}
unsafe impl<T: DataElem> FromBits<T> for usize {}
unsafe impl<T: DataElem> FromBits<T> for isize {}
unsafe impl<T: DataElem> FromBits<T> for [u8; 2] {}
unsafe impl<T: DataElem> FromBits<T> for [i8; 2] {}
unsafe impl<T: DataElem> FromBits<T> for [u16; 2] {}
unsafe impl<T: DataElem> FromBits<T> for [i16; 2] {}
unsafe impl<T: DataElem> FromBits<T> for [u8; 4] {}
unsafe impl<T: DataElem> FromBits<T> for [i8; 4] {}

/// Width of data elements.
#[derive(Clone, Copy)]
enum DataWidth {
    /// 8-bit width.
    Bits8,
    /// 16-bit width.
    Bits16,
    /// 32-bit width.
    Bits32,
}

impl DataWidth {
    /// Converts the numeric width to an instance of `DataWidth`.
    ///
    /// **Panics** if the width is not valid.
    fn width_of<T>() -> Self {
        match ::core::mem::size_of::<T>() {
            1 => DataWidth::Bits8,
            2 => DataWidth::Bits16,
            4 => DataWidth::Bits32,
            width => panic!("Illegal width for DMA transfer: {}", width),
        }
    }

    /// Returns the bitwise representation for the CCR register.
    fn to_bits(self) -> u8 {
        match self {
            DataWidth::Bits8 => 0b00,
            DataWidth::Bits16 => 0b01,
            DataWidth::Bits32 => 0b10,
        }
    }
}

impl<'body, 'data, Channel, OutBuf, Handler> Transfer<'body, 'data, Channel, OutBuf, Handler>
where
    Channel: DmaChannel,
    Handler: DmaHandler,
{
    /// Returns `true` iff the transfer is complete or there was a fatal error.
    pub fn is_done(&self) -> bool {
        self.channel.transfer_error() || self.channel.transfer_complete()
    }

    /// Waits for the DMA transfer to finish (blocking).
    ///
    /// **Panics** if there was a transfer error. (This occurs when a DMA transfer
    /// is attempted to/from a reserved address space. This can happen if the stack
    /// is configured to be in CCM RAM and a DMA transfer is attempted to/from a
    /// stack-allocated buffer, since the DMA cannot access CCM RAM.)
    pub fn wait(self) -> (Channel, ScopeGuard<'body, 'data, Handler>, OutBuf) {
        let Transfer {
            mut guard,
            mut channel,
            out_buf,
        } = self;

        while !channel.transfer_error() && !channel.transfer_complete() {}

        // Disable DMA portion of guard handler.
        if let Some(handler) = guard.handler_mut() {
            handler.disable_dma_handler();
        }

        // Ensure that the compiler does not try to use previously-read values from
        // `dst` instead of reading the new values written by the DMA transfer.
        atomic::compiler_fence(atomic::Ordering::SeqCst);

        if channel.transfer_error() {
            // The hardware automatically disables the channel in the error case.
            debug_assert!(!channel.is_enabled());
            panic!("DMA transfer error for channel {:?}", channel);
        }

        channel.clear_enabled();
        (channel, guard, out_buf)
    }

    /// Disables the DMA transfer without waiting for it to finish.
    pub fn disable(self) -> (Channel, ScopeGuard<'body, 'data, Handler>, OutBuf) {
        let Transfer {
            mut guard,
            mut channel,
            out_buf,
        } = self;
        channel.clear_enabled();
        if let Some(handler) = guard.handler_mut() {
            handler.disable_dma_handler();
        }
        atomic::compiler_fence(atomic::Ordering::SeqCst);
        (channel, guard, out_buf)
    }
}

macro_rules! dma {
    ($DMAx:ident, $Dmax:ident, $DmaxChannel:ident, $dmax:ident, $dmaxenr:ident, [
        $(
            $Channeli:ident: (
                $chani:ident,
                gif => ($gifi:ident, $cgifi:ident),
                tcif => ($tcifi:ident, $ctcifi:ident),
                htif => ($htifi:ident, $chtifi:ident),
                teif => ($teifi:ident, $cteifi:ident),
                ccr => ($ccri:ident, $CCRi:ident),
                cndtr => ($cndtri:ident, $CNDTRi:ident),
                cpar => ($cpari:ident, $CPARi:ident),
                cmar => ($cmari:ident, $CMARi:ident),
            ),
        )+
    ]) => {
        /// DMA
        pub mod $dmax {
            use stm32f30x::$DMAx;
            $(
                use stm32f30x::dma1::{
                    $ccri,
                    $CCRi,
                    // $cndtri,
                    $CNDTRi,
                    // $cpari,
                    $CPARi,
                    // $cmari,
                    $CMARi
                };
            )+
            use super::*;

            /// DMA parts
            pub struct Parts {
                /// Opaque wrapper to hold the non-HAL DMA token.
                pub $dmax: $Dmax,
                $(
                    /// Channel
                    pub $chani: $Channeli,
                )+
            }

            impl DmaExt for $DMAx {
                type Parts = Parts;

                fn split(self, ahb: &mut AHB) -> Parts {
                    ahb.enr().modify(|_, w| w.$dmaxenr().enabled());

                    Parts {
                        $dmax: $Dmax { reg: self },
                        $(
                            $chani: $Channeli  { _0: () },
                        )+
                    }
                }
            }

            impl Parts {
                /// Disables the DMA controller.
                ///
                /// Note that this takes ownership of `self` to ensure that
                /// nothing still has access to a DMA channel and all the
                /// channels are disabled. It returns the non-HAL token so that
                /// you can call `.split()` again if desired.
                pub fn disable(self, ahb: &mut AHB) -> $DMAx {
                    ahb.enr().modify(|_, w| w.$dmaxenr().disabled());
                    self.$dmax.reg
                }
            }

            /// DMA wrapper.
            pub struct $Dmax {
                reg: $DMAx,
            }

            $(
                /// Token that represents control of the DMA channel.
                #[derive(Debug)]
                pub struct $Channeli {
                    _0: (),
                }

                impl $Channeli {
                    /// Waits for the ongoing transfer to complete and disables the channel.
                    ///
                    /// This function is intended only for use in `ScopeGuard` handlers.
                    ///
                    /// This is unsafe because it modifies the DMA channel without borrowing the
                    /// channel token. The caller must ensure that (1) the DMA channel cannot be
                    /// modified concurrently with the call to this function, (2) the DMA channel
                    /// is currently enabled, and (3) no `Transfer` instances with this cahnnel
                    /// are accessible after this function is called.
                    unsafe fn wait_unchecked() {
                        let reg = &(*$DMAx::ptr());
                        let mut isr = reg.isr.read();
                        // Wait for transfer error or transfer complete.
                        while isr.$teifi().bit_is_clear() && isr.$tcifi().bit_is_clear() {
                            isr = reg.isr.read();
                        }

                        // Ensure that the compiler does not try to use old values from the memory
                        // buffers.
                        atomic::compiler_fence(atomic::Ordering::SeqCst);

                        if isr.$teifi().bit_is_set() {
                            // The hardware automatically disables the channel in the error case.
                            debug_assert!(reg.$ccri.read().en().bit_is_clear());
                            panic!("DMA transfer error for channel {:?}", stringify!($Channeli));
                        }
                        reg.$ccri.modify(|_, w| w.en().clear_bit());
                    }

                    // /// Immediately disables the channel.
                    // ///
                    // /// This function is intended only for use in `ScopeGuard` handlers.
                    // ///
                    // /// This is unsafe because it modifies the DMA channel without borrowing the
                    // /// channel token. The caller must ensure that (1) the DMA channel cannot be
                    // /// modified concurrently with the call to this function, (2) the DMA channel
                    // /// is currently enabled, and (3) no `Transfer` instances with this cahnnel
                    // /// are accessible after this function is called.
                    // unsafe fn disable_unchecked() {
                    //     let reg = &(*$DMAx::ptr());
                    //     reg.$ccri.modify(|_, w| w.en().clear_bit());

                    //     // Ensure that the compiler does not try to use old values from the memory
                    //     // buffers.
                    //     atomic::compiler_fence(atomic::Ordering::SeqCst);

                    //     if reg.isr.read().$teifi().bit_is_set() {
                    //         panic!("DMA transfer error for channel {:?}", stringify!($Channeli));
                    //     }
                    // }

                    /// Returns the CNDTR register.
                    ///
                    /// This is safe as long as the register is not written while the channel is
                    /// enabled.
                    unsafe fn cndtr_mut(&mut self) -> &$CNDTRi {
                        // The channel has exclusive access to its register.
                        &(*$DMAx::ptr()).$cndtri
                    }

                    /// Returns the CPAR register.
                    ///
                    /// This is safe as long as the register is not written while the channel is
                    /// enabled.
                    unsafe fn cpar_mut(&mut self) -> &$CPARi {
                        // The channel has exclusive access to its register.
                        &(*$DMAx::ptr()).$cpari
                    }

                    /// Returns the CMAR register.
                    ///
                    /// This is safe as long as the register is not written while the channel is
                    /// enabled.
                    unsafe fn cmar_mut(&mut self) -> &$CMARi {
                        // The channel has exclusive access to its register.
                        &(*$DMAx::ptr()).$cmari
                    }

                    // /// Returns the value of the CNDTR register.
                    // fn cndtr(&self) -> $cndtri::R {
                    //     // The channel has exclusive access to its register.
                    //     unsafe { (*$DMAx::ptr()).$cndtri.read() }
                    // }

                    // /// Returns the value of the CPAR register.
                    // fn cpar(&self) -> $cpari::R {
                    //     // The channel has exclusive access to its register.
                    //     unsafe { (*$DMAx::ptr()).$cpari.read() }
                    // }

                    // /// Returns the value of the CMAR register.
                    // fn cmar(&self) -> $cmari::R {
                    //     // The channel has exclusive access to its register.
                    //     unsafe { (*$DMAx::ptr()).$cmari.read() }
                    // }

                    /// Returns the CCR register.
                    ///
                    /// This register can be written while the channel is disabled or enabled.
                    fn ccr_mut(&mut self) -> &$CCRi {
                        // The channel has exclusive access to its register
                        unsafe { &(*$DMAx::ptr()).$ccri }
                    }

                    /// Returns the value of the CCR register.
                    fn ccr(&self) -> $ccri::R {
                        // The channel has exclusive access to its register.
                        unsafe { (*$DMAx::ptr()).$ccri.read() }
                    }
                }

                impl DmaChannelPriv for $Channeli {
                    unsafe fn enable_periph_to_mem<'body, 'data, P, M, H>(
                        mut self,
                        mut guard: ScopeGuard<'body, 'data, WaitHandler<H>>,
                        periph: *const P,
                        mem: &'data mut [M],
                    ) -> Transfer<'body, 'data, Self, &'data mut [M], WaitHandler<H>>
                    where
                        P: DataElem,
                        M: DataElem + FromBits<P>,
                        H: ScopeEndHandler,
                    {
                        self.clear_all_flags();
                        self.cpar_mut().write(|w| w.pa().bits(periph as u32));
                        self.cmar_mut().write(|w| w.ma().bits(mem.as_ptr() as u32));
                        if mem.len() > ::core::u16::MAX as usize {
                            panic!("DMA request buffer is too long: {}", mem.len())
                        }
                        self.cndtr_mut().write(|w| w.ndt().bits(mem.len() as u16));
                        // Note that this is `write`, not `modify`, so it disables the interrupts.
                        self.ccr_mut().write(|w| {
                            w.dir().clear_bit();
                            w.circ().clear_bit();
                            w.pinc().clear_bit();
                            w.minc().set_bit();
                            w.psize().bits(DataWidth::width_of::<P>().to_bits());
                            w.msize().bits(DataWidth::width_of::<M>().to_bits());
                            w.mem2mem().clear_bit()
                        });

                        // Set up the guard to wait for the transfer to complete and disable the
                        // channel before `mem` goes out of scope.
                        //
                        // This is safe and we don't have to worry about concurrent access to
                        // the register because:
                        //
                        // 1. The channel has exclusive access to its register.
                        //
                        // 2. Since `Transfer` has the `'body` lifetime from the `ScopeGuard`,
                        //    the handler cannot not called while the `Transfer` is alive, so
                        //    the `Transfer` instance cannot access the register concurrently
                        //    with this handler.
                        //
                        // 3. The handler is disabled before returning the channel in
                        //    `.wait()`, so the `$Channeli` instance cannot access the register
                        //    concurrently with this handler.
                        if let Some(handler) = guard.handler_mut() {
                            handler.dma = Some($Channeli::wait_unchecked);
                        }

                        // Ensure that all reads from `mem` have completed before enabling the
                        // transfer.
                        atomic::compiler_fence(atomic::Ordering::SeqCst);

                        self.ccr_mut().modify(|_, w| w.en().set_bit());
                        Transfer {
                            guard,
                            channel: self,
                            out_buf: mem,
                        }
                    }

                    unsafe fn enable_mem_to_periph<'body, 'data, M, P, H>(
                        mut self,
                        mut guard: ScopeGuard<'body, 'data, WaitHandler<H>>,
                        mem: &'data [M],
                        periph: *mut P,
                    ) -> Transfer<'body, 'data, Self, (), WaitHandler<H>>
                    where
                        P: DataElem,
                        M: DataElem + FromBits<P>,
                        H: ScopeEndHandler,
                    {
                        self.clear_all_flags();
                        self.cpar_mut().write(|w| w.pa().bits(periph as u32));
                        self.cmar_mut().write(|w| w.ma().bits(mem.as_ptr() as u32));
                        if mem.len() > ::core::u16::MAX as usize {
                            panic!("DMA request buffer is too long: {}", mem.len())
                        }
                        self.cndtr_mut().write(|w| w.ndt().bits(mem.len() as u16));
                        // Note that this is `write`, not `modify`, so it disables the interrupts.
                        self.ccr_mut().write(|w| {
                            w.dir().set_bit();
                            w.circ().clear_bit();
                            w.pinc().clear_bit();
                            w.minc().set_bit();
                            w.psize().bits(DataWidth::width_of::<P>().to_bits());
                            w.msize().bits(DataWidth::width_of::<M>().to_bits());
                            w.mem2mem().clear_bit()
                        });

                        // Set up the guard to wait for the transfer to complete and disable the
                        // channel before `mem` goes out of scope.
                        //
                        // This is safe and we don't have to worry about concurrent access to
                        // the register because:
                        //
                        // 1. The channel has exclusive access to its register.
                        //
                        // 2. Since `Transfer` has the `'body` lifetime from the `ScopeGuard`,
                        //    the handler cannot not called while the `Transfer` is alive, so
                        //    the `Transfer` instance cannot access the register concurrently
                        //    with this handler.
                        //
                        // 3. The handler is disabled before returning the channel in
                        //    `.wait()`, so the `$Channeli` instance cannot access the register
                        //    concurrently with this handler.
                        if let Some(handler) = guard.handler_mut() {
                            handler.dma = Some($Channeli::wait_unchecked);
                        }

                        // Ensure that all writes to `mem` have completed before enabling the
                        // transfer.
                        atomic::compiler_fence(atomic::Ordering::SeqCst);

                        self.ccr_mut().modify(|_, w| w.en().set_bit());
                        Transfer {
                            guard,
                            channel: self,
                            out_buf: (),
                        }
                    }

                    fn is_enabled(&self) -> bool {
                        self.ccr().en().bit_is_set()
                    }

                    fn clear_enabled(&mut self) {
                        self.ccr_mut().modify(|_, w| w.en().clear_bit());
                    }
                }

                impl DmaChannel for $Channeli {
                    fn start_mem_to_mem<'body, 'data, S, D>(
                        mut self,
                        mut guard: ScopeGuard<'body, 'data, WaitHandler<()>>,
                        src: &'data [S],
                        dst: &'data mut [D],
                    ) -> Transfer<'body, 'data, Self, &'data mut [D], WaitHandler<()>>
                    where
                        S: DataElem,
                        D: DataElem + FromBits<S>,
                    {
                        self.clear_all_flags();
                        // This implementation treats `src` as the peripheral.
                        unsafe {
                            self.cpar_mut().write(|w| w.pa().bits(src.as_ptr() as u32));
                            self.cmar_mut().write(|w| w.ma().bits(dst.as_ptr() as u32));
                        }
                        assert_eq!(src.len(), dst.len());
                        if src.len() > ::core::u16::MAX as usize {
                            panic!("DMA request buffer is too long: {}", src.len())
                        }
                        unsafe {
                            self.cndtr_mut().write(|w| w.ndt().bits(src.len() as u16));
                        }
                        // Note that this is `write`, not `modify`, so it disables the interrupts.
                        self.ccr_mut().write(|w| {
                            w.dir().clear_bit();
                            w.circ().clear_bit();
                            w.pinc().set_bit();
                            w.minc().set_bit();
                            unsafe { w.psize().bits(DataWidth::width_of::<S>().to_bits()); }
                            unsafe { w.msize().bits(DataWidth::width_of::<D>().to_bits()); }
                            w.mem2mem().set_bit()
                        });

                        // Set up the guard to wait for the transfer to complete and disable the
                        // channel before the borrows of `src` and `dest` go out of scope.
                        //
                        // This is safe and we don't have to worry about concurrent access to the
                        // register because:
                        //
                        // 1. The channel has exclusive access to its register.
                        //
                        // 2. Since `Transfer` has the `'body` lifetime from the `ScopeGuard`, the
                        //    handler cannot not called while the `Transfer` is alive, so the
                        //    `Transfer` instance cannot access the register concurrently with this
                        //    handler.
                        //
                        // 3. The handler is disabled before returning the channel in `.wait()`, so
                        //    the `$Channeli` instance cannot access the register concurrently with
                        //    this handler.
                        if let Some(handler) = guard.handler_mut() {
                            handler.dma = Some($Channeli::wait_unchecked);
                        }

                        // Ensure that all writes to `src` and all reads from `dst` have completed
                        // before enabling the transfer.
                        atomic::compiler_fence(atomic::Ordering::SeqCst);

                        self.ccr_mut().modify(|_, w| w.en().set_bit());
                        Transfer {
                            guard,
                            channel: self,
                            out_buf: dst,
                        }
                    }

                    fn priority(&self) -> Priority {
                        Priority::from_bits(self.ccr().pl().bits())
                    }

                    fn set_priority(&mut self, priority: Priority) {
                        self.ccr_mut().modify(|_, w| unsafe { w.pl().bits(priority.to_bits()) })
                    }

                    fn global_interrupt(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$DMAx::ptr()).isr.read().$gifi().bit() }
                    }

                    fn clear_global_interrupt(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$DMAx::ptr()).ifcr.write(|w| w.$cgifi().set_bit()) }
                    }

                    fn transfer_complete(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$DMAx::ptr()).isr.read().$tcifi().bit() }
                    }

                    fn clear_transfer_complete(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$DMAx::ptr()).ifcr.write(|w| w.$ctcifi().set_bit()) }
                    }

                    fn half_transfer_complete(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$DMAx::ptr()).isr.read().$htifi().bit() }
                    }

                    fn clear_half_transfer_complete(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$DMAx::ptr()).ifcr.write(|w| w.$chtifi().set_bit()) }
                    }

                    fn transfer_error(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$DMAx::ptr()).isr.read().$teifi().bit() }
                    }

                    fn clear_transfer_error(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$DMAx::ptr()).ifcr.write(|w| w.$cteifi().set_bit()) }
                    }

                    fn clear_all_flags(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe {
                            (*$DMAx::ptr()).ifcr.write(|w| {
                                w.$cgifi().set_bit();
                                w.$ctcifi().set_bit();
                                w.$chtifi().set_bit();
                                w.$cteifi().set_bit()
                            })
                        }
                    }
                }
            )+
        }
    }
}

dma!(
    DMA1,
    Dma1,
    Dma1Channel,
    dma1,
    dmaen,
    [
        Channel1:
            (
                chan1,
                gif => (gif1, cgif1),
                tcif => (tcif1, ctcif1),
                htif => (htif1, chtif1),
                teif => (teif1, cteif1),
                ccr => (ccr1, CCR1),
                cndtr => (cndtr1, CNDTR1),
                cpar => (cpar1, CPAR1),
                cmar => (cmar1, CMAR1),
            ),
        Channel2:
            (
                chan2,
                gif => (gif2, cgif2),
                tcif => (tcif2, ctcif2),
                htif => (htif2, chtif2),
                teif => (teif2, cteif2),
                ccr => (ccr2, CCR2),
                cndtr => (cndtr2, CNDTR2),
                cpar => (cpar2, CPAR2),
                cmar => (cmar2, CMAR2),
            ),
        Channel3:
            (
                chan3,
                gif => (gif3, cgif3),
                tcif => (tcif3, ctcif3),
                htif => (htif3, chtif3),
                teif => (teif3, cteif3),
                ccr => (ccr3, CCR3),
                cndtr => (cndtr3, CNDTR3),
                cpar => (cpar3, CPAR3),
                cmar => (cmar3, CMAR3),
            ),
        Channel4:
            (
                chan4,
                gif => (gif4, cgif4),
                tcif => (tcif4, ctcif4),
                htif => (htif4, chtif4),
                teif => (teif4, cteif4),
                ccr => (ccr4, CCR4),
                cndtr => (cndtr4, CNDTR4),
                cpar => (cpar4, CPAR4),
                cmar => (cmar4, CMAR4),
            ),
        Channel5:
            (
                chan5,
                gif => (gif5, cgif5),
                tcif => (tcif5, ctcif5),
                htif => (htif5, chtif5),
                teif => (teif5, cteif5),
                ccr => (ccr5, CCR5),
                cndtr => (cndtr5, CNDTR5),
                cpar => (cpar5, CPAR5),
                cmar => (cmar5, CMAR5),
            ),
        Channel6:
            (
                chan6,
                gif => (gif6, cgif6),
                tcif => (tcif6, ctcif6),
                htif => (htif6, chtif6),
                teif => (teif6, cteif6),
                ccr => (ccr6, CCR6),
                cndtr => (cndtr6, CNDTR6),
                cpar => (cpar6, CPAR6),
                cmar => (cmar6, CMAR6),
            ),
        Channel7:
            (
                chan7,
                gif => (gif7, cgif7),
                tcif => (tcif7, ctcif7),
                htif => (htif7, chtif7),
                teif => (teif7, cteif7),
                ccr => (ccr7, CCR7),
                cndtr => (cndtr7, CNDTR7),
                cpar => (cpar7, CPAR7),
                cmar => (cmar7, CMAR7),
            ),
    ]
);
dma!(
    DMA2,
    Dma2,
    Dma2Channel,
    dma2,
    dma2en,
    [
        Channel1:
            (
                chan1,
                gif => (gif1, cgif1),
                tcif => (tcif1, ctcif1),
                htif => (htif1, chtif1),
                teif => (teif1, cteif1),
                ccr => (ccr1, CCR1),
                cndtr => (cndtr1, CNDTR1),
                cpar => (cpar1, CPAR1),
                cmar => (cmar1, CMAR1),
            ),
        Channel2:
            (
                chan2,
                gif => (gif2, cgif2),
                tcif => (tcif2, ctcif2),
                htif => (htif2, chtif2),
                teif => (teif2, cteif2),
                ccr => (ccr2, CCR2),
                cndtr => (cndtr2, CNDTR2),
                cpar => (cpar2, CPAR2),
                cmar => (cmar2, CMAR2),
            ),
        Channel3:
            (
                chan3,
                gif => (gif3, cgif3),
                tcif => (tcif3, ctcif3),
                htif => (htif3, chtif3),
                teif => (teif3, cteif3),
                ccr => (ccr3, CCR3),
                cndtr => (cndtr3, CNDTR3),
                cpar => (cpar3, CPAR3),
                cmar => (cmar3, CMAR3),
            ),
        Channel4:
            (
                chan4,
                gif => (gif4, cgif4),
                tcif => (tcif4, ctcif4),
                htif => (htif4, chtif4),
                teif => (teif4, cteif4),
                ccr => (ccr4, CCR4),
                cndtr => (cndtr4, CNDTR4),
                cpar => (cpar4, CPAR4),
                cmar => (cmar4, CMAR4),
            ),
        Channel5:
            (
                chan5,
                gif => (gif5, cgif5),
                tcif => (tcif5, ctcif5),
                htif => (htif5, chtif5),
                teif => (teif5, cteif5),
                ccr => (ccr5, CCR5),
                cndtr => (cndtr5, CNDTR5),
                cpar => (cpar5, CPAR5),
                cmar => (cmar5, CMAR5),
            ),
    ]
);
