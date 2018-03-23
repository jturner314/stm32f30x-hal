//! Direct memory access controllers (DMA)

// This should be true for all STM32 devices.
#![cfg(target_pointer_width = "32")]

// provide general methods that take ownership and use type states
// can also provide blocking methods that just take &mut

// THIS API IS UNSOUND

use core::marker::PhantomData;
use rcc::AHB;
use vcell::VolatileCell;

/// The DMA channel is enabled (type state).
///
/// The lifetime `'a` is the lifetime of the borrows of the source and destination.
pub struct Enabled<'a> {
    life: PhantomData<&'a ()>,
}

/// The DMA channel is disabled (type state).
pub struct Disabled;

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
            8 => DataWidth::Bits8,
            16 => DataWidth::Bits16,
            32 => DataWidth::Bits32,
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
                    pub $chani: $Channeli<Disabled>,
                )+
            }

            impl DmaExt for $DMAx {
                type Parts = Parts;

                fn split(self, ahb: &mut AHB) -> Parts {
                    ahb.enr().modify(|_, w| w.$dmaxenr().enabled());

                    Parts {
                        $dmax: $Dmax { reg: self },
                        $(
                            $chani: $Channeli  {
                                _state: PhantomData,
                            },
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
                pub struct $Channeli<State> {
                    _state: PhantomData<State>,
                }

                impl $Channeli<Disabled> {
                    /// Returns the CNDTR register.
                    ///
                    /// (This register must not be written while the channel is enabled.)
                    fn cndtr_mut(&mut self) -> &$CNDTRi {
                        // The channel has exclusive access to its register.
                        unsafe { &(*$DMAx::ptr()).$cndtri }
                    }

                    /// Returns the CPAR register.
                    ///
                    /// (This register must not be written while the channel is enabled.)
                    fn cpar_mut(&mut self) -> &$CPARi {
                        // The channel has exclusive access to its register.
                        unsafe { &(*$DMAx::ptr()).$cpari }
                    }

                    /// Returns the CMAR register.
                    ///
                    /// (This register must not be written while the channel is enabled.)
                    fn cmar_mut(&mut self) -> &$CMARi {
                        // The channel has exclusive access to its register.
                        unsafe { &(*$DMAx::ptr()).$cmari }
                    }

                    // /// Enables a DMA transfer from a peripheral register to a memory buffer. (The
                    // /// number of data to be transferred is `mem.len()`.)
                    // ///
                    // /// **Panics** if `mem.len() > ::core::u16::MAX as usize`.
                    // ///
                    // /// **Warning**: If `periph` is not actually a peripheral register, then this
                    // /// transfer will never finish because the DMA will never receive any transfer
                    // /// requests.
                    // pub(crate) fn enable_periph_to_mem<'a, P, M>(
                    //     mut self,
                    //     periph: &'a VolatileCell<P>,
                    //     mem: &'a mut [VolatileCell<M>],
                    //     priority: Priority,
                    // ) -> $Channeli<Enabled<'a>>
                    // where
                    //     P: DataElem,
                    //     M: DataElem + FromBits<P>,
                    // {
                    //     self.cpar_mut().write(|w| unsafe {
                    //         w.pa().bits(periph as *const VolatileCell<P> as *const P as u32)
                    //     });
                    //     self.cmar_mut().write(|w| unsafe {
                    //         w.ma().bits(mem.as_ptr() as u32)
                    //     });
                    //     if mem.len() > ::core::u16::MAX as usize {
                    //         panic!("DMA request buffer is too long: {}", mem.len())
                    //     }
                    //     self.cndtr_mut().write(|w| unsafe {
                    //         w.ndt().bits(mem.len() as u16)
                    //     });
                    //     // Note that this is `write`, not `modify`, so it disables the interrupts.
                    //     self.ccr_mut().write(|w| {
                    //         w.dir().clear_bit();
                    //         w.circ().clear_bit();
                    //         w.pinc().clear_bit();
                    //         w.minc().set_bit();
                    //         unsafe { w.psize().bits(DataWidth::width_of::<P>().to_bits()); }
                    //         unsafe { w.msize().bits(DataWidth::width_of::<M>().to_bits()); }
                    //         unsafe { w.pl().bits(priority.to_bits()); }
                    //         w.mem2mem().clear_bit()
                    //     });
                    //     self.ccr_mut().modify(|_, w| w.en().set_bit());
                    //     $Channeli {
                    //         _state: PhantomData,
                    //     }
                    // }

                    // /// Enables a DMA transfer from a memory buffer to a peripheral register. (The
                    // /// number of data to be transferred is `mem.len()`.)
                    // ///
                    // /// **Panics** if `mem.len() > ::core::u16::MAX as usize`.
                    // ///
                    // /// **Warning**: If `periph` is not actually a peripheral register, then this
                    // /// transfer will never finish because the DMA will never receive any transfer
                    // /// requests.
                    // pub(crate) fn enable_mem_to_periph<'a, M, P>(
                    //     mut self,
                    //     mem: &'a [M],
                    //     periph: &'a mut VolatileCell<P>,
                    //     priority: Priority,
                    // ) -> $Channeli<Enabled<'a>>
                    // where
                    //     P: DataElem,
                    //     M: DataElem + FromBits<P>,
                    // {
                    //     self.cpar_mut().write(|w| unsafe {
                    //         w.pa().bits(periph as *const VolatileCell<P> as *const P as u32)
                    //     });
                    //     self.cmar_mut().write(|w| unsafe {
                    //         w.ma().bits(mem.as_ptr() as u32)
                    //     });
                    //     if mem.len() > ::core::u16::MAX as usize {
                    //         panic!("DMA request buffer is too long: {}", mem.len())
                    //     }
                    //     self.cndtr_mut().write(|w| unsafe {
                    //         w.ndt().bits(mem.len() as u16)
                    //     });
                    //     // Note that this is `write`, not `modify`, so it disables the interrupts.
                    //     self.ccr_mut().write(|w| {
                    //         w.dir().set_bit();
                    //         w.circ().clear_bit();
                    //         w.pinc().clear_bit();
                    //         w.minc().set_bit();
                    //         unsafe { w.psize().bits(DataWidth::width_of::<P>().to_bits()); }
                    //         unsafe { w.msize().bits(DataWidth::width_of::<M>().to_bits()); }
                    //         unsafe { w.pl().bits(priority.to_bits()); }
                    //         w.mem2mem().clear_bit()
                    //     });
                    //     self.ccr_mut().modify(|_, w| w.en().set_bit());
                    //     $Channeli {
                    //         _state: PhantomData,
                    //     }
                    // }

                    /// Starts a DMA transfer from `src` to `dst`. (The number of data to be
                    /// transferred is `src.len()`, which must be equal to `dst.len()`.)
                    ///
                    /// **Panics** if `src.len() > ::core::u16::MAX as usize` or if `src.len() !=
                    /// dst.len()`.
                    pub fn start_mem_to_mem<'a, S, D>(
                        mut self,
                        src: &'a [S],
                        dst: &'a mut [VolatileCell<D>],
                        priority: Priority,
                    ) -> $Channeli<Enabled<'a>>
                    where
                        S: DataElem,
                        D: DataElem + FromBits<S>,
                    {
                        // This implementation treats `src` as the peripheral.
                        self.cpar_mut().write(|w| unsafe {
                            w.pa().bits(src.as_ptr() as u32)
                        });
                        self.cmar_mut().write(|w| unsafe {
                            w.ma().bits(dst.as_ptr() as u32)
                        });
                        assert_eq!(src.len(), dst.len());
                        if src.len() > ::core::u16::MAX as usize {
                            panic!("DMA request buffer is too long: {}", src.len())
                        }
                        self.cndtr_mut().write(|w| unsafe {
                            w.ndt().bits(src.len() as u16)
                        });
                        // Note that this is `write`, not `modify`, so it disables the interrupts.
                        self.ccr_mut().write(|w| {
                            w.dir().clear_bit();
                            w.circ().clear_bit();
                            w.pinc().set_bit();
                            w.minc().set_bit();
                            unsafe { w.psize().bits(DataWidth::width_of::<S>().to_bits()); }
                            unsafe { w.msize().bits(DataWidth::width_of::<D>().to_bits()); }
                            unsafe { w.pl().bits(priority.to_bits()); }
                            w.mem2mem().set_bit()
                        });
                        self.ccr_mut().modify(|_, w| w.en().set_bit());
                        $Channeli {
                            _state: PhantomData,
                        }
                    }
                }

                impl<'a> $Channeli<Enabled<'a>> {
                    /// Waits for the DMA transfer to finish (blocking).
                    ///
                    /// Returns `Err` if there was a transfer error. (This occurs when a DMA
                    /// transfer is attempted to/from a reserved address space.)
                    pub fn finish_blocking(mut self) -> Result<$Channeli<Disabled>, $Channeli<Disabled>> {
                        while !self.transfer_error() && !self.transfer_complete() {}
                        if self.transfer_error() {
                            // The hardware automatically disables the channel in the error case.
                            debug_assert!(self.ccr().en().bit_is_clear());
                            self.clear_all_flags();
                            Err($Channeli {
                                _state: PhantomData,
                            })
                        } else {
                            self.ccr_mut().modify(|_, w| w.en().clear_bit());
                            self.clear_all_flags();
                            Ok($Channeli {
                                _state: PhantomData,
                            })
                        }
                    }
                }

                impl<State> $Channeli<State> {
                    /// Returns the CCR register.
                    ///
                    /// (This register can be written while the channel is disabled or enabled.)
                    fn ccr_mut(&mut self) -> &$CCRi {
                        // The channel has exclusive access to its register
                        unsafe { &(*$DMAx::ptr()).$ccri }
                    }

                    /// Returns the value of the CCR register.
                    fn ccr(&self) -> $ccri::R {
                        // The channel has exclusive access to its register.
                        unsafe { (*$DMAx::ptr()).$ccri.read() }
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

                    /// Returns the global interrupt flag for this channel.
                    pub fn global_interrupt(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$DMAx::ptr()).isr.read().$gifi().bit() }
                    }

                    /// Clears the global interrupt flag for this channel.
                    pub fn clear_global_interrupt(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$DMAx::ptr()).ifcr.write(|w| w.$cgifi().set_bit()) }
                    }

                    /// Returns the transfer complete flag for this channel.
                    pub fn transfer_complete(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$DMAx::ptr()).isr.read().$tcifi().bit() }
                    }

                    /// Clears the transfer complete flag for this channel.
                    pub fn clear_transfer_complete(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$DMAx::ptr()).ifcr.write(|w| w.$ctcifi().set_bit()) }
                    }

                    /// Returns the half transfer complete flag for this channel.
                    pub fn half_transfer_complete(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$DMAx::ptr()).isr.read().$htifi().bit() }
                    }

                    /// Clears the half transfer complete flag for this channel.
                    pub fn clear_half_transfer_complete(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$DMAx::ptr()).ifcr.write(|w| w.$chtifi().set_bit()) }
                    }

                    /// Returns the transfer error flag for this channel.
                    pub fn transfer_error(&self) -> bool {
                        // NOTE(unsafe) atomic read with no side effects
                        unsafe { (*$DMAx::ptr()).isr.read().$teifi().bit() }
                    }

                    /// Clears the transfer error flag for this channel.
                    pub fn clear_transfer_error(&mut self) {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$DMAx::ptr()).ifcr.write(|w| w.$cteifi().set_bit()) }
                    }

                    /// Clears all of the flags at the same time.
                    pub fn clear_all_flags(&mut self) {
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
    ]
);
// dma!(
//     DMA2,
//     Dma1,
//     Dma2Channel,
//     dma2,
//     dma2en,
//     [
//         Channel1:
//             (
//                 chan1,
//                 (gif1, cgif1),
//                 (tcif1, ctcif1),
//                 (htif1, chtif1),
//                 (teif1, cteif1),
//                 ccr => (ccr1, CCR1),
//                 cndtr => (cndtr1, CNDTR1),
//             ),
//     ]
// );
