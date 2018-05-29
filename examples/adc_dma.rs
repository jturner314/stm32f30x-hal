//! Serial interface DMA RX transfer test

#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m_rt as rt;
#[macro_use(iprint, iprintln)]
extern crate cortex_m;
extern crate panic_semihosting;
extern crate stm32f30x_hal as hal;
#[macro_use(scope)]
extern crate strong_scope_guard;

use cortex_m::asm;
use hal::adc;
use hal::adc::adc12::channels::{Adc1Channel, Adc2Channel};
use hal::delay::Delay;
use hal::dma::{dma1, DmaExt, Priority};
use hal::prelude::*;
use hal::stm32f30x;

use rt::ExceptionFrame;

entry!(main);

fn main() -> ! {
    let core_periph = cortex_m::Peripherals::take().unwrap();
    let stm32_periph = stm32f30x::Peripherals::take().unwrap();

    let mut itm = core_periph.ITM;
    let mut flash = stm32_periph.FLASH.constrain();
    let mut rcc = stm32_periph.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(core_periph.SYST, clocks);

    let dma1::Parts {
        chan1: mut dma1_chan1,
        ..
    } = stm32_periph.DMA1.split(&mut rcc.ahb);
    let mut gpioa = stm32_periph.GPIOA.split(&mut rcc.ahb);
    let pa0 = gpioa.pa0.into_analog(&mut gpioa.moder);
    let pa6 = gpioa.pa6.into_analog(&mut gpioa.moder);
    let (adc12, adc12_channels) =
        stm32_periph
            .ADC1_2
            .split(stm32_periph.ADC1, stm32_periph.ADC2, &mut rcc.ahb, clocks);

    // Configure ADC1.
    let mut adc12 = adc12.map_adc1(|unpowered| {
        let mut disabled = unpowered.power_on(&mut delay);
        disabled.calibrate_single_ended(&mut delay);
        let mut enabled = disabled.enable();
        enabled.set_alignment(adc::Alignment::Right);
        enabled.set_resolution(adc::Resolution::Bit12);
        unsafe {
            enabled.with_sequence_unchecked(&[Adc1Channel::from((&adc12_channels.adc1_in1, &pa0))])
        }
    });

    // DMA with a single ADC.
    let mut buf: [u16; 8] = [7; 8];
    iprintln!(&mut itm.stim[0], "buf (before) = {:?}", buf);
    let (adc12, mut dma1_chan1) = scope!(|guard| {
        dma1_chan1.set_priority(Priority::VeryHigh);
        let running = adc12.adc1.start_dma(&mut buf, guard, dma1_chan1);
        // Can do other things here while the ADC and DMA are running.
        let (stopped, dma1_chan1, _, _) = running.wait();
        adc12.adc1 = stopped;
        (adc12, dma1_chan1)
    });
    iprintln!(&mut itm.stim[0], "buf (after) = {:?}", buf);

    // Configure the ADCs in dual mode.
    let mut adc12 = adc12.map_adc2(|adc2| adc2.power_on(&mut delay));
    adc12.adc2.calibrate_single_ended(&mut delay);
    let adc12 = adc12.map_adc1(|adc1| adc1.drop_sequence().disable());
    let mut adc12 = adc12.into_dual().enable_both();
    adc12.adc2.set_alignment(adc::Alignment::Right);
    adc12.adc2.set_resolution(adc::Resolution::Bit12);
    let adc12 = adc12.with_sequences(
        &[Adc1Channel::from((&adc12_channels.adc1_in1, &pa0))],
        &[Adc2Channel::from((&adc12_channels.adc2_in3, &pa6))],
    ).unwrap_or_else(|_| panic!());

    // DMA with dual ADCs.
    let mut buf: [[u16; 2]; 8] = [[1, 2]; 8];
    iprintln!(&mut itm.stim[0], "buf (before) = {:?}", buf);
    let _adc12 = scope!(|guard| {
        dma1_chan1.set_priority(Priority::VeryHigh);
        let running = adc12.start_dma(&mut buf, guard, dma1_chan1);
        // Can do other things here while the ADCs and DMA are running.
        let (adc12, _, _, _) = running.wait();
        adc12
    });
    iprintln!(&mut itm.stim[0], "buf (after) = {:?}", buf);

    asm::bkpt();

    loop {}
}

exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
