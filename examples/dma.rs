//! Serial interface DMA RX transfer test

#![deny(unsafe_code)]
#![deny(warnings)]
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
    let _clocks = rcc.cfgr.freeze(&mut flash.acr);

    let dma1::Parts { mut chan1, .. } = stm32_periph.DMA1.split(&mut rcc.ahb);

    let src: [i32; 4] = [1, 2, 3, 4];
    let mut dst: [i32; 4] = [0, 0, 0, 0];
    scope!(|guard| {
        iprintln!(&mut itm.stim[0], "src (before) = {:?}", src);
        iprintln!(&mut itm.stim[0], "dst (before) = {:?}", dst);

        chan1.set_priority(Priority::default());
        let transfer = chan1.start_mem_to_mem(guard, &src, &mut dst);
        transfer.wait();
    });
    iprintln!(&mut itm.stim[0], "src (after) = {:?}", src);
    iprintln!(&mut itm.stim[0], "dst (after) = {:?}", dst);

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
