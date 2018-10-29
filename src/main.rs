//! Testing PWM output
//! Ramp the duty cycle up and down

#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
extern crate cortex_m;
extern crate panic_semihosting;
extern crate stm32f103xx_hal as hal;

use hal::prelude::*;
use hal::stm32f103xx;
use rt::{entry, exception, ExceptionFrame};
use cortex_m::asm;

#[entry]
fn main() -> ! {
    let p = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    // let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // TIM2
    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

    let mut pwm2_3 = p.TIM2
        .pwm(
            (c1, c2, c3, c4),
            &mut afio.mapr,
            25.khz(),
            clocks,
            &mut rcc.apb1,
        )
        .2; // 2 -> channel 3

    let wait = 1_000_000;
    let max = pwm2_3.get_max_duty() as i32;
    let min = max/10;
    let mut duty : i32 = max;
    let mut is_counting_up = true;

    pwm2_3.enable();

    loop {
        pwm2_3.set_duty(duty as u16);

        if (is_counting_up && duty >= max) || (!is_counting_up && duty <= min) {
            is_counting_up = !is_counting_up;
        }

        if is_counting_up {
            duty = duty + 10;
        } else {
            duty = duty - 10;
        }
    
        for _ in 0..wait {
            asm::nop();
        }
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
