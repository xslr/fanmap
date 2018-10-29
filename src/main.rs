//! A showcase of the `app!` macro syntax
#![no_std]
#![no_main]
#![deny(unsafe_code)]

extern crate cortex_m;
#[macro_use(entry)]
extern crate cortex_m_rt as rt;
extern crate cortex_m_rtfm as rtfm;
extern crate stm32f103xx_hal as hal;
extern crate panic_abort;

use cortex_m::asm;
use hal::stm32f103xx;
use rtfm::{app, Threshold};

app! {
    device: stm32f103xx,

    resources: {
        static CO_OWNED: u32 = 0;
        static ON: bool = false;
        static OWNED: bool = false;
        static SHARED: bool = false;
    },

    init: {
        // This is the path to the `init` function
        //
        // `init` doesn't necessarily has to be in the root of the crate
        path: main::init,
    },

    idle: {
        // This is a path to the `idle` function
        //
        // `idle` doesn't necessarily has to be in the root of the crate
        path: main::idle,
        resources: [OWNED, SHARED],
    },

    tasks: {
        SYS_TICK: {
            path: sys_tick,
            // If omitted priority is assumed to be 1
            // priority: 1,
            resources: [CO_OWNED, ON, SHARED],
        },

        TIM2: {
            // Tasks are enabled, between `init` and `idle`, by default but they
            // can start disabled if `false` is specified here
            enabled: false,
            path: tim2,
            priority: 1,
            resources: [CO_OWNED],
        },
    },
}

mod main {
    use rtfm::{self, Resource, Threshold};

    pub fn init(_p: ::init::Peripherals, _r: ::init::Resources) {
        // configure_pins();
        // timer init
    }

    pub fn idle(t: &mut Threshold, mut r: ::idle::Resources) -> ! {
        loop {
            *r.OWNED = !*r.OWNED;

            if *r.OWNED {
                if r.SHARED.claim(t, |shared, _| *shared) {
                    rtfm::wfi();
                }
            } else {
                r.SHARED.claim_mut(t, |shared, _| *shared = !*shared);
            }
        }
    }

    pub fn configure_pins() {
        use cortex_m::asm;
        use hal::prelude::*;

        let p = ::stm32f103xx::Peripherals::take().unwrap();

        let mut flash = p.FLASH.constrain();
        let mut rcc = p.RCC.constrain();

        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut afio = p.AFIO.constrain(&mut rcc.apb2);
        // let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

        let c1 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let c2 = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
        let c3 = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
        let c4 = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

        let mut pwm = p.TIM4.pwm(
            (c1, c2, c3, c4),
            &mut afio.mapr,
            1.khz(),
            clocks,
            &mut rcc.apb1,
        )
        .3;

        let max = pwm.get_max_duty();

        pwm.enable();

        // full
        // pwm.set_duty(max);
        // asm::bkpt();

        // dim
        pwm.set_duty(max / 4);
        asm::bkpt();

        // zero
        // pwm.set_duty(0);
        // asm::bkpt();
    }
}

fn sys_tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    *r.ON = !*r.ON;

    *r.CO_OWNED += 1;
}

fn tim2(_t: &mut Threshold, mut r: TIM2::Resources) {
    *r.CO_OWNED += 1;
}

#[entry]
fn main_wrapper() -> ! {
    main();
    loop {
        asm::bkpt();
    }
}
