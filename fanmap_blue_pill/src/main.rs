#![no_std]
#![no_main]
#![feature(lang_items)]

mod cdc_acm;

extern crate cortex_m;
#[macro_use] extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32f103xx_hal as hal;
extern crate usb_device;
extern crate stm32f103xx_usb;
extern crate arrayvec;

use hal::prelude::*;
use hal::stm32f103xx;
use rt::ExceptionFrame;
use arrayvec::ArrayVec;

use usb_device::prelude::*;
use stm32f103xx_usb::UsbBus;


const MSG_LEN: usize = 4;
static DEFAULT_CPU_DUTY: i32 = 50;
static DEFAULT_SYS_DUTY: i32 = 30;
static SIGNATURE_RX: [u8; 2] = [0xAB, 0xCD];
static SIGNATURE_TX: [u8; 2] = [0x78, 0x34];

enum RxOffset {
    Signature = 0,
    CpuDuty = 2,
    SysDuty = 3,
}

#[no_mangle]
struct MessageProcessor {
    rx_buf: ArrayVec<[u8; MSG_LEN]>,
    tx_buf: ArrayVec<[u8; MSG_LEN]>,
    cpu_duty: (u8, bool),
    sys_duty: (u8, bool),
    speed_current: [u8; 5],
}

#[no_mangle]
impl MessageProcessor {
    #[no_mangle]
    fn new() -> MessageProcessor {
        MessageProcessor {
            rx_buf: ArrayVec::new(),
            tx_buf: ArrayVec::new(),
            cpu_duty: (DEFAULT_CPU_DUTY as u8, false),
            sys_duty: (DEFAULT_SYS_DUTY as u8, false),
            speed_current: [0; 5],
        }
    }

    pub fn rx_msg(&mut self, buf: &[u8]) {
        // if SIGNATURE_RX[0] != buf[1] || SIGNATURE_RX[1] != buf[0] {
        //     return;
        // }

        for elem in buf {
            if self.rx_buf.is_full() {
                break;
            }

            self.rx_buf.push(*elem);
        }

        if self.rx_buf.len() == MSG_LEN {
            let new_cpu_duty = self.rx_buf[RxOffset::CpuDuty as usize];
            if new_cpu_duty != self.cpu_duty.0 {
                self.cpu_duty = (new_cpu_duty, true);
            }

            let new_sys_duty = self.rx_buf[RxOffset::SysDuty as usize];
            if new_sys_duty != self.sys_duty.0 {
                self.sys_duty = (new_sys_duty, true);
            }

            self.tx_buf.clone_from(&self.rx_buf);
            self.rx_buf.clear();
        }
    }

    #[no_mangle]
    pub fn tx_msg(&self) -> &[u8] {
        self.tx_buf.as_slice()
    }

    #[no_mangle]
    pub fn consume_tx(&mut self, count: usize) {
        self.tx_buf.clear();
    }

    pub fn cpu_duty_set(&mut self) {
        self.cpu_duty.1 = false;
    }

    pub fn sys_duty_set(&mut self) {
        self.sys_duty.1 = false;
    }
}

#[entry]
fn main() -> ! {
    let dp = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr
        .hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    let usb_bus = UsbBus::usb(dp.USB, &mut rcc.apb1);
    usb_bus.borrow_mut().enable_reset(&clocks, &mut gpioa.crh, gpioa.pa12);

    let serial = cdc_acm::SerialPort::new(&usb_bus);

    let usb_dev = UsbDevice::new(&usb_bus, UsbVidPid(0x5824, 0x27dd))
        .manufacturer("Subrat Meher")
        .product("Fan speed controller")
        .serial_number("12341234")
        .device_class(cdc_acm::USB_CLASS_CDC)
        .build(&[&serial]);

    usb_dev.force_reset().expect("reset failed");

    let mut msg_proc = MessageProcessor::new();

    // TIM2
    let c1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let c2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
    let c3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let c4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

    let mut pwm2  = dp.TIM2
        .pwm(
            (c1, c2, c3, c4),
            &mut afio.mapr,
            25.khz(),
            clocks,
            &mut rcc.apb1,
        );

    let max = pwm2.3.get_max_duty() as i32;

    pwm2.2.enable();
    pwm2.2.set_duty((max*DEFAULT_CPU_DUTY/100) as u16);   // cpu

    pwm2.3.enable();
    pwm2.3.set_duty((max*DEFAULT_SYS_DUTY/100) as u16);   // other fans

    loop {
        usb_dev.poll();

        if usb_dev.state() == UsbDeviceState::Configured {
            let mut buf = [0u8; 128];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    msg_proc.rx_msg(&buf[0..count]);
                },
                _ => { },
            }

            let tx_result =
            {
                let tx_data: &[u8] = msg_proc.tx_msg();
                if tx_data.len() > 0 {
                    serial.write(tx_data)
                } else {
                    Err(usb_device::UsbError::NoData)
                }
            };

            match tx_result {
                Ok(n) if n > 0 => { msg_proc.consume_tx(n); },
                Ok(_) => { /* no data written, but no error */ },
                Err(_) => { },
            }
        }

        if msg_proc.cpu_duty.1 == true {
            pwm2.2.set_duty((max*(msg_proc.cpu_duty.0 as i32)/100) as u16);   // cpu
            msg_proc.cpu_duty.1 = false;
        }

        if msg_proc.sys_duty.1 == true {
            pwm2.3.set_duty((max*(msg_proc.sys_duty.0 as i32)/100) as u16);   // other fans
            msg_proc.sys_duty.1 = false;
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
