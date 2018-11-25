#![no_std]
#![no_main]
#![feature(alloc)]
#![feature(alloc_error_handler)]
#![feature(lang_items)]

mod cdc_acm;

extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m;
#[macro_use] extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32f103xx_hal as hal;
extern crate usb_device;
extern crate stm32f103xx_usb;

use alloc::vec::Vec;
use alloc::collections::VecDeque;
use alloc_cortex_m::CortexMHeap;
use hal::prelude::*;
use hal::stm32f103xx;
use rt::ExceptionFrame;

use usb_device::prelude::*;
use stm32f103xx_usb::UsbBus;


#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();


#[no_mangle]
struct MessageProcessor {
    rx_buf: VecDeque<u8>,
    tx_buf: Vec<u8>
}

#[no_mangle]
impl MessageProcessor {
    #[no_mangle]
    fn new() -> MessageProcessor {
        MessageProcessor {
            rx_buf: VecDeque::with_capacity(128),
            tx_buf: Vec::with_capacity(128),
        }
    }

    #[no_mangle]
    pub fn rx_msg(&mut self, buf: &[u8]) {
        self.rx_buf.extend(buf.iter());

        // process message
        {
            let (rx_slice0,rx_slice1) = self.rx_buf.as_slices();
            self.tx_buf.extend_from_slice(rx_slice0);
            self.tx_buf.extend_from_slice(rx_slice1);
        }

        self.rx_buf.clear();
    }

    #[no_mangle]
    pub fn tx_msg(&self) -> &[u8] {
        self.tx_buf.as_slice()
    }

    #[no_mangle]
    pub fn consume_tx(&mut self, count: usize) {
        let new_len = self.tx_buf.len() - count;
        self.tx_buf.resize(new_len, 0);
    }
}

#[entry]
fn main() -> ! {
    // Initialize the allocator BEFORE using it
    let start = rt::heap_start() as usize;
    let size = 1024; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    let dp = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr
        .hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

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

    loop {
        usb_dev.poll();

        if usb_dev.state() == UsbDeviceState::Configured {
            let mut buf = [0u8; 128];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    msg_proc.rx_msg(&buf[0..count]);
                    let mut tx_result = {
                        let tx_data: &[u8] = msg_proc.tx_msg();
                        if tx_data.len() > 0 {
                            serial.write(tx_data)
                        } else {
                            Err(usb_device::UsbError::NoData)
                        }
                    };

                    match tx_result {
                        Ok(count) if count > 0 => {
                            msg_proc.consume_tx(count);
                        },
                        Ok(_) => {
                            // nothing was written, but no error
                        },
                        Err(_) => { panic!("Write failed"); }
                    }
                },
                _ => { },
            }
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

#[alloc_error_handler]
fn rust_oom(_: core::alloc::Layout) -> ! {
    panic!("Out of memory.");
}

#[no_mangle]
pub unsafe extern "C" fn rust_begin_unwind(
    _args: ::core::fmt::Arguments,
    _file: &'static str,
    _line: u32,
) -> ! {
    panic!("rust_begin_unwind");
}
