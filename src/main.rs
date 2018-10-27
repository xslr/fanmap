#![no_std]
#![no_main]
#![warn(unused)]

#![feature(asm)]

extern crate cortex_m_rt as rt;
extern crate panic_semihosting;

use core::ptr;
use rt::entry;

// LED0 = PC13;

pub const RCC_APB2ENR: *mut u32 = 0x4002_1018 as *mut u32;
pub const RCC_APB2ENR_IOPCEN: u32 = 1 << 4;
pub const GPIOC_CRH: *mut u32 = 0x4001_1004 as *mut u32;
pub const GPIOC_BSRR: *mut u32 = 0x4001_1010 as *mut u32;

#[entry]
fn main() -> ! {
    unsafe {
        // Enable GPIOC
        ptr::write_volatile(RCC_APB2ENR, ptr::read_volatile(RCC_APB2ENR) | RCC_APB2ENR_IOPCEN);
        // Set PC13 Mode = Output
        ptr::write_volatile(GPIOC_CRH, 0x44544444);
        loop {
            // Set PC13
            ptr::write_volatile(GPIOC_BSRR, 1 << (13 + 16));
            // Delay approx 1/2 second
            for _ in 0..125_000 { asm!("nop") }
            // Reset Set PC13
            ptr::write_volatile(GPIOC_BSRR, 1 << 13);
            // Delay approx 1/2 second
            for _ in 0..125_000 { asm!("nop") }
        }
    }
}
