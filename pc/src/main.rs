extern crate serial;
extern crate sysinfo;

use std::cmp;
use std::io::{Read, Write};
use sysinfo::{System, SystemExt, ComponentExt};
use std::{thread, time};
use serial::SerialPort;

fn main() {
    let mut sys = System::new();
    let mut port = serial::open("/dev/ttyACM0").unwrap();
    let _port_cfg_result = port.reconfigure(&|settings| {
        settings.set_baud_rate(serial::Baud115200)?;
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    });

    let _port_set_timeout_result = port.set_timeout(time::Duration::from_millis(500));

    let hyst = 5;
    let target_temp = 70;
    let mut fan_duty: u8 = 10;
    let mut wbuf: Vec<u8> = vec![0xAB, 0xCD, 20, 15];
    let mut rbuf: Vec<u8> = vec![];
    loop {
        let temp = get_max_temp(&mut sys);
        let temp_distance: i32 = target_temp - temp;
        let next_update_millis = if temp_distance < hyst {
            fan_duty = if fan_duty < 98 { fan_duty + 2 } else { 100 };
            2000
        } else if temp_distance > (2 * hyst) {
            fan_duty = if fan_duty > 2 { fan_duty - 2 } else { 0 };
            2000
        } else {
            500
        };

        wbuf[2] = fan_duty;  // cpu
        // wbuf[3] = fan_duty;  // sys
        let _write_result = port.write(&wbuf[..]);
        let _read_result = port.read(&mut rbuf[..]);

        println!("t={} d={}, rbuf={:?}", temp, fan_duty, rbuf);

        thread::sleep(time::Duration::from_millis(next_update_millis));
    }
}

fn get_max_temp(sys: &mut System) -> i32 {
    let mut max_temp = 0;

    sys.refresh_all();
    // Components temperature:
    for component in sys.get_components_list() {
        let name = component.get_label();
        let temp = component.get_temperature() as i32;
        if name.contains("Core") {
            max_temp = cmp::max(temp, max_temp);

        }
        // print!("{:?} ", temp);
    }
    // println!("");

    return max_temp;
}
