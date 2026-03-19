mod controller;
mod packet;
mod report_05;
mod transport;

use crate::controller::Controller;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();

    let controller = match args.get(1).map(String::as_str) {
        Some("usb") => {
            println!("[sw2ctl] Opening USB transport");
            Controller::open_usb()?
        }
        Some("ble") if args.len() >= 4 => {
            let adapter_mac = &args[2];
            let controller_mac = &args[3];
            println!(
                "[sw2ctl] Opening BLE transport: adapter={} controller={}",
                adapter_mac, controller_mac
            );
            Controller::open_ble(adapter_mac, controller_mac)?
        }
        _ => {
            eprintln!(
                "Usage: {} usb | ble <ADAPTER_MAC> <CONTROLLER_MAC>",
                args[0]
            );
            std::process::exit(1);
        }
    };

    println!("[sw2ctl] Connected: {:?}", controller.device_info);

    loop {
        match controller.get_input() {
            Ok(n) => {
                //println!("[sw2ctl] HID data: {:02X?} ", &buf[1..n]);

                //
            }
            Err(e) => {
                eprintln!("[sw2ctl] HID read error, {e}")
            }
        }
    }

    Ok(())
}
