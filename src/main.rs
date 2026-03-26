mod controller;
mod packet;
mod report;
mod slimeinterface;
mod transport;
mod virtualdevice;
mod visualizer;

use std::thread;

use crate::{controller::Controller, virtualdevice::create_virtual_controller};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();

    let mut controller = match args.get(1).map(String::as_str) {
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

    println!(
        "[sw2ctl] Connected to {:?}: {:?}",
        controller.controller_type, controller.device_info
    );

    let mut vdevice = create_virtual_controller()?;
    let handle = thread::spawn(move || {
        loop {
            match controller.get_input() {
                Ok(n) => {
                    if let Ok(q) = controller.update_orientation(&n.motion, &n.magnetometer) {}
                    if let Err(e) = n.emit_to_device(&mut vdevice) {
                        eprintln!("[sw2ctl] emit error: {e}");
                    }
                }
                Err(e) => {
                    eprintln!("[sw2ctl] HID read error, {e}")
                }
            }
        }
    });

    handle.join().unwrap();
    Ok(())
}
