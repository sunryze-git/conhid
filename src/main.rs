mod controller;
mod packet;
mod report;
mod transport;
mod virtualdevice;

use std::thread;

use crate::{controller::Controller, virtualdevice::create_virtual_controller};

enum ControllerSpec {
    Usb,
    Ble {
        adapter_mac: String,
        controller_mac: String,
    },
}

fn parse_specs(args: &[String]) -> Result<Vec<ControllerSpec>, String> {
    let mut specs = Vec::new();
    let mut i = 0;

    while i < args.len() {
        match args[i].as_str() {
            "usb" => {
                specs.push(ControllerSpec::Usb);
                i += 1;
            }
            "ble" => {
                let adapter_mac = args
                    .get(i + 1)
                    .ok_or_else(|| "ble requires <ADAPTER_MAC> <CONTROLLER_MAC>".to_string())?
                    .clone();
                let controller_mac = args
                    .get(i + 2)
                    .ok_or_else(|| "ble requires <ADAPTER_MAC> <CONTROLLER_MAC>".to_string())?
                    .clone();
                specs.push(ControllerSpec::Ble {
                    adapter_mac,
                    controller_mac,
                });
                i += 3;
            }
            other => return Err(format!("Unknown token '{other}'")),
        }
    }

    if specs.is_empty() {
        return Err("No controllers specified".to_string());
    }

    Ok(specs)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let all_args: Vec<String> = std::env::args().collect();
    let prog = &all_args[0];

    let specs = parse_specs(&all_args[1..]).unwrap_or_else(|e| {
        eprintln!("[sw2ctl] {e}");
        eprintln!("Usage: {prog} (usb | ble <ADAPTER_MAC> <CONTROLLER_MAC>)+");
        eprintln!("Examples:");
        eprintln!("  {prog} usb");
        eprintln!("  {prog} ble AA:BB:CC:DD:EE:FF 11:22:33:44:55:66");
        eprintln!("  {prog} usb ble AA:BB:CC:DD:EE:FF 11:22:33:44:55:66");
        std::process::exit(1);
    });

    println!("[sw2ctl] Opening {} controller(s)", specs.len());

    let mut handles = Vec::with_capacity(specs.len());

    for (idx, spec) in specs.into_iter().enumerate() {
        let label = format!("controller-{idx}");

        let mut controller = match spec {
            ControllerSpec::Usb => {
                println!("[{label}] Opening USB transport");
                Controller::open_usb()?
            }
            ControllerSpec::Ble {
                ref adapter_mac,
                ref controller_mac,
            } => {
                println!(
                    "[{label}] Opening BLE transport: adapter={adapter_mac} controller={controller_mac}"
                );
                Controller::open_ble(adapter_mac, controller_mac)?
            }
        };

        println!(
            "[{label}] Connected to {:?}: {:?}",
            controller.controller_type, controller.device_info
        );

        let mut vdevice = create_virtual_controller()?;

        let handle = thread::Builder::new().name(label.clone()).spawn(move || {
            loop {
                controller.wait_for_input();
                while let Some(n) = controller.get_input() {
                    let _ = controller.update_orientation(&n.motion, &n.magnetometer);
                    if let Err(e) = n.emit_to_device(&mut vdevice) {
                        eprintln!("[{label}] emit error: {e}");
                    }
                }
            }
        })?;

        handles.push(handle);
    }

    for handle in handles {
        match handle.join() {
            Err(e) => {
                eprintln!("[sw2ctl] A controller thread panicked: {e:?}");
            }
            _ => (),
        }
    }

    Ok(())
}
