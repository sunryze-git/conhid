mod controller;
mod packet;
mod report;
mod slimeinterface;
mod transport;
mod virtualdevice;
mod visualizer;

use std::{sync::mpsc, thread, time::Duration};

use nalgebra::Vector3;
use uf_ahrs::{Ahrs, Vqf, VqfParams};

use crate::{controller::Controller, virtualdevice::create_virtual_controller};

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
    let mut vdevice = create_virtual_controller()?;
    let dt = Duration::from_secs_f32(1.0 / 250.0);
    let mut vqf = Vqf::new(dt, VqfParams::default());
    const ACCEL_LSB_PER_G: f32 = 4096.0;
    const GYRO_LSB_PER_DPS: f32 = 8.2;
    const MAG_SCALE: f32 = 0.15;
    loop {
        match controller.get_input() {
            Ok(n) => {
                let accel = Vector3::new(
                    (n.motion.accel_x as f32 / ACCEL_LSB_PER_G) * 9.81,
                    (n.motion.accel_y as f32 / ACCEL_LSB_PER_G) * 9.81,
                    (n.motion.accel_z as f32 / ACCEL_LSB_PER_G) * 9.81,
                );
                let gyro = Vector3::new(
                    (n.motion.gyro_x as f32 / GYRO_LSB_PER_DPS).to_radians(),
                    (n.motion.gyro_y as f32 / GYRO_LSB_PER_DPS).to_radians(),
                    (n.motion.gyro_z as f32 / GYRO_LSB_PER_DPS).to_radians(),
                );
                let mag = Vector3::new(
                    n.magnetometer.x as f32 * MAG_SCALE,
                    n.magnetometer.y as f32 * MAG_SCALE,
                    n.magnetometer.z as f32 * MAG_SCALE,
                );
                vqf.update(gyro, accel, mag);
                let orientation = vqf.orientation();
                n.emit_to_device(&mut vdevice)?;
            }
            Err(e) => {
                eprintln!("[sw2ctl] HID read error, {e}")
            }
        }

        //handle_ff_events(&mut device, &controller, &mut seq, &mut ff_effects)?;
    }

    Ok(())
}
