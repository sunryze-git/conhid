use super::Transport;
use crate::transport::{TRANSPORT_MTU, TRANSPORT_TIMEOUT, TransportData};
use rusb::{DeviceHandle, GlobalContext};
use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::Duration;

const NINTENDO_VID: u16 = 0x057E;
const SW2_PIDS: &[u16] = &[
    0x2060, // pro con 2
    0x2066, // joycon L
    0x2067, // joycon R
    0x2068, // NSO GC
    0x2069, // pro con 2
];

// Interfaces
const HID_INTERFACE: u8 = 0;
const BULK_INTERFACE: u8 = 1;

const HID_EP_IN: u8 = 0x81;
const BULK_EP_IN: u8 = 0x82;

const EP_OUT: u8 = 0x02;
const HID_OUT: u8 = 0x01;

pub struct UsbTransport {
    handle: DeviceHandle<GlobalContext>,
    kernel_driver_attached: [bool; 2],
    pending: Mutex<VecDeque<TransportData>>,
}

impl UsbTransport {
    pub fn open() -> std::io::Result<Self> {
        let devices = rusb::devices().unwrap();

        for device in devices.iter() {
            let desc = device.device_descriptor().unwrap();

            if desc.vendor_id() == NINTENDO_VID && SW2_PIDS.contains(&desc.product_id()) {
                let handle = device
                    .open()
                    .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

                let mut kernel_driver_attached = [false; 2];
                for iface in [HID_INTERFACE, BULK_INTERFACE] {
                    kernel_driver_attached[iface as usize] =
                        match handle.kernel_driver_active(iface) {
                            Ok(true) => {
                                handle.detach_kernel_driver(iface).map_err(|e| {
                                    std::io::Error::new(std::io::ErrorKind::Other, e)
                                })?;
                                true
                            }
                            _ => false,
                        };
                    handle
                        .claim_interface(iface)
                        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
                }

                return Ok(UsbTransport {
                    handle,
                    kernel_driver_attached,
                    pending: Mutex::default(),
                });
            }
        }
        Err(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "No Switch 2 controller was found connected to the system.",
        ))
    }
}

impl Transport for UsbTransport {
    fn send_command(&self, buf: &[u8]) -> Result<(), std::io::Error> {
        self.handle
            .write_bulk(EP_OUT, buf, Duration::from_millis(TRANSPORT_TIMEOUT))
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
        Ok(())
    }

    fn recv_response(&self) -> Result<TransportData, std::io::Error> {
        let mut buf = [0u8; TRANSPORT_MTU];
        let bytes_read = self
            .handle
            .read_bulk(
                BULK_EP_IN,
                &mut buf,
                Duration::from_millis(TRANSPORT_TIMEOUT),
            )
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
        Ok(TransportData {
            payload: buf,
            len: bytes_read,
        })
    }

    fn recv_hid(&self) -> Result<TransportData, std::io::Error> {
        // Return the pending report, if there is one
        {
            let mut pending = self.pending.lock().unwrap();
            if let Some(data) = pending.pop_front() {
                return Ok(data);
            }
        }

        let mut buf = [0u8; TRANSPORT_MTU];
        let bytes_read = self
            .handle
            .read_interrupt(
                HID_EP_IN,
                &mut buf,
                Duration::from_millis(TRANSPORT_TIMEOUT),
            )
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        // split into 64-byte reports
        let mut pending = self.pending.lock().unwrap();
        let mut offset = 64; // first report returned directly below
        while offset + 64 <= bytes_read {
            let mut payload = [0u8; TRANSPORT_MTU];
            payload[..64].copy_from_slice(&buf[offset..offset + 64]);
            pending.push_back(TransportData { payload, len: 64 });
            offset += 64;
        }

        Ok(TransportData {
            payload: buf,
            len: 64, // always exactly one report
        })
    }

    fn send_hid_cmd(&self, buf: &[u8]) -> std::io::Result<()> {
        self.handle
            .write_interrupt(HID_OUT, buf, Duration::from_millis(TRANSPORT_TIMEOUT))
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
        Ok(())
    }
}

impl Drop for UsbTransport {
    fn drop(&mut self) {
        for iface in 0..2u8 {
            let _ = self.handle.release_interface(iface);
            if self.kernel_driver_attached[iface as usize] {
                let _ = self.handle.attach_kernel_driver(iface);
            }
        }
    }
}
