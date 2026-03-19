use crate::transport::TRANSPORT_MTU;
use crate::transport::TRANSPORT_TIMEOUT;
use crate::transport::Transport;
use crate::transport::TransportData;
use std::ffi::CString;

// The BLE transport has to be done in C, it makes the most sense there due to raw socket connectivity.
// This is because the controllers are extremely non-standard. They don't follow most Bluetooth spec.
// They will actively disconnect upon an SMP attempt, which BlueZ will automatically do.
// Using sockets bypasses this, however sockets are extremely stupid to do in Rust and its better to just
// keep using the same stuff as before.
#[link(name = "ble_transport")]
unsafe extern "C" {
    fn ble_transport_open(
        adapter_mac: *const i8,
        controller_mac: *const i8,
        handles: *mut std::ffi::c_void,
    ) -> *mut std::ffi::c_void;

    fn ble_transport_close(t: *mut std::ffi::c_void);

    fn ble_transport_send(t: *mut std::ffi::c_void, buf: *const u8, len: usize) -> i32;

    fn ble_transport_recv(
        t: *mut std::ffi::c_void,
        buf: *mut u8,
        len: usize,
        timeout_ms: i32,
        expected_handle: u16,
    ) -> i32;
}

pub struct BleTransport {
    ptr: *mut std::ffi::c_void,
}

impl BleTransport {
    pub fn open(adapter_mac: &str, controller_mac: &str) -> Result<Self, std::io::Error> {
        let adapter = CString::new(adapter_mac).unwrap();
        let controller = CString::new(controller_mac).unwrap();

        // c calls are unsafe because we have no idea what its doing
        let ptr = unsafe {
            ble_transport_open(adapter.as_ptr(), controller.as_ptr(), std::ptr::null_mut())
        };

        if ptr.is_null() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::ConnectionRefused,
                "failed to open BLE transport",
            ));
        }

        Ok(BleTransport { ptr })
    }
}

impl Transport for BleTransport {
    fn send_command(&self, buf: &[u8]) -> std::io::Result<()> {
        let ret = unsafe { ble_transport_send(self.ptr, buf.as_ptr(), buf.len()) };

        if ret < 0 {
            return Err(std::io::Error::from_raw_os_error(-ret));
        }

        Ok(())
    }

    fn recv_response(&self) -> Result<TransportData, std::io::Error> {
        let mut buf = [0u8; TRANSPORT_MTU];
        let ret = unsafe {
            ble_transport_recv(
                self.ptr,
                buf.as_mut_ptr(),
                buf.len(),
                TRANSPORT_TIMEOUT as i32,
                0x000A,
            )
        };

        if ret < 0 {
            return Err(std::io::Error::from_raw_os_error(-ret));
        }

        Ok(TransportData {
            payload: buf,
            len: ret as usize,
        })
    }

    fn recv_hid(&self) -> Result<TransportData, std::io::Error> {
        todo!()
    }

    fn send_hid_cmd(&self, buf: &[u8]) -> std::io::Result<()> {
        todo!()
    }
}

impl Drop for BleTransport {
    fn drop(&mut self) {
        unsafe { ble_transport_close(self.ptr) }
    }
}
