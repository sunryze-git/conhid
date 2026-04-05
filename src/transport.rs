pub mod ble;
pub mod usb;

// Constants
pub const TRANSPORT_MTU: usize = 64;
pub const TRANSPORT_TIMEOUT: u64 = 500;

pub struct TransportData {
    pub payload: [u8; TRANSPORT_MTU],
    pub len: usize,
}

pub trait Transport: Send {
    fn send_command(&self, buf: &[u8]) -> std::io::Result<()>;
    fn recv_response(&self) -> std::io::Result<TransportData>;
    fn recv_hid(&self) -> std::io::Result<TransportData>;
    fn send_hid_cmd(&self, buf: &[u8]) -> std::io::Result<()>;
}
