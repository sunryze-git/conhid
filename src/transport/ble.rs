use std::{
    fs::File,
    io::{self, Read, Write},
    mem,
    os::fd::{AsRawFd, FromRawFd, RawFd},
    thread,
    time::{Duration, Instant},
};

use crate::transport::*;
use libc::{AF_BLUETOOTH, SOCK_SEQPACKET, bind, connect, sockaddr, socket, socklen_t};

// constants
const BTPROTO_L2CAP: libc::c_int = 0;
const BDADDR_LE_PUBLIC: u8 = 0x01;
const ATT_CID: u16 = 4;

const ATT_OP_HANDLE_NOTIFY: u8 = 0x1B;
const ATT_OP_WRITE_REQ: u8 = 0x12;
const ATT_OP_WRITE_CMD: u8 = 0x52;

#[derive(Clone, Copy)]
pub struct BleHandles {
    pub cmd_write: u16,
    pub cmd_resp: u16,
    pub hid_input: u16,
}

impl Default for BleHandles {
    fn default() -> Self {
        Self {
            cmd_write: 0x0014,
            cmd_resp: 0x001A,
            hid_input: 0x000A,
        }
    }
}

#[repr(C)]
#[derive(Default)]
struct SockaddrL2 {
    l2_family: libc::sa_family_t,
    l2_psm: libc::c_ushort,
    l2_bdaddr: [u8; 6],
    l2_cid: libc::c_ushort,
    l2_bdaddr_type: u8,
}

fn parse_mac(s: &str) -> io::Result<[u8; 6]> {
    let mut b = [0u8; 6];
    let mut count = 0;

    for (i, part) in s.split(':').enumerate() {
        if i >= 6 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "Too many segments",
            ));
        }
        b[i] = u8::from_str_radix(part, 16)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "Invalid hex"))?;
        count += 1;
    }

    if count != 6 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "Incomplete MAC",
        ));
    }

    b.reverse();
    Ok(b)
}

// unsafe and without protection!
fn connect_att(adapter_mac: &str, controller_mac: &str) -> io::Result<RawFd> {
    let fd = unsafe { socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP) };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }

    let result = (|| {
        let mut local = SockaddrL2::default();
        local.l2_family = AF_BLUETOOTH as u16;
        local.l2_cid = ATT_CID.to_le();
        local.l2_bdaddr_type = BDADDR_LE_PUBLIC;
        local.l2_bdaddr = parse_mac(adapter_mac)?;

        if unsafe {
            bind(
                fd,
                &local as *const _ as *const sockaddr,
                mem::size_of::<SockaddrL2>() as socklen_t,
            )
        } < 0
        {
            return Err(io::Error::last_os_error());
        }

        let mut remote = SockaddrL2::default();
        remote.l2_family = AF_BLUETOOTH as u16;
        remote.l2_cid = ATT_CID.to_le();
        remote.l2_bdaddr_type = BDADDR_LE_PUBLIC;
        remote.l2_bdaddr = parse_mac(controller_mac)?;

        println!("[ble] Connecting to {} ...", controller_mac);
        if unsafe {
            connect(
                fd,
                &remote as *const _ as *const sockaddr,
                mem::size_of::<SockaddrL2>() as socklen_t,
            )
        } < 0
        {
            return Err(io::Error::last_os_error());
        }

        Ok(fd)
    })();

    match result {
        Ok(fd) => Ok(fd),
        Err(e) => {
            unsafe { libc::close(fd) };
            Err(e)
        }
    }
}

pub struct BleTransport {
    sock: File,
    handles: BleHandles,
}

impl BleTransport {
    pub fn open(
        adapter_mac: &str,
        mac_addr: &str,
        handles: Option<BleHandles>,
    ) -> io::Result<Self> {
        let fd = connect_att(adapter_mac, mac_addr)?;
        let mut transport = Self {
            sock: unsafe { File::from_raw_fd(fd) },
            handles: handles.unwrap_or_default(),
        };

        transport.exchange_mtu()?;
        transport.enable_notifications()?;
        transport.drain()?;

        Ok(transport)
    }

    fn exchange_mtu(&mut self) -> io::Result<()> {
        self.sock.write_all(&[0x02, 0x00, 0x02])?;
        let mut resp = [0u8; 64];
        let n = self.sock.read(&mut resp)?;
        if n >= 3 && resp[0] == 0x03 {
            let mtu = u16::from_le_bytes([resp[1], resp[2]]);
            println!("[ble] MTU exchanged, server MTU: {mtu}");
        }
        Ok(())
    }

    fn enable_notifications(&mut self) -> io::Result<()> {
        for &h in &[
            self.handles.cmd_write,
            self.handles.cmd_resp,
            self.handles.hid_input,
        ] {
            let handle = h + 1;
            let [h_low, h_high] = handle.to_le_bytes();
            self.sock
                .write_all(&[ATT_OP_WRITE_REQ, h_low, h_high, 0x01, 0x00])?;
            thread::sleep(Duration::from_millis(5));
        }
        Ok(())
    }

    fn drain(&mut self) -> io::Result<()> {
        let fd = self.sock.as_raw_fd();
        unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFL, 0);
            libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
        }

        let mut buf = [0u8; 256];
        while self.sock.read(&mut buf).is_ok_and(|n| n > 0) {}

        unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFL, 0);
            libc::fcntl(fd, libc::F_SETFL, flags & !libc::O_NONBLOCK);
        }
        Ok(())
    }

    fn set_read_timeout(&self, timeout: Duration) -> io::Result<()> {
        let tv = libc::timeval {
            tv_sec: timeout.as_secs() as libc::time_t,
            tv_usec: timeout.subsec_micros() as libc::suseconds_t,
        };
        let ret = unsafe {
            libc::setsockopt(
                self.sock.as_raw_fd(),
                libc::SOL_SOCKET,
                libc::SO_RCVTIMEO,
                &tv as *const _ as *const libc::c_void,
                mem::size_of::<libc::timeval>() as socklen_t,
            )
        };
        if ret < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
}

impl Transport for BleTransport {
    fn send_command(&self, buf: &[u8]) -> io::Result<()> {
        self.send_write_cmd(self.handles.cmd_write, buf)
    }

    fn recv_response(&self) -> io::Result<TransportData> {
        self.recv_notification(self.handles.cmd_resp)
    }

    fn recv_hid(&self) -> io::Result<TransportData> {
        self.recv_notification(self.handles.hid_input)
    }

    fn send_hid_cmd(&self, buf: &[u8]) -> io::Result<()> {
        self.send_write_cmd(self.handles.hid_input, buf)
    }
}

// Private Funcs
impl BleTransport {
    fn send_write_cmd(&self, handle: u16, buf: &[u8]) -> io::Result<()> {
        if buf.len() + 3 > TRANSPORT_MTU {
            return Err(io::Error::other("MTU Exceeded"));
        }
        let mut packet = [0u8; TRANSPORT_MTU];
        packet[0] = ATT_OP_WRITE_CMD;
        let [h_low, h_high] = handle.to_le_bytes();
        packet[1] = h_low;
        packet[2] = h_high;
        packet[3..3 + buf.len()].copy_from_slice(buf);

        (&self.sock).write_all(&packet[..buf.len() + 3])
    }

    fn recv_notification(&self, expected_handle: u16) -> io::Result<TransportData> {
        let start = Instant::now();
        let timeout = Duration::from_millis(TRANSPORT_TIMEOUT as u64);
        let mut raw = [0u8; TRANSPORT_MTU];

        loop {
            let elapsed = start.elapsed();
            if elapsed >= timeout {
                return Err(io::Error::from(io::ErrorKind::TimedOut));
            }

            self.set_read_timeout(timeout - elapsed)?;

            let n = match (&self.sock).read(&mut raw) {
                Ok(n) => n,
                Err(e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    return Err(io::Error::from(io::ErrorKind::TimedOut));
                }
                Err(e) => return Err(e),
            };

            if n >= 3 && raw[0] == ATT_OP_HANDLE_NOTIFY {
                let h = u16::from_le_bytes([raw[1], raw[2]]);
                if h == expected_handle {
                    let payload_len = (n - 3).min(TRANSPORT_MTU);
                    let mut data = TransportData {
                        payload: [0u8; TRANSPORT_MTU],
                        len: payload_len,
                    };
                    data.payload[..payload_len].copy_from_slice(&raw[3..3 + payload_len]);
                    return Ok(data);
                }
            }
        }
    }
}
