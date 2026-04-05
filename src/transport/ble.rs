use std::{
    cell::RefCell,
    fs::File,
    io::{self, Read, Write},
    mem,
    os::fd::{AsRawFd, FromRawFd},
    thread,
    time::{Duration, Instant},
};

use libc::{AF_BLUETOOTH, SOCK_SEQPACKET, bind, close, connect, sockaddr, socket, socklen_t};

use crate::transport::*;

// constants
const BTPROTO_L2CAP: libc::c_int = 0;
const BDADDR_LE_PUBLIC: u8 = 0x01;
const ATT_CID: u16 = 4;

const ATT_OP_HANDLE_NOTIFY: u8 = 0x1B;
const ATT_OP_WRITE_REQ: u8 = 0x12;
const ATT_OP_WRITE_CMD: u8 = 0x52;

const INPUT_REPORT_1: u16 = 0x001B;
const CCCD_NOTIFY: u16 = 0x002F;

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
struct SockaddrL2 {
    l2_family: libc::sa_family_t,
    l2_psm: libc::c_ushort,
    l2_bdaddr: [u8; 6],
    l2_cid: libc::c_ushort,
    l2_bdaddr_type: u8,
}

impl SockaddrL2 {
    fn zeroed() -> Self {
        unsafe { mem::zeroed() }
    }
}

fn parse_mac(s: &str) -> io::Result<[u8; 6]> {
    let mut b = [0u8; 6];
    let parts: Vec<&str> = s.split(':').collect();
    if parts.len() != 6 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("bad MAC: {s}"),
        ));
    }
    for (i, p) in parts.iter().enumerate() {
        b[i] = u8::from_str_radix(p, 16)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
    }

    b.reverse();
    Ok(b)
}

// unsafe and without protection!
fn connect_att(adapter_mac: &str, controller_mac: &str) -> io::Result<libc::c_int> {
    let fd = unsafe { socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP) };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }

    let mut local = SockaddrL2::zeroed();
    local.l2_family = AF_BLUETOOTH as u16;
    local.l2_cid = ATT_CID as libc::c_ushort;
    local.l2_bdaddr_type = BDADDR_LE_PUBLIC;
    local.l2_bdaddr = parse_mac(adapter_mac)?;

    if unsafe {
        bind(
            fd,
            &local as *const SockaddrL2 as *const sockaddr,
            mem::size_of::<SockaddrL2>() as socklen_t,
        )
    } < 0
    {
        let e = io::Error::last_os_error();
        unsafe { close(fd) };
        return Err(e);
    }

    let mut remote = SockaddrL2::zeroed();
    remote.l2_family = AF_BLUETOOTH as u16;
    remote.l2_cid = ATT_CID as libc::c_ushort;
    remote.l2_bdaddr_type = BDADDR_LE_PUBLIC;
    remote.l2_bdaddr = parse_mac(controller_mac)?;

    println!("[ble] Connecting to {} ...", controller_mac);
    if unsafe {
        connect(
            fd,
            &remote as *const SockaddrL2 as *const sockaddr,
            mem::size_of::<SockaddrL2>() as socklen_t,
        )
    } < 0
    {
        let e = io::Error::last_os_error();
        unsafe { close(fd) };
        return Err(e);
    }

    Ok(fd)
}

struct AttSocket(RefCell<File>);

impl AttSocket {
    unsafe fn from_raw_fd(fd: libc::c_int) -> Self {
        Self(RefCell::new(unsafe { File::from_raw_fd(fd) }))
    }

    fn as_raw_fd(&self) -> libc::c_int {
        self.0.borrow().as_raw_fd()
    }

    fn write_all(&self, buf: &[u8]) -> io::Result<()> {
        self.0.borrow_mut().write_all(buf)
    }

    fn read_packet(&self, buf: &mut [u8]) -> io::Result<usize> {
        self.0.borrow_mut().read(buf)
    }
}

fn exchange_mtu(sock: &AttSocket) -> io::Result<()> {
    sock.write_all(&[0x02, 0x00, 0x02])?;

    let mut resp = [0u8; 64];
    let n = sock.read_packet(&mut resp)?;
    if n >= 3 && resp[0] == 0x03 {
        let mtu = u16::from_le_bytes([resp[1], resp[2]]);
        println!("[ble] MTU exchanged, server MTU: {mtu}");
    } else {
        println!("[ble] MTU exchange failed or timed out");
    }
    Ok(())
}

fn enable_notifications(sock: &AttSocket, handles: &BleHandles) -> io::Result<()> {
    for &handle in &[
        handles.cmd_write + 1,
        handles.cmd_resp + 1,
        handles.hid_input + 1,
    ] {
        sock.write_all(&[
            ATT_OP_WRITE_REQ,
            (handle & 0xFF) as u8,
            ((handle >> 8) & 0xFF) as u8,
            0x01,
            0x00,
        ])?;
        thread::sleep(Duration::from_micros(5_000)); // std, not usleep
    }
    Ok(())
}

fn drain(sock: &mut AttSocket) -> io::Result<()> {
    use std::os::unix::io::AsRawFd;

    let fd = sock.0.borrow().as_raw_fd();
    unsafe {
        let flags = libc::fcntl(fd, libc::F_GETFL, 0);
        libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
    }

    let mut buf = [0u8; 256];
    loop {
        match sock.read_packet(&mut buf) {
            Ok(0) => break,
            Ok(_) => thread::sleep(Duration::from_micros(1_000)),
            Err(_) => break,
        }
    }

    unsafe {
        let flags = libc::fcntl(fd, libc::F_GETFL, 0);
        libc::fcntl(fd, libc::F_SETFL, flags & !libc::O_NONBLOCK);
    }

    Ok(())
}

fn set_read_timeout(sock: &AttSocket, timeout: Duration) -> io::Result<()> {
    let tv = libc::timeval {
        tv_sec: timeout.as_secs() as libc::time_t,
        tv_usec: timeout.subsec_micros() as libc::suseconds_t,
    };
    let ret = unsafe {
        libc::setsockopt(
            sock.as_raw_fd(),
            libc::SOL_SOCKET,
            libc::SO_RCVTIMEO,
            &tv as *const libc::timeval as *const libc::c_void,
            mem::size_of::<libc::timeval>() as socklen_t,
        )
    };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

fn recv_notification(sock: &AttSocket, expected_handle: u16) -> io::Result<TransportData> {
    let deadline = Instant::now();
    let timeout = Duration::from_millis(TRANSPORT_TIMEOUT);
    let mut raw = [0u8; TRANSPORT_MTU];

    loop {
        let elapsed = deadline.elapsed();
        if elapsed >= timeout {
            return Err(io::Error::from(io::ErrorKind::TimedOut));
        }

        set_read_timeout(sock, timeout - elapsed)?;

        let n = match sock.read_packet(&mut raw) {
            Ok(n) => n,
            Err(e)
                if e.kind() == io::ErrorKind::WouldBlock || e.kind() == io::ErrorKind::TimedOut =>
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

pub struct BleTransport {
    sock: AttSocket,
    handles: BleHandles,
}

impl BleTransport {
    pub fn open(
        adapter_mac: &str,
        mac_addr: &str,
        handles: Option<BleHandles>,
    ) -> io::Result<Self> {
        let handles = handles.unwrap_or_default();

        let fd = { connect_att(adapter_mac, mac_addr)? };
        // From here on: owned by AttSocket, no more manual close() calls.
        let mut sock = { unsafe { AttSocket::from_raw_fd(fd) } };

        exchange_mtu(&sock)?;
        enable_notifications(&sock, &handles)?;
        drain(&mut sock)?;

        Ok(Self { sock, handles })
    }
}

impl Transport for BleTransport {
    fn send_command(&self, buf: &[u8]) -> io::Result<()> {
        send_write_cmd(&self.sock, self.handles.cmd_write, buf)
    }

    fn recv_response(&self) -> io::Result<TransportData> {
        recv_notification(&self.sock, self.handles.cmd_resp)
    }

    fn recv_hid(&self) -> io::Result<TransportData> {
        recv_notification(&self.sock, self.handles.hid_input)
    }

    fn send_hid_cmd(&self, buf: &[u8]) -> io::Result<()> {
        send_write_cmd(&self.sock, self.handles.hid_input, buf)
    }
}

fn send_write_cmd(sock: &AttSocket, handle: u16, buf: &[u8]) -> io::Result<()> {
    if buf.len() + 3 > TRANSPORT_MTU {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "payload exceeds MTU",
        ));
    }
    let mut packet = [0u8; TRANSPORT_MTU];
    packet[0] = ATT_OP_WRITE_CMD;
    packet[1] = (handle & 0xFF) as u8;
    packet[2] = ((handle >> 8) & 0xFF) as u8;
    packet[3..3 + buf.len()].copy_from_slice(buf);
    sock.write_all(&packet[..buf.len() + 3])
}
