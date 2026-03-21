use crate::packet::*;
use crate::report::{InputReport, ParseError};
use crate::transport::Transport;
use crate::transport::ble::{BleHandles, BleTransport};
use crate::transport::usb::UsbTransport;

const REPORT_TYPE: u8 = 0x05;

pub struct Controller {
    transport: Box<dyn Transport>,
    transport_kind: TransportType,
    pub device_info: DeviceInfo,
    pub controller_type: ControllerType,
}

#[derive(Debug)]
pub struct DeviceInfo {
    pub name: String,
    pub serial: String,
    pub vendor: u16,
    pub product: u16,
    pub left_stick_cal: [u8; 9],
    pub right_stick_cal: [u8; 9],
    pub firmware_version: (u8, u8, u8),
    pub bluetooth_version: (u8, u8, u8),
}
impl Default for DeviceInfo {
    fn default() -> Self {
        DeviceInfo {
            name: String::new(),
            serial: String::new(),
            vendor: 0,
            product: 0,
            left_stick_cal: [0u8; 9],
            right_stick_cal: [0u8; 9],
            firmware_version: (0, 0, 0),
            bluetooth_version: (0, 0, 0),
        }
    }
}

pub enum ControllerType {
    ProController2,
    JoyConL,
    JoyConR,
    NsoGc,
}

impl Controller {
    fn init_sequence(kind: TransportType) -> std::result::Result<Vec<Packet>, PacketError> {
        Ok(vec![
            Packet::new(
                Command::Init(InitSubCmd::UsbInit),
                kind,
                vec![0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            )?,
            Packet::new(Command::Cmd07(Cmd07SubCmd::Init), kind, vec![])?,
            Packet::new(Command::Cmd16(Cmd16SubCmd::Unknown01), kind, vec![])?,
            Packet::new(
                Command::Battery(BatterySubCmd::Unknown07),
                kind,
                vec![0x00, 0x00, 0x00, 0x00],
            )?,
            Packet::new(
                Command::Led(LedSubCmd::SetPattern),
                kind,
                vec![0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            )?,
            Packet::new(
                Command::Features(FeaturesSubCmd::SetMask),
                kind,
                vec![0x27, 0x00, 0x00, 0x00],
            )?,
            Packet::new(
                Command::Features(FeaturesSubCmd::Enable),
                kind,
                vec![0x27, 0x00, 0x00, 0x00],
            )?,
            Packet::new(
                Command::Haptics(HapticsSubCmd::PlaySample),
                kind,
                vec![0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            )?,
            Packet::new(Command::Cmd11(Cmd11SubCmd::Unknown03), kind, vec![])?,
            Packet::new(
                Command::Init(InitSubCmd::InputReport),
                kind,
                vec![REPORT_TYPE, 0x00, 0x00, 0x00],
            )?,
            Packet::new(Command::Cmd18(Cmd18SubCmd::Unknown01), kind, vec![])?,
        ])
    }

    pub fn open_usb() -> std::io::Result<Self> {
        let transport = UsbTransport::open()?;
        Self::init(Box::new(transport), TransportType::Usb)
    }

    pub fn open_ble(adapter_mac: &str, controller_mac: &str) -> std::io::Result<Self> {
        let transport =
            BleTransport::open(adapter_mac, controller_mac, Some(BleHandles::default()))?;
        Self::init(Box::new(transport), TransportType::Ble)
    }

    pub fn get_input(&self) -> std::result::Result<InputReport, ParseError> {
        let resp = self.transport.recv_hid()?;
        // clearscreen::clear().expect("bad");
        // println!("{:02X?}", &resp.payload[20..20 + 24]);

        // let start = 20;
        // let section = &resp.payload[start..start + 24];

        // let ints: Vec<i32> = section
        //     .chunks(4)
        //     .map(|c| i32::from_le_bytes(c.try_into().unwrap()))
        //     .collect();

        // println!("6 x u32 from the section:");
        // for (i, val) in ints.iter().enumerate() {
        //     println!("  [{i}] 0x{val:08X}  ({val})");
        // }

        match self.transport_kind {
            TransportType::Usb => InputReport::from_bytes(&resp.payload[1..resp.len]),
            TransportType::Ble => InputReport::from_bytes(&resp.payload[..resp.len]),
        }
    }

    pub fn do_rumble(&self, seq: &mut u8, rumble: &HdRumble) -> std::io::Result<()> {
        let mut data = [0u8; 64];
        let seq_nibble = *seq & 0x0F;

        data[0] = 0x02;

        match self.controller_type {
            ControllerType::ProController2 => {
                let (lhdr, lsamples) = rumble.left.encode(seq_nibble);
                let (rhdr, rsamples) = rumble.right.encode(seq_nibble);
                data[1] = lhdr;
                data[2..17].copy_from_slice(&lsamples);
                data[17] = rhdr;
                data[18..33].copy_from_slice(&rsamples);
            }
            ControllerType::JoyConL => {
                let (hdr, samples) = rumble.left.encode(seq_nibble);
                data[1] = hdr;
                data[2..27].copy_from_slice(&samples);
            }
            ControllerType::JoyConR => {
                let (hdr, samples) = rumble.right.encode(seq_nibble);
                data[1] = hdr;
                data[2..27].copy_from_slice(&samples);
            }
            ControllerType::NsoGc => {
                let (hdr, samples) = rumble.left.encode(seq_nibble);
                data[1] = hdr;
                data[2..5].copy_from_slice(&samples[0..3]);
            }
        }

        *seq = seq.wrapping_add(1) & 0x0F;

        println!("{:02X?}", &data);
        self.transport.send_hid_cmd(&data)
    }

    fn request(&self, packet: Packet) -> std::io::Result<Packet> {
        self.transport.send_command(&packet.to_bytes())?;
        let resp = self.transport.recv_response()?;
        Packet::from_response(&resp.payload[..resp.len])
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))
    }

    fn run_sequence(&self, packets: Vec<Packet>) -> std::io::Result<()> {
        for packet in packets {
            println!("[sw2ctl] [REQUEST ] {packet:?} {:02X?}", packet.to_bytes());
            let resp = self.request(packet)?;
            println!("[sw2ctl] [RESPONSE] {resp:?} {:02X?}", resp.to_bytes());
        }
        Ok(())
    }

    fn read_device_info(&self) -> std::io::Result<DeviceInfo> {
        let mut info = DeviceInfo::default();

        // Get Firmware Versions
        let resp = self.request(Packet::new(
            Command::FirmwareInfo(FirmwareInfoSubCmd::GetVersion),
            self.transport_kind,
            vec![],
        )?)?;
        info.firmware_version = (resp.data[0], resp.data[1], resp.data[2]);
        info.bluetooth_version = (resp.data[4], resp.data[5], resp.data[6]);

        // Get Serial Number
        let resp = self.request(Packet::new(
            Command::Spi(SpiSubCmd::MemoryRead),
            self.transport_kind,
            vec![0x10, 0x7E, 0x00, 0x00, 0x02, 0x30, 0x01, 0x00],
        )?)?;
        info.serial = String::from_utf8_lossy(&resp.data[8..])
            .trim_end_matches('\0')
            .to_string();

        // Get Vendor + Product ID
        let resp = self.request(Packet::new(
            Command::Spi(SpiSubCmd::MemoryRead),
            self.transport_kind,
            vec![0x10, 0x7E, 0x00, 0x00, 0x02, 0x30, 0x01, 0x00],
        )?)?;
        info.vendor = u16::from_le_bytes([resp.data[8], resp.data[9]]);
        info.product = u16::from_le_bytes([resp.data[10], resp.data[11]]);

        // Get Primary Stick Calibration
        let resp = self.request(Packet::new(
            Command::Spi(SpiSubCmd::MemoryRead),
            self.transport_kind,
            vec![0x09, 0x7E, 0x00, 0x00, 0xA8, 0x30, 0x01, 0x00],
        )?)?;
        info.left_stick_cal.copy_from_slice(&resp.data[8..17]);

        // Get Secondary Stick Calibration
        let resp = self.request(Packet::new(
            Command::Spi(SpiSubCmd::MemoryRead),
            self.transport_kind,
            vec![0x09, 0x7E, 0x00, 0x00, 0xE8, 0x30, 0x01, 0x00],
        )?)?;
        info.right_stick_cal.copy_from_slice(&resp.data[8..17]);

        Ok(info)
    }

    fn init(transport: Box<dyn Transport>, kind: TransportType) -> std::io::Result<Self> {
        let mut ctrl = Self {
            transport,
            transport_kind: kind,
            device_info: DeviceInfo::default(),
            controller_type: ControllerType::ProController2,
        };

        let packets = Self::init_sequence(kind)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        println!(
            "[sw2ctl] Running init sequence ({} commands)",
            packets.len()
        );
        ctrl.run_sequence(packets)?;

        println!("[sw2ctl] Reading device info");
        ctrl.device_info = ctrl.read_device_info()?;

        Ok(ctrl)
    }
}

#[derive(Clone, Copy)]
pub struct LraSample {
    pub freq: u16, // 10 bits
    pub amp: u16,  // 10 bits
}

impl LraSample {
    pub fn encode(&self) -> [u8; 3] {
        let mut b = [0u8; 3];
        b[0] = (self.freq & 0xFF) as u8;
        b[1] = ((self.freq >> 8) & 0x03) as u8 | ((self.amp & 0x3F) << 2) as u8;
        b[2] = ((self.amp >> 6) & 0x0F) as u8;
        b
    }

    pub fn silent() -> Self {
        Self { freq: 0, amp: 0 }
    }
}

pub struct LraMotor {
    pub samples: Vec<LraSample>,
}

impl LraMotor {
    pub fn single(sample: LraSample) -> Self {
        Self {
            samples: vec![sample],
        }
    }

    pub fn silent() -> Self {
        Self::single(LraSample::silent())
    }

    pub fn encode(&self, seq_nibble: u8) -> (u8, [u8; 15]) {
        let num_samples = self.samples.len().clamp(1, 5) as u8;
        let header = (seq_nibble & 0x0F) | ((num_samples & 0x03) << 4) | (1 << 6);
        let mut buf = [0u8; 15];
        for (i, s) in self.samples.iter().take(5).enumerate() {
            buf[i * 3..(i * 3) + 3].copy_from_slice(&s.encode());
        }
        (header, buf)
    }
}

pub struct HdRumble {
    pub left: LraMotor,
    pub right: LraMotor,
}

impl HdRumble {
    pub fn simple(left: LraSample, right: LraSample) -> Self {
        Self {
            left: LraMotor::single(left),
            right: LraMotor::single(right),
        }
    }

    pub fn silent() -> Self {
        Self {
            left: LraMotor::silent(),
            right: LraMotor::silent(),
        }
    }
}
