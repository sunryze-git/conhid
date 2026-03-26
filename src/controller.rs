use nalgebra::{UnitQuaternion, Vector3};

use crate::packet::*;
use crate::report::{InputReport, MagData, MotionData, ParseError};
use crate::transport::Transport;
use crate::transport::ble::{BleHandles, BleTransport};
use crate::transport::usb::UsbTransport;

// Specify the report ID to get back. This currently only works on USB.
const REPORT_TYPE: u8 = 0x05;

use vqf_rs::{Params, VQF};

pub struct Controller {
    transport: Box<dyn Transport>,
    transport_kind: TransportType,
    vqf: VQF,
    previous_time: Option<u32>,
    sample_count: u128,
    pub device_info: DeviceInfo,
    pub controller_type: ControllerType,
}

#[derive(Debug, Default)]
pub struct DeviceInfo {
    pub serial: String,
    pub vendor: u16,
    pub product: u16,
    pub firmware_version: (u8, u8, u8),
    pub bluetooth_version: (u8, u8, u8),
    pub factory_calibration: FactoryCalibration,
}

#[derive(Debug, Default)]
pub struct StickCalibration {
    x_max: u16,
    y_max: u16,
    x_center: u16,
    y_center: u16,
    x_min: u16,
    y_min: u16,
}

#[derive(Debug, Default)]
pub struct FactoryCalibration {
    /// Data is in degrees Celsius
    pub temperature: f32,

    /// Data is in m/sec^2
    pub accel_bias: Vector3<f32>,

    /// Data is in radians/sec
    pub gyro_bias: Vector3<f32>,

    /// Data is in µT (microtesla)
    pub mag_bias: Vector3<f32>,
    pub primary_stick: StickCalibration,
    pub secondary_stick: StickCalibration,
}

#[derive(Debug)]
pub enum ControllerType {
    ProController2,
    JoyConL,
    JoyConR,
    NsoGc,
    Unknown,
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
                vec![0xFF, 0x00, 0x00, 0x00],
            )?,
            Packet::new(
                Command::Features(FeaturesSubCmd::Enable),
                kind,
                vec![0xFF, 0x00, 0x00, 0x00],
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
        match self.transport_kind {
            TransportType::Usb => InputReport::from_bytes(&resp.payload[1..resp.len]),
            TransportType::Ble => InputReport::from_bytes(&resp.payload[..resp.len]),
        }
    }

    // Different controller types seem to have different timestamp scales.
    fn micros_per_tick(&self) -> f64 {
        match self.controller_type {
            ControllerType::ProController2 => 25.60,
            ControllerType::JoyConL => 1.00,
            ControllerType::JoyConR => 1.00,
            ControllerType::NsoGc => 1.00,
            ControllerType::Unknown => {
                panic!("Attempted to call micros_per_tick on an unknown controller type.")
            }
        }
    }

    /// Updates the internal VQF filter, polling rate, and returns the current VQF Quaternion.
    pub fn update_orientation(
        &mut self,
        motion: &MotionData,
        magnetometer: &MagData,
    ) -> Result<UnitQuaternion<f32>, &str> {
        const ACCEL_SCALE: f64 = 4096.0; // 4096 counts / g
        const GYRO_SCALE: f64 = 16.4; // 16.4 counts / degree / sec
        const GYRO_TEMP_COEFF: f64 = 0.015_f64; // 0.015 Celsius / Degree / Sec
        const MAG_SCALE: f64 = 0.15; // 0.15 uT per LSB
        const G: f64 = 9.80665; // Gravity Constant
        self.sample_count += 1;

        // Skip the first 200 reports, as it will have invalid dt calculation, and to stabilize.
        if self.sample_count < 100 {
            self.previous_time = Some(motion.timestamp);
            return Err("warming up");
        }

        // Calculate wall time passed in microseconds
        let hw_ticks = motion.timestamp.wrapping_sub(self.previous_time.unwrap());
        let dt = (hw_ticks as f64 * self.micros_per_tick()) * 1e-6;

        // update previous
        self.previous_time = Some(motion.timestamp);

        // Produces accelerometer data in meters/second^2
        let accel = [
            motion.accel_x / ACCEL_SCALE * G,
            motion.accel_y / ACCEL_SCALE * G,
            motion.accel_z / ACCEL_SCALE * G,
        ];

        // Produces gyroscope data in radians / sec
        let gyro_bias = self.device_info.factory_calibration.gyro_bias;
        let temp_delta =
            motion.temperature() - self.device_info.factory_calibration.temperature as f64;
        let gyro_temp_correction = (temp_delta * GYRO_TEMP_COEFF).to_radians();
        let gyro = [
            (motion.gyro_x / GYRO_SCALE).to_radians() - gyro_bias.x as f64 - gyro_temp_correction,
            (motion.gyro_y / GYRO_SCALE).to_radians() - gyro_bias.y as f64 - gyro_temp_correction,
            (motion.gyro_z / GYRO_SCALE).to_radians() - gyro_bias.z as f64 - gyro_temp_correction,
        ];

        // Produces magnetometer data in microTesla
        let mag = [
            magnetometer.x * MAG_SCALE,
            magnetometer.y * MAG_SCALE,
            magnetometer.z * MAG_SCALE,
        ];

        // Updates the VQF
        self.vqf.update_gyr_with_ts(gyro, dt);
        self.vqf.update_acc(accel);
        //self.vqf.update_mag(mag);

        let q = self.vqf.quat_6d();
        let unit_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            q.0 as f32, q.1 as f32, q.2 as f32, q.3 as f32,
        ));

        Ok(unit_quat)
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
            ControllerType::Unknown => {
                panic!("Attempted to perform a rumble command on an unknown controller type.")
            }
        }

        *seq = seq.wrapping_add(1) & 0x0F;

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

    fn read_device_info(&mut self) -> std::io::Result<DeviceInfo> {
        let mut info = DeviceInfo::default();
        let unpack = |b: &[u8]| -> (u16, u16) {
            let val0 = (b[0] as u16) | (((b[1] & 0x0F) as u16) << 8);
            let val1 = ((b[1] >> 4) as u16) | ((b[2] as u16) << 4);
            (val0, val1)
        };

        // Get Firmware Versions
        let resp = self.request(Packet::new(
            Command::FirmwareInfo(FirmwareInfoSubCmd::GetVersion),
            self.transport_kind,
            vec![],
        )?)?;
        info.firmware_version = (resp.data[0], resp.data[1], resp.data[2]);
        info.bluetooth_version = (resp.data[4], resp.data[5], resp.data[6]);
        self.controller_type = match resp.data[3] {
            0x00 => ControllerType::JoyConL,
            0x01 => ControllerType::JoyConR,
            0x02 => ControllerType::ProController2,
            0x03 => ControllerType::NsoGc,
            _ => ControllerType::Unknown,
        };

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
            vec![0x10, 0x7E, 0x00, 0x00, 0x12, 0x30, 0x01, 0x00],
        )?)?;
        info.vendor = u16::from_le_bytes([resp.data[8], resp.data[9]]);
        info.product = u16::from_le_bytes([resp.data[10], resp.data[11]]);

        // Get Primary Stick Calibration
        let resp = self.request(Packet::new(
            Command::Spi(SpiSubCmd::MemoryRead),
            self.transport_kind,
            vec![0x09, 0x7E, 0x00, 0x00, 0xA8, 0x30, 0x01, 0x00],
        )?)?;
        let cal = &resp.data[8..];
        let (x_max, y_max) = unpack(&cal[0..3]);
        let (x_center, y_center) = unpack(&cal[3..6]);
        let (x_min, y_min) = unpack(&cal[6..9]);
        info.factory_calibration.primary_stick = StickCalibration {
            x_max,
            y_max,
            x_center,
            y_center,
            x_min,
            y_min,
        };

        // Get Secondary Stick Calibration
        let resp = self.request(Packet::new(
            Command::Spi(SpiSubCmd::MemoryRead),
            self.transport_kind,
            vec![0x09, 0x7E, 0x00, 0x00, 0xE8, 0x30, 0x01, 0x00],
        )?)?;
        let cal = &resp.data[8..];
        let (x_max, y_max) = unpack(&cal[0..3]);
        let (x_center, y_center) = unpack(&cal[3..6]);
        let (x_min, y_min) = unpack(&cal[6..9]);
        info.factory_calibration.secondary_stick = StickCalibration {
            x_max,
            y_max,
            x_center,
            y_center,
            x_min,
            y_min,
        };

        // Get Gyroscope Factory Calibration ADDRESS 0x13040
        // Returns float32 values { temp, gyro_x, gyro_y, gyro_x }
        let resp = self.request(Packet::new(
            Command::Spi(SpiSubCmd::MemoryRead),
            self.transport_kind,
            vec![0x10, 0x7E, 0x00, 0x00, 0x40, 0x30, 0x01, 0x00],
        )?)?;
        let floats: Vec<f32> = resp.data[8..]
            .chunks_exact(4)
            .map(|chunk| f32::from_le_bytes(chunk.try_into().unwrap()))
            .collect();
        info.factory_calibration.temperature = floats[0];
        info.factory_calibration.gyro_bias = Vector3::new(floats[1], floats[2], floats[3]);

        // Get Accelerometer / Magnetometer Factory Calibration ADDRESS 0x13100
        // Returns float32 values { mag_x, mag_y, mag_z, accel_x, accel_y, accel_z }
        let resp = self.request(Packet::new(
            Command::Spi(SpiSubCmd::MemoryRead),
            self.transport_kind,
            vec![0x18, 0x7E, 0x00, 0x00, 0x00, 0x31, 0x01, 0x00],
        )?)?;
        let floats: Vec<f32> = resp.data[8..]
            .chunks_exact(4)
            .map(|chunk| f32::from_le_bytes(chunk.try_into().unwrap()))
            .collect();
        info.factory_calibration.mag_bias = Vector3::new(floats[0], floats[1], floats[2]);
        info.factory_calibration.accel_bias = Vector3::new(floats[3], floats[4], floats[5]);

        Ok(info)
    }

    fn init(transport: Box<dyn Transport>, kind: TransportType) -> std::io::Result<Self> {
        let params = Params {
            tau_acc: 3.0,
            tau_mag: 20.0,
            rest_bias_est_enabled: true,
            mag_dist_rejection_enabled: true,
            bias_sigma_init: 0.5,
            bias_forgetting_time: 100.0,
            bias_clip: 2.0, // °/s
            bias_sigma_rest: 0.03,
            rest_min_t: 1.5,
            rest_filter_tau: 0.5,
            rest_th_gyr: 2.0, // °/s
            rest_th_acc: 0.5, // m/s²
            ..Params::default()
        };

        let mut ctrl = Self {
            transport,
            transport_kind: kind,
            vqf: VQF::new(1.0, None, None, Some(params)),
            previous_time: None,
            sample_count: 0,
            device_info: DeviceInfo::default(),
            controller_type: ControllerType::Unknown,
        };

        let packets = Self::init_sequence(kind)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        println!(
            "[sw2ctl] Running init sequence ({} commands)",
            packets.len()
        );
        ctrl.run_sequence(packets)?;

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
