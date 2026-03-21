use core::fmt;
use std::io::Error;

use evdev::{AbsoluteAxisCode, EventType, InputEvent, KeyCode, uinput::VirtualDevice};

pub struct InputReport {
    pub counter: u32,
    pub buttons: Buttons,
    pub left_stick: StickData,
    pub right_stick: StickData,
    pub mouse: MouseData,
    pub magnetometer: MagData,
    pub battery_mv: u16,
    pub charging_state: u8,
    pub battery_current: u16,
    pub motion: MotionData,
    pub left_trigger: u8,
    pub right_trigger: u8,
}

pub struct Buttons {
    // Right
    pub zr: bool,
    pub r: bool,
    pub slr: bool,
    pub srr: bool,
    pub a: bool,
    pub b: bool,
    pub x: bool,
    pub y: bool,
    // Center
    pub c: bool,
    pub capture: bool,
    pub home: bool,
    pub left_stick: bool,
    pub right_stick: bool,
    pub plus: bool,
    pub minus: bool,
    // Left
    pub zl: bool,
    pub l: bool,
    pub sll: bool,
    pub srl: bool,
    pub dpad_left: bool,
    pub dpad_right: bool,
    pub dpad_up: bool,
    pub dpad_down: bool,
    // Misc
    pub headset: bool,
    pub gl: bool,
    pub gr: bool,
}

impl Buttons {
    fn from_bytes(b: &[u8; 4]) -> Self {
        Self {
            zr: b[0] & 0x80 != 0,
            r: b[0] & 0x40 != 0,
            slr: b[0] & 0x20 != 0,
            srr: b[0] & 0x10 != 0,
            a: b[0] & 0x08 != 0,
            b: b[0] & 0x04 != 0,
            x: b[0] & 0x02 != 0,
            y: b[0] & 0x01 != 0,

            c: b[1] & 0x40 != 0,
            capture: b[1] & 0x20 != 0,
            home: b[1] & 0x10 != 0,
            left_stick: b[1] & 0x08 != 0,
            right_stick: b[1] & 0x04 != 0,
            plus: b[1] & 0x02 != 0,
            minus: b[1] & 0x01 != 0,

            zl: b[2] & 0x80 != 0,
            l: b[2] & 0x40 != 0,
            sll: b[2] & 0x20 != 0,
            srl: b[2] & 0x10 != 0,
            dpad_left: b[2] & 0x08 != 0,
            dpad_right: b[2] & 0x04 != 0,
            dpad_up: b[2] & 0x02 != 0,
            dpad_down: b[2] & 0x01 != 0,

            headset: b[3] & 0x10 != 0,
            gl: b[3] & 0x02 != 0,
            gr: b[3] & 0x01 != 0,
        }
    }
}

pub struct StickData {
    pub x: u16,
    pub y: u16,
}

pub struct MouseData {
    pub pos_x: i16,
    pub pos_y: i16,
    pub surface_quality: u16,
    pub distance: u16,
}

impl MouseData {
    fn from_bytes(b: &[u8; 8]) -> Self {
        Self {
            pos_x: i16::from_le_bytes([b[0], b[1]]),
            pos_y: i16::from_le_bytes([b[2], b[3]]),
            surface_quality: u16::from_le_bytes([b[4], b[5]]),
            distance: u16::from_le_bytes([b[6], b[7]]),
        }
    }
}

pub struct MagData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl MagData {
    fn from_bytes(b: &[u8; 6]) -> Self {
        Self {
            x: i16::from_le_bytes([b[0], b[1]]),
            y: i16::from_le_bytes([b[2], b[3]]),
            z: i16::from_le_bytes([b[4], b[5]]),
        }
    }
}

pub struct MotionData {
    pub timestamp: u32,
    temperature: i16,
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

impl MotionData {
    fn from_bytes(b: &[u8; 18]) -> Self {
        Self {
            timestamp: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            temperature: i16::from_le_bytes([b[4], b[5]]),
            accel_x: i16::from_le_bytes([b[6], b[7]]),
            accel_y: i16::from_le_bytes([b[8], b[9]]),
            accel_z: i16::from_le_bytes([b[10], b[11]]),
            gyro_x: i16::from_le_bytes([b[12], b[13]]),
            gyro_y: i16::from_le_bytes([b[14], b[15]]),
            gyro_z: i16::from_le_bytes([b[16], b[17]]),
        }
    }

    /// Returns temperature in Celsius
    pub fn temperature(&self) -> f64 {
        25f64 + (self.temperature as f64 / 127.0)
    }

    /// Returns a filtered quaternion
    pub fn to_quat() {
        todo!()
    }
}

#[derive(Debug)]
pub enum ParseError {
    BufferTooShort,
    InvalidSlice(&'static str),
    Io(()),
}

impl From<Error> for ParseError {
    fn from(_e: Error) -> Self {
        ParseError::Io(())
    }
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

fn dpad_to_hat(up: bool, down: bool, left: bool, right: bool) -> (i32, i32) {
    let x = right as i32 - left as i32;
    let y = down as i32 - up as i32;
    (x, y)
}

impl InputReport {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, ParseError> {
        if buf.len() < 0x3F {
            return Err(ParseError::BufferTooShort);
        }
        let s = |field| move |_| ParseError::InvalidSlice(field);

        Ok(Self {
            counter: u32::from_le_bytes(buf[0x00..0x04].try_into().map_err(s("counter"))?),
            buttons: Buttons::from_bytes(buf[0x04..0x08].try_into().map_err(s("buttons"))?),
            left_stick: StickData::from_packed(&buf[0x0A..0x0D]),
            right_stick: StickData::from_packed(&buf[0x0D..0x10]),
            mouse: MouseData::from_bytes(buf[0x10..0x18].try_into().map_err(s("mouse"))?),
            magnetometer: MagData::from_bytes(
                buf[0x19..0x1F].try_into().map_err(s("magnetometer"))?,
            ),
            battery_mv: u16::from_le_bytes(buf[0x1F..0x21].try_into().map_err(s("battery_mv"))?),
            charging_state: buf[0x21],
            battery_current: u16::from_le_bytes(
                buf[0x22..0x24].try_into().map_err(s("battery_current"))?,
            ),
            motion: MotionData::from_bytes(buf[0x2A..0x3C].try_into().map_err(s("motion"))?),
            left_trigger: buf[0x3C],
            right_trigger: buf[0x3D],
        })
    }

    pub fn emit_to_device(
        &self,
        device: &mut VirtualDevice,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut events: Vec<InputEvent> = Vec::new();

        let b = &self.buttons;
        let btn_map: &[(KeyCode, bool)] = &[
            (KeyCode::BTN_SOUTH, b.b),
            (KeyCode::BTN_EAST, b.a),
            (KeyCode::BTN_NORTH, b.x),
            (KeyCode::BTN_WEST, b.y),
            (KeyCode::BTN_TL, b.l),
            (KeyCode::BTN_TR, b.r),
            (KeyCode::BTN_TL2, b.zl),
            (KeyCode::BTN_TR2, b.zr),
            (KeyCode::BTN_SELECT, b.minus),
            (KeyCode::BTN_START, b.plus),
            (KeyCode::BTN_THUMBL, b.left_stick),
            (KeyCode::BTN_THUMBR, b.right_stick),
            (KeyCode::BTN_MODE, b.home),
            (KeyCode::BTN_TRIGGER_HAPPY1, b.capture),
            (KeyCode::BTN_TRIGGER_HAPPY2, b.gl),
            (KeyCode::BTN_TRIGGER_HAPPY3, b.gr),
            (KeyCode::BTN_TRIGGER_HAPPY4, b.sll),
            (KeyCode::BTN_TRIGGER_HAPPY5, b.slr),
            (KeyCode::BTN_TRIGGER_HAPPY6, b.srl),
            (KeyCode::BTN_TRIGGER_HAPPY7, b.srr),
        ];
        for (key, pressed) in btn_map {
            events.push(InputEvent::new(EventType::KEY.0, key.0, *pressed as i32));
        }

        let (hat_x, hat_y) = dpad_to_hat(b.dpad_up, b.dpad_down, b.dpad_left, b.dpad_right);
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_HAT0X.0,
            hat_x,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_HAT0Y.0,
            hat_y,
        ));

        let (lx, ly) = self.left_stick.to_evdev_axes();
        let (rx, ry) = self.right_stick.to_evdev_axes();
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_X.0,
            lx,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_Y.0,
            ly,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_RX.0,
            rx,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_RY.0,
            ry,
        ));

        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_Z.0,
            self.left_trigger as i32,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_RZ.0,
            self.right_trigger as i32,
        ));

        let m = &self.motion;
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_TILT_X.0,
            m.gyro_x as i32,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_TILT_Y.0,
            m.gyro_y as i32,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_MISC.0,
            m.gyro_z as i32,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_BRAKE.0,
            m.accel_x as i32,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_GAS.0,
            m.accel_y as i32,
        ));
        events.push(InputEvent::new(
            EventType::ABSOLUTE.0,
            AbsoluteAxisCode::ABS_WHEEL.0,
            m.accel_z as i32,
        ));

        // Flush
        events.push(InputEvent::new(EventType::SYNCHRONIZATION.0, 0, 0));

        device.emit(&events)?;
        Ok(())
    }
}

impl StickData {
    fn from_packed(bytes: &[u8]) -> Self {
        let x = (bytes[0] as u16) | (((bytes[1] & 0x0F) as u16) << 8);
        let y = ((bytes[1] as u16) >> 4) | ((bytes[2] as u16) << 4);
        Self { x, y }
    }

    pub fn to_evdev_axes(&self) -> (i32, i32) {
        let scale = |v: u16| -> i32 { ((v as i32 - 2048) * 32767) / 2048 };
        (scale(self.x), -scale(self.y)) // negate Y
    }
}
