use std::fmt;

// Command / SubCommand Union
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    Nfc(NfcSubCmd),
    Spi(SpiSubCmd),
    Init(InitSubCmd),
    Cmd04,
    Cmd05,
    Power(PowerSubCmd),
    Cmd07(Cmd07SubCmd),
    ChargingGrip,
    Led(LedSubCmd),
    Haptics(HapticsSubCmd),
    Battery(BatterySubCmd),
    Features(FeaturesSubCmd),
    FirmwareUpdate,
    Cmd0E,
    Cmd0F,
    FirmwareInfo(FirmwareInfoSubCmd),
    Cmd11(Cmd11SubCmd),
    Cmd12,
    Cmd13,
    Cmd14,
    Pairing(PairingSubCmd),
    Cmd16(Cmd16SubCmd),
    Cmd17,
    Cmd18(Cmd18SubCmd),
}

impl Command {
    pub fn cmd_byte(&self) -> u8 {
        match self {
            Self::Nfc(_) => 0x01,
            Self::Spi(_) => 0x02,
            Self::Init(_) => 0x03,
            Self::Cmd04 => 0x04,
            Self::Cmd05 => 0x05,
            Self::Power(_) => 0x06,
            Self::Cmd07(_) => 0x07,
            Self::ChargingGrip => 0x08,
            Self::Led(_) => 0x09,
            Self::Haptics(_) => 0x0A,
            Self::Battery(_) => 0x0B,
            Self::Features(_) => 0x0C,
            Self::FirmwareUpdate => 0x0D,
            Self::Cmd0E => 0x0E,
            Self::Cmd0F => 0x0F,
            Self::FirmwareInfo(_) => 0x10,
            Self::Cmd11(_) => 0x11,
            Self::Cmd12 => 0x12,
            Self::Cmd13 => 0x13,
            Self::Cmd14 => 0x14,
            Self::Pairing(_) => 0x15,
            Self::Cmd16(_) => 0x16,
            Self::Cmd17 => 0x17,
            Self::Cmd18(_) => 0x18,
        }
    }

    pub fn sub_cmd_byte(&self) -> u8 {
        match self {
            Self::Nfc(s) => *s as u8,
            Self::Spi(s) => *s as u8,
            Self::Init(s) => *s as u8,
            Self::Power(s) => *s as u8,
            Self::Cmd07(s) => *s as u8,
            Self::Led(s) => *s as u8,
            Self::Haptics(s) => *s as u8,
            Self::Battery(s) => *s as u8,
            Self::Features(s) => *s as u8,
            Self::FirmwareInfo(s) => *s as u8,
            Self::Cmd11(s) => *s as u8,
            Self::Pairing(s) => *s as u8,
            Self::Cmd16(s) => *s as u8,
            Self::Cmd18(s) => *s as u8,
            // No sub-command
            Self::Cmd04
            | Self::Cmd05
            | Self::ChargingGrip
            | Self::FirmwareUpdate
            | Self::Cmd0E
            | Self::Cmd0F
            | Self::Cmd12
            | Self::Cmd13
            | Self::Cmd14
            | Self::Cmd17 => 0x00,
        }
    }

    pub fn from_bytes(cmd: u8, sub: u8) -> Result<Self, PacketError> {
        match cmd {
            0x01 => NfcSubCmd::try_from(sub).map(Self::Nfc),
            0x02 => SpiSubCmd::try_from(sub).map(Self::Spi),
            0x03 => InitSubCmd::try_from(sub).map(Self::Init),
            0x04 => Ok(Self::Cmd04),
            0x05 => Ok(Self::Cmd05),
            0x06 => PowerSubCmd::try_from(sub).map(Self::Power),
            0x07 => Cmd07SubCmd::try_from(sub).map(Self::Cmd07),
            0x08 => Ok(Self::ChargingGrip),
            0x09 => LedSubCmd::try_from(sub).map(Self::Led),
            0x0A => HapticsSubCmd::try_from(sub).map(Self::Haptics),
            0x0B => BatterySubCmd::try_from(sub).map(Self::Battery),
            0x0C => FeaturesSubCmd::try_from(sub).map(Self::Features),
            0x0D => Ok(Self::FirmwareUpdate),
            0x0E => Ok(Self::Cmd0E),
            0x0F => Ok(Self::Cmd0F),
            0x10 => FirmwareInfoSubCmd::try_from(sub).map(Self::FirmwareInfo),
            0x11 => Cmd11SubCmd::try_from(sub).map(Self::Cmd11),
            0x12 => Ok(Self::Cmd12),
            0x13 => Ok(Self::Cmd13),
            0x14 => Ok(Self::Cmd14),
            0x15 => PairingSubCmd::try_from(sub).map(Self::Pairing),
            0x16 => Cmd16SubCmd::try_from(sub).map(Self::Cmd16),
            0x17 => Ok(Self::Cmd17),
            0x18 => Cmd18SubCmd::try_from(sub).map(Self::Cmd18),
            _ => Err(PacketError::UnknownCommand(cmd)),
        }
    }
}

// SubCommands
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NfcSubCmd {
    Unknown0C = 0x0C,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiSubCmd {
    ReadMemory = 0x01,
    WriteMemory = 0x02,
    EraseMemory = 0x03,
    MemoryRead = 0x04,
    MemoryWrite = 0x05,
    Unknown06 = 0x06,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitSubCmd {
    WakeConsole = 0x01,
    StorePairing = 0x07,
    UsbInit = 0x0D,
    InputReport = 0x0A,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerSubCmd {
    Shutdown = 0x02,
    Reboot = 0x03,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cmd07SubCmd {
    Init = 0x01,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedSubCmd {
    Player1 = 0x01,
    Player2 = 0x02,
    Player3 = 0x03,
    Player4 = 0x04,
    AllOn = 0x05,
    AllOff = 0x06,
    SetPattern = 0x07,
    SetFlash = 0x08,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HapticsSubCmd {
    PlaySample = 0x02,
    SendData = 0x08,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatterySubCmd {
    GetVoltage = 0x03,
    GetCharge = 0x04,
    Unknown07 = 0x07,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeaturesSubCmd {
    GetInfo = 0x01,
    SetMask = 0x02,
    ClearMask = 0x03,
    Enable = 0x04,
    Disable = 0x05,
    Configure = 0x06,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirmwareInfoSubCmd {
    GetVersion = 0x01,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PairingSubCmd {
    SetAddress = 0x01,
    ConfirmLtk = 0x02,
    Finalize = 0x03,
    ExchangeKeys = 0x04,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cmd18SubCmd {
    Unknown01 = 0x01,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cmd11SubCmd {
    Unknown03 = 0x03,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cmd16SubCmd {
    Unknown01 = 0x01,
}

// Command Implementation
macro_rules! impl_try_from_u8 {
    ($t:ty, $($val:expr => $variant:expr),+ $(,)?) => {
        impl TryFrom<u8> for $t {
            type Error = PacketError;
            fn try_from(v: u8) -> Result<Self, Self::Error> {
                match v {
                    $($val => Ok($variant)),+,
                    _ => Err(PacketError::UnknownSubCommand(v)),
                }
            }
        }
    };
}

impl_try_from_u8!(NfcSubCmd,
    0x0C => NfcSubCmd::Unknown0C,
);
impl_try_from_u8!(SpiSubCmd,
    0x01 => SpiSubCmd::ReadMemory,
    0x02 => SpiSubCmd::WriteMemory,
    0x03 => SpiSubCmd::EraseMemory,
    0x04 => SpiSubCmd::MemoryRead,
    0x05 => SpiSubCmd::MemoryWrite,
    0x06 => SpiSubCmd::Unknown06
);
impl_try_from_u8!(InitSubCmd,
    0x01 => InitSubCmd::WakeConsole,
    0x07 => InitSubCmd::StorePairing,
    0x0A => InitSubCmd::InputReport,
    0x0D => InitSubCmd::UsbInit,
);
impl_try_from_u8!(PowerSubCmd,
    0x02 => PowerSubCmd::Shutdown,
    0x03 => PowerSubCmd::Reboot,
);
impl_try_from_u8!(Cmd07SubCmd,
    0x01 => Cmd07SubCmd::Init,
);
impl_try_from_u8!(LedSubCmd,
    0x01 => LedSubCmd::Player1,
    0x02 => LedSubCmd::Player2,
    0x03 => LedSubCmd::Player3,
    0x04 => LedSubCmd::Player4,
    0x05 => LedSubCmd::AllOn,
    0x06 => LedSubCmd::AllOff,
    0x07 => LedSubCmd::SetPattern,
    0x08 => LedSubCmd::SetFlash,
);
impl_try_from_u8!(HapticsSubCmd,
    0x02 => HapticsSubCmd::PlaySample,
    0x08 => HapticsSubCmd::SendData,
);
impl_try_from_u8!(BatterySubCmd,
    0x03 => BatterySubCmd::GetVoltage,
    0x04 => BatterySubCmd::GetCharge,
    0x07 => BatterySubCmd::Unknown07,
);
impl_try_from_u8!(FeaturesSubCmd,
    0x01 => FeaturesSubCmd::GetInfo,
    0x02 => FeaturesSubCmd::SetMask,
    0x03 => FeaturesSubCmd::ClearMask,
    0x04 => FeaturesSubCmd::Enable,
    0x05 => FeaturesSubCmd::Disable,
    0x06 => FeaturesSubCmd::Configure,
);
impl_try_from_u8!(FirmwareInfoSubCmd,
    0x01 => FirmwareInfoSubCmd::GetVersion,
);
impl_try_from_u8!(Cmd11SubCmd,
    0x03 => Cmd11SubCmd::Unknown03,
);
impl_try_from_u8!(PairingSubCmd,
    0x01 => PairingSubCmd::SetAddress,
    0x02 => PairingSubCmd::ConfirmLtk,
    0x03 => PairingSubCmd::Finalize,
    0x04 => PairingSubCmd::ExchangeKeys,
);
impl_try_from_u8!(Cmd16SubCmd,
    0x01 => Cmd16SubCmd::Unknown01,
);
impl_try_from_u8!(Cmd18SubCmd,
    0x01 => Cmd18SubCmd::Unknown01,
);

// Transport / Request Types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TransportType {
    #[default]
    Usb = 0x00,
    Ble = 0x01,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketStatus {
    Success = 0x01,
    Failure = 0x04,
    Request = 0x91,
}

// Header
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    pub command: Command,
    pub packet_status: PacketStatus,
    pub transport_type: TransportType,
    pub data_length: u8,
}

impl Header {
    pub const SIZE: usize = 8;

    fn to_bytes(&self) -> [u8; Self::SIZE] {
        [
            self.command.cmd_byte(),
            self.packet_status as u8,
            self.transport_type as u8,
            self.command.sub_cmd_byte(),
            0x00,
            self.data_length,
            0x00,
            0x00,
        ]
    }

    fn from_bytes(b: &[u8; Self::SIZE]) -> Result<Self, PacketError> {
        let command = Command::from_bytes(b[0], b[3])?;

        let request_type = match b[1] {
            0x91 => PacketStatus::Request,
            0x04 => PacketStatus::Failure,
            0x01 => PacketStatus::Success,
            v => {
                return Err(PacketError::InvalidRequestType(v));
            }
        };

        let transport_type = match b[2] {
            0x00 => TransportType::Usb,
            0x01 => TransportType::Ble,
            v => return Err(PacketError::InvalidTransportType(v)),
        };

        Ok(Self {
            command,
            packet_status: request_type,
            transport_type,
            data_length: b[5],
        })
    }
}

// Packet
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    pub header: Header,
    pub data: Vec<u8>,
}

impl Packet {
    pub fn new(
        command: Command,
        transport_type: TransportType,
        data: Vec<u8>,
    ) -> Result<Self, PacketError> {
        let data_length =
            u8::try_from(data.len()).map_err(|_| PacketError::DataTooLong(data.len()))?;

        Ok(Self {
            header: Header {
                command,
                packet_status: PacketStatus::Request,
                transport_type,
                data_length,
            },
            data,
        })
    }

    pub fn from_response(bytes: &[u8]) -> Result<Self, PacketError> {
        if bytes.len() < Header::SIZE {
            return Err(PacketError::BufferTooShort {
                expected: Header::SIZE,
                got: bytes.len(),
            });
        }

        let header = Header::from_bytes(
            bytes[..Header::SIZE]
                .try_into()
                .expect("slice is exactly Header::SIZE"),
        )?;

        let data = bytes[Header::SIZE..].to_vec();

        Ok(Self { header, data })
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(Header::SIZE + self.data.len());
        buf.extend_from_slice(&self.header.to_bytes());
        buf.extend_from_slice(&self.data);
        buf
    }
}

impl From<PacketError> for std::io::Error {
    fn from(e: PacketError) -> Self {
        std::io::Error::other(e)
    }
}

// Errors
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PacketError {
    BufferTooShort { expected: usize, got: usize },
    DataLengthMismatch { expected: usize, got: usize },
    DataTooLong(usize),
    UnknownCommand(u8),
    UnknownSubCommand(u8),
    InvalidRequestType(u8),
    InvalidTransportType(u8),
}

impl fmt::Display for PacketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferTooShort { expected, got } => {
                write!(f, "buffer too short: need {expected}, got {got}")
            }
            Self::DataLengthMismatch { expected, got } => {
                write!(f, "data length mismatch: header says {expected}, got {got}")
            }
            Self::DataTooLong(n) => write!(f, "data too long: {n} bytes exceeds u8::MAX"),
            Self::UnknownCommand(v) => write!(f, "unknown command byte: {v:#04x}"),
            Self::UnknownSubCommand(v) => write!(f, "unknown sub-command byte: {v:#04x}"),
            Self::InvalidRequestType(v) => write!(f, "invalid request type byte: {v:#04x}"),
            Self::InvalidTransportType(v) => write!(f, "invalid transport type byte: {v:#04x}"),
        }
    }
}
impl std::error::Error for PacketError {}
