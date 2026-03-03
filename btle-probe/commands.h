// Switch 2 Controller Command Definitions
// Derived from USB/Bluetooth protocol analysis and firmware research.
//
// Command Header Structure (8 bytes at offset 0x0):
//   [0x0] 0x1  Command ID
//   [0x1] 0x1  Direction: REQUEST_MARKER (0x91) = Host->Device, 0x01 = Device->Host
//   [0x2] 0x1  Transport: COMM_USB (0x00) or COMM_BLE (0x01)
//   [0x3] 0x1  Subcommand ID
//   [0x4] 0x1  Unknown
//   [0x5] 0x1  Data Length (request) / ACK (response)
//   [0x6] 0x2  Reserved (always 0x0000)

#pragma once

// Direction / transport markers
#define REQUEST_MARKER      0x91  // Host -> Device
#define RESPONSE_MARKER     0x01  // Device -> Host

#define COMM_USB            0x00
#define COMM_BLE            0x01

// ---------------------------------------------------------------------------
// COMMAND IDs
// ---------------------------------------------------------------------------

#define NFC                 0x01  // NFC          -- NFC read/write commands
#define SPI                 0x02  // Flash Memory -- Internal flash read/write/erase
#define INIT                0x03  // Init         -- Initialisation; USB init, input report selection, pairing bypass
#define CMD_04              0x04  // Unknown      -- Possibly unused
#define CMD_05              0x05  // Unknown
#define POWER               0x06  // Power        -- Shutdown/reboot controller
#define CMD_07              0x07  // Unknown      -- First command sent during initialisation
#define CHARGING_GRIP       0x08  // ChargingGrip -- Charging grip info & button enable
#define LED                 0x09  // PlayerLEDs   -- Set player LED pattern
#define HAPTICS             0x0A  // Vibration    -- Play vibration/sound presets or raw data
#define BATTERY             0x0B  // Battery      -- Battery voltage & charge status
#define FEATURES            0x0C  // FeatureSelect-- Enable/disable report features (IMU, mouse, magnetometer etc.)
#define FIRMWARE            0x0D  // FirmwareUpdate
#define CMD_0E              0x0E  // Unknown
#define CMD_0F              0x0F  // Unknown
#define FIRMWARE_INFO       0x10  // FirmwareInfo -- Read firmware version information
#define CMD_11              0x11  // Unknown
#define CMD_12              0x12  // Unknown      -- Seems to be unused
#define CMD_13              0x13  // Unknown      -- JoyCon only
#define CMD_14              0x14  // Unknown
#define PAIRING             0x15  // BTPairing    -- Custom BLE pairing (address exchange, LTK derivation)
#define CMD_16              0x16  // Unknown
#define CMD_17              0x17  // Unknown
#define CMD_18              0x18  // Unknown

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x01 (NFC)
// ---------------------------------------------------------------------------

#define NFC_UNKNOWN_01          0x01
#define NFC_UNKNOWN_02          0x02
#define NFC_UNKNOWN_03          0x03  // Known request: 01 91 00 03 00 05 00 00 | 00 e8 03 2c 01
#define NFC_UNKNOWN_04          0x04  // Known request: 01 91 00 04 00 00 00 00
#define NFC_STATE               0x05  // Get current MCU/NFC device state
#define NFC_READ_DEVICE         0x06  // Send read properties
#define NFC_WIRE_DEVICE         0x08  // Send write properties
#define NFC_UNKNOWN_0C          0x0C  // Response contains 4 bytes of unknown data
#define NFC_READ_BUFFER         0x14  // Write MCU buffer (note: formerly named MCU_WRITE_BUFFER)
#define NFC_WRITE_BUFFER        0x15  // Read MCU buffer with offset; response contains 4+ bytes

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x02 (Flash Memory / SPI)
// ---------------------------------------------------------------------------
// Max USB read: 0x50 bytes, BLE: 0x4F bytes. Addresses >= 0x200000 wrap to 0.
// Write permissions only for addresses >= 0x1F5000 (SPI_WRITE_MEM).
// Block ops (SPI_READ_BLOCK / SPI_WRITE_BLOCK) always transfer 0x40 bytes.
// Erase (SPI_ERASE_SECTOR) erases a 0x1000-byte sector, setting all to 0xFF.

#define SPI_READ_BLOCK          0x01  // Read  0x40-byte block from flash
#define SPI_WRITE_BLOCK         0x02  // Write 0x40-byte block to flash
#define SPI_ERASE_SECTOR        0x03  // Erase 0x1000-byte sector (sets all to 0xFF)
#define SPI_READ                0x04  // Read  up to 0x50 bytes (USB) / 0x4F bytes (BLE)
#define SPI_WRITE               0x05  // Write up to 0x80 bytes (addr must be >= 0x1F5000)
#define SPI_UNKNOWN_06          0x06  // Unknown

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x03 (Init)
// ---------------------------------------------------------------------------
// INIT_WAKE_CONSOLE wakes the console via BLE when argument is nonzero.
// INIT_STORE_PAIRING bypasses 0x15 pairing; writes BT host address + LTK directly.
// INIT_USB_INIT     required before the controller will send USB input reports.
// INIT_INPUT_REPORT selects HID input report format (default: 0x07/08/09/0A).

#define INIT_WAKE_CONSOLE       0x01  // Wake console via BLE (if arg != 0)
#define INIT_UNKNOWN_02         0x02
#define INIT_STORE_PAIRING      0x07  // Write BT host address (6B, rev) + LTK (16B, rev) directly
#define INIT_UNKNOWN_08         0x08
#define INIT_UNKNOWN_09         0x09
#define INIT_INPUT_REPORT       0x0A  // Select HID input report ID (0x05 or 0x07/08/09/0A)
#define INIT_UNKNOWN_0C         0x0C
#define INIT_USB_INIT           0x0D  // Initialise USB; supply host BT address (byte-reversed)
#define INIT_UNKNOWN_0F         0x0F

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x06 (Power / Controller Control)
// ---------------------------------------------------------------------------
// POWER_SHUTDOWN is sent to USB controllers when the console sleeps (docked).
// POWER_REBOOT   is sent after a firmware update to reload firmware.

#define POWER_UNKNOWN_01       0x01
#define POWER_SHUTDOWN         0x02  // Shutdown controller (called on USB sleep)
#define POWER_REBOOT           0x03  // Reboot / reload firmware

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x07 (Unknown)
// ---------------------------------------------------------------------------
// CMD_07_INIT is the very first command sent during the initialisation sequence.

#define CMD_07_INIT             0x01  // First init command; response: 1 byte (boolean?)
#define CMD_07_UNKNOWN_02       0x02

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x08 (Charging Grip)
// ---------------------------------------------------------------------------
// Data mirrors factory data at flash offset 0x13000–0x14FFF.

#define GRIP_GET_INFO_20        0x01  // Get 0x20 bytes of charging grip data
#define GRIP_ENABLE_BUTTONS     0x02  // Enable GL/GR buttons on the grip (boolean arg)
#define GRIP_GET_INFO_40        0x03  // Get 0x40 bytes of charging grip data

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x09 (Player LEDs)
// ---------------------------------------------------------------------------

#define LED_PLAYER_1            0x01  // Player 1 LED on, others off  (equiv. LED_SET_PATTERN 0b0001)
#define LED_PLAYER_2            0x02  // Player 2 LED on, others off  (equiv. LED_SET_PATTERN 0b0010)
#define LED_PLAYER_3            0x03  // Player 3 LED on, others off  (equiv. LED_SET_PATTERN 0b0100)
#define LED_PLAYER_4            0x04  // Player 4 LED on, others off  (equiv. LED_SET_PATTERN 0b1000)
#define LED_ALL_ON              0x05  // All LEDs on                  (equiv. LED_SET_PATTERN 0b1111)
#define LED_ALL_OFF             0x06  // All LEDs off                 (equiv. LED_SET_PATTERN 0b0000)
#define LED_SET_PATTERN         0x07  // Set LED pattern via bitmask (bits 0-3 = LEDs 1-4)
#define LED_SET_FLASH           0x08  // Flash current LED pattern (boolean arg)

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x0A (Vibration)
// ---------------------------------------------------------------------------

#define VIBRATION_UNKNOWN_01    0x01
#define VIBRATION_PLAY_SAMPLE   0x02  // Play preset vibration/sound sample by ID
#define VIBRATION_UNKNOWN_03    0x03
#define VIBRATION_UNKNOWN_04    0x04
#define VIBRATION_UNKNOWN_05    0x05
#define VIBRATION_UNKNOWN_06    0x06
#define VIBRATION_UNKNOWN_07    0x07
#define VIBRATION_SEND_DATA     0x08  // Send raw vibration data (0x14 bytes; format TBD)
#define VIBRATION_UNKNOWN_09    0x09

// Vibration sample preset IDs (used with VIBRATION_PLAY_SAMPLE)
#define VIBRATION_NONE          0x00  // Silence / stop current playback
#define VIBRATION_BUZZ          0x01  // Low-frequency buzz ~1s
#define VIBRATION_FIND          0x02  // High-freq buzz + beep-beep ("Search For Controllers")
#define VIBRATION_CONNECT       0x03  // Soft click-click (connection sound)
#define VIBRATION_PAIRING       0x04  // High-to-higher beep-beep (low battery?)
#define VIBRATION_STRONG_THUNK  0x05  // Like VIBRATION_CONNECT but stronger
#define VIBRATION_DUN           0x06  // Short high-frequency beep
#define VIBRATION_DING          0x07  // Short higher-frequency beep

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x0B (Battery)
// ---------------------------------------------------------------------------

#define BATTERY_GET_VOLTAGE     0x03  // Returns current battery voltage in mV (2 bytes LE)
#define BATTERY_GET_CHARGE      0x04  // Returns charge status (2 bytes) + flags (2 bytes)
#define BATTERY_UNKNOWN_06      0x06  // Response: 0x00000011
#define BATTERY_UNKNOWN_07      0x07

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x0C (Feature Select)
// ---------------------------------------------------------------------------
// Workflow: SET_MASK -> ENABLE/DISABLE features -> optionally CONFIGURE.
// Feature flags outside the current mask are ignored by ENABLE/DISABLE.

#define FEATURE_GET_INFO        0x01  // Query feature info for given flags bitmask
#define FEATURE_SET_MASK        0x02  // Set feature mask (must call before ENABLE/DISABLE)
#define FEATURE_CLEAR_MASK      0x03  // Clear feature mask (all features disabled until re-set)
#define FEATURE_ENABLE          0x04  // Enable features by flags bitmask
#define FEATURE_DISABLE         0x05  // Disable features by flags bitmask
#define FEATURE_CONFIGURE       0x06  // Configure feature parameters

// Feature flag bitmasks (used with all 0x0C subcommands)
#define FEATURE_BUTTON          0x01  // Button state reporting
#define FEATURE_STICK           0x02  // Analog stick reporting
#define FEATURE_MOTION          0x04  // IMU: linear accelerometer + gyro
#define FEATURE_UNKNOWN_08      0x08  // Unused
#define FEATURE_MOUSE           0x10  // Mouse data (JoyCon only)
#define FEATURE_CURRENT         0x20  // Rumble / battery current (disabling doesn't affect BLE)
#define FEATURE_UNKNOWN_40      0x40  // Unused
#define FEATURE_MAGNETOMETER    0x80  // Magnetometer
#define FEATURE_ALL             0xFF  // All features

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x0D (Firmware Update)
// ---------------------------------------------------------------------------

#define FIRMWARE_UNKNOWN_01     0x01  // Init?
#define FIRMWARE_UNKNOWN_02     0x02
#define FIRMWARE_PROPERTIES     0x03  // Full firmware size info
#define FIRMWARE_DATA           0x04  // Firmware data in 0x4C-byte chunks
#define FIRMWARE_UNKNOWN_05     0x05
#define FIRMWARE_UNKNOWN_06     0x06
#define FIRMWARE_UNKNOWN_07     0x07  // Finalize? (reboot via POWER_REBOOT after this)

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x10 (Firmware Info)
// ---------------------------------------------------------------------------
// Response layout:
//   [0x0] 3B  Controller firmware version (major.minor.micro)
//   [0x3] 1B  Controller type: 0x00=JoyCon-L, 0x01=JoyCon-R, 0x02=Pro, 0x03=Gamecube
//   [0x4] 3B  Bluetooth patch version (major.minor.micro)
//   [0x7] 1B  Padding
//   [0x8] 3B  DSP firmware version (Pro Controller with updated FW only)

#define FIRMWARE_GET_VERSION    0x01  // Read firmware/BT/DSP version and controller type

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x11 (Unknown)
// ---------------------------------------------------------------------------

#define CMD_11_UNKNOWN_01       0x01
#define CMD_11_UNKNOWN_03       0x03
#define CMD_11_UNKNOWN_04       0x04

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x13 (Unknown -- JoyCon only)
// ---------------------------------------------------------------------------

#define CMD_13_UNKNOWN_01       0x01
#define CMD_13_UNKNOWN_02       0x02
#define CMD_13_UNKNOWN_03       0x03

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x14 (Unknown)
// ---------------------------------------------------------------------------

#define CMD_14_UNKNOWN_01       0x01
#define CMD_14_UNKNOWN_02       0x02

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x15 (Bluetooth Pairing)
// ---------------------------------------------------------------------------
// Custom pairing sequence instead of standard BLE SMP.
// Required for auto-reconnect and wake-from-sleep to work.
//
// Sequence:
//   1. PAIRING_EXCHANGE_KEYS  -- derive LTK as: LTK = host_key (A1) XOR device_key (B1)
//   2. PAIRING_SET_ADDRESS    -- exchange BT adapter addresses
//   3. PAIRING_CONFIRM_LTK   -- AES-128-ECB(LTK, challenge_A2) = response_B2
//   4. PAIRING_FINALIZE       -- commit addresses + LTK to controller memory
//
// Device key (B1) is always: 5C F6 EE 79 2C DF 05 E1 BA 2B 63 25 C4 1A 5F 10

#define PAIRING_SET_ADDRESS     0x01  // Exchange BT adapter addresses (host sends list, device responds)
#define PAIRING_CONFIRM_LTK     0x02  // Send AES-128 challenge; device returns encrypted response
#define PAIRING_FINALIZE        0x03  // Commit pairing (host address + LTK) to flash
#define PAIRING_EXCHANGE_KEYS   0x04  // Exchange 16-byte public keys to derive LTK

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x16 (Unknown)
// ---------------------------------------------------------------------------

#define CMD_16_UNKNOWN_01       0x01  // Response: 0x18 bytes (all 0x00)

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x17 (Unknown)
// ---------------------------------------------------------------------------

#define CMD_17_UNKNOWN_02       0x02  // Request: 7 bytes unknown data

// ---------------------------------------------------------------------------
// SUBCOMMANDS -- cmdId 0x18 (Unknown)
// ---------------------------------------------------------------------------

#define CMD_18_UNKNOWN_01       0x01  // Response: 8 bytes unknown
#define CMD_18_UNKNOWN_02       0x02
#define CMD_18_UNKNOWN_03       0x03  // Echo: 1 byte request / 1 byte response
#define CMD_18_UNKNOWN_04       0x04