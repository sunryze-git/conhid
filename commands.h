#define REQUEST_MARKER  0x91
#define COMM_USB 0x00
#define COMM_BLE 0x01

// COMMANDS
#define MCU             0x01 // MCU          -- NFC read/write commands
#define SPI             0x02 // SPI          -- Flash memory read/write
#define INIT            0x03 // Init         -- Initialization, console serial/address exchange
#define REPORT_06       0x06 // Report06     -- Unknown
#define REPORT_07       0x07 // Report07     -- Unknown
#define REPORT_08       0x08 // Report08     -- Unknown
#define LED             0x09 // PlayerLights -- Set LED pattern
#define HAPTICS         0x0A // Vibration    -- Play vibration/sound presets
#define FEATURES        0x0C // Feature      -- Enable/disable features (IMU, mouse, magnetometer etc.)
#define FIRMWARE        0x0D // Firmware     -- Firmware update
#define REPORT_10       0x10 // Report10     -- Unknown
#define REPORT_11       0x11 // Report11     -- Unknown
#define PAIRING         0x15 // Pairing      -- Pairing data transfer
#define REPORT_16       0x16 // Report16     -- Unknown
#define REPORT_18       0x18 // Report18     -- Unknown

// SUBCOMMANDS
// cmdId 0x01 - MCU
#define MCU_UNKNOWN_02          0x02
#define MCU_UNKNOWN_03          0x03
#define MCU_UNKNOWN_04          0x04
#define MCU_STATE               0x05  // Return current MCU device state
#define MCU_READ_DEVICE         0x06  // Send read properties
#define MCU_WIRE_DEVICE         0x08  // Send write properties
#define MCU_UNKNOWN_0C          0x0C
#define MCU_READ_BUFFER         0x14  // Read MCU buffer
#define MCU_WRITE_BUFFER        0x15  // Write MCU buffer

// cmdId 0x02 - SPI
#define SPI_READ                0x04  // Read up to 0x40 bytes
#define SPI_WRITE               0x05  // Write up to 0x40 bytes

// cmdId 0x03 - Init
#define INIT_UNKNOWN_01         0x01
#define INIT_UNKNOWN_07         0x07  // Unknown, set final LTK key?
#define INIT_UNKNOWN_0A         0x0A
#define INIT_UNKNOWN_0C         0x0C
#define INIT_CONSOLE_ADDRESS    0x0D  // Has console address

// cmdId 0x06 - Report06
#define REPORT06_UNKNOWN_03     0x03

// cmdId 0x07 - Report07
#define REPORT07_UNKNOWN_01     0x01

// cmdId 0x08 - Report08
#define REPORT08_UNKNOWN_01     0x01
#define REPORT08_UNKNOWN_02     0x02

// cmdId 0x09 - PlayerLights
#define LED_SET_PATTERN         0x07  // Set LED pattern

// cmdId 0x0A - Vibration
#define VIBRATION_PLAY_SAMPLE   0x02  // Play preset sound/vibration
#define VIBRATION_UNKNOWN_08    0x08

// cmdId 0x0C - Feature
#define FEATURE_INIT            0x02  // Initialize feature
#define FEATURE_FINALIZE        0x03  // Finalize feature
#define FEATURE_ENABLE          0x04  // Enable feature
#define FEATURE_DISABLE         0x05  // Disable feature
#define FEATURE_UNKNOWN_06      0x06

// cmdId 0x0C - Feature // Feature list
#define FEATURE_BUTTON          0x01
#define FEATURE_STICK           0x02
#define FEATURE_MOTION          0x04
#define FEATURE_UNKNOWN_08      0x08
#define FEATURE_MOUSE           0x10
#define FEATURE_CURRENT         0x20
#define FEATURE_UNKNOWN_40      0x40
#define FEATURE_MAGNETOMETER    0x80

// cmdId 0x0D - Firmware
#define FIRMWARE_UNKNOWN_01     0x01  // Init?
#define FIRMWARE_UNKNOWN_02     0x02
#define FIRMWARE_PROPERTIES     0x03  // Full FW size info
#define FIRMWARE_DATA           0x04  // Firmware data in 0x4C chunks
#define FIRMWARE_UNKNOWN_05     0x05
#define FIRMWARE_UNKNOWN_06     0x06
#define FIRMWARE_UNKNOWN_07     0x07  // Finalize?

// cmdId 0x10 - Report10
#define REPORT10_UNKNOWN_01     0x01

// cmdId 0x11 - Report11
#define REPORT11_UNKNOWN_03     0x03

// cmdId 0x15 - Pairing
#define PAIRING_SET_ADDRESS     0x01
#define PAIRING_UNKNOWN_02      0x02
#define PAIRING_UNKNOWN_03      0x03
#define PAIRING_UNKNOWN_04      0x04

// cmdId 0x16 - Report16
#define REPORT16_UNKNOWN_01     0x01

// cmdId 0x18 - Report18
#define REPORT18_UNKNOWN_01     0x01

// Feature flags (used with cmdId 0x0C)
#define FEATURE_BUTTON          0x01
#define FEATURE_STICK           0x02
#define FEATURE_MOTION          0x04
#define FEATURE_UNKNOWN_08      0x08
#define FEATURE_MOUSE           0x10
#define FEATURE_CURRENT         0x20
#define FEATURE_UNKNOWN_40      0x40
#define FEATURE_MAGNETOMETER    0x80
#define FEATURE_ALL             0xFF

// Vibration sample presets (used with VIBRATION_PLAY_SAMPLE)
#define VIBRATION_NONE          0x00
#define VIBRATION_BUZZ          0x01  // 1s buzz
#define VIBRATION_FIND          0x02  // High pitch buzz + beep beep
#define VIBRATION_CONNECT       0x03  // Button click sound
#define VIBRATION_PAIRING       0x04  // Pairing sound
#define VIBRATION_STRONG_THUNK  0x05
#define VIBRATION_DUN           0x06  // Screen recording?
#define VIBRATION_DING          0x07  // Screen recording?