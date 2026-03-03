# ConHid

ConHid is a program intending to bring support to Linux for the Switch 2 generation of Nintendo controllers. This includes:

- Pro Controller 2
- Joy Con 2
- GameCube Controller for Switch 2 

## Current Limitations
The limitations right now come from the driver being in userspace. Being in userspace makes it easier to produce a workable result, but prevents us from having working rumble support since it is not possible to perform. The two methods we have at performing in userspace is uinput and uhid. uhid is what the driver currently uses which provides important adjustments to controller data, before passing the HID packet to ``hid-generic``.

uinput is also a promising system to use however the issues with it is that it is janky.

Moving to a kernel module fixes all these issues.

## Intended Implementation
I intend to bring this to support the entire Switch 2 generation of controllers, and these subsets of features:

- Buttons, Joysticks (basic)
- IMU (Gyroscope, Magnetometer (Joy Con 2), Accelerometer)
- Mouse Input (Joy Con 2)
- Haptics / Rumble

Some future ideas which I have not verified, but could be possible:

- NFC
- Audio (Pro Controller 2)

## Packets

Each controller seems to have varying packet types.

### Pro Controller

This one is the most interesting as it appears to have two distinct communication protocols over each transport layer (BTLE and USB)

BTLE Packet:    ``XX 88 20 00 00 00 09 48 89 28 18 7E 38 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 28 92 00 03 0F 83 C4 D5 8F CD 33 F1 F8 06 09 40 47 7A 01 DE 37 E8 BB 14 41 D9 5F 40 3E A2 CE 9C 25 1D A2 60 E3 0F 78 B0 0B 00 00 00 00 00 00``

USB Packet:     ``09 41 23 00 00 00 10 18 8B 1C 88 7F 38 00 00 1E 9F 39 00 0C 00 F8 79 03 02 33 A9 F2 00 6B 6F 56 78 55 90 FB 38 0A 1E 06 60 F0 2C 0E 30 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00``

The Pro Controller also has two seperate modes regarding individual packets. One of them is basic report mode (on USB, known is 0x30), and on BTLE, is the 63-byte packet. Basic report mode seems to report information as expected however IMU data is encoded as 16-bit integers. In full report mode (0x09 on USB, 112-byte packet on BTLE) the IMU data is instead encoded as 32-bit LE integers. The 1G for the accelerometer is determined to be 2^28 in this mode.

### Joy Con

BTLE Packet:    ``XX E8 18 00 00 07 4F 88 7E 38 00 00 00 00 FF 00 28 EF 74 02 0F 63 DB FC 79 59 48 58 55 B6 D3 DB 04 08 8F 1A 41 33 FC 36 40 12 17 F9 C3 E6 03 A0 F2 5D 87 E1 08 47 EB A5 1D 00 00 00 00 00 00 00``

### Switch 2 Pro Controller - USB Full Report (0x09)
Packet Size: 64-bytes (USB)
IMU Encoding: 32-bit Little Endian (1G = 2^28)

|Offset     |Size   |Field          | Description                   |
|-          |-      |-              |-                              |
|``0x00``   |1      |Report ID      |0x09 for full                  |
|``0x01``   |1      |Packet ID      |Incremental sequence counter   |
|``0x02``   |1      |Status?        |Connection status?             |
|``0x03``   |3      |Buttons        |Bitmask of 3-bytes             |
|``0x06``   |3      |Left Stick     |12-bit X, 12-bit Y Packed      |
|``0x09``   |3      |Right Stick    |12-bit X, 12-bit Y Packed      |
|``0x0C``   |1      |Vib Code       |0x38 when no, 0x30 when yes    |
|``0x0F``   |1      |IMU Len?       |IMU data length?               |
|``0x15``   |24     |IMU Data       |6axes x 4bytes (int32_t LE)    |

### Switch 2 Pro Controller - BTLE Full Report (RECV 002E)
Packet Size: 112-bytes
IMU Encoding: 32-bit Little Endian (1G = 2^28)

|Offset     |Size   |Field          | Description                   | Type              |
|-          |-      |-              |-                              |-                  |
|``0x00``   |1      |Packet ID      |Incremental sequence counter   |``uint8_t``        |
|``0x01``   |1      |Status?        |Connection status?             |``uint8_t``        |
|``0x02``   |3      |Buttons        |Digital Button Bitmask         |``uint24_t LE``    |
|``0x05``   |3      |Left Stick     |12-bit X, 12-bit Y Packed      |``uint8_t[3]``     |
|``0x08``   |3      |Right Stick    |12-bit X, 12-bit Y Packed      |``uint8_t[3]``     |
|``0x0B``   |1      |Vib Code       |0x38 when no, 0x30 when yes    |``uint8_t``        |
|``0x40``   |1      |IMU Length     |Size of following data         |``uint8_t``        |
|``0x41``   |4      |IMU Clock      |High-speed timestamp           |``uint32_t LE``    |
|``0x45``   |4      |Accel X        |Linear Acceleration            |``int32_t LE``     |
|``0x49``   |4      |Accel Y        |Linear Acceleration            |``int32_t LE``     |
|``0x4D``   |4      |Accel Z        |Linear Acceleration            |``int32_t LE``     |
|``0x51``   |4      |Gyro  X        |Rotational Velocity (Pitch)    |``int32_t LE``     |
|``0x55``   |4      |Gyro  Y        |Rotational Velocity (Yaw)      |``int32_t LE``     |
|``0x59``   |4      |Gyro  Z        |Rotational Velocity (Roll)     |``int32_t LE``     |



### Switch 2 Joy Con - BTLE Report (RECV 000E)
Packet Size: 63-bytes (BTLE)
IMU Encoding: 32-bit Little Endian (1G = 2^28)

|Offset     |Size   |Field          | Description                   | Type              |
|-          |-      |-              |-                              |-                  |
|``0x00``   |1      |Packet ID      |Incremental Sequence Counter   |``uint8_t``        |
|``0x01``   |1      |Status?        |Connection status              |``uint8_t``        |
|``0x02``   |2      |Buttons        |Digital Button Bitmask         |``uint16_t LE``    |
|``0x05``   |3      |Right Stick    |12-bit X, 12-bit Y Packed      |``uint8_t[3]``     |
|``0x08``   |1      |Vib Code       |0x38 when no, 0x30 when yes    |``uint8_t``        |
|``0x09``   |2      |Mouse X        |Relative Horizontal Move       |``uint16_t LE``    |
|``0x0B``   |2      |Mouse Y        |Relative Vertical Move         |``uint16_t LE``    |
|``0x0D``   |1      |Mouse Dist     |Distance to surface            |``uint8_t``        |
|``0x0F``   |1      |IMU Length     |Size of following data         |``uint8_t``        |
|``0x10``   |4      |IMU Clock      |High-speed timestamp           |``uint32_t LE``    |
|``0x14``   |4      |Accel X        |Linear Acceleration            |``int32_t LE``     |
|``0x18``   |4      |Accel Y        |Linear Acceleration            |``int32_t LE``     |
|``0x1C``   |4      |Accel Z        |Linear Acceleration            |``int32_t LE``     |
|``0x20``   |4      |Gyro  X        |Rotational Velocity (Pitch)    |``int32_t LE``     |
|``0x24``   |4      |Gyro  Y        |Rotational Velocity (Yaw)      |``int32_t LE``     |
|``0x28``   |4      |Gyro  Z        |Rotational Velocity (Roll)     |``int32_t LE``     |
|``0x2C``   |4      |Magn  X        |Magnetic Field Strength        |``int32_t LE``     |
|``0x30``   |4      |Magn  Y        |Magnetic Field Strength        |``int32_t LE``     |
|``0x34``   |4      |Magn  Z        |Magnetic Field Strength        |``int32_t LE``     |