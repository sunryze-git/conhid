#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------
// Raw SPI calibration data as read from the controller flash.
// Populated during the init sequence from SPI READ responses.
// -----------------------------------------------------------------------

typedef struct {
    // Stick factory calibration — SPI 0x0130A8 (left) / 0x0130E8 (right)
    // Each is 11 bytes: center_x/y, max_x/y, min_x/y packed as 12-bit pairs.
    // Zero until the corresponding SPI read completes.
    uint8_t stick_l_raw[11];
    uint8_t stick_r_raw[11];

    // Motion factory calibration — SPI 0x013100 (0x18 bytes)
    // Bytes 0-11:  int16 bias offsets: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    // Bytes 12-23: float scale factors: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    uint8_t motion_raw[24];

    // User calibration magic check — SPI 0x1FC040 first 2 bytes.
    // If == 0xB2A1, user_stick_l_raw / user_stick_r_raw are valid.
    uint16_t user_magic;
    uint8_t  user_stick_l_raw[11];
    uint8_t  user_stick_r_raw[11];

    // Which fields have been filled in
    int stick_l_loaded;
    int stick_r_loaded;
    int motion_loaded;
    int user_loaded;
} Sw2CalibrationRaw;

// -----------------------------------------------------------------------
// Processed calibration — ready to use at report time.
// -----------------------------------------------------------------------

typedef struct {
    // Stick axes — 12-bit raw space
    uint16_t l_center_x, l_center_y;
    uint16_t l_max_x,    l_max_y;
    uint16_t l_min_x,    l_min_y;

    uint16_t r_center_x, r_center_y;
    uint16_t r_max_x,    r_max_y;
    uint16_t r_min_x,    r_min_y;

    // IMU
    int16_t  accel_offset[3];  // subtract from raw before scaling
    int16_t  gyro_offset[3];
    float    accel_scale[3];   // multiply after offset
    float    gyro_scale[3];

    int valid;  // 0 until Parse() succeeds
} Sw2Calibration;

// -----------------------------------------------------------------------
// Parse raw SPI bytes into the processed struct.
// Call after all SPI reads have completed.
// If user calibration is present it takes priority over factory.
// -----------------------------------------------------------------------
void Calibration_Parse(const Sw2CalibrationRaw* raw, Sw2Calibration* out);

// -----------------------------------------------------------------------
// Normalize a raw 12-bit stick axis to a signed 16-bit value.
// Returns 0 if calibration is not valid.
// -----------------------------------------------------------------------
int16_t Calibration_NormalizeAxis(const Sw2Calibration* cal,
                                   uint16_t raw,
                                   uint16_t center,
                                   uint16_t max_val,
                                   uint16_t min_val);

// Convenience wrappers that pick the right center/max/min automatically.
int16_t Calibration_NormalizeLX(const Sw2Calibration* cal, uint16_t raw);
int16_t Calibration_NormalizeLY(const Sw2Calibration* cal, uint16_t raw);
int16_t Calibration_NormalizeRX(const Sw2Calibration* cal, uint16_t raw);
int16_t Calibration_NormalizeRY(const Sw2Calibration* cal, uint16_t raw);

// Scale a raw IMU int32 sample to a calibrated int16 for HID output.
int16_t Calibration_ScaleAccel(const Sw2Calibration* cal, int axis, int32_t raw);
int16_t Calibration_ScaleGyro (const Sw2Calibration* cal, int axis, int32_t raw);

// -----------------------------------------------------------------------
// SPI address constants (for use in initializer)
// -----------------------------------------------------------------------
#define SPI_ADDR_FACTORY_CAL_L    0x0130A8  // 11 bytes
#define SPI_ADDR_FACTORY_CAL_R    0x0130E8  // 11 bytes
#define SPI_ADDR_FACTORY_MOTION   0x013100  // 24 bytes
#define SPI_ADDR_USER_CAL_L       0x1FC040  // 11 bytes (check magic first)
#define SPI_ADDR_USER_CAL_R       0x1FC060  // 11 bytes
#define SPI_CAL_USER_MAGIC        0xB2A1

#ifdef __cplusplus
}
#endif