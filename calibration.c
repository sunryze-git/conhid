#include "calibration.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

// -----------------------------------------------------------------------
// 12-bit stick unpacking
// Nintendo packs two 12-bit values into 3 bytes: AAAA AAAA BBBB AAAA BBBB BBBB
// -----------------------------------------------------------------------

static void unpack_stick_cal(const uint8_t* d, uint16_t* x, uint16_t* y) {
    *x =  d[0]        | ((uint16_t)(d[1] & 0x0F) << 8);
    *y = (d[1] >> 4)  | ((uint16_t) d[2]          << 4);
}

// The 11-byte factory cal block layout for Pro Controller 2:
//   [0..2]  = center x/y          (absolute 12-bit value)
//   [3..5]  = max delta x/y       (offset added to center)
//   [6..8]  = min delta x/y       (offset subtracted from center)
//   [9..10] = deadzone (ignored)
//
// NOTE: Older Switch 1 documentation lists this as [max, center, min] but
// real Pro Controller 2 SPI data shows center first. Verified against
// live captures: center comes out at ~2149,2039 which is correct for a
// resting 12-bit ADC. The old order produced center=1146,1231 which is wrong.

static void parse_stick_block(const uint8_t* d,
                               uint16_t* center_x, uint16_t* center_y,
                               uint16_t* max_x,    uint16_t* max_y,
                               uint16_t* min_x,    uint16_t* min_y) {
    uint16_t cx, cy, dx, dy, nx, ny;
    unpack_stick_cal(d + 0, &cx, &cy);   // center
    unpack_stick_cal(d + 3, &dx, &dy);   // max delta
    unpack_stick_cal(d + 6, &nx, &ny);   // min delta

    *center_x = cx;
    *center_y = cy;
    *max_x    = cx + dx;
    *max_y    = cy + dy;
    *min_x    = (cx >= nx) ? cx - nx : 0;
    *min_y    = (cy >= ny) ? cy - ny : 0;

    // Clamp to 12-bit range
    if (*max_x > 0xFFF) *max_x = 0xFFF;
    if (*max_y > 0xFFF) *max_y = 0xFFF;
}

// -----------------------------------------------------------------------
// Motion calibration
// Bytes 0-11: six int16 bias offsets (accel xyz, gyro xyz)
// Bytes 12-23: six float scale factors (accel xyz, gyro xyz)
// -----------------------------------------------------------------------

static void parse_motion_block(const uint8_t* d,
                                int16_t accel_offset[3], int16_t gyro_offset[3],
                                float   accel_scale[3],  float   gyro_scale[3]) {
    for (int i = 0; i < 3; i++) {
        accel_offset[i] = (int16_t)(d[i*2]     | ((uint16_t)d[i*2 + 1] << 8));
        gyro_offset[i]  = (int16_t)(d[6 + i*2] | ((uint16_t)d[6 + i*2 + 1] << 8));
    }
    // Floats are little-endian IEEE 754
    for (int i = 0; i < 3; i++) {
        uint32_t bits;
        memcpy(&bits, d + 12 + i*4, 4);
        float f;
        memcpy(&f, &bits, 4);
        accel_scale[i] = f;
    }
    for (int i = 0; i < 3; i++) {
        uint32_t bits;
        memcpy(&bits, d + 12 + 12 + i*4, 4);
        float f;
        memcpy(&f, &bits, 4);
        gyro_scale[i] = f;

        // Sanity check: if scale is 0 or non-finite, use a safe default
        if (!isnormal(gyro_scale[i]) || gyro_scale[i] == 0.0f)
            gyro_scale[i] = 1.0f;
        if (!isnormal(accel_scale[i]) || accel_scale[i] == 0.0f)
            accel_scale[i] = 1.0f;
    }
}

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------

void Calibration_Parse(const Sw2CalibrationRaw* raw, Sw2Calibration* out) {
    memset(out, 0, sizeof(*out));

    // Decide which stick cal to use: user overrides factory if magic matches
    const uint8_t* l_data = raw->stick_l_raw;
    const uint8_t* r_data = raw->stick_r_raw;

    if (raw->user_loaded && raw->user_magic == SPI_CAL_USER_MAGIC) {
        l_data = raw->user_stick_l_raw;
        r_data = raw->user_stick_r_raw;
        printf("[cal] Using user stick calibration\n");
    } else {
        printf("[cal] Using factory stick calibration\n");
    }

    if (raw->stick_l_loaded) {
        parse_stick_block(l_data,
                          &out->l_center_x, &out->l_center_y,
                          &out->l_max_x,    &out->l_max_y,
                          &out->l_min_x,    &out->l_min_y);
    } else {
        // Safe fallback — centered 12-bit range
        out->l_center_x = out->l_center_y = 2048;
        out->l_max_x    = out->l_max_y    = 4095;
        out->l_min_x    = out->l_min_y    = 0;
        printf("[cal] Left stick cal missing, using defaults\n");
    }

    if (raw->stick_r_loaded) {
        parse_stick_block(r_data,
                          &out->r_center_x, &out->r_center_y,
                          &out->r_max_x,    &out->r_max_y,
                          &out->r_min_x,    &out->r_min_y);
    } else {
        out->r_center_x = out->r_center_y = 2048;
        out->r_max_x    = out->r_max_y    = 4095;
        out->r_min_x    = out->r_min_y    = 0;
        printf("[cal] Right stick cal missing, using defaults\n");
    }

    if (raw->motion_loaded) {
        parse_motion_block(raw->motion_raw,
                           out->accel_offset, out->gyro_offset,
                           out->accel_scale,  out->gyro_scale);
    } else {
        // Identity fallback
        for (int i = 0; i < 3; i++) {
            out->accel_offset[i] = 0;
            out->gyro_offset[i]  = 0;
            out->accel_scale[i]  = 1.0f;
            out->gyro_scale[i]   = 1.0f;
        }
        printf("[cal] Motion cal missing, using defaults\n");
    }

    out->valid = 1;
}

int16_t Calibration_NormalizeAxis(const Sw2Calibration* cal,
                                   uint16_t raw,
                                   uint16_t center,
                                   uint16_t max_val,
                                   uint16_t min_val) {
    if (!cal->valid) return 0;

    // Clamp to known range first
    if (raw > max_val) raw = max_val;
    if (raw < min_val) raw = min_val;

    float result;
    if (raw >= center) {
        float range = (float)(max_val - center);
        if (range < 1.0f) return 0;
        result = ((float)(raw - center) / range) * 32767.0f;
    } else {
        float range = (float)(center - min_val);
        if (range < 1.0f) return 0;
        result = -((float)(center - raw) / range) * 32768.0f;
    }

    // Clamp to [-32767, 32767] — NOT -32768.
    // NormalizeLY negates this value. Negating -32768 in int16_t overflows
    // back to -32768 (undefined behaviour / wraps on x86), which makes the
    // stick appear to snap back to its opposite extreme at full deflection.
    if (result >  32767.0f) result =  32767.0f;
    if (result < -32767.0f) result = -32767.0f;
    return (int16_t)result;
}

int16_t Calibration_NormalizeLX(const Sw2Calibration* cal, uint16_t raw) {
    return Calibration_NormalizeAxis(cal, raw,
        cal->l_center_x, cal->l_max_x, cal->l_min_x);
}

int16_t Calibration_NormalizeLY(const Sw2Calibration* cal, uint16_t raw) {
    // Y axis is inverted in HID convention (up = negative)
    return -Calibration_NormalizeAxis(cal, raw,
        cal->l_center_y, cal->l_max_y, cal->l_min_y);
}

int16_t Calibration_NormalizeRX(const Sw2Calibration* cal, uint16_t raw) {
    return Calibration_NormalizeAxis(cal, raw,
        cal->r_center_x, cal->r_max_x, cal->r_min_x);
}

int16_t Calibration_NormalizeRY(const Sw2Calibration* cal, uint16_t raw) {
    return -Calibration_NormalizeAxis(cal, raw,
        cal->r_center_y, cal->r_max_y, cal->r_min_y);
}

int16_t Calibration_ScaleAccel(const Sw2Calibration* cal, int axis, int32_t raw) {
    if (!cal->valid || axis < 0 || axis > 2) return (int16_t)raw;
    float v = ((float)raw - cal->accel_offset[axis]) * cal->accel_scale[axis];
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

int16_t Calibration_ScaleGyro(const Sw2Calibration* cal, int axis, int32_t raw) {
    if (!cal->valid || axis < 0 || axis > 2) return (int16_t)raw;
    float v = ((float)raw - cal->gyro_offset[axis]) * cal->gyro_scale[axis];
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}