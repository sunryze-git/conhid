#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>
#include "structs.h"
#include "calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

bool Initializer_IsCompatible(libusb_device* dev,
                               const DeviceId* allowed, int allowed_count);

libusb_device_handle* Initializer_PrepareDevice(libusb_device* dev,
                                                 Sw2CalibrationRaw* cal_raw);

void Initializer_GetDeviceId(libusb_device* dev, char* buf, int buf_len);

#ifdef __cplusplus
}
#endif