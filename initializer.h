#pragma once
#include <libusb-1.0/libusb.h>
#include <vector>
#include <string>
#include <cstdint>
#include "structs.h"

class Initializer {
public:
    static bool IsCompatible(libusb_device* dev, const std::vector<DeviceId>& allowed);
    static libusb_device_handle* PrepareDevice(libusb_device* dev);
    static std::string GetDeviceId(libusb_device* dev);

private:
    static void BulkCommand(libusb_device_handle* handle, const std::vector<uint8_t>& payload);
    static void Init(libusb_device_handle* handle);
};