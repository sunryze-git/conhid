#pragma once

#include <libusb-1.0/libusb.h>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

namespace HapticProtocol {
    const uint8_t PacketHeader = 0x10;      // Based on your C# 'PacketHeader'
    const uint8_t StaticState = 0x00;       // Matches 'StaticState'
    const uint8_t ModeEnabled = 0x01;       // Matches 'ModeEnabled'
    const int BufferSize = 12;              // Based on your offsets
    const int SequenceOffset = 1;
    const int LeftMotorOffset = 6;          // Derived from your 'LeftMotorStateOffset'
    const double IntervalMs = 4.0;          // Your 4ms pacing
}

#pragma pack(push, 1)
struct RumblePacket {
    uint8_t header = HapticProtocol::PacketHeader;
    uint8_t sequence;
    uint8_t staticState;
    uint8_t hfMode;
    uint8_t hfValue;
    uint8_t padding; // Alignment filler
    uint8_t lfHeader;
    uint8_t lfSequence;
    uint8_t lfStaticState;
    uint8_t lfMode;
    uint8_t lfValue;
    uint8_t finalPadding;
};
#pragma pack(pop)

class Controller {
public:
    Controller(libusb_device* dev, libusb_device_handle* handle, std::string deviceId);
    ~Controller();

    // Prevent copying to avoid double-closing handles
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    void SetRumble(int hf, int lf);
private:
    void PollLoop();

    libusb_device* dev_;
    libusb_device_handle* handle_;
    std::string deviceId_;
    
    std::atomic<bool> running_{true};
    std::thread pollThread_;
    int virtualDeviceFd_{-1};

    std::mutex rumbleMutex_;
    int currentHF_ = 0;
    int currentLF_ = 0;
    uint8_t rumblePacketCount_ = 0;
    
    void SendRumblePacket(); // The "Submit" equivalent
};