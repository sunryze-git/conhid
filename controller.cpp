#include "controller.h"
#include <chrono>
#include <iostream>
#include <linux/uhid.h>
#include <unistd.h>
#include "virtualdevice.h"

#pragma pack(push, 1)
struct PhysicalProReport {
    uint8_t  reportId;    // 0x09
    uint16_t counter;     // 51 20 -> 0x2051

    // 3 Bytes of Buttons
    uint8_t  buttons1;    // Right side (ABXY, etc)
    uint8_t  buttons2;    // Left side (D-Pad, etc)
    uint8_t  buttons3;    // System (Home, etc)

    // Joysticks (12-bit packed)
    // 0x12 0xD8 0x89 -> LX: 0xD12, LY: 0x89D (approx)
    // It's easier to handle these as raw bytes and mask them.
    uint8_t  sticksLeft[3];  
    uint8_t  sticksRight[3];

    uint8_t  padding[8];  // The "00 00" before IMU
    
    uint8_t  imuData[44]; // The rest of the "IMU shit"
};
#pragma pack(pop)

#pragma pack(push, 1)
struct VirtualReport {
    uint8_t reportId = 0x09;
    uint8_t padding_header[2];
    
    // --- Byte 3: Right Side ---
    uint8_t b : 1;
    uint8_t a : 1;
    uint8_t y : 1;
    uint8_t x : 1;
    uint8_t rb : 1;
    uint8_t rt : 1;
    uint8_t plus : 1;
    uint8_t rClick : 1;

    // --- Byte 4: D-Pad (Hat) + Left Side ---
    uint8_t dpad : 4;   // 0-7 for directions, 8 for neutral
    uint8_t lb : 1;
    uint8_t lt : 1;
    uint8_t minus : 1;
    uint8_t lClick : 1;

    // --- Byte 5: System/Extra ---
    uint8_t home : 1;
    uint8_t capture : 1;
    uint8_t paddleGR : 1;
    uint8_t paddleGL : 1;
    uint8_t chat : 1;
    uint8_t padding1 : 3; // Fills the rest of Byte 5

    // --- Bytes 6-11: 12-bit Joysticks (Packed) ---
    // Two 12-bit values fit into 3 bytes. We repeat for Left and Right.
    uint8_t lStickData[3];
    uint8_t rStickData[3];

    // --- Bytes 12-19: Gap Padding ---
    uint8_t gap[8];

    // --- Bytes 20-43: IMU (32-bit values) ---
    int32_t accelX;    // Offset 20 (imuBase + 0)
    int32_t gyroX;     // Offset 24 (imuBase + 4)
    int32_t gyroY;     // Offset 28 (imuBase + 8)
    int32_t accelY;    // Offset 32 (imuBase + 12)
    int32_t gyroZ;     // Offset 36 (imuBase + 16)
    int32_t accelZ;    // Offset 40 (imuBase + 20)

    // --- Remainder: Padding to 64 bytes ---
    uint8_t finalPadding[20];
};
#pragma pack(pop)

Controller::Controller(libusb_device* dev, libusb_device_handle* handle, std::string deviceId)
    : dev_(dev), handle_(handle), deviceId_(deviceId) {
    
    libusb_ref_device(dev_);
    virtualDeviceFd_ = VirtualDevice::CreateUhidDevice(); 
    pollThread_ = std::thread(&Controller::PollLoop, this);
    
    std::cout << "[" << deviceId_ << "] Controller initialized and polling." << std::endl;
}

Controller::~Controller() {
    running_ = false;

    if (pollThread_.joinable()) {
        pollThread_.join();
    }

    if (handle_) {
        libusb_release_interface(handle_, 0);
        libusb_release_interface(handle_, 1);
        libusb_close(handle_);
    }

    if (virtualDeviceFd_ != -1) {
        close(virtualDeviceFd_);
    }

    libusb_unref_device(dev_);
    std::cout << "[" << deviceId_ << "] Controller disposed." << std::endl;
}

uint8_t GetHatValue(bool up, bool down, bool left, bool right) {
    if (up && right) return 1;
    if (right && down) return 3;
    if (down && left) return 5;
    if (left && up) return 7;
    if (up) return 0;
    if (right) return 2;
    if (down) return 4;
    if (left) return 6;
    return 8; // Neutral (Centered)
}

void PackSticks(uint8_t* target, uint16_t x, uint16_t y) {
    target[0] = x & 0xFF;                  
    target[1] = ((x >> 8) & 0x0F) | ((y << 4) & 0xF0); 
    target[2] = (y >> 4) & 0xFF;           
}

void PrintControllerState(const VirtualReport& out, uint16_t lx, uint16_t ly, uint16_t rx, uint16_t ry) {
    // \033[K clears the line, \r returns to the start
    printf("\r\033[K"); 
    
    // Main Face, Shoulders, and System/Paddles
    printf("Btns:[A:%d B:%d X:%d Y:%d L:%d R:%d ZL:%d ZR:%d] [+:%d -:%d H:%d C:%d] [GR:%d GL:%d CH:%d] ",
        out.a, out.b, out.x, out.y, 
        out.lb, out.rb, out.lt, out.rt,
        out.plus, out.minus, out.home, out.capture,
        out.paddleGR, out.paddleGL, out.chat);

    // Navigation and Sticks
    printf("| DPAD:%d | L:%4d,%4d | R:%4d,%4d ",
        out.dpad, lx, ly, rx, ry);

    // IMU Data - Using %11d to accommodate the full 32-bit signed range + sign
    printf("\n\033[KIMU: [ACCEL X:%11d Y:%11d Z:%11d] [GYRO Z:%11d]",
        out.accelX, out.accelY, out.accelZ,
        out.gyroZ);
    
    // Move cursor back up one line so the next \r\033[K works on the top line
    printf("\033[1A"); 

    fflush(stdout); 
}

void PrintIMUDebug(int32_t* raw) {
    printf("\r\033[K");
    printf("IMU RAW: [0:%11d] [1:%11d] [2:%11d] [3:%11d] [4:%11d] [5:%11d]",
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
    fflush(stdout);
}

void Controller::SetRumble(int hf, int lf) {
    std::lock_guard<std::mutex> lock(rumbleMutex_);
    currentHF_ = hf;
    currentLF_ = lf;
}

void Controller::SendRumblePacket() {
    uint8_t buf[64]; // HapticProtocol.BufferSize
    memset(buf, 0, sizeof(buf));

    // Calculate sequence: SequenceBase (0x50) + count % 16
    uint8_t seq = (uint8_t)(0x50 + (rumblePacketCount_++ % 16));

    {
        std::lock_guard<std::mutex> lock(rumbleMutex_);
        
        buf[0] = 0x02; // PacketHeader
        buf[1] = seq;  // SequenceOffset

        // Right Motor (Offset 2)
        buf[2] = 130;  // StaticState
        buf[3] = (uint8_t)((currentHF_ > 0 ? 0x10 : 0x00) | (1 & 0x03)); // ModeEnabled | 1
        buf[4] = (uint8_t)(currentHF_ & 0x0F);

        // Left Motor (Offset 16)
        buf[16] = 0x00; // Manual init to 0x00 as per C#
        buf[17] = seq;  // Offset + 1
        buf[18] = 130;  // Offset + 2 (StaticState)
        buf[19] = (uint8_t)((currentLF_ > 0 ? 0x10 : 0x00) | (1 & 0x03)); // ModeEnabled
        buf[20] = (uint8_t)(currentLF_ & 0x0F);
    }

    int transferred = 0;
    int rc = libusb_interrupt_transfer(handle_, 0x01, buf, sizeof(buf), &transferred, 100);
}

void Controller::PollLoop() {
    uint8_t buffer[64];
    int transferred = 0;
    auto lastRumbleTime = std::chrono::steady_clock::now();

    while (running_) {
        int rc = libusb_interrupt_transfer(handle_, 0x81, buffer, sizeof(buffer), &transferred, 5);

        // Handle Physical Input
        if (rc == 0 && transferred > 0) {
            PhysicalProReport* in = reinterpret_cast<PhysicalProReport*>(buffer);

            // Initialize Virtual Report
            VirtualReport out;
            memset(&out, 0, sizeof(out));
            out.reportId = 0x09;

            // Byte 3 Map Buttons (Physical -> Virtual)
            out.b     = (in->buttons1 & 0x01) != 0; // Bit 0
            out.a     = (in->buttons1 & 0x02) != 0; // Bit 1
            out.y     = (in->buttons1 & 0x04) != 0; // Bit 2
            out.x     = (in->buttons1 & 0x08) != 0; // Bit 3
            out.rb    = (in->buttons1 & 0x10) != 0; // Bit 4
            out.rt    = (in->buttons1 & 0x20) != 0; // Bit 5
            out.plus  = (in->buttons1 & 0x40) != 0; // Bit 6
            out.rClick = (in->buttons1 & 0x80) != 0; // Bit 7

            // Map D-Pad to hat
            bool d_down  = (in->buttons2 & 0x01) != 0;
            bool d_right = (in->buttons2 & 0x02) != 0;
            bool d_left  = (in->buttons2 & 0x04) != 0;
            bool d_up    = (in->buttons2 & 0x08) != 0;
            out.dpad = GetHatValue(d_up, d_down, d_left, d_right);

            // Map remainder of Byte 4
            out.lb     = (in->buttons2 & 0x10) != 0;
            out.lt     = (in->buttons2 & 0x20) != 0;
            out.minus  = (in->buttons2 & 0x40) != 0;
            out.lClick = (in->buttons2 & 0x80) != 0;

            // Map System Buttons
            out.home      = (in->buttons3 & 0x01) != 0;
            out.capture   = (in->buttons3 & 0x02) != 0;
            out.paddleGR  = (in->buttons3 & 0x04) != 0;
            out.paddleGL  = (in->buttons3 & 0x08) != 0;
            out.chat      = (in->buttons3 & 0x10) != 0;

            // Process Joysticks
            uint16_t lx = in->sticksLeft[0] | ((in->sticksLeft[1] & 0x0F) << 8);
            uint16_t ly = (in->sticksLeft[1] >> 4) | (in->sticksLeft[2] << 4);
            uint16_t rx = in->sticksRight[0] | ((in->sticksRight[1] & 0x0F) << 8);
            uint16_t ry = (in->sticksRight[1] >> 4) | (in->sticksRight[2] << 4);

            // Clamp that shit
            uint16_t clx = (lx > 4095) ? 4095 : lx;
            uint16_t cly = (ly > 4095) ? 4095 : ly;
            uint16_t crx = (rx > 4095) ? 4095 : rx;
            uint16_t cry = (ry > 4095) ? 4095 : ry;

            // Pack Sticks
            // Y values are inverted.
            PackSticks(out.lStickData, clx, 4095 - cly);
            PackSticks(out.rStickData, crx, 4095 - cry);

            // IMU Data
            int32_t* raw = reinterpret_cast<int32_t*>(in->imuData);
            out.gyroX  = raw[0]; 
            out.gyroY  = raw[1];
            out.gyroZ  = raw[2];
            out.accelX = raw[3];
            out.accelY = raw[4];
            out.accelZ = raw[5];

            // uncomment to print controller data
            //PrintControllerState(out, lx, ly, rx, ry);
            //PrintIMUDebug(raw);

            // uncomment to print button data hex
            //printf("\rRAW: %02X %02X %02X | B-Map: %d", in->buttons1, in->buttons2, in->buttons3, (in->buttons2 & 0x01));
            //fflush(stdout);

            // Wrap and write to UHID
            struct uhid_event ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = UHID_INPUT2;
            ev.u.input2.size = sizeof(out);
            memcpy(ev.u.input2.data, &out, sizeof(out));
            
            if (write(virtualDeviceFd_, &ev, sizeof(ev)) < 0) {
                std::cerr << "UHID Write Error: " << strerror(errno) << std::endl;
            }
        } else if (rc != LIBUSB_ERROR_TIMEOUT && rc != 0) {
            std::cerr << "[" << deviceId_ << "] Poll error: " << libusb_error_name(rc) << std::endl;
            break; 
        }

        // Read from UHID
        struct uhid_event ev_in;
        ssize_t ret = read(virtualDeviceFd_, &ev_in, sizeof(ev_in));
        if (ret > 0 && ev_in.type == UHID_OUTPUT) {
            // data[0] is Report ID (0x01)
            // data[1] is LF (Strong)
            // data[2] is HF (Weak)
            if (ev_in.u.output.data[0] == 0x01 && ev_in.u.output.size >= 3) {
                int lf_val = ev_in.u.output.data[1] >> 4;
                int hf_val = ev_in.u.output.data[2] >> 4;
                this->SetRumble(hf_val, lf_val);
            }
        }

        // Rumble Pacing
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRumbleTime).count();
        if (elapsed >= 4) {
            if (currentHF_ > 0 || currentLF_ > 0) {
                SendRumblePacket();
            }
            lastRumbleTime = now;
        }
    }
}