#include "virtualdevice.h"
#include <fcntl.h>
#include <iostream>
#include <errno.h>

// I hate This
#define D_USAGE_PAGE(x)          0x05, x
#define D_USAGE_PAGE16(x, y)     0x06, x, y
#define D_USAGE(x)               0x09, x
#define D_USAGE_L(x, y)          0x0a, x, y
#define D_COLLECTION(x)          0xa1, x
#define D_END_COLLECTION         0xc0
#define D_REPORT_ID(x)           0x85, x
#define D_LOGICAL_MIN(x)         0x15, x
#define D_LOGICAL_MIN32(x, y, z, w) 0x17, x, y, z, w
#define D_LOGICAL_MAX(x)         0x25, x
#define D_LOGICAL_MAX16(x, y)    0x26, x, y
#define D_LOGICAL_MAX32(x,y,z,w) 0x27, x, y, z, w
#define D_REPORT_SIZE(x)         0x75, x
#define D_REPORT_COUNT(x)        0x95, x
#define D_INPUT(x)               0x81, x
#define D_OUTPUT(x)              0x91, x

int VirtualDevice::CreateUhidDevice() {
    // Open the UHID character device
    int fd = open("/dev/uhid", O_RDWR | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[-] Failed to open /dev/uhid: " << strerror(errno) << std::endl;
        return -1;
    }

    struct uhid_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = UHID_CREATE2;

    strncpy((char*)ev.u.create2.name, "Nintendo Switch 2 Pro Controller", sizeof(ev.u.create2.name));
    ev.u.create2.bus = BUS_USB;
    ev.u.create2.vendor = 0x057E;
    ev.u.create2.product = 0x2069;
    ev.u.create2.version = 2;

    std::vector<uint8_t> rd = {
        D_USAGE_PAGE(0x01), D_USAGE(0x05), D_COLLECTION(0x01),

        D_REPORT_ID(0x09),

        // --- BYTES 1 & 2: Skip/Counters ---
        D_REPORT_SIZE(0x08), D_REPORT_COUNT(0x02), D_INPUT(0x03),

        // --- BYTE 3: Right Side (8 Bits) ---
        D_USAGE_PAGE(0x09),      // Button Page
        D_LOGICAL_MIN(0x00), 
        D_LOGICAL_MAX(0x01),
        D_REPORT_SIZE(0x01), 
        D_REPORT_COUNT(0x08),    // This tells the kernel: "Next 8 bits are buttons"
            D_USAGE(0x01),       // B1
            D_USAGE(0x02),       // B2
            D_USAGE(0x04),       // B3
            D_USAGE(0x05),       // B4
            D_USAGE(0x08),       // B5
            D_USAGE(0x0A),       // B6
            D_USAGE(0x0C),       // B7
            D_USAGE(0x0F),       // B8
        D_INPUT(0x02),           // Finish the 8-bit block

        // --- BYTE 4: Hat (4 Bits) + Left Side (4 Bits) ---
        D_USAGE_PAGE(0x01),      // Generic Desktop
        D_USAGE(0x39),           // Hat Switch
        D_LOGICAL_MIN(0x00), 
        D_LOGICAL_MAX(0x07),
        D_REPORT_SIZE(0x04), 
        D_REPORT_COUNT(0x01), 
        D_INPUT(0x42),           // Finish 4-bit Hat

        D_USAGE_PAGE(0x09),      // Back to Buttons
        D_REPORT_SIZE(0x01), 
        D_REPORT_COUNT(0x04),    // This fills the remaining 4 bits of Byte 4
            D_USAGE(0x07), 
            D_USAGE(0x09), 
            D_USAGE(0x0B), 
            D_USAGE(0x0E), 
        D_INPUT(0x02),           // Finish the 4-bit block

        // --- BYTE 5: System/Extra (Home, Capture, GL, GR, Chat) ---
        D_USAGE_PAGE(0x09),
        D_REPORT_SIZE(0x01), 
        D_REPORT_COUNT(0x05), // 5 buttons
            D_USAGE(0x11), // Home
            D_USAGE(0x12), // Capture
            D_USAGE(0x13), // Paddle GR
            D_USAGE(0x14), // Paddle GL
            D_USAGE(0x15), // Chat
        D_INPUT(0x02),

        D_REPORT_COUNT(0x03), // 3 bits of padding to finish the byte
        D_INPUT(0x03),

        // --- BYTES 6-11: STICKS ---
        D_USAGE_PAGE(0x01), D_LOGICAL_MIN(0x00), D_LOGICAL_MAX16(0xff, 0x0f),
        D_REPORT_SIZE(0x0c), 
        D_REPORT_COUNT(0x04), // 4 axes total: LX, LY, RX, RY
        D_USAGE(0x30), D_USAGE(0x31), D_USAGE(0x33), D_USAGE(0x34),
        D_INPUT(0x02),

        // --- BYTES 12-19: Padding ---
        D_REPORT_SIZE(0x08), D_REPORT_COUNT(0x08), D_INPUT(0x03),

        // --- BYTES 20-43: IMU ---
        D_USAGE_PAGE(0x01),      // Generic Desktop
        D_USAGE(0x04),           // Change to "Joystick" or "Multi-axis Controller"
        D_COLLECTION(0x00),      // Physical
            D_LOGICAL_MIN32(0x00, 0x00, 0x00, 0x80), 
            D_LOGICAL_MAX32(0xff, 0xff, 0xff, 0x7f), 
            D_REPORT_SIZE(0x20), 
            D_REPORT_COUNT(0x06), 
            
            // Use "Vendor Defined" or "Secondary" axis usages 
            // so they don't overwrite 0x30 (X) and 0x31 (Y)
            D_USAGE_PAGE(0xff), // Vendor Defined Page
            D_USAGE(0x01), D_USAGE(0x02), D_USAGE(0x03), // Accel X, Y, Z
            D_USAGE(0x04), D_USAGE(0x05), D_USAGE(0x06), // Gyro X, Y, Z
            D_INPUT(0x02),
        D_END_COLLECTION,

        // --- REMAINDER ---
        D_REPORT_SIZE(0x08), D_REPORT_COUNT(0x14), D_INPUT(0x03),

        D_END_COLLECTION
    };

    ev.u.create2.rd_size = rd.size();
    memcpy(ev.u.create2.rd_data, rd.data(), rd.size());

    if (write(fd, &ev, sizeof(ev)) < 0) {
        std::cerr << "[-] UHID Write FAILED: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}