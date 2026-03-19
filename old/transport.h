#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRANSPORT_USB,
    TRANSPORT_BLE,
} TransportType;

//
// A Transport wraps one physical connection to a controller.
// Callers use the function pointers — they never touch the union directly.
//
typedef struct Transport Transport;

struct Transport {
    TransportType type;

    // Send a command payload to the controller.
    // For USB: bulk OUT endpoint 0x02.
    // For BLE: ATT_WRITE_CMD to write_handle 0x0014.
    void (*send)(Transport* t, const uint8_t* data, int len);

    // Send a LRA haptic/rumble packet to the controller.
    // This is a completely separate subsystem from send():
    //   For USB: interrupt OUT endpoint 0x01, 64-byte fixed packet, no timeout.
    //   For BLE: ATT_WRITE_CMD to haptic_write_handle (separate handle).
    // Returns 0 on success, libusb error code (negative) on failure.
    int (*send_haptic)(Transport* t, const uint8_t* data, int len);

    // Return a fd that becomes readable when the controller sends data.
    int  (*get_poll_fd)(Transport* t);

    // Read one raw input report into buf (max buf_len bytes).
    // Returns number of bytes read, 0 if nothing ready, -1 on error/disconnect.
    int  (*read_report)(Transport* t, uint8_t* buf, int buf_len);

    // Clean up — closes sockets/handles but does NOT free the Transport itself.
    void (*destroy)(Transport* t);

    //
    // Backend-specific state
    //
    union {
        struct {
            // libusb pieces — owned by the USB transport
            void*   dev;       // libusb_device*
            void*   handle;    // libusb_device_handle*
            void*   transfer;  // struct libusb_transfer*  (async input)

            // Pipe used to wake the poll fd when async callback fires.
            int     wake_pipe[2];

            // Single-slot report buffer written by callback, read by read_report.
            uint8_t report_buf[64];
            int     report_len;
            volatile int report_ready;
        } usb;

        struct {
            int      att_sock;              // raw L2CAP ATT socket
            char     mac[18];               // controller MAC string
            uint16_t write_handle;          // 0x0014 — command write
            uint16_t input_handle;          // 0x000E — input notify
            uint16_t haptic_write_handle;   // haptic/rumble write handle (BLE)
        } ble;
    };
};

// -----------------------------------------------------------------------
// USB transport
// -----------------------------------------------------------------------
Transport* Transport_CreateUSB(void* dev, void* handle);

// -----------------------------------------------------------------------
// BLE transport
// -----------------------------------------------------------------------
// haptic_write_handle: the ATT handle for LRA haptic packets (separate from
// the normal command write_handle). Pass 0 if BLE haptics are not supported.
Transport* Transport_CreateBLE(int att_sock, const char* mac,
                               uint16_t write_handle, uint16_t input_handle,
                               uint16_t haptic_write_handle);

void Transport_Destroy(Transport* t);

#ifdef __cplusplus
}
#endif