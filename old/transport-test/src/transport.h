#pragma once

#include <stdint.h>
#include <stddef.h>

/* Maximum raw packet size on the wire */
#define TRANSPORT_MTU           0x100

/* Default send/recv timeout in milliseconds */
#define TRANSPORT_TIMEOUT_MS    500

/* Transport type tags */
#define TRANSPORT_TYPE_USB      0x00
#define TRANSPORT_TYPE_BLE      0x01

/* Return codes */
#define TRANSPORT_OK            0
#define TRANSPORT_ERR_IO       -1   /* Send/recv failed                  */
#define TRANSPORT_ERR_TIMEOUT  -2   /* No response within timeout window  */
#define TRANSPORT_ERR_PARAM    -3   /* Bad argument                       */
#define TRANSPORT_ERR_OPEN     -4   /* Could not open device              */
#define TRANSPORT_ERR_CLOSED   -5   /* Transport already closed           */

typedef struct transport transport_t;

struct transport {
    /*
     * Send `len` bytes from `buf` to the controller.
     * Returns TRANSPORT_OK or a negative error code.
     */
    int  (*send)(transport_t *t, const uint8_t *buf, size_t len);

    /*
     * Block until `len` bytes arrive in `buf`, or `timeout_ms` expires.
     * Returns TRANSPORT_OK, TRANSPORT_ERR_TIMEOUT, or another negative code.
     */
    int  (*recv)(transport_t *t, uint8_t *buf, size_t len, int timeout_ms, uint16_t expected_handle);

    /*
     * Close and free the transport. After this call `t` must not be used.
     */
    void (*close)(transport_t *t);

    /* TRANSPORT_TYPE_USB or TRANSPORT_TYPE_BLE -- used by protocol layer
     * to fill the correct comm_type byte in the packet header.          */
    uint8_t  type;

    /* Implementation-private state (libusb handle, HCI socket fd, etc.) */
    void    *priv;
};

transport_t *usb_transport_open(uint16_t vid, uint16_t pid);
typedef struct {
    uint16_t cmd_write;     /* Command write handle  (default 0x0016) */
    uint16_t cmd_resp1;     /* Response notify handle #1 (default 0x001A) */
    uint16_t hid_input;     /* HID input report handle   (default 0x000E) */
    uint16_t hid_input_ccc; /* CCC descriptor for HID input (default 0x000F) */
} ble_handles_t;

transport_t *ble_transport_open(const char *adapter_mac, const char *mac_addr, const ble_handles_t *handles);