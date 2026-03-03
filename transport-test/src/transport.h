#pragma once

#include <stdint.h>
#include <stddef.h>

/*
 * transport.h -- Switch 2 controller transport abstraction
 *
 * Both USB (libusb) and BLE (BlueZ HCI) implement this interface identically.
 * All command code talks only to transport_t; it never calls libusb or BlueZ
 * directly.
 */

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
    int  (*recv)(transport_t *t, uint8_t *buf, size_t len, int timeout_ms);

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

/* -------------------------------------------------------------------------
 * USB transport  (transport_usb.c / libusb-1.0)
 * ------------------------------------------------------------------------- */

/*
 * Open the first Switch 2 controller found on USB with the given VID/PID.
 * Pass 0 for both to use the default Nintendo VID (0x057E) and scan for any
 * known Switch 2 product ID (>= 0x2060).
 *
 * Returns a heap-allocated transport_t on success, NULL on failure.
 * The caller owns it and must call t->close(t) when done.
 */
transport_t *usb_transport_open(uint16_t vid, uint16_t pid);

/* -------------------------------------------------------------------------
 * BLE transport  (transport_ble.c / BlueZ HCI sockets)
 * ------------------------------------------------------------------------- */

/*
 * Open a BLE transport to the controller at `mac_addr` (colon-separated,
 * e.g. "AA:BB:CC:DD:EE:FF").  The device must already be connected -- this
 * function does not perform scanning or GATT service discovery; it expects
 * the GATT handles to have been resolved beforehand and stored in the
 * ble_handles_t struct pointed to by `handles`.
 *
 * Passing NULL for `handles` uses the default handle layout from the
 * bluetooth_interface spec (Pro Controller 2 values).
 *
 * Returns a heap-allocated transport_t on success, NULL on failure.
 */
typedef struct {
    uint16_t cmd_write;     /* Command write handle  (default 0x0016) */
    uint16_t cmd_resp1;     /* Response notify handle #1 (default 0x001E) */
    uint16_t cmd_resp2;     /* Response notify handle #2 (default 0x001A, JoyCon only) */
    uint16_t cmd_resp3;     /* Response notify handle #3 (default 0x0022) */
    uint16_t hid_input;     /* HID input report handle   (default 0x000E) */
    uint16_t hid_input_ccc; /* CCC descriptor for HID input (default 0x000F) */
    uint16_t report_rate;   /* Report-rate descriptor    (default 0x0010) */
} ble_handles_t;

/*
 * mac_addr: controller Bluetooth address (e.g. "94:8E:6D:2C:A6:E2")
 * handles:  GATT handle layout, or NULL for Pro Controller 2 defaults
 *
 * The adapter is auto-detected via hci_get_route(). Run as root or with
 * CAP_NET_RAW. Stop bluetoothd first if it has claimed the adapter.
 */
/* adapter_mac: PC Bluetooth adapter MAC (e.g. "CC:5E:F8:B2:53:00") */
transport_t *ble_transport_open(const char *adapter_mac, const char *mac_addr, const ble_handles_t *handles);