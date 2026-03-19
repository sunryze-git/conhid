#pragma once
#include <stdint.h>
#include <stddef.h>

#define TRANSPORT_MTU           0x100
#define TRANSPORT_TIMEOUT_MS    500
#define TRANSPORT_TYPE_BLE      0x01

typedef struct transport transport_t;

struct transport {
    int  (*send)(transport_t *t, const uint8_t *buf, size_t len);
    int  (*recv)(transport_t *t, uint8_t *buf, size_t len, int timeout_ms, uint16_t expected_handle);
    void (*close)(transport_t *t);
    uint8_t  type;
    void    *priv;
};

typedef struct {
    uint16_t cmd_write;     /* Command write handle  (default 0x0016) */
    uint16_t cmd_resp1;     /* Response notify handle #1 (default 0x001A) */
    uint16_t hid_input;     /* HID input report handle   (default 0x000E) */
    uint16_t hid_input_ccc; /* CCC descriptor for HID input (default 0x000F) */
} ble_handles_t;

transport_t *ble_transport_open(const char *adapter_mac, const char *mac_addr, const ble_handles_t *handles);
int         ble_transport_send(transport_t *t, const uint8_t *buf, size_t len);
int         ble_transport_recv(transport_t *t, uint8_t *buf, size_t len, int timeout_ms, uint16_t expected_handle);
void        ble_transport_close(transport_t *t);