#include "transport.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>

#include <libusb-1.0/libusb.h>

// -----------------------------------------------------------------------
// ATT constants (BLE only)
// -----------------------------------------------------------------------

#define ATT_WRITE_CMD       0x52
#define ATT_HANDLE_NOTIFY   0x1B

// -----------------------------------------------------------------------
// USB backend
// -----------------------------------------------------------------------

static void usb_send(Transport* t, const uint8_t* data, int len) {
    libusb_device_handle* handle = (libusb_device_handle*)t->usb.handle;
    int transferred = 0;
    int rc = libusb_bulk_transfer(handle, 0x02,
                                  (unsigned char*)data, len,
                                  &transferred, 1000);
    if (rc != 0)
        fprintf(stderr, "[transport/usb] bulk write EP0x02 error: %s\n",
                libusb_error_name(rc));
}

/*
 * usb_send_haptic — LRA rumble packets go to interrupt OUT endpoint 0x01.
 *
 * This is a completely separate USB pipe from the bulk command endpoint 0x02.
 * The controller expects 64-byte interrupt transfers here at ~4ms intervals
 * while rumble is active. Using the wrong endpoint (bulk 0x02) results in
 * the packets being silently accepted by the OS but ignored by the controller
 * firmware, which is why nothing was felt despite correct-looking log output.
 *
 * libusb_interrupt_transfer is synchronous but fast (~<1ms for 64 bytes) so
 * it's fine to call from the rumble pump thread.
 */
static int usb_send_haptic(Transport* t, const uint8_t* data, int len) {
    libusb_device_handle* handle = (libusb_device_handle*)t->usb.handle;
    int transferred = 0;

    /* Cap at 64 bytes — the interrupt endpoint's max packet size */
    if (len > 64) len = 64;

    int rc = libusb_interrupt_transfer(handle, 0x01,
                                       (unsigned char*)data, len,
                                       &transferred, 100 /* ms timeout */);
    if (rc != 0 && rc != LIBUSB_ERROR_TIMEOUT)
        fprintf(stderr, "[transport/usb] interrupt write EP0x01 error: %s  "
                "(transferred=%d)\n", libusb_error_name(rc), transferred);

    return rc;
}

static int usb_get_poll_fd(Transport* t) {
    return t->usb.wake_pipe[0];
}

static int usb_read_report(Transport* t, uint8_t* buf, int buf_len) {
    if (!t->usb.report_ready)
        return 0;

    int n = t->usb.report_len;
    if (n > buf_len) n = buf_len;
    memcpy(buf, t->usb.report_buf, n);

    t->usb.report_ready = 0;
    t->usb.report_len   = 0;

    char discard;
    read(t->usb.wake_pipe[0], &discard, 1);

    return n;
}

static void usb_destroy(Transport* t) {
    libusb_device_handle* handle = (libusb_device_handle*)t->usb.handle;
    libusb_device*        dev    = (libusb_device*)t->usb.dev;

    t->usb.report_ready = -1;
    if (t->usb.transfer) {
        libusb_cancel_transfer((struct libusb_transfer*)t->usb.transfer);
        while (t->usb.transfer != NULL) {
            struct timeval tv = {0, 1000};
            libusb_handle_events_timeout(NULL, &tv);
        }
    }

    close(t->usb.wake_pipe[0]);
    close(t->usb.wake_pipe[1]);

    libusb_release_interface(handle, 0);
    libusb_release_interface(handle, 1);
    libusb_close(handle);
    libusb_unref_device(dev);
}

static void usb_transfer_callback(struct libusb_transfer* transfer) {
    Transport* t = (Transport*)transfer->user_data;

    if (t->usb.report_ready < 0) {
        libusb_free_transfer(transfer);
        t->usb.transfer = NULL;
        return;
    }

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED && transfer->actual_length > 0) {
        if (!t->usb.report_ready) {
            memcpy(t->usb.report_buf, transfer->buffer, transfer->actual_length);
            t->usb.report_len   = transfer->actual_length;
            t->usb.report_ready = 1;
            char wake = 1;
            write(t->usb.wake_pipe[1], &wake, 1);
        }
    } else if (transfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
        fprintf(stderr, "[transport/usb] transfer error: %d\n", transfer->status);
    }

    if (libusb_submit_transfer(transfer) != 0) {
        fprintf(stderr, "[transport/usb] resubmit failed\n");
        libusb_free_transfer(transfer);
        t->usb.transfer = NULL;
    }
}

Transport* Transport_CreateUSB(void* dev, void* handle) {
    Transport* t = (Transport*)calloc(1, sizeof(Transport));
    if (!t) return NULL;

    t->type           = TRANSPORT_USB;
    t->send           = usb_send;
    t->send_haptic    = usb_send_haptic;
    t->get_poll_fd    = usb_get_poll_fd;
    t->read_report    = usb_read_report;
    t->destroy        = usb_destroy;

    t->usb.dev        = dev;
    t->usb.handle     = handle;
    t->usb.report_ready = 0;
    t->usb.report_len   = 0;

    libusb_ref_device((libusb_device*)dev);

    if (pipe(t->usb.wake_pipe) < 0) {
        fprintf(stderr, "[transport/usb] pipe failed: %s\n", strerror(errno));
        libusb_unref_device((libusb_device*)dev);
        free(t);
        return NULL;
    }

    fcntl(t->usb.wake_pipe[0], F_SETFL, O_NONBLOCK);

    struct libusb_transfer* xfer = libusb_alloc_transfer(0);
    if (!xfer) {
        fprintf(stderr, "[transport/usb] alloc_transfer failed\n");
        close(t->usb.wake_pipe[0]);
        close(t->usb.wake_pipe[1]);
        libusb_unref_device((libusb_device*)dev);
        free(t);
        return NULL;
    }

    libusb_fill_interrupt_transfer(
        xfer,
        (libusb_device_handle*)handle,
        0x81,                       // interrupt IN endpoint
        t->usb.report_buf,
        sizeof(t->usb.report_buf),
        usb_transfer_callback,
        t,
        0
    );

    t->usb.transfer = xfer;

    if (libusb_submit_transfer(xfer) != 0) {
        fprintf(stderr, "[transport/usb] initial submit failed\n");
        libusb_free_transfer(xfer);
        t->usb.transfer = NULL;
        close(t->usb.wake_pipe[0]);
        close(t->usb.wake_pipe[1]);
        libusb_unref_device((libusb_device*)dev);
        free(t);
        return NULL;
    }

    return t;
}

// -----------------------------------------------------------------------
// BLE backend
// -----------------------------------------------------------------------

static void ble_send(Transport* t, const uint8_t* data, int len) {
    uint8_t buf[512];
    if (len + 3 > (int)sizeof(buf)) {
        fprintf(stderr, "[transport/ble] payload too large (%d)\n", len);
        return;
    }
    buf[0] = ATT_WRITE_CMD;
    buf[1] =  t->ble.write_handle       & 0xFF;
    buf[2] = (t->ble.write_handle >> 8) & 0xFF;
    memcpy(&buf[3], data, len);

    if (write(t->ble.att_sock, buf, len + 3) < 0)
        fprintf(stderr, "[transport/ble] att write error: %s\n", strerror(errno));
}

static int ble_send_haptic(Transport* t, const uint8_t* data, int len) {
    if (t->ble.haptic_write_handle == 0) {
        fprintf(stderr, "[transport/ble] no haptic handle configured\n");
        return -1;
    }

    uint8_t buf[512];
    if (len + 3 > (int)sizeof(buf)) {
        fprintf(stderr, "[transport/ble] haptic payload too large (%d)\n", len);
        return -1;
    }
    buf[0] = ATT_WRITE_CMD;
    buf[1] =  t->ble.haptic_write_handle       & 0xFF;
    buf[2] = (t->ble.haptic_write_handle >> 8) & 0xFF;
    memcpy(&buf[3], data, len);

    ssize_t n = write(t->ble.att_sock, buf, len + 3);
    if (n < 0) {
        fprintf(stderr, "[transport/ble] haptic att write error: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int ble_get_poll_fd(Transport* t) {
    return t->ble.att_sock;
}

static int ble_read_report(Transport* t, uint8_t* buf, int buf_len) {
    uint8_t raw[512];

    ssize_t n = read(t->ble.att_sock, raw, sizeof(raw));
    if (n <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }

    if (n < 3) return 0;
    if (raw[0] != ATT_HANDLE_NOTIFY) return 0;

    uint16_t handle = raw[1] | (raw[2] << 8);
    if (handle != t->ble.input_handle) return 0;

    int payload_len = (int)n - 3;
    if (payload_len > buf_len) payload_len = buf_len;
    memcpy(buf, raw + 3, payload_len);
    return payload_len;
}

static void ble_destroy(Transport* t) {
    close(t->ble.att_sock);
}

Transport* Transport_CreateBLE(int att_sock, const char* mac,
                               uint16_t write_handle, uint16_t input_handle,
                               uint16_t haptic_write_handle) {
    Transport* t = (Transport*)calloc(1, sizeof(Transport));
    if (!t) return NULL;

    t->type           = TRANSPORT_BLE;
    t->send           = ble_send;
    t->send_haptic    = ble_send_haptic;
    t->get_poll_fd    = ble_get_poll_fd;
    t->read_report    = ble_read_report;
    t->destroy        = ble_destroy;

    t->ble.att_sock           = att_sock;
    t->ble.write_handle       = write_handle;
    t->ble.input_handle       = input_handle;
    t->ble.haptic_write_handle = haptic_write_handle;
    strncpy(t->ble.mac, mac, sizeof(t->ble.mac) - 1);

    int flags = fcntl(att_sock, F_GETFL, 0);
    fcntl(att_sock, F_SETFL, flags | O_NONBLOCK);

    return t;
}

// -----------------------------------------------------------------------
// Common
// -----------------------------------------------------------------------

void Transport_Destroy(Transport* t) {
    if (!t) return;
    t->destroy(t);
    free(t);
}