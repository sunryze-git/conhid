#include "transport.h"

#include <libusb-1.0/libusb.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define NINTENDO_VID        0x057E
static const uint16_t SW2_PIDS[] = {
    0x2060,  /* Pro Controller 2   */
    0x2066,  /* JoyCon (L) 2       */
    0x2067,  /* JoyCon (R) 2       */
    0x2068,  /* NSO Gamecube       */
    0x2069,  /* Pro Controller 2   */
    0x0000   /* sentinel           */
};

#define HID_INTERFACE           0
#define HID_EP_OUT              0x02    /* Interrupt OUT -- host -> device */
#define HID_EP_IN               0x81    /* Interrupt IN  -- device -> host */
#define HID_EP_PACKET_SIZE      64

typedef struct {
    libusb_context        *ctx;
    libusb_device_handle  *dev;
    int                    kernel_driver_detached;
} usb_priv_t;

static int usb_send(transport_t *t, const uint8_t *buf, size_t len)
{
    usb_priv_t *p = (usb_priv_t *)t->priv;
    int transferred = 0;

    if (!p || !p->dev) return TRANSPORT_ERR_CLOSED;

    if (len > TRANSPORT_MTU) return TRANSPORT_ERR_PARAM;

    /* libusb requires a non-const pointer; the transfer does not modify buf */
    int rc = libusb_bulk_transfer(p->dev,
                                       HID_EP_OUT,
                                       (uint8_t *)buf,
                                       (int)len,
                                       &transferred,
                                       TRANSPORT_TIMEOUT_MS);
    if (rc != LIBUSB_SUCCESS) {
        fprintf(stderr, "[usb] send error: %s\n", libusb_strerror(rc));
        return TRANSPORT_ERR_IO;
    }

    return TRANSPORT_OK;
}

static int usb_recv(transport_t *t, uint8_t *buf, size_t len, int timeout_ms, uint16_t expected_handle)
{
    usb_priv_t *p = (usb_priv_t *)t->priv;
    int transferred = 0;

    if (!p || !p->dev) return TRANSPORT_ERR_CLOSED;

    if (len > TRANSPORT_MTU) return TRANSPORT_ERR_PARAM;

    int rc = libusb_interrupt_transfer(p->dev,
                                       HID_EP_IN,
                                       buf,
                                       (int)len,
                                       &transferred,
                                       timeout_ms);
    if (rc == LIBUSB_ERROR_TIMEOUT) return TRANSPORT_ERR_TIMEOUT;

    if (rc != LIBUSB_SUCCESS) {
        fprintf(stderr, "[usb] recv error: %s\n", libusb_strerror(rc));
        return TRANSPORT_ERR_IO;
    }

    return TRANSPORT_OK;
}

static void usb_close(transport_t *t)
{
    usb_priv_t *p = (usb_priv_t *)t->priv;

    if (p) {
        if (p->dev) {
            libusb_release_interface(p->dev, HID_INTERFACE);
            if (p->kernel_driver_detached) libusb_attach_kernel_driver(p->dev, HID_INTERFACE);
            libusb_close(p->dev);
        }
        if (p->ctx) libusb_exit(p->ctx);
        free(p);
    }
    free(t);
}

static int is_sw2_pid(uint16_t pid)
{
    for (int i = 0; SW2_PIDS[i] != 0x0000; i++) if (SW2_PIDS[i] == pid) return 1;
    return 0;
}

transport_t *usb_transport_open(uint16_t vid, uint16_t pid)
{
    usb_priv_t  *priv = calloc(1, sizeof(usb_priv_t));
    transport_t *t    = calloc(1, sizeof(transport_t));

    if (!priv || !t) goto fail;

    /* Default to Nintendo VID and scan for any known SW2 PID */
    if (vid == 0) vid = NINTENDO_VID;

    if (libusb_init(&priv->ctx) < 0) {
        fprintf(stderr, "[usb] libusb_init failed\n");
        goto fail;
    }

    /* Open device -------------------------------------------------------- */
    if (pid != 0) {
        priv->dev = libusb_open_device_with_vid_pid(priv->ctx, vid, pid);
    } else {
        /* Scan for any known SW2 PID */
        libusb_device **list = NULL;
        ssize_t cnt = libusb_get_device_list(priv->ctx, &list);
        if (cnt >= 0) {
            for (ssize_t i = 0; i < cnt && !priv->dev; i++) {
                struct libusb_device_descriptor desc;
                if (libusb_get_device_descriptor(list[i], &desc) < 0) continue;
                if (desc.idVendor == vid && is_sw2_pid(desc.idProduct)) {
                    libusb_open(list[i], &priv->dev);
                    pid = desc.idProduct;
                }
            }
            libusb_free_device_list(list, 1);
        }
    }

    if (!priv->dev) {
        fprintf(stderr, "[usb] no Switch 2 controller found (VID=%04x PID=%04x)\n", vid, pid);
        goto fail;
    }

    /* Detach kernel HID driver if attached -------------------------------- */
    if (libusb_kernel_driver_active(priv->dev, HID_INTERFACE) == 1) {
        if (libusb_detach_kernel_driver(priv->dev, HID_INTERFACE) < 0) {
            fprintf(stderr, "[usb] failed to detach kernel driver\n");
            goto fail;
        }
        priv->kernel_driver_detached = 1;
    }

    if (libusb_claim_interface(priv->dev, HID_INTERFACE) < 0) {
        fprintf(stderr, "[usb] failed to claim interface\n");
        goto fail;
    }

    if (libusb_claim_interface(priv->dev, 1) < 0) {
        fprintf(stderr, "[usb] failed to claim interface\n");
        goto fail;
    }

    t->send  = usb_send;
    t->recv  = usb_recv;
    t->close = usb_close;
    t->type  = TRANSPORT_TYPE_USB;
    t->priv  = priv;

    return t;

fail:
    free(t);
    if (priv) {
        if (priv->dev)  libusb_close(priv->dev);
        if (priv->ctx)  libusb_exit(priv->ctx);
        free(priv);
    }
    return NULL;
}