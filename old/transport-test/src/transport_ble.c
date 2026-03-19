#include "command_definitions.h"
#include "transport.h"

#include <asm-generic/errno.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/select.h>

#define ATT_CID              4
#define ATT_OP_HANDLE_NOTIFY 0x1B
#define ATT_OP_WRITE_REQ     0x12
#define ATT_OP_WRITE_RSP     0x13
#define ATT_OP_WRITE_CMD     0x52

static const ble_handles_t CCCD_HANDLES = {
    .cmd_write     = 0x0014,
    .cmd_resp1     = 0x001A,
    .hid_input     = 0x000A,
};

typedef struct {
    int           att_fd;
    ble_handles_t handles;
} ble_priv_t;

static void drain(int sock)
{
    fd_set fds;
    struct timeval tv = {0, 20000};
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    while (select(sock + 1, &fds, NULL, NULL, &tv) > 0) {
        uint8_t buf[256];
        read(sock, buf, sizeof(buf));
        tv.tv_usec = 20000;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
    }
}

static int enable_notifications(int sock) {
    uint16_t cccd_handles[] = {
        INPUT_REPORT_1, 0x002F
    };

    for (size_t i = 0; i < sizeof(cccd_handles) / sizeof(cccd_handles[0]); i++) {
        uint8_t buf[] = {
            ATT_OP_WRITE_REQ,
            cccd_handles[i] & 0xFF,
            (cccd_handles[i] >> 8) & 0xFF,
            0x01,
            0x00
        };
        int ret = write(sock, buf, sizeof(buf));
        if (ret < 0) {
            fprintf(stderr, "Failed to enable notifications for handle 0x%04X", cccd_handles[i]);
            return ret;
        }
        usleep(5000); 
    }
    return 0;
}

static int connect_att(const char *adapter_mac, const char *controller_mac)
{
    int sock = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sock < 0) { perror("socket"); return -1; }

    struct sockaddr_l2 local = {0};
    local.l2_family      = AF_BLUETOOTH;
    local.l2_cid         = htobs(ATT_CID);
    local.l2_bdaddr_type = BDADDR_LE_PUBLIC;
    str2ba(adapter_mac, &local.l2_bdaddr);

    if (bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        perror("bind"); close(sock); return -1;
    }

    struct sockaddr_l2 remote = {0};
    remote.l2_family      = AF_BLUETOOTH;
    remote.l2_cid         = htobs(ATT_CID);
    remote.l2_bdaddr_type = BDADDR_LE_PUBLIC;
    str2ba(controller_mac, &remote.l2_bdaddr);

    printf("[ble] Connecting to %s ...\n", controller_mac);
    if (connect(sock, (struct sockaddr *)&remote, sizeof(remote)) < 0) {
        perror("connect"); close(sock); return -1;
    }

    /* MTU exchange -- identical to probe.c */
    uint8_t mtu_req[] = {0x02, 0x00, 0x02};
    if (write(sock, mtu_req, sizeof(mtu_req)) < 0) {
        perror("MTU req write failed"); close(sock); return -1;
    }
    uint8_t resp[64];
    int n = read(sock, resp, sizeof(resp));
    if (n > 0 && resp[0] == 0x03) {
        uint16_t server_mtu = resp[1] | (resp[2] << 8);
        printf("[ble] MTU exchanged, server MTU: %d\n", server_mtu);
    } else {
        printf("[ble] MTU exchange failed or timed out\n");
    }

    return sock;
}

static int ble_send(transport_t *t, const uint8_t *buf, size_t len)
{
    ble_priv_t *p = (ble_priv_t *)t->priv;
    if (!p || p->att_fd < 0) return -ESHUTDOWN;

    uint8_t packet[TRANSPORT_MTU] = {
        ATT_OP_WRITE_CMD,
        p->handles.cmd_write & 0xFF,
        (p->handles.cmd_write >> 8) & 0xFF
    };
    memcpy(&packet[3], buf, len);

    int ret = write(p->att_fd, packet, len+3);
    if (ret < 0) return -errno;
    return ret;
}

// Recieve Data from the transport. Returns size of data, or -1 if IO error.
static int ble_recv(transport_t *t, uint8_t *buf, size_t len, int timeout_ms, uint16_t expected_handle)
{
    ble_priv_t *p = (ble_priv_t *)t->priv;
    if (!p || p->att_fd < 0) return -ESHUTDOWN;
    
    fd_set fds;
    struct timeval tv;
    uint8_t raw[TRANSPORT_MTU];

    while (timeout_ms > 0) {
        FD_ZERO(&fds);
        FD_SET(p->att_fd, &fds);
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int ready = select(p->att_fd + 1, &fds, NULL, NULL, &tv);
        if (ready < 0) return -errno;
        if (ready == 0) return -ETIMEDOUT;

        int n = read(p->att_fd, raw, sizeof(raw));
        if (n <= 0) return n;

        if (raw[0] == ATT_OP_HANDLE_NOTIFY && n >= 3) {
            uint16_t h = raw[1] | (raw[2] << 8);
            if (h == expected_handle) {
                size_t payload_len = (size_t)(n-3);
                if (payload_len > len) payload_len = len;
                memcpy(buf, &raw[3], payload_len);
                return (int)payload_len;
            }
        }
    }

    return -ETIMEDOUT;
}

static void ble_close(transport_t *t)
{
    ble_priv_t *p = (ble_priv_t *)t->priv;
    if (p) {
        if (p->att_fd >= 0) close(p->att_fd);
        free(p);
    }
    free(t);
}

transport_t *ble_transport_open(const char *adapter_mac, const char *mac_addr, const ble_handles_t *handles)
{
    if (!adapter_mac || !mac_addr) return NULL;

    ble_priv_t  *priv = calloc(1, sizeof(ble_priv_t));
    transport_t *t    = calloc(1, sizeof(transport_t));
    if (!priv || !t) goto fail;

    priv->att_fd = -1;
    priv->handles = handles ? *handles : CCCD_HANDLES;

    priv->att_fd = connect_att(adapter_mac, mac_addr);
    if (priv->att_fd < 0) goto fail;

    if (enable_notifications(priv->att_fd) < 0) goto fail;

    drain(priv->att_fd);

    t->send  = ble_send;
    t->recv  = ble_recv;
    t->close = ble_close;
    t->type  = TRANSPORT_TYPE_BLE;
    t->priv  = priv;
    return t;

fail:
    free(t);
    if (priv) {
        if (priv->att_fd >= 0) close(priv->att_fd);
        free(priv);
    }
    return NULL;
}