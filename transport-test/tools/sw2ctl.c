/*
 * tools/sw2ctl.c -- Switch 2 controller BLE/USB test tool
 *
 * Usage:
 *   sw2ctl usb                   -- open first Switch 2 over USB
 *   sw2ctl ble <ADAPTER_MAC> <CONTROLLER_MAC>  -- open via BLE (adapter auto-detected)
 *
 * Runs the full init sequence from probe.c, then listens for input
 * report notifications and prints them raw.
 */

#include "../src/transport.h"
#include "../src/commands.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <poll.h>

#define PADDING     0x00

/* -------------------------------------------------------------------------
 * Full init sequence -- taken directly from probe.c
 * Each command is sent then we wait up to 200ms for a response on the
 * command response handles before moving to the next one.
 * ------------------------------------------------------------------------- */

typedef struct { const uint8_t *data; int len; } Cmd;

static const uint8_t init[] =                   {INIT,REQUEST_MARKER,COMM_BLE,INIT_USB_INIT,0x00,0x08,                  PADDING,PADDING,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t unknown_init_01[] =        {CMD_07,REQUEST_MARKER,COMM_BLE,CMD_07_INIT,0x00,0x00,                  PADDING,PADDING};
static const uint8_t unknown_init_02[] =        {CMD_16,REQUEST_MARKER,COMM_BLE,CMD_16_UNKNOWN_01,0x00,0x00,            PADDING,PADDING};
static const uint8_t unknown_batt_01[] =        {BATTERY,REQUEST_MARKER,COMM_BLE,BATTERY_UNKNOWN_07,0x00,0x04,          PADDING,PADDING,0x00,0x00,0x00,0x00};
static const uint8_t led_set_p1[] =             {LED,REQUEST_MARKER,COMM_BLE,LED_SET_PATTERN,0x00,0x08,                 PADDING,PADDING,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t play_conn_vib[] =          {HAPTICS,REQUEST_MARKER,COMM_BLE,VIBRATION_PLAY_SAMPLE,0x00,0x08,       PADDING,PADDING,VIBRATION_PAIRING,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t features_prepare_all[] =   {FEATURES,REQUEST_MARKER,COMM_BLE,FEATURE_SET_MASK,0x00,0x04,           PADDING,PADDING,FEATURE_ALL,0x00,0x00,0x00};                
static const uint8_t features_enable_all[] =    {FEATURES,REQUEST_MARKER,COMM_BLE,FEATURE_ENABLE,0x00,0x04,             PADDING,PADDING,FEATURE_ALL,0x00,0x00,0x00};
//static const uint8_t read_unknown_01[] =        {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,                        PADDING,PADDING,0x40,0x7E,0x00,0x00,0x80,0x30,0x01,0x00};   
//static const uint8_t read_unknown_02[] =        {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,                        PADDING,PADDING,0x40,0x7E,0x00,0x00,0xC0,0x30,0x01,0x00};
//static const uint8_t left_stick_cal[] =         {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,                        PADDING,PADDING,0x40,0x7E,0x00,0x00,0x40,0xC0,0x1F,0x00};
//static const uint8_t read_unknown_03[] =        {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,                        PADDING,PADDING,0x10,0x7E,0x00,0x00,0x40,0x30,0x01,0x00};
//static const uint8_t read_unknown_04[] =        {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,                        PADDING,PADDING,0x18,0x7E,0x00,0x00,0x00,0x31,0x01,0x00};
static const uint8_t unknown_cmd_03[] =         {CMD_11,REQUEST_MARKER,COMM_BLE,CMD_11_UNKNOWN_03,0x00,0x00,            PADDING,PADDING};
//static const uint8_t c16[] =                    {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,                        PADDING,PADDING,0x20,0x7E,0x00,0x00,0x60,0x30,0x01,0x00};
static const uint8_t select_pro_mode[] =        {INIT,REQUEST_MARKER,COMM_BLE,INIT_INPUT_REPORT,0x00,0x04,              PADDING,PADDING,0x09,0x00,0x00,0x00};
static const uint8_t get_firmware_info[] =      {FIRMWARE_INFO,REQUEST_MARKER,COMM_BLE,FIRMWARE_GET_VERSION,0x00,0x00,  PADDING,PADDING};
static const uint8_t nfc_unknown[] =            {NFC,REQUEST_MARKER,COMM_BLE,NFC_UNKNOWN_0C,0x00,0x00,                  PADDING,PADDING};
static const uint8_t unknown_cmd_04[] =         {CMD_18,REQUEST_MARKER,COMM_BLE,CMD_18_UNKNOWN_01,0x00,0x00,            PADDING,PADDING};
//static const uint8_t c22[] =                    {INIT,REQUEST_MARKER,COMM_BLE,INIT_WAKE_CONSOLE,0x00,0x00,              PADDING,PADDING};
//static const uint8_t c23[] =                    {HAPTICS,REQUEST_MARKER,COMM_BLE,VIBRATION_PLAY_SAMPLE,0x00,0x04,       PADDING,PADDING,0x03,0x00,0x00};

#define INIT_CMD_ENTRY(CMD) {CMD,sizeof(CMD)}

static const Cmd init_cmds[] = {
    INIT_CMD_ENTRY(init),
    INIT_CMD_ENTRY(unknown_init_01),
    INIT_CMD_ENTRY(unknown_init_02),
    INIT_CMD_ENTRY(unknown_batt_01),
    INIT_CMD_ENTRY(led_set_p1),
    INIT_CMD_ENTRY(play_conn_vib),
    INIT_CMD_ENTRY(unknown_cmd_03),
    INIT_CMD_ENTRY(select_pro_mode),
    INIT_CMD_ENTRY(get_firmware_info),
    INIT_CMD_ENTRY(nfc_unknown),
    INIT_CMD_ENTRY(unknown_cmd_04),
    INIT_CMD_ENTRY(features_prepare_all),
    INIT_CMD_ENTRY(features_enable_all),
};

#define NUM_INIT_CMDS (int)(sizeof(init_cmds) / sizeof(init_cmds[0]))

/* -------------------------------------------------------------------------
 * Send one init command and wait briefly for a response, printing both.
 * Mirrors probe.c's ble_send_init_cmd() -- we wait up to 200ms for an
 * ATT notification on any handle, then move on regardless.
 * ------------------------------------------------------------------------- */

#define ATT_HANDLE_NOTIFY 0x1B

static void send_init_cmd(transport_t *t, int idx, const uint8_t *payload, int len)
{
    printf("[CMD %02d/%02d] >> ", idx, NUM_INIT_CMDS - 1);
    for (int i = 0; i < len; i++) printf("%02X ", payload[i]);
    printf("\n");

    int rc = t->send(t, payload, (size_t)len);
    if (rc != TRANSPORT_OK) {
        fprintf(stderr, "  send failed: %d\n", rc);
        return;
    }

    uint8_t resp[TRANSPORT_MTU];
    rc = t->recv(t, resp, sizeof(resp), 200);
    if (rc == 0) printf(" << (no data received)\n");
    if (rc <  0) fprintf(stderr, " recv error: %s", strerror(-rc));
    if (rc > 0) {
        printf(" << ");
        for (int i = 0; i < rc; i++) printf("%02X ", resp[i]);
        printf("\n");
    }

    usleep(100000);

    if (payload[0] == FIRMWARE_INFO) {
        uint8_t controller_major = resp[0];
        uint8_t controller_minor = resp[1];
        uint8_t controller_micro = resp[2];
        uint8_t controller_type  = resp[3];
        uint8_t bluetooth_major  = resp[4];
        uint8_t bluetooth_minor  = resp[5];
        uint8_t bluetooth_micro  = resp[6];
        uint8_t dsp_major        = resp[8];
        uint8_t dsp_minor        = resp[9];
        uint8_t dsp_micro        = resp[10];

        printf("[Detected Firmware]\n");
        printf("Firmware Version : %d.%d.%d\n", controller_major, controller_minor, controller_micro);
        printf("Bluetooth Version: %d.%d.%d\n", bluetooth_major, bluetooth_minor, bluetooth_micro);
        printf("DSP Version      : %d.%d.%d\n", dsp_major, dsp_minor, dsp_micro);
        printf("Controller Type  : %d\n", controller_type);
    }
}

/* -------------------------------------------------------------------------
 * Listen loop -- print all incoming notifications after init
 * ------------------------------------------------------------------------- */

static void listen_loop(transport_t *t)
{
    printf("\nListening for input reports (Ctrl+C to stop)...\n");
    uint8_t buf[TRANSPORT_MTU];
    for (;;) {
        int rc = t->recv(t, buf, sizeof(buf), 5000);
        if (rc == TRANSPORT_ERR_TIMEOUT) {
            printf("  (no data for 5s)\n");
            continue;
        }
        if (rc != TRANSPORT_OK) {
            fprintf(stderr, "recv error: %d\n", rc);
            break;
        }
        printf("NOTIFY: \n");
        for (size_t i = 0; i < sizeof(buf); i++) printf("%02X ", buf[i]);
        printf("\n");
    }
}

/* -------------------------------------------------------------------------
 * Main
 * ------------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s usb | ble <ADAPTER_MAC> <CONTROLLER_MAC>\n", argv[0]);
        return 1;
    }

    transport_t *t = NULL;

    if (strcmp(argv[1], "usb") == 0) {
        printf("[sw2ctl] Opening USB transport...\n");
        t = usb_transport_open(0, 0);
    } else if (strcmp(argv[1], "ble") == 0 && argc >= 4) {
        printf("[sw2ctl] Opening BLE transport: adapter=%s controller=%s...\n", argv[2], argv[3]);
        t = ble_transport_open(argv[2], argv[3], NULL);
    } else {
        fprintf(stderr, "Usage: %s usb | ble <ADAPTER_MAC> <CONTROLLER_MAC>\n", argv[0]);
        return 1;
    }

    if (!t) {
        fprintf(stderr, "[sw2ctl] Failed to open transport.\n");
        return 1;
    }

    printf("[sw2ctl] Transport open (type=%s)\n",
           t->type == TRANSPORT_TYPE_USB ? "USB" : "BLE");

    /* Run full init sequence */
    printf("\n[sw2ctl] Running init sequence (%d commands)...\n", NUM_INIT_CMDS);
    for (int i = 0; i < NUM_INIT_CMDS; i++)
        send_init_cmd(t, i, init_cmds[i].data, init_cmds[i].len);

    printf("\n[sw2ctl] Init complete.\n");

    /* Then listen for input reports */
    listen_loop(t);

    t->close(t);
    printf("[sw2ctl] Transport closed.\n");
    return 0;
}