#include "../src/transport.h"
#include "../src/command_definitions.h"
#include "../src/responses.h"

#include <asm-generic/errno.h>
#include <bits/time.h>
#include <pulse/def.h>
#include <pulse/sample.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <poll.h>

#define PADDING     0x00

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
static const uint8_t get_battery_v[] =          {BATTERY,REQUEST_MARKER,COMM_BLE,BATTERY_GET_VOLTAGE,0x00,0x00,         PADDING, PADDING};
static const uint8_t get_battery_c[] =          {BATTERY,REQUEST_MARKER,COMM_BLE,BATTERY_GET_CHARGE,0x00,0x00,          PADDING, PADDING};


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
    INIT_CMD_ENTRY(get_battery_c),
    INIT_CMD_ENTRY(get_battery_v),
};
#define NUM_INIT_CMDS (int)(sizeof(init_cmds) / sizeof(init_cmds[0]))

static void print_command_response(resp_cmd_header* header, uint8_t* resp) {
    // FIRMWARE - GET VERSION
    if (header->id == FIRMWARE_INFO && header->sub_id == FIRMWARE_GET_VERSION) {
        resp_get_firmware_ver_info version;
        memcpy(&version, resp, sizeof(version));

        printf("[Detected Firmware]\n");
        printf("Firmware Version : %d.%d.%d\n", version.controller_major, version.controller_minor, version.controller_micro);
        printf("Bluetooth Version: %d.%d.%d\n", version.bluetooth_major, version.bluetooth_minor, version.bluetooth_micro);
        printf("DSP Version      : %d.%d.%d\n", version.dsp_major, version.dsp_minor, version.dsp_micro);
        printf("Controller Type  : %d\n", version.controller_type);
    }

    // BATTERY - GET VOLTAGE
    if (header->id == BATTERY && header->sub_id == BATTERY_GET_VOLTAGE) {
        resp_get_battery_voltage volt;
        memcpy(&volt, resp, sizeof(volt));
        printf("[Battery Voltage]\n");
        printf("Battery Voltage: %d mV\n", volt.voltage);
    }

    // BATTERY - GET CHARGE
    if (header->id == BATTERY && header->sub_id == BATTERY_GET_CHARGE) {
        resp_get_charge_status charge;
        memcpy(&charge, resp, sizeof(charge));
        printf("[Battery Charge Status]\n");
        printf("Battery Charge Status: %d\n", charge.status);
    }
}

static void send_init_cmd(transport_t *t, int idx, const uint8_t *payload, int len)
{
    printf("[CMD %02d/%02d] >> ", idx, NUM_INIT_CMDS - 1);
    for (int i = 0; i < len; i++) printf("%02X ", payload[i]);
    printf("\n");

    int rc = t->send(t, payload, (size_t)len);
    if (rc == 0) {
        fprintf(stderr, " send failed, no bytes written.\n");
        return;
    } else if (rc < 0) {
        fprintf(stderr, " send failed: %s\n", strerror(-rc));
        return;
    }

    uint8_t resp[TRANSPORT_MTU];
    rc = t->recv(t, resp, sizeof(resp), 500, 0x001A);
    if (rc == 0) {
        printf(" << (no data received)\n");
        return;
    }
    if (rc <  0) {
        fprintf(stderr, " recv error: %s\n", strerror(-rc));
        return;
    }

    resp_cmd_header* header = (resp_cmd_header*)resp;
    printf(" << CMD ID: 0x%02X | DIR: 0x%02X | TRANSPORT: 0x%02X | SUB CMD: 0x%02X\n", header->id, header->direction, header->transport, header->sub_id);

    print_command_response(header, resp);
}

static void listen_loop(transport_t *t)
{
    printf("\nListening for input reports (Ctrl+C to stop)...\n");

    uint8_t buf[TRANSPORT_MTU];
    struct timespec ts;
    
    while (1) {
        int rc = t->recv(t, buf, sizeof(buf), 5000, 0x002E);
    
        if (rc == -ETIMEDOUT) {
            printf("  (no data)\n");
            continue;
        } else if (rc < 0) {
            fprintf(stderr, "recv error: %s\n", strerror(-rc));
            break;
        } else if (rc == 0) {
            fprintf(stderr, " data returned is 0 bytes\n");
        }
    
        clock_gettime(CLOCK_MONOTONIC, &ts);
        printf("[%ld.%06ld]: NOTIFY: \n", ts.tv_sec, ts.tv_nsec / 1000);
        
        hid_ble_output_report_2E* packet = (hid_ble_output_report_2E*)buf;
        for (size_t i = 0; i < sizeof(packet->data); i++) {
            printf("%02X ", packet->data[i]);
        }
        printf("\n");

        // testing with if this is pcm audio data

        // hid_output_report report;
        // memcpy(&report, buf, sizeof(report));
        // print_hid_report(&report);
    }
}

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

    printf("[sw2ctl] Transport open (type=%s)\n", t->type == TRANSPORT_TYPE_USB ? "USB" : "BLE");

    printf("\n[sw2ctl] Running init sequence (%d commands)...\n", NUM_INIT_CMDS);
    for (int i = 0; i < NUM_INIT_CMDS; i++) send_init_cmd(t, i, init_cmds[i].data, init_cmds[i].len);

    printf("\n[sw2ctl] Init complete.\n");

    listen_loop(t);
    t->close(t);
    printf("[sw2ctl] Transport closed.\n");
    return 0;
}