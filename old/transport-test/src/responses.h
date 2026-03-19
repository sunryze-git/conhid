// Command Response Structs
#pragma once
#include <stdint.h>


// COMMAND HEADERS
typedef struct {
    uint8_t id;
    uint8_t direction;
    uint8_t transport;
    uint8_t sub_id;
    uint8_t _reserved[0x4];
} __attribute__((packed)) resp_cmd_header;

typedef struct {
    uint8_t cmd_id;
    uint8_t direction;
    uint8_t transport;
    uint8_t subcmd_id;
    uint8_t _reserved1;
    uint8_t data_len;
    uint8_t _reserved2[0x2];
} __attribute__((packed)) req_cmd_header;

// COMMAND 0x02 FLASH MEMORY RESPONSES
typedef struct {
    uint8_t bytes_read;
    uint8_t _reserved[0x3];
    uint32_t read_address;
    uint8_t data[0x40];
} __attribute__((packed)) resp_read_memory_block; // cmd 0x01

typedef struct {
    uint8_t bytes_read;
    uint8_t _reserved[0x3];
    uint32_t read_address;
    uint8_t data[0x40];
} __attribute__((packed)) resp_write_memory_block; // cmd 0x02

typedef struct {
    uint8_t bytes_read;
    uint8_t _reserved[0x3];
    uint32_t read_address;
    uint8_t data[0x40];
} __attribute__((packed)) resp_erase_memory_sector; // cmd 0x03

typedef struct {
    uint8_t bytes_read;
    uint8_t _reserved[0x3];
    uint32_t read_address;
    uint8_t data[];
} __attribute__((packed)) resp_memory_read; // cmd 0x04

typedef struct {
    uint32_t _reserved;
    uint32_t write_address;
} __attribute__((packed)) resp_memory_write; // cmd 0x05


// COMMAND 0x03 INITIALIZATION RESPONSES
typedef struct {
    uint8_t _reserved[0x4];
} __attribute__((packed)) resp_init_usb; // cmd 0x0D

// COMMAND 0x07 UNKNOWN RESPONSES
typedef struct {
    uint8_t _reserved;
} __attribute__((packed)) resp_unknown07_01; // cmd 0x01

// COMMNAND 0x08 CHARGING GRIP RESPONSES
typedef struct {
    uint8_t _reserved[0x4];
    uint8_t data[0x20];
} __attribute__((packed)) resp_get_grip_info_0x20; // cmd 0x01

typedef struct {
    uint8_t _reserved[0x4];
    uint8_t data[0x40];
} __attribute__((packed)) resp_get_grip_info_0x40; // cmd 0x03

// COMMAND 0x0B BATTERY RESPONSES
typedef struct {
    uint16_t voltage;
    uint8_t _reserved[0x2];
} __attribute__((packed)) resp_get_battery_voltage; // cmd 0x03

typedef struct {
    uint16_t status;
    uint8_t _reserved[0x2];
} __attribute__((packed)) resp_get_charge_status; // cmd 0x04

typedef struct {
    uint8_t _reserved[0x4];
} __attribute__((packed)) resp_battery_unknown_06; // cmd 0x06

// COMMAND 0x0C FEATURE SELECT RESPONSES
typedef struct {
    uint8_t _reserved[0x4];
    uint8_t feature_info[0x8];
} __attribute__((packed)) resp_get_feature_info; // cmd 0x01

typedef struct {
    uint8_t _reserved[0x4];
} __attribute__((packed)) resp_set_feature_mask; // cmd 0x02

typedef struct {
    uint8_t _reserved[0x4];
} __attribute__((packed)) resp_clear_feature_mask; // cmd 0x03

typedef struct {
    uint8_t _reserved[0x4];
} __attribute__((packed)) resp_enable_features; // cmd 0x04

typedef struct {
    uint8_t _reserved[0x4];
} __attribute__((packed)) resp_disable_features; // cmd 0x05

typedef struct {
    uint8_t _reserved[0x4];
    uint8_t data_len;
    uint8_t data[0x20];
} __attribute__((packed)) resp_configure_features; // cmd 0x06

// COMMAND 0x10 FIRMWARE INFO RESPONSES
typedef struct {
    uint8_t controller_major;
    uint8_t controller_minor;
    uint8_t controller_micro;
    uint8_t controller_type;
    uint8_t bluetooth_major;
    uint8_t bluetooth_minor;
    uint8_t bluetooth_micro;
    uint8_t reserved;
    uint8_t dsp_major;
    uint8_t dsp_minor;
    uint8_t dsp_micro;
} __attribute__((packed)) resp_get_firmware_ver_info; // cmd 0x01

// COMMAND 0x15 BLUETOOTH PAIRING
typedef struct {
    uint8_t _reserved[0x2];
    uint8_t count;
    uint8_t address[0x6];
} __attribute__((packed)) resp_exchange_address; // cmd 0x01

typedef struct {
    uint8_t _reserved;
    uint8_t response[0x10];
} __attribute__((packed)) resp_confirm_ltk; // cmd 0x02

typedef struct {
    uint8_t _reserved;
} __attribute__((packed)) resp_finalize_pairing; // cmd 0x03

typedef struct {
    uint8_t _reserved;
    uint8_t device_key[0x10];
} __attribute__((packed)) resp_exchange_keys; // cmd 0x04


// HID RESPONSES

typedef struct {
    uint32_t counter;
    uint32_t buttons; // bitfield
    uint16_t _reserved1;
    uint8_t left_stick[0x3]; // 12-bit packed
    uint8_t right_stick[0x3]; 
    uint16_t mouse_x;
    uint16_t mouse_y;
    uint16_t _mouse_unknown;
    uint16_t mouse_dist;
    uint8_t _reserved2;
    uint16_t mag_x;
    uint16_t mag_y;
    uint16_t mag_z;
    uint16_t battery_voltage;
    uint8_t  battery_chrg_rate;
    int16_t battery_current;
    uint8_t  _reserved3[0x6];
    int32_t motion_timestamp;
    int16_t motion_temperature;
    int16_t motion_accel_x;
    int16_t motion_accel_y;
    int16_t motion_accel_z;
    int16_t motion_gyro_x;
    int16_t motion_gyro_y;
    int16_t motion_gyro_z;
    uint8_t  left_analog_trigger;
    uint8_t  right_analog_trigger;
    uint8_t  reserved4;
} __attribute__((packed)) hid_output_report_05;

typedef struct {
    uint8_t header[15];
    uint8_t data[50];
} __attribute__((packed)) hid_ble_output_report_2E;