#include "initializer.h"
#include "calibration.h"
#include "commands.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

// -----------------------------------------------------------------------
// SPI reply layout
//
// A SPI READ reply on USB bulk 0x82 is:
//   [0]    = cmdId  (0x02)
//   [1]    = 0x01   (reply marker)
//   [2]    = 0x00   (USB comm type)
//   [3]    = 0x04   (SPI_READ subcmd)
//   [4]    = unk
//   [5]    = result (0xF8 = ACK USB)
//   [6-7]  = unk
//   [8-11] = read_length (uint32 LE)
//   [12-15]= address     (uint32 LE)
//   [16+]  = data
// -----------------------------------------------------------------------

#define SPI_REPLY_HEADER   16
#define SPI_CMD_ID         0x02
#define SPI_SUBCMD_READ    0x04

static void BulkCommand(libusb_device_handle* handle,
                        const uint8_t* payload, int payload_len,
                        uint8_t* reply_out, int* reply_len_out) {
    int transferred = 0;
    int rc;

    rc = libusb_bulk_transfer(handle, 0x02,
                              (unsigned char*)payload, payload_len,
                              &transferred, 1000);
    if (rc != 0) {
        fprintf(stderr, "[init] bulk write error: %s\n", libusb_error_name(rc));
        if (reply_len_out) *reply_len_out = 0;
        return;
    }

    usleep(50 * 1000);

    if (reply_out && reply_len_out) {
        uint8_t buf[80];
        rc = libusb_bulk_transfer(handle, 0x82, buf, sizeof(buf),
                                  &transferred, 1000);
        if (rc == 0) {
            int n = transferred < 80 ? transferred : 80;
            memcpy(reply_out, buf, n);
            *reply_len_out = n;
        } else {
            fprintf(stderr, "[init] bulk read error: %s\n", libusb_error_name(rc));
            *reply_len_out = 0;
        }
    } else {
        // Fire and forget — still need to drain the reply
        uint8_t discard[80];
        libusb_bulk_transfer(handle, 0x82, discard, sizeof(discard),
                             &transferred, 1000);
    }

    usleep(50 * 1000);
}

// Convenience macro for commands where we don't care about the reply
#define CMD(handle, ...) \
    do { \
        uint8_t _p[] = { __VA_ARGS__ }; \
        BulkCommand(handle, _p, sizeof(_p), NULL, NULL); \
    } while(0)

// -----------------------------------------------------------------------
// Parse a SPI READ reply and extract the data payload.
// Returns the number of data bytes extracted (0 on error).
// -----------------------------------------------------------------------
static int ExtractSpiData(const uint8_t* reply, int reply_len,
                           uint32_t expected_addr,
                           uint8_t* data_out, int data_max) {
    if (reply_len < SPI_REPLY_HEADER) return 0;
    if (reply[0] != SPI_CMD_ID)       return 0;
    if (reply[3] != SPI_SUBCMD_READ)  return 0;

    uint32_t got_addr = reply[12] | ((uint32_t)reply[13] << 8)
                      | ((uint32_t)reply[14] << 16) | ((uint32_t)reply[15] << 24);
    if (got_addr != expected_addr) {
        fprintf(stderr, "[init] SPI addr mismatch: expected 0x%06X got 0x%06X\n",
                expected_addr, got_addr);
        return 0;
    }

    int data_len = reply_len - SPI_REPLY_HEADER;
    if (data_len > data_max) data_len = data_max;
    memcpy(data_out, reply + SPI_REPLY_HEADER, data_len);
    return data_len;
}

// -----------------------------------------------------------------------
// Init sequence + calibration extraction
// -----------------------------------------------------------------------

static void Init(libusb_device_handle* handle, Sw2CalibrationRaw* cal) {
    uint8_t reply[80];
    int     rlen;

    // --- Fixed init commands (no useful reply data) ---
    CMD(handle, 0x03,0x91,0x00,0x0D,0x00,0x08,0x00,0x00,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF);
    CMD(handle, 0x07,0x91,0x00,0x01,0x00,0x00,0x00,0x00);
    CMD(handle, 0x16,0x91,0x00,0x01,0x00,0x00,0x00,0x00);
    CMD(handle, 0x0B,0x91,0x00,0x07,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00);
    CMD(handle, 0x15,0x91,0x00,0x01,0x00,0x0E,0x00,0x00,0x00,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF);
    CMD(handle, 0x15,0x91,0x00,0x02,0x00,0x11,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF);
    CMD(handle, 0x15,0x91,0x00,0x03,0x00,0x01,0x00,0x00,0x00);
    CMD(handle, 0x09,0x91,0x00,0x07,0x00,0x08,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
    CMD(handle, 0x0C,0x91,0x00,0x02,0x00,0x04,0x00,0x00,0xFF,0x00,0x00,0x00); // FEATURE_INIT  all
    CMD(handle, 0x0C,0x91,0x00,0x04,0x00,0x04,0x00,0x00,0xFF,0x00,0x00,0x00); // FEATURE_ENABLE all

    // --- SPI: CalibrationA block at 0x013080 (0x40 bytes) ---
    // Contains factory stick cal A at offset +0x28 = 0x0130A8
    {
        static const uint8_t spi_cmd[] = {
            0x02,0x91,0x00,0x04,0x00,0x08,0x00,0x00,
            0x40,0x7E,0x00,0x00, 0x80,0x30,0x01,0x00
        };
        BulkCommand(handle, spi_cmd, sizeof(spi_cmd), reply, &rlen);
        uint8_t block[64];
        if (ExtractSpiData(reply, rlen, 0x013080, block, sizeof(block)) >= 40) {
            // FactoryCalJoystickA is at 0x0130A8 = offset 0x28 into the block
            if (cal) {
                memcpy(cal->stick_l_raw, block + 0x28, 11);
                cal->stick_l_loaded = 1;
                printf("[init] Left stick factory cal loaded\n");
            }
        }
    }

    // --- SPI: CalibrationB block at 0x0130C0 (0x40 bytes) ---
    // Contains factory stick cal B at offset +0x28 = 0x0130E8
    {
        static const uint8_t spi_cmd[] = {
            0x02,0x91,0x00,0x04,0x00,0x08,0x00,0x00,
            0x40,0x7E,0x00,0x00, 0xC0,0x30,0x01,0x00
        };
        BulkCommand(handle, spi_cmd, sizeof(spi_cmd), reply, &rlen);
        uint8_t block[64];
        if (ExtractSpiData(reply, rlen, 0x0130C0, block, sizeof(block)) >= 40) {
            if (cal) {
                memcpy(cal->stick_r_raw, block + 0x28, 11);
                cal->stick_r_loaded = 1;
                printf("[init] Right stick factory cal loaded\n");
            }
        }
    }

    // --- SPI: User cal left at 0x1FC040 (0x40 bytes, check magic) ---
    {
        static const uint8_t spi_cmd[] = {
            0x02,0x91,0x00,0x04,0x00,0x08,0x00,0x00,
            0x40,0x7E,0x00,0x00, 0x40,0xC0,0x1F,0x00
        };
        BulkCommand(handle, spi_cmd, sizeof(spi_cmd), reply, &rlen);
        uint8_t block[64];
        if (ExtractSpiData(reply, rlen, 0x1FC040, block, sizeof(block)) >= 13 && cal) {
            uint16_t magic = block[0] | ((uint16_t)block[1] << 8);
            if (magic == SPI_CAL_USER_MAGIC) {
                // User cal left stick starts at byte 2
                memcpy(cal->user_stick_l_raw, block + 2, 11);
                cal->user_magic  = magic;
                cal->user_loaded = 1;
                printf("[init] User left stick cal found\n");
            }
        }
    }

    // --- SPI: Factory motion cal at 0x013100 (0x18 = 24 bytes) ---
    {
        static const uint8_t spi_cmd[] = {
            0x02,0x91,0x00,0x04,0x00,0x08,0x00,0x00,
            0x18,0x7E,0x00,0x00, 0x00,0x31,0x01,0x00
        };
        BulkCommand(handle, spi_cmd, sizeof(spi_cmd), reply, &rlen);
        uint8_t block[24];
        if (ExtractSpiData(reply, rlen, 0x013100, block, sizeof(block)) == 24 && cal) {
            memcpy(cal->motion_raw, block, 24);
            cal->motion_loaded = 1;
            printf("[init] Motion factory cal loaded\n");
        }
    }

    // --- SPI: read at 0x013040 (magnetometer cal, 0x10 bytes) ---
    {
        static const uint8_t spi_cmd[] = {
            0x02,0x91,0x00,0x04,0x00,0x08,0x00,0x00,
            0x10,0x7E,0x00,0x00, 0x40,0x30,0x01,0x00
        };
        BulkCommand(handle, spi_cmd, sizeof(spi_cmd), reply, &rlen);
        // Not yet used — stored for future magnetometer support
        (void)reply; (void)rlen;
    }

    // --- REPORT_11 / IMU float scale factors ---
    CMD(handle, 0x11,0x91,0x00,0x03,0x00,0x00,0x00,0x00);

    // --- SPI: 0x013060 (joycon-only region, 0x20 bytes) ---
    {
        static const uint8_t spi_cmd[] = {
            0x02,0x91,0x00,0x04,0x00,0x08,0x00,0x00,
            0x20,0x7E,0x00,0x00, 0x60,0x30,0x01,0x00
        };
        BulkCommand(handle, spi_cmd, sizeof(spi_cmd), reply, &rlen);
        (void)reply; (void)rlen;
    }

    // --- Remaining fixed init ---
    CMD(handle, 0x0A,0x91,0x00,0x08,0x00,0x14,0x00,0x00,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x35,0x00,0x46,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
    CMD(handle, 0x03,0x91,0x00,0x0A,0x00,0x04,0x00,0x00,0x09,0x00,0x00,0x00);
    CMD(handle, 0x10,0x91,0x00,0x01,0x00,0x00,0x00,0x00);
    CMD(handle, 0x01,0x91,0x00,0x0C,0x00,0x00,0x00,0x00);
    CMD(handle, 0x18,0x91,0x00,0x01,0x00,0x00,0x00,0x00);
    CMD(handle, 0x03,0x91,0x00,0x01,0x00,0x00,0x00);
    CMD(handle, HAPTICS,REQUEST_MARKER,COMM_BLE,VIBRATION_PLAY_SAMPLE,0x00,0x04,0x00,0x00,0x03,0x00,0x00); // connect sound
    CMD(handle, 0x09,0x91,0x00,0x07,0x00,0x08,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // LED P1

    CMD(handle, 0x0C,0x91,0x00,0x02,0x00,0x04,0x00,0x00,0xFF,0x00,0x00,0x00); // FEATURE_INIT  all
    CMD(handle, 0x0C,0x91,0x00,0x04,0x00,0x04,0x00,0x00,0xFF,0x00,0x00,0x00); // FEATURE_ENABLE all
}

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------

bool Initializer_IsCompatible(libusb_device* dev,
                               const DeviceId* allowed, int allowed_count) {
    struct libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(dev, &desc) < 0) return false;

    for (int i = 0; i < allowed_count; i++) {
        if (desc.idVendor  == allowed[i].vid &&
            desc.idProduct == allowed[i].pid)
            return true;
    }
    return false;
}

libusb_device_handle* Initializer_PrepareDevice(libusb_device* dev,
                                                 Sw2CalibrationRaw* cal_raw) {
    libusb_device_handle* handle = NULL;
    int rc;

    rc = libusb_open(dev, &handle);
    if (rc != 0) {
        fprintf(stderr, "[-] libusb_open failed: %s\n", libusb_error_name(rc));
        return NULL;
    }

    for (int i = 0; i < 2; i++) {
        if (libusb_kernel_driver_active(handle, i) == 1) {
            rc = libusb_detach_kernel_driver(handle, i);
            if (rc != 0)
                fprintf(stderr, "[-] detach_kernel_driver %d: %s\n",
                        i, libusb_error_name(rc));
        }
        rc = libusb_claim_interface(handle, i);
        if (rc != 0) {
            fprintf(stderr, "[-] claim_interface %d failed: %s\n",
                    i, libusb_error_name(rc));
            libusb_close(handle);
            return NULL;
        }
    }

    libusb_set_interface_alt_setting(handle, 0, 0);
    libusb_set_interface_alt_setting(handle, 1, 0);
    libusb_clear_halt(handle, 0x02);
    libusb_clear_halt(handle, 0x82);

    if (cal_raw) memset(cal_raw, 0, sizeof(*cal_raw));
    Init(handle, cal_raw);

    return handle;
}

void Initializer_GetDeviceId(libusb_device* dev, char* buf, int buf_len) {
    snprintf(buf, buf_len, "%03d:%03d",
             libusb_get_bus_number(dev),
             libusb_get_device_address(dev));
}