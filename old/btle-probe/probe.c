#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include "commands.h"
#include <poll.h>
#include <ctype.h>

// -----------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------

#define CONTROLLER_MAC  "94:8E:6D:2C:A6:E2"
//#define CONTROLLER_MAC  "BC:89:A6:98:25:2F"
#define ADAPTER_MAC     "CC:5E:F8:B2:53:00"

// Known handles from btmon + Windows implementation
// input reports (notify):  ab7de9be-89fe-49ad-828f-118f09df7fd2  -> 0x0009
// write commands (write):  649d4ac9-8eb7-4e6c-af44-1ea54fe5f005  -> 0x000B
#define HANDLE_INPUT    0x0009
#define HANDLE_CCCD     0x000B
#define HANDLE_WRITE    0x0014

// Service 2 range to search for CCCD
#define SERVICE2_START  0x0008
#define SERVICE2_END    0x0032

// -----------------------------------------------------------------------
// ATT opcodes
// -----------------------------------------------------------------------

#define ATT_CID                 4
#define ATT_ERROR_RSP           0x01
#define ATT_WRITE_REQ           0x12
#define ATT_WRITE_RSP           0x13
#define ATT_WRITE_CMD           0x52
#define ATT_HANDLE_NOTIFY       0x1B
#define ATT_READ_BY_TYPE_REQ    0x08
#define ATT_READ_BY_TYPE_RSP    0x09

// CCCD UUID (0x2902)
#define UUID_CCCD_LO    0x02
#define UUID_CCCD_HI    0x29

// -----------------------------------------------------------------------
// Init sequence
// -----------------------------------------------------------------------

typedef struct { const uint8_t* data; int len; } Cmd;

static const uint8_t c00[]      = {INIT,REQUEST_MARKER,COMM_BLE,INIT_USB_INIT,0x00,0x08,0x00,0x00,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t c01[]      = {CMD_07,REQUEST_MARKER,COMM_BLE,CMD_07_INIT,0x00,0x00,0x00,0x00};
static const uint8_t c02[]      = {CMD_16,REQUEST_MARKER,COMM_BLE,CMD_16_UNKNOWN_01,0x00,0x00,0x00,0x00};
static const uint8_t c03[]      = {BATTERY,REQUEST_MARKER,COMM_BLE,BATTERY_UNKNOWN_07,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t c04[]      = {PAIRING,REQUEST_MARKER,COMM_BLE,PAIRING_SET_ADDRESS,0x00,0x0E,0x00,0x00,0x00,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t c05[]      = {PAIRING,REQUEST_MARKER,COMM_BLE,PAIRING_CONFIRM_LTK,0x00,0x11,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t c06[]      = {PAIRING,REQUEST_MARKER,COMM_BLE,PAIRING_FINALIZE,0x00,0x01,0x00,0x00,0x00};
static const uint8_t c07[]      = {LED,REQUEST_MARKER,COMM_BLE,LED_SET_PATTERN,0x00,0x08,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t c08[]      = {FEATURES,REQUEST_MARKER,COMM_BLE,FEATURE_SET_MASK,0x00,0x04,0x00,0x00,FEATURE_ALL,0x00,0x00,0x00};
static const uint8_t c09[]      = {FEATURES,REQUEST_MARKER,COMM_BLE,FEATURE_ENABLE,0x00,0x04,0x00,0x00,FEATURE_ALL,0x00,0x00,0x00};
static const uint8_t c10[]      = {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,0x00,0x00,0x40,0x7E,0x00,0x00,0x80,0x30,0x01,0x00}; // Gets MAC
static const uint8_t c11[]      = {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,0x00,0x00,0x40,0x7E,0x00,0x00,0xC0,0x30,0x01,0x00}; // Gets Factory Stick Calibration Data
static const uint8_t c12[]      = {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,0x00,0x00,0x40,0x7E,0x00,0x00,0x40,0xC0,0x1F,0x00};
static const uint8_t c13[]      = {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,0x00,0x00,0x10,0x7E,0x00,0x00,0x40,0x30,0x01,0x00};
static const uint8_t c14[]      = {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,0x00,0x00,0x18,0x7E,0x00,0x00,0x00,0x31,0x01,0x00};
static const uint8_t c15[]      = {CMD_11,REQUEST_MARKER,COMM_BLE,CMD_11_UNKNOWN_03,0x00,0x00,0x00,0x00}; // Get Factory IMU Calibration Float scale factors
static const uint8_t c16[]      = {SPI,REQUEST_MARKER,COMM_BLE,SPI_READ,0x00,0x08,0x00,0x00,0x20,0x7E,0x00,0x00,0x60,0x30,0x01,0x00};
static const uint8_t c17[]      = {HAPTICS,REQUEST_MARKER,COMM_BLE,VIBRATION_CONNECT,0x00,0x14,0x00,0x00,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x35,0x00,0x46,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t c18[]      = {INIT,REQUEST_MARKER,COMM_BLE,INIT_INPUT_REPORT,0x00,0x04,0x00,0x00,0x09,0x00,0x00,0x00};
static const uint8_t c19[]      = {FIRMWARE_INFO,REQUEST_MARKER,COMM_BLE,0x01,0x00,0x00,0x00,0x00};
static const uint8_t c20[]      = {NFC,REQUEST_MARKER,COMM_BLE,NFC_UNKNOWN_0C,0x00,0x00,0x00,0x00};
static const uint8_t c21[]      = {CMD_18,REQUEST_MARKER,COMM_BLE,CMD_18_UNKNOWN_01,0x00,0x00,0x00,0x00};
static const uint8_t c22[]      = {INIT,REQUEST_MARKER,COMM_BLE,INIT_WAKE_CONSOLE,0x00,0x00,0x00};
static const uint8_t c23[]      = {HAPTICS,REQUEST_MARKER,COMM_BLE,VIBRATION_PLAY_SAMPLE,0x00,0x04,0x00,0x00,0x03,0x00,0x00};

static const Cmd init_cmds[] = {
    {c00,sizeof(c00)},{c01,sizeof(c01)},{c02,sizeof(c02)},{c03,sizeof(c03)},
    {c04,sizeof(c04)},{c05,sizeof(c05)},{c06,sizeof(c06)},{c07,sizeof(c07)},
    {c08,sizeof(c08)},{c09,sizeof(c09)},{c10,sizeof(c10)},{c11,sizeof(c11)},
    {c12,sizeof(c12)},{c13,sizeof(c13)},{c14,sizeof(c14)},{c15,sizeof(c15)},
    {c16,sizeof(c16)},{c17,sizeof(c17)},{c18,sizeof(c18)},{c19,sizeof(c19)},
    {c20,sizeof(c20)},{c21,sizeof(c21)},{c22,sizeof(c22)},{c23,sizeof(c23)},
};
#define NUM_CMDS (sizeof(init_cmds) / sizeof(init_cmds[0]))


// -----------------------------------------------------------------------
// ATT helpers
// -----------------------------------------------------------------------

static int att_write_req(int sock, uint16_t handle, const uint8_t* data, int len) {
    uint8_t buf[512];
    buf[0] = ATT_WRITE_REQ;
    buf[1] = handle & 0xFF;
    buf[2] = (handle >> 8) & 0xFF;
    memcpy(&buf[3], data, len);

    if (write(sock, buf, len + 3) < 0) { perror("write"); return -1; }

    uint8_t resp[64];
    int n = read(sock, resp, sizeof(resp));
    if (n < 0) { perror("read"); return -1; }

    if (resp[0] == ATT_WRITE_RSP) return 1;

    if (resp[0] == ATT_ERROR_RSP && n >= 5) {
        printf("  ATT Error on handle 0x%04X: error=0x%02X\n",
            resp[2] | (resp[3] << 8), resp[4]);
    }
    return 0;
}

static void att_write_cmd(int sock, uint16_t handle, const uint8_t* data, int len) {
    uint8_t buf[512];
    buf[0] = ATT_WRITE_CMD;  // 0x52
    buf[1] = handle & 0xFF;
    buf[2] = (handle >> 8) & 0xFF;
    memcpy(&buf[3], data, len);
    write(sock, buf, len + 3);
    // no read - no response expected
}

// Search for CCCD (UUID 0x2902) in a handle range using Read By Type
// Returns the CCCD handle or 0 if not found
static uint16_t find_cccd(int sock, uint16_t start, uint16_t end) {
    printf("Searching for CCCD (0x2902) in handle range 0x%04X-0x%04X...\n", start, end);

    uint8_t req[7];
    req[0] = ATT_READ_BY_TYPE_REQ;
    req[1] = start & 0xFF;
    req[2] = (start >> 8) & 0xFF;
    req[3] = end & 0xFF;
    req[4] = (end >> 8) & 0xFF;
    req[5] = UUID_CCCD_LO;  // 0x2902 little-endian
    req[6] = UUID_CCCD_HI;

    if (write(sock, req, sizeof(req)) < 0) { perror("write"); return 0; }

    uint8_t resp[256];
    int n = read(sock, resp, sizeof(resp));
    if (n < 0) { perror("read"); return 0; }

    if (resp[0] == ATT_READ_BY_TYPE_RSP && n >= 6) {
        // resp[1] = length of each attribute data
        // resp[2..3] = first handle (little-endian)
        uint16_t cccd_handle = resp[2] | (resp[3] << 8);
        printf("Found CCCD at handle 0x%04X\n", cccd_handle);
        return cccd_handle;
    }

    if (resp[0] == ATT_ERROR_RSP && n >= 5) {
        printf("  CCCD not found via Read By Type (error=0x%02X)\n", resp[4]);
    }

    return 0;
}

static void drain(int sock) {
    fd_set fds;
    struct timeval tv = {0, 20000};
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    while (select(sock + 1, &fds, NULL, NULL, &tv) > 0) {
        uint8_t buf[256];
        int n = read(sock, buf, sizeof(buf));
        if (n > 0 && buf[0] != ATT_HANDLE_NOTIFY) {
            printf("  Drained: opcode=0x%02X len=%d\n", buf[0], n);
        }
        tv.tv_usec = 20000;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
    }
}

// -----------------------------------------------------------------------
// Connect
// -----------------------------------------------------------------------

static int connect_att(void) {
    int sock = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sock < 0) { perror("socket"); return -1; }

    struct sockaddr_l2 local = {0};
    local.l2_family      = AF_BLUETOOTH;
    local.l2_cid         = htobs(ATT_CID);
    local.l2_bdaddr_type = BDADDR_LE_PUBLIC;
    str2ba(ADAPTER_MAC, &local.l2_bdaddr);

    if (bind(sock, (struct sockaddr*)&local, sizeof(local)) < 0) {
        perror("bind"); close(sock); return -1;
    }

    struct sockaddr_l2 remote = {0};
    remote.l2_family      = AF_BLUETOOTH;
    remote.l2_cid         = htobs(ATT_CID);
    remote.l2_bdaddr_type = BDADDR_LE_PUBLIC;
    str2ba(CONTROLLER_MAC, &remote.l2_bdaddr);

    printf("Connecting to %s ...\n", CONTROLLER_MAC);
    if (connect(sock, (struct sockaddr*)&remote, sizeof(remote)) < 0) {
        perror("connect"); close(sock); return -1;
    }

    uint8_t mtu_req[] = {0x02, 0x00, 0x02}; 
    if (write(sock, mtu_req, sizeof(mtu_req)) < 0) {
        perror("MTU req write failed");
        close(sock); return -1;
    }

    // Wait for MTU Response to ensure the ATT layer is synced
    uint8_t resp[64];
    int n = read(sock, resp, sizeof(resp));
    if (n > 0 && resp[0] == 0x03) {
        uint16_t server_mtu = resp[1] | (resp[2] << 8);
        printf("MTU Exchanged. Server supports: %d\n", server_mtu);
    } else {
        printf("MTU Exchange failed or timed out. This may cause disconnects later.\n");
    }

    printf("Connected.\n");
    return sock;
}

// -----------------------------------------------------------------------
// Enable notifications
// -----------------------------------------------------------------------

static int read_att_response(int sock) {
    // Keep reading and discarding notifications until we get a real ATT response
    while (1) {
        fd_set fds;
        struct timeval tv = {0, 500000}; // 500ms timeout
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        if (select(sock + 1, &fds, NULL, NULL, &tv) <= 0) return -1; // timeout

        uint8_t resp[512];
        int n = read(sock, resp, sizeof(resp));
        if (n <= 0) return -1;

        if (resp[0] == ATT_HANDLE_NOTIFY) {
            // Discard and keep waiting
            continue;
        }
        return resp[0]; // return the opcode
    }
}

static int enable_notifications(int sock) {
    // Based on your Lua script: 
    // 0x000E: Standard HID
    // 0x0012: Vibration Only
    // 0x0016: Long Form / Commands
    // 0x002A: Pro Extended Report
    // We target the CCCD handles (Handle + 1)
    uint16_t cccd_handles[] = {
        0x000B, 0x000F, 0x0013, 0x0017, 
        0x001B, 0x001F, 0x0023, 0x0027, 
        0x002B, 0x002F
    };
    uint8_t notify_enable[] = {0x01, 0x00};
    int success_count = 0;

    printf("\nEnabling channels based on Dissector logic...\n");

    for (int i = 0; i < (int)(sizeof(cccd_handles) / sizeof(cccd_handles[0])); i++) {
        uint8_t buf[5];
        buf[0] = ATT_WRITE_REQ;
        buf[1] = cccd_handles[i] & 0xFF;
        buf[2] = (cccd_handles[i] >> 8) & 0xFF;
        buf[3] = 0x01;
        buf[4] = 0x00;
        
        // We use write instead of att_write_req to avoid the blocking read
        // which was likely causing your previous hang.
        if (write(sock, buf, 5) >= 0) {
            success_count++;
            usleep(5000); // 5ms gap to prevent flooding the controller
        }
    }

    printf("Enabled %d channels. Controller ready.\n", success_count);
    return success_count;
}

// -----------------------------------------------------------------------
// Send init sequence
// -----------------------------------------------------------------------

static void ble_send_init_cmd(int sock, const uint8_t* payload, int len) {
    att_write_cmd(sock, HANDLE_WRITE, payload, len);

    fd_set fds;
    struct timeval tv = {0, 200000}; // 200ms
    FD_ZERO(&fds);
    FD_SET(sock, &fds);

    while (select(sock + 1, &fds, NULL, NULL, &tv) > 0) {
        uint8_t buf[512];
        int n = read(sock, buf, sizeof(buf));
        if (n <= 0) break;

        if (buf[0] == ATT_HANDLE_NOTIFY) {
            uint16_t handle = buf[1] | (buf[2] << 8);
            if (handle == 0x001A) {
                printf("  << ACK 0x001A [%d bytes]:\n", n - 3);
                break;
            }
            // Log unexpected notifications rather than silently dropping them
            printf("  ?? NOTIFY on unexpected handle 0x%04X [%d]: ", handle, n - 3);
            for (int i = 3; i < n; i++) printf("%02X ", buf[i]);
            printf("\n");
        }

        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        tv.tv_usec = 200000;
    }
}

static void send_init(int sock) {
    printf("\nSending BLE-wrapped init sequence...\n");
    for (int i = 0; i < (int)NUM_CMDS; i++) {
        printf("\n[CMD %02d/%02d]\n", i, (int)NUM_CMDS - 1);
        ble_send_init_cmd(sock, init_cmds[i].data, init_cmds[i].len);
    }
    printf("\nInit complete.\n");
}

// -----------------------------------------------------------------------
// Input report parsing (mirrors input_handler.lua)
// -----------------------------------------------------------------------

// Input report type IDs
#define INPUT_REPORT_NULL     0x00
#define INPUT_REPORT_02       0x02
#define INPUT_REPORT_LEFT     0x07
#define INPUT_REPORT_RIGHT    0x08
#define INPUT_REPORT_PRO      0x09
#define INPUT_REPORT_GC       0x0A

// Button bitmasks (standard layout)
#define BTN_B           0x000001
#define BTN_A           0x000002
#define BTN_Y           0x000004
#define BTN_X           0x000008
#define BTN_R           0x000010
#define BTN_ZR          0x000020
#define BTN_PLUS        0x000040
#define BTN_STICK_R     0x000080
#define BTN_DOWN        0x000100
#define BTN_RIGHT       0x000200
#define BTN_LEFT        0x000400
#define BTN_UP          0x000800
#define BTN_L           0x001000
#define BTN_ZL          0x002000
#define BTN_MINUS       0x004000
#define BTN_STICK_L     0x008000
#define BTN_HOME        0x010000
#define BTN_CAPTURE     0x020000
#define BTN_GR          0x040000
#define BTN_GL          0x080000
#define BTN_C           0x100000
#define BTN_SR          0x400000
#define BTN_SL          0x800000

// Button bitmasks (format A - BLE common report)
#define BTN_A_Y           0x00000001
#define BTN_A_X           0x00000002
#define BTN_A_B           0x00000004
#define BTN_A_A           0x00000008
#define BTN_A_SR_RIGHT    0x00000010
#define BTN_A_SL_RIGHT    0x00000020
#define BTN_A_R           0x00000040
#define BTN_A_ZR          0x00000080
#define BTN_A_MINUS       0x00000100
#define BTN_A_PLUS        0x00000200
#define BTN_A_STICK_R     0x00000400
#define BTN_A_STICK_L     0x00000800
#define BTN_A_HOME        0x00001000
#define BTN_A_CAPTURE     0x00002000
#define BTN_A_C           0x00004000
#define BTN_A_DOWN        0x00010000
#define BTN_A_UP          0x00020000
#define BTN_A_RIGHT       0x00040000
#define BTN_A_LEFT        0x00080000
#define BTN_A_SR_LEFT     0x00100000
#define BTN_A_SL_LEFT     0x00200000
#define BTN_A_L           0x00400000
#define BTN_A_ZL          0x00800000
#define BTN_A_GR          0x01000000
#define BTN_A_GL          0x02000000
#define BTN_A_HEADSET     0x10000000

// Read a little-endian uint16 from a buffer
static inline uint16_t read_le16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

// Read a little-endian int16 from a buffer
static inline int16_t read_le16s(const uint8_t *p) {
    return (int16_t)(p[0] | (p[1] << 8));
}

// Read a little-endian uint32 from a buffer
static inline uint32_t read_le32(const uint8_t *p) {
    return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

// Read a 12-bit packed axis pair (3 bytes -> two 12-bit values)
// Layout: byte0=lo8 of X, byte1=hi4 of X | lo4 of Y (nibbles), byte2=hi8 of Y
static inline void read_stick_12bit(const uint8_t *p, uint16_t *axis_x, uint16_t *axis_y) {
    *axis_x = p[0] | ((p[1] & 0x0F) << 8);
    *axis_y = (p[2] << 4) | (p[1] >> 4);
}

// Normalise a raw stick axis value to [-1.0, +1.0]
static float parse_raw_axis(uint16_t raw, uint16_t center, uint16_t max_range, uint16_t min_range) {
    int32_t value = (int32_t)raw - (int32_t)center;
    if (value > 0) return (float)value / (float)max_range;
    return (float)value / (float)min_range;
}

// Parse left stick (calibration constants from input_handler.lua)
static void parse_left_stick(const uint8_t *p, float *x, float *y) {
    uint16_t raw_x, raw_y;
    read_stick_12bit(p, &raw_x, &raw_y);
    *x = parse_raw_axis(raw_x, 1968, 1626, 1695);
    *y = parse_raw_axis(raw_y, 2194, 1534, 1527);
}

// Parse right stick (calibration constants from input_handler.lua)
static void parse_right_stick(const uint8_t *p, float *x, float *y) {
    uint16_t raw_x, raw_y;
    read_stick_12bit(p, &raw_x, &raw_y);
    *x = parse_raw_axis(raw_x, 2253, 1457, 1612);
    *y = parse_raw_axis(raw_y, 2104, 1574, 1629);
}

// Decode a 12-bit IMU timestamp sample (same packing as stick axes)
static uint16_t parse_imu_sample(const uint8_t *p) {
    return (uint16_t)(p[0] | ((p[1] & 0x0F) << 8));
}

// Convert raw temperature value to degrees Celsius
// Formula from input_handler.lua: 25 + (raw / 127.0)
static float parse_temperature(int16_t raw) {
    return 25.0f + ((float)raw / 127.0f);
}

// Build a human-readable button string from standard button bitmask.
// out must be at least 128 bytes.
static void format_buttons(uint32_t bits, char *out, size_t out_sz) {
    static const struct { uint32_t bit; const char *name; } map[] = {
        { BTN_DOWN,    "down"    }, { BTN_UP,      "up"      },
        { BTN_RIGHT,   "right"   }, { BTN_LEFT,    "left"    },
        { BTN_SR,      "SR"      }, { BTN_SL,      "SL"      },
        { BTN_L,       "L"       }, { BTN_ZL,      "ZL"      },
        { BTN_Y,       "Y"       }, { BTN_X,       "X"       },
        { BTN_B,       "B"       }, { BTN_A,       "A"       },
        { BTN_R,       "R"       }, { BTN_ZR,      "ZR"      },
        { BTN_MINUS,   "minus"   }, { BTN_PLUS,    "plus"    },
        { BTN_STICK_R, "stickR"  }, { BTN_STICK_L, "stickL"  },
        { BTN_HOME,    "home"    }, { BTN_CAPTURE, "capture" },
        { BTN_C,       "C"       }, { BTN_GR,      "GR"      },
        { BTN_GL,      "GL"      },
    };
    int first = 1;
    size_t pos = 0;
    pos += snprintf(out + pos, out_sz - pos, "(");
    for (size_t i = 0; i < sizeof(map)/sizeof(map[0]); i++) {
        if (bits & map[i].bit) {
            pos += snprintf(out + pos, out_sz - pos, "%s%s", first ? "" : ", ", map[i].name);
            first = 0;
        }
    }
    if (first) pos += snprintf(out + pos, out_sz - pos, "none");
    snprintf(out + pos, out_sz - pos, ")");
}

// Build a human-readable button string from format-A bitmask (BLE common report).
static void format_buttons_a(uint32_t bits, char *out, size_t out_sz) {
    static const struct { uint32_t bit; const char *name; } map[] = {
        { BTN_A_DOWN,    "down"    }, { BTN_A_UP,      "up"      },
        { BTN_A_RIGHT,   "right"   }, { BTN_A_LEFT,    "left"    },
        { BTN_A_SR_LEFT, "SR"      }, { BTN_A_SL_LEFT, "SL"      },
        { BTN_A_SR_RIGHT,"SR"      }, { BTN_A_SL_RIGHT,"SL"      },
        { BTN_A_L,       "L"       }, { BTN_A_ZL,      "ZL"      },
        { BTN_A_Y,       "Y"       }, { BTN_A_X,       "X"       },
        { BTN_A_B,       "B"       }, { BTN_A_A,       "A"       },
        { BTN_A_R,       "R"       }, { BTN_A_ZR,      "ZR"      },
        { BTN_A_MINUS,   "minus"   }, { BTN_A_PLUS,    "plus"    },
        { BTN_A_STICK_R, "stickR"  }, { BTN_A_STICK_L, "stickL"  },
        { BTN_A_HOME,    "home"    }, { BTN_A_CAPTURE, "capture" },
        { BTN_A_C,       "C"       }, { BTN_A_GR,      "GR"      },
        { BTN_A_GL,      "GL"      }, { BTN_A_HEADSET, "headset" },
    };
    int first = 1;
    size_t pos = 0;
    pos += snprintf(out + pos, out_sz - pos, "(");
    for (size_t i = 0; i < sizeof(map)/sizeof(map[0]); i++) {
        if (bits & map[i].bit) {
            pos += snprintf(out + pos, out_sz - pos, "%s%s", first ? "" : ", ", map[i].name);
            first = 0;
        }
    }
    if (first) pos += snprintf(out + pos, out_sz - pos, "none");
    snprintf(out + pos, out_sz - pos, ")");
}

// Print a simple IMU motion block (simple format: 2-byte timestamp only).
// Returns the IMU sample value for use in the info string.
static uint16_t print_motion_simple(const uint8_t *buf, int len) {
    if (len < 2) return 0;
    uint16_t sample = parse_imu_sample(buf);
    printf("    IMU sample: %u  raw[", sample);
    for (int i = 0; i < len; i++) printf("%02X%s", buf[i], i < len - 1 ? " " : "");
    printf("]\n");
    return sample;
}

// Print a full IMU motion block (format 2: 4-byte timestamp, temperature, accel xyz, gyro xyz).
// Buffer layout from input_handler.lua parse_motion2():
//   0x00: uint32 imu_sample (microseconds)
//   0x04: int16  temperature
//   0x06: int16  accel_x
//   0x08: int16  accel_y
//   0x0A: int16  accel_z
//   0x0C: int16  gyro_x
//   0x0E: int16  gyro_y
//   0x10: int16  gyro_z  (18 bytes total)
static void print_motion2(const uint8_t *buf, int len) {
    if (len < 18) {
        printf("    IMU: buffer too short (%d bytes)\n", len);
        return;
    }
    uint32_t sample_us  = read_le32(buf + 0x00);
    int16_t  temp_raw   = read_le16s(buf + 0x04);
    int16_t  accel_x    = read_le16s(buf + 0x06);
    int16_t  accel_y    = read_le16s(buf + 0x08);
    int16_t  accel_z    = read_le16s(buf + 0x0A);
    int16_t  gyro_x     = read_le16s(buf + 0x0C);
    int16_t  gyro_y     = read_le16s(buf + 0x0E);
    int16_t  gyro_z     = read_le16s(buf + 0x10);

    float temp_c = parse_temperature(temp_raw);
    float ts_sec = (float)sample_us / 1000000.0f;

    printf("    IMU: ts=%.3fs  temp=%.2fC  "
           "accel=(%d, %d, %d)  gyro=(%d, %d, %d)\n",
           ts_sec, temp_c,
           accel_x, accel_y, accel_z,
           gyro_x, gyro_y, gyro_z);
}

//
// parse_input_report() - main entry point.
//
// Mirrors switch2hid_protocol_dissector() from input_handler.lua.
// payload: pointer to the HID report data (after the ATT notify header)
// plen:    length of the payload in bytes
// handle:  the ATT notify handle (used to pick wireless format)
//
static void parse_input_report(const uint8_t *payload, int plen, uint16_t handle) {
    if (plen < 1) return;

    uint8_t report_type = payload[0];
    char btn_str[128];
    float sx, sy;

    switch (report_type) {

    // ----------------------------------------------------------------
    // 0x00 - Null / empty report
    // ----------------------------------------------------------------
    case INPUT_REPORT_NULL:
        printf("  [INPUT] Empty input report\n");
        break;

    // ----------------------------------------------------------------
    // 0x07 - Left Joy-Con input report
    // Layout (from parse_left_input_report):
    //   [0]       report type
    //   [1]       packet_id
    //   [2]       status
    //   [3..5]    buttons (3 bytes, LE)
    //   [6..8]    left stick (3 bytes, 12-bit packed)
    //   [9]       vibration_code
    //   [10..11]  mouse_x (LE16)
    //   [12..13]  mouse_y (LE16)
    //   [14]      (padding / reserved)
    //   [15]      imu_length
    //   [16..]    IMU motion buffer (imu_length bytes)
    // ----------------------------------------------------------------
    case INPUT_REPORT_LEFT: {
        if (plen < 16) { printf("  [INPUT/LEFT] Packet too short\n"); break; }
        uint8_t  packet_id    = payload[1];
        uint8_t  status       = payload[2];
        uint32_t buttons      = payload[3] | ((uint32_t)payload[4] << 8) | ((uint32_t)payload[5] << 16);
        uint16_t mouse_x      = read_le16(payload + 10);
        uint16_t mouse_y      = read_le16(payload + 12);
        uint8_t  imu_length   = payload[15];

        parse_left_stick(payload + 6, &sx, &sy);
        format_buttons(buttons, btn_str, sizeof(btn_str));

        printf("  [INPUT/LEFT]  pkt=%02X status=%02X  btns=%s  "
               "Lstick=(%.2f, %.2f)  mouse=(0x%04X, 0x%04X)  imuLen=%d\n",
               packet_id, status, btn_str, sx, sy,
               mouse_x, mouse_y, imu_length);

        if (imu_length > 0 && plen >= 16 + imu_length)
            print_motion_simple(payload + 16, imu_length);
        break;
    }

    // ----------------------------------------------------------------
    // 0x08 - Right Joy-Con input report
    // Layout (from parse_right_input_report) - same structure as Left
    // except [6..8] is right stick and mouse_x/y at same offsets.
    // ----------------------------------------------------------------
    case INPUT_REPORT_RIGHT: {
        if (plen < 16) { printf("  [INPUT/RIGHT] Packet too short\n"); break; }
        uint8_t  packet_id    = payload[1];
        uint8_t  status       = payload[2];
        uint32_t buttons      = payload[3] | ((uint32_t)payload[4] << 8) | ((uint32_t)payload[5] << 16);
        uint16_t mouse_x      = read_le16(payload + 10);
        uint16_t mouse_y      = read_le16(payload + 12);
        uint8_t  imu_length   = payload[15];

        parse_right_stick(payload + 6, &sx, &sy);
        format_buttons(buttons, btn_str, sizeof(btn_str));

        printf("  [INPUT/RIGHT] pkt=%02X status=%02X  btns=%s  "
               "Rstick=(%.2f, %.2f)  mouse=(0x%04X, 0x%04X)  imuLen=%d\n",
               packet_id, status, btn_str, sx, sy,
               mouse_x, mouse_y, imu_length);

        if (imu_length > 0 && plen >= 16 + imu_length)
            print_motion_simple(payload + 16, imu_length);
        break;
    }

    // ----------------------------------------------------------------
    // 0x09 - Pro Controller input report
    // Layout (from parse_pro_input_report):
    //   [0]       report type
    //   [1]       packet_id
    //   [2]       status
    //   [3..5]    buttons (3 bytes, LE)
    //   [6..8]    left  stick (3 bytes, 12-bit packed)
    //   [9..11]   right stick (3 bytes, 12-bit packed)
    //   [12]      vibration_code
    //   [13]      (padding)
    //   [14]      audio_status
    //   [15]      imu_length
    //   [16..]    IMU motion buffer
    // ----------------------------------------------------------------
    case INPUT_REPORT_PRO: {
        if (plen < 16) { printf("  [INPUT/PRO] Packet too short\n"); break; }
        uint8_t  packet_id    = payload[1];
        uint8_t  status       = payload[2];
        uint32_t buttons      = payload[3] | ((uint32_t)payload[4] << 8) | ((uint32_t)payload[5] << 16);
        uint8_t  vib_code     = payload[12];
        uint8_t  audio_status = payload[14];
        uint8_t  imu_length   = payload[15];
        float    rx, ry;

        parse_left_stick(payload + 6, &sx, &sy);
        parse_right_stick(payload + 9, &rx, &ry);
        format_buttons(buttons, btn_str, sizeof(btn_str));

        printf("  [INPUT/PRO]   pkt=%02X status=%02X  btns=%s  "
               "Lstick=(%.2f, %.2f)  Rstick=(%.2f, %.2f)  vib=%02X  audio=%02X  imuLen=%d\n",
               packet_id, status, btn_str, sx, sy, rx, ry,
               vib_code, audio_status, imu_length);

        if (imu_length > 0 && plen >= 16 + imu_length)
            print_motion_simple(payload + 16, imu_length);
        break;
    }

    // ----------------------------------------------------------------
    // 0x0A - GameCube Controller input report
    // Layout (from parse_gc_input_report):
    //   [0]       report type
    //   [1]       packet_id
    //   [2]       status
    //   [3..5]    buttons (3 bytes, LE)
    //   [6..8]    left  stick (3 bytes, 12-bit packed)
    //   [9..11]   right stick (3 bytes, 12-bit packed)
    //   [12]      vibration_code
    //   [13]      left  analog trigger
    //   [14]      right analog trigger
    //   [15]      imu_length
    //   [16..]    IMU motion buffer
    // ----------------------------------------------------------------
    case INPUT_REPORT_GC: {
        if (plen < 16) { printf("  [INPUT/GC] Packet too short\n"); break; }
        uint8_t  packet_id   = payload[1];
        uint8_t  status      = payload[2];
        uint32_t buttons     = payload[3] | ((uint32_t)payload[4] << 8) | ((uint32_t)payload[5] << 16);
        uint8_t  vib_code    = payload[12];
        uint8_t  trig_l      = payload[13];
        uint8_t  trig_r      = payload[14];
        uint8_t  imu_length  = payload[15];
        float    rx, ry;

        parse_left_stick(payload + 6, &sx, &sy);
        parse_right_stick(payload + 9, &rx, &ry);
        format_buttons(buttons, btn_str, sizeof(btn_str));

        printf("  [INPUT/GC]    pkt=%02X status=%02X  btns=%s  "
               "Lstick=(%.2f, %.2f)  Rstick=(%.2f, %.2f)  "
               "Ltrig=0x%02X  Rtrig=0x%02X  vib=%02X  imuLen=%d\n",
               packet_id, status, btn_str, sx, sy, rx, ry,
               trig_l, trig_r, vib_code, imu_length);

        if (imu_length > 0 && plen >= 16 + imu_length)
            print_motion_simple(payload + 16, imu_length);
        break;
    }

    // ----------------------------------------------------------------
    // Default: try to detect wireless BLE report formats.
    //
    // For the BLE simple input handle (0x000E) the Lua uses
    // parse_wireless_input_report() which has NO leading report-type
    // byte - the payload starts directly with packet_id.
    //
    // For the BLE common input handle (0x000A) the Lua uses
    // parse_wireless_input_reportA() - also starts with packet_id (4 bytes).
    //
    // The handle lets us pick the right format.
    // ----------------------------------------------------------------
    default: {
        // BLE simple wireless report (handle 0x000E)
        // Layout (parse_wireless_input_report):
        //   [0]       packet_id
        //   [1]       status
        //   [2..4]    buttons (3 bytes, LE)
        //   [5..7]    left  stick
        //   [8..10]   right stick
        //   [11]      vibration_code
        //   [12]      left  analog trigger
        //   [13]      right analog trigger
        //   [14]      imu_length
        //   [15..]    IMU motion buffer
        if (handle == 0x000E && plen >= 15) {
            uint8_t  packet_id  = payload[0];
            uint8_t  status     = payload[1];
            uint32_t buttons    = payload[2] | ((uint32_t)payload[3] << 8) | ((uint32_t)payload[4] << 16);
            uint8_t  vib_code   = payload[11];
            uint8_t  trig_l     = payload[12];
            uint8_t  trig_r     = payload[13];
            uint8_t  imu_length = payload[14];
            float    rx, ry;

            parse_left_stick(payload + 5, &sx, &sy);
            parse_right_stick(payload + 8, &rx, &ry);
            format_buttons(buttons, btn_str, sizeof(btn_str));

            printf("  [WIRELESS]    pkt=%02X status=%02X  btns=%s  "
                   "Lstick=(%.2f, %.2f)  Rstick=(%.2f, %.2f)  "
                   "Ltrig=0x%02X  Rtrig=0x%02X  vib=%02X  imuLen=%d\n",
                   packet_id, status, btn_str, sx, sy, rx, ry,
                   trig_l, trig_r, vib_code, imu_length);

            if (imu_length > 0 && plen >= 15 + imu_length)
                print_motion_simple(payload + 15, imu_length);
            break;
        }

        // BLE common input report (handle 0x000A) - format A
        // Layout (parse_wireless_input_reportA):
        //   [0x00..0x03]  packet_id  (4 bytes, LE)
        //   [0x04..0x07]  buttons    (4 bytes, LE)
        //   [0x08..0x09]  (unknown/padding)
        //   [0x0A..0x0C]  left  stick (3 bytes, 12-bit packed)
        //   [0x0D..0x0F]  right stick
        //   [0x10..0x11]  mouse_x (LE16)
        //   [0x12..0x13]  mouse_y (LE16)
        //   [0x14..0x15]  mouse_unkA (LE16)
        //   [0x16..0x17]  mouse_distance (LE16)
        //   [0x18]        (padding)
        //   [0x19..0x1A]  magnetometer_x (LE16)
        //   [0x1B..0x1C]  magnetometer_y (LE16)
        //   [0x1D..0x1E]  magnetometer_z (LE16)
        //   [0x1F..0x20]  battery (LE16)
        //   [0x21]        (padding)
        //   [0x22..0x23]  current (LE16)
        //   [0x24..0x29]  (padding / reserved)
        //   [0x2A..0x3B]  IMU motion buffer (18 bytes, full motion2 format)
        //   [0x3C]        left  analog trigger
        //   [0x3D]        right analog trigger
        if (handle == 0x000A && plen >= 0x3E) {
            uint32_t packet_id      = read_le32(payload + 0x00);
            uint32_t buttons        = read_le32(payload + 0x04);
            uint16_t mouse_x        = read_le16(payload + 0x10);
            uint16_t mouse_y        = read_le16(payload + 0x12);
            uint16_t mouse_unkA     = read_le16(payload + 0x14);
            uint16_t mouse_dist     = read_le16(payload + 0x16);
            int16_t  magnet_x       = read_le16s(payload + 0x19);
            int16_t  magnet_y       = read_le16s(payload + 0x1B);
            int16_t  magnet_z       = read_le16s(payload + 0x1D);
            uint16_t battery        = read_le16(payload + 0x1F);
            uint16_t current        = read_le16(payload + 0x22);
            uint8_t  trig_l         = payload[0x3C];
            uint8_t  trig_r         = payload[0x3D];
            float    rx, ry;

            parse_left_stick(payload + 0x0A, &sx, &sy);
            parse_right_stick(payload + 0x0D, &rx, &ry);
            format_buttons_a(buttons, btn_str, sizeof(btn_str));

            printf("  [WIRELESS/A]  pkt=%08X  btns=%s  "
                   "Lstick=(%.2f, %.2f)  Rstick=(%.2f, %.2f)  "
                   "mouse=(0x%04X, 0x%04X) unkA=0x%04X dist=0x%04X  "
                   "mag=(%d, %d, %d)  bat=%u  cur=%u  "
                   "Ltrig=0x%02X  Rtrig=0x%02X\n",
                   packet_id, btn_str, sx, sy, rx, ry,
                   mouse_x, mouse_y, mouse_unkA, mouse_dist,
                   magnet_x, magnet_y, magnet_z,
                   battery, current, trig_l, trig_r);

            print_motion2(payload + 0x2A, 18);
            break;
        }

        // Unknown / unrecognised report - dump raw bytes
        printf("  [INPUT/?]     type=0x%02X handle=0x%04X len=%d\n",
               report_type, handle, plen);
        break;
    }

    } // switch
}

// -----------------------------------------------------------------------
// Listen for input reports
// -----------------------------------------------------------------------

static void listen_loop(int sock) {
    printf("\nListening for notifications (Ctrl+C to stop)...\n");
    while (1) {
        uint8_t buf[512];
        int n = read(sock, buf, sizeof(buf));
        if (n < 0) { perror("read"); break; }
        if (n == 0) { printf("Disconnected.\n"); break; }

        if (buf[0] != ATT_HANDLE_NOTIFY) continue; // ignore everything except notifications

        uint16_t handle = buf[1] | (buf[2] << 8);
        uint8_t *payload = &buf[3];
        int plen = n - 3;

        printf("NOTIFY 0x%04X [%3d]: ", handle, plen);
        for (int i = 0; i < plen; i++) printf("%02X ", payload[i]);
        printf("\n");

        // Decode input reports on known HID input handles
        if (handle == 0x0009 || handle == 0x000E || handle == 0x000A)
            parse_input_report(payload, plen, handle);
    }
}

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------


// Mods
// Optimized hex parser for machine-to-machine communication
static size_t hex_to_bytes(const char *hex, uint8_t *bytes, size_t max_len) {
    size_t count = 0;
    while (*hex && count < max_len) {
        while (*hex && !isxdigit(*hex)) hex++;
        if (!*hex) break;
        unsigned int temp;
        sscanf(hex, "%2x", &temp);
        bytes[count++] = (uint8_t)temp;
        hex += 2;
    }
    return count;
}

static void run_bridge(int sock) {
    struct pollfd fds[2];
    fds[0].fd = fileno(stdin);
    fds[0].events = POLLIN;
    fds[1].fd = sock;
    fds[1].events = POLLIN;

    uint8_t buf[512];
    char line[1024];

    // Signal to Python that the connection is ready
    printf("STATUS READY\n");
    fflush(stdout);

    while (poll(fds, 2, -1) > 0) {
        // 1. Handle Commands FROM Python (stdin)
        if (fds[0].revents & POLLIN) {
            if (fgets(line, sizeof(line), stdin)) {
                uint8_t cmd[256];
                size_t len = hex_to_bytes(line, cmd, sizeof(cmd));
                if (len > 0) {
                    // We use your existing handle constant
                    att_write_cmd(sock, HANDLE_WRITE, cmd, len);
                }
            }
        }

        // 2. Handle Data FROM Controller (Bluetooth)
        if (fds[1].revents & POLLIN) {
            int n = read(sock, buf, sizeof(buf));
            if (n <= 0) break; // Disconnected

            if (buf[0] == ATT_HANDLE_NOTIFY) {
                uint16_t handle = buf[1] | (buf[2] << 8);
                printf("RECV %04X ", handle);
                for (int i = 3; i < n; i++) printf("%02X", buf[i]);
                printf("\n");
                fflush(stdout);

                // Decode input reports on known HID input handles
                if (handle == 0x0009 || handle == 0x000E || handle == 0x000A) {
                    parse_input_report(buf + 3, n - 3, handle);
                    fflush(stdout);
                }
            }
        }
    }
}

int main(void) {
    int sock = connect_att();
    if (sock < 0) return 1;

    //find_all_handles(sock);

    // Step 1: Enable notifications FIRST (per Windows implementation)
    enable_notifications(sock);
    drain(sock);

    //probe_writable_handles(sock);

    // Step 2: Send init sequence
    send_init(sock);
    drain(sock);

    // Step 3: Interactive REPL (type 'listen' to start streaming, 'quit' to exit)
    //run_bridge(sock);
    listen_loop(sock);

    close(sock);
    return 0;
}