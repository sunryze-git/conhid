#include "controller.h"
#include "virtualdevice.h"
#include "transport.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <linux/input.h>
#include <linux/uinput.h>

/* -----------------------------------------------------------------------
 * BLE haptics commands
 * -------------------------------------------------------------------- */

#define REQUEST_MARKER        0x91
#define COMM_BLE              0x01
#define HAPTICS               0x0A
#define VIBRATION_PLAY_SAMPLE 0x02

static void SendHapticsSample(Controller* c, uint8_t sample_id) {
    uint8_t cmd[] = {
        HAPTICS, REQUEST_MARKER, COMM_BLE, VIBRATION_PLAY_SAMPLE,
        0x00, 0x04, 0x00, 0x00,
        sample_id, 0x00, 0x00
    };
    c->transport->send_haptic(c->transport, cmd, sizeof(cmd));
}

static void SendUsbRumble(Controller* c, int hf, int lf) {
    uint8_t buf[64];
    memset(buf, 0, sizeof(buf));

    uint8_t seq = (uint8_t)(0x50 + (c->rumblePacketCount++ % 16));

    buf[0]  = 0x02;   /* PacketHeader */
    buf[1]  = seq;

    /* Right motor (HF) — RightMotorStateOffset = 2 */
    buf[2]  = 130;    /* StaticState */
    buf[3]  = (uint8_t)((hf > 0 ? 0x10 : 0x00) | (1 & 0x03));
    buf[4]  = (uint8_t)(hf & 0x0F);

    /* Left motor (LF) — LeftMotorStateOffset = 16 */
    buf[16] = 0x00;
    buf[17] = seq;
    buf[18] = 130;    /* StaticState */
    buf[19] = (uint8_t)((lf > 0 ? 0x10 : 0x00) | (1 & 0x03));
    buf[20] = (uint8_t)(lf & 0x0F);

    /* send_haptic → libusb_interrupt_transfer to EP 0x01 */
    c->transport->send_haptic(c->transport, buf, sizeof(buf));
}

/* -----------------------------------------------------------------------
 * uinput emit helpers
 * -------------------------------------------------------------------- */

static void emit(int fd, uint16_t type, uint16_t code, int32_t value) {
    struct input_event ev = {0};
    ev.type  = type;
    ev.code  = code;
    ev.value = value;
    if (write(fd, &ev, sizeof(ev)) < 0)
        fprintf(stderr, "[-] uinput write error: %s\n", strerror(errno));
}

static void emit_sync(int fd) {
    emit(fd, EV_SYN, SYN_REPORT, 0);
}

/* -----------------------------------------------------------------------
 * D-pad hat: raw hat value (0-8) → ABS_HAT0X / ABS_HAT0Y
 * -------------------------------------------------------------------- */

static uint8_t GetHatValue(int up, int down, int left, int right) {
    if (up    && right) return 1;
    if (right && down)  return 3;
    if (down  && left)  return 5;
    if (left  && up)    return 7;
    if (up)    return 0;
    if (right) return 2;
    if (down)  return 4;
    if (left)  return 6;
    return 8;
}

static void EmitHat(int fd, uint8_t hat) {
    static const int8_t hat_x[9] = { 0, 1, 1, 1, 0,-1,-1,-1, 0 };
    static const int8_t hat_y[9] = {-1,-1, 0, 1, 1, 1, 0,-1, 0 };
    uint8_t h = hat <= 8 ? hat : 8;
    emit(fd, EV_ABS, ABS_HAT0X, hat_x[h]);
    emit(fd, EV_ABS, ABS_HAT0Y, hat_y[h]);
}

/* -----------------------------------------------------------------------
 * Controller state → uinput
 * -------------------------------------------------------------------- */

typedef struct {
    int b, a, y, x, rb, rt, plus, rClick;
    int lb, lt, minus, lClick;
    uint8_t dpad;
    int home, capture, paddleGR, paddleGL, chat;
    int16_t lx, ly, rx, ry;
    int16_t accelX, accelY, accelZ;
    int16_t gyroX,  gyroY,  gyroZ;
} ControllerState;

static void EmitControllerState(int fd, const ControllerState* s) {
    emit(fd, EV_KEY, BTN_SOUTH,  s->b);
    emit(fd, EV_KEY, BTN_EAST,   s->a);
    emit(fd, EV_KEY, BTN_NORTH,  s->y);
    emit(fd, EV_KEY, BTN_WEST,   s->x);
    emit(fd, EV_KEY, BTN_TR,     s->rb);
    emit(fd, EV_KEY, BTN_TR2,    s->rt);
    emit(fd, EV_KEY, BTN_START,  s->plus);
    emit(fd, EV_KEY, BTN_THUMBR, s->rClick);

    emit(fd, EV_KEY, BTN_TL,     s->lb);
    emit(fd, EV_KEY, BTN_TL2,    s->lt);
    emit(fd, EV_KEY, BTN_SELECT, s->minus);
    emit(fd, EV_KEY, BTN_THUMBL, s->lClick);

    EmitHat(fd, s->dpad);

    emit(fd, EV_KEY, BTN_MODE,           s->home);
    emit(fd, EV_KEY, BTN_Z,              s->capture);
    emit(fd, EV_KEY, BTN_TRIGGER_HAPPY1, s->paddleGL);
    emit(fd, EV_KEY, BTN_TRIGGER_HAPPY2, s->paddleGR);
    emit(fd, EV_KEY, BTN_TRIGGER_HAPPY3, s->chat);

    emit(fd, EV_ABS, ABS_X,  s->lx);
    emit(fd, EV_ABS, ABS_Y,  s->ly);
    emit(fd, EV_ABS, ABS_RX, s->rx);
    emit(fd, EV_ABS, ABS_RY, s->ry);

    emit_sync(fd);
}

/* -----------------------------------------------------------------------
 * USB input report processing
 * -------------------------------------------------------------------- */

static void ProcessUsbReport(Controller* c, const uint8_t* buffer, int length) {
    if (length < (int)sizeof(PhysicalProReport)) return;
    printf("Raw Report [%d bytes]:\n", length);
    for (int i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);
        if ((i + 1) % 16 == 0) printf("\n"); // Wrap every 16 bytes
    }
    printf("\n");

    const PhysicalProReport* in = (const PhysicalProReport*)buffer;
    ControllerState s = {0};

    s.b      = (in->buttons1 & 0x01) != 0;
    s.a      = (in->buttons1 & 0x02) != 0;
    s.y      = (in->buttons1 & 0x04) != 0;
    s.x      = (in->buttons1 & 0x08) != 0;
    s.rb     = (in->buttons1 & 0x10) != 0;
    s.rt     = (in->buttons1 & 0x20) != 0;
    s.plus   = (in->buttons1 & 0x40) != 0;
    s.rClick = (in->buttons1 & 0x80) != 0;

    int d_down  = (in->buttons2 & 0x01) != 0;
    int d_right = (in->buttons2 & 0x02) != 0;
    int d_left  = (in->buttons2 & 0x04) != 0;
    int d_up    = (in->buttons2 & 0x08) != 0;
    s.dpad   = GetHatValue(d_up, d_down, d_left, d_right);
    s.lb     = (in->buttons2 & 0x10) != 0;
    s.lt     = (in->buttons2 & 0x20) != 0;
    s.minus  = (in->buttons2 & 0x40) != 0;
    s.lClick = (in->buttons2 & 0x80) != 0;

    s.home     = (in->buttons3 & 0x01) != 0;
    s.capture  = (in->buttons3 & 0x02) != 0;
    s.paddleGR = (in->buttons3 & 0x04) != 0;
    s.paddleGL = (in->buttons3 & 0x08) != 0;
    s.chat     = (in->buttons3 & 0x10) != 0;

    uint16_t lx_raw = in->sticksLeft[0]  | ((in->sticksLeft[1]  & 0x0F) << 8);
    uint16_t ly_raw = (in->sticksLeft[1]  >> 4) | ((uint16_t)in->sticksLeft[2]  << 4);
    uint16_t rx_raw = in->sticksRight[0] | ((in->sticksRight[1] & 0x0F) << 8);
    uint16_t ry_raw = (in->sticksRight[1] >> 4) | ((uint16_t)in->sticksRight[2] << 4);

    s.lx = Calibration_NormalizeLX(&c->cal, lx_raw);
    s.ly = Calibration_NormalizeLY(&c->cal, ly_raw);
    s.rx = Calibration_NormalizeRX(&c->cal, rx_raw);
    s.ry = Calibration_NormalizeRY(&c->cal, ry_raw);

    const int32_t* raw_imu = (const int32_t*)in->imuData;
    s.gyroX  = Calibration_ScaleGyro (&c->cal, 0, raw_imu[0]);
    s.gyroY  = Calibration_ScaleGyro (&c->cal, 1, raw_imu[1]);
    s.gyroZ  = Calibration_ScaleGyro (&c->cal, 2, raw_imu[2]);
    s.accelX = Calibration_ScaleAccel(&c->cal, 0, raw_imu[3]);
    s.accelY = Calibration_ScaleAccel(&c->cal, 1, raw_imu[4]);
    s.accelZ = Calibration_ScaleAccel(&c->cal, 2, raw_imu[5]);

    EmitControllerState(c->virtualDeviceFd, &s);
}

/* -----------------------------------------------------------------------
 * BLE input report processing
 * -------------------------------------------------------------------- */

static void ProcessBleReport(Controller* c, const uint8_t* buffer, int length) {
    if (length < 16) return;

    const BleInputReport* in = (const BleInputReport*)buffer;
    ControllerState s = {0};

    uint8_t b1 = in->buttons_lo;
    uint8_t b2 = in->buttons_hi;

    s.b      = (b1 & 0x01) != 0;
    s.a      = (b1 & 0x02) != 0;
    s.y      = (b1 & 0x04) != 0;
    s.x      = (b1 & 0x08) != 0;
    s.rb     = (b1 & 0x10) != 0;
    s.rt     = (b1 & 0x20) != 0;
    s.plus   = (b1 & 0x40) != 0;
    s.rClick = (b1 & 0x80) != 0;

    int d_down  = (b2 & 0x01) != 0;
    int d_right = (b2 & 0x02) != 0;
    int d_left  = (b2 & 0x04) != 0;
    int d_up    = (b2 & 0x08) != 0;
    s.dpad   = GetHatValue(d_up, d_down, d_left, d_right);
    s.lb     = (b2 & 0x10) != 0;
    s.lt     = (b2 & 0x20) != 0;
    s.minus  = (b2 & 0x40) != 0;
    s.lClick = (b2 & 0x80) != 0;

    uint16_t lx_raw = in->stickLeft[0] | ((in->stickLeft[1] & 0x0F) << 8);
    uint16_t ly_raw = (in->stickLeft[1] >> 4) | ((uint16_t)in->stickLeft[2] << 4);
    s.lx = Calibration_NormalizeLX(&c->cal, lx_raw);
    s.ly = Calibration_NormalizeLY(&c->cal, ly_raw);

    if (in->imuLength >= 12 && length >= 16 + 12) {
        const uint8_t* imu = buffer + 16;
        int16_t ax, ay, az, gx, gy, gz;
        memcpy(&ax, imu + 0, 2);
        memcpy(&ay, imu + 2, 2);
        memcpy(&az, imu + 4, 2);
        memcpy(&gx, imu + 6, 2);
        memcpy(&gy, imu + 8, 2);
        memcpy(&gz, imu + 10, 2);

        s.accelX = Calibration_ScaleAccel(&c->cal, 0, ax);
        s.accelY = Calibration_ScaleAccel(&c->cal, 1, ay);
        s.accelZ = Calibration_ScaleAccel(&c->cal, 2, az);
        s.gyroX  = Calibration_ScaleGyro (&c->cal, 0, gx);
        s.gyroY  = Calibration_ScaleGyro (&c->cal, 1, gy);
        s.gyroZ  = Calibration_ScaleGyro (&c->cal, 2, gz);
    }

    EmitControllerState(c->virtualDeviceFd, &s);
}

/* -----------------------------------------------------------------------
 * Rumble pump thread
 *
 * Wakes on pthread_cond when rumble is armed, sends 64-byte packets every
 * 4ms via transport->send_haptic (interrupt OUT EP 0x01 for USB), then
 * drains a stop-hold tail of RUMBLE_STOP_HOLD_PACKETS before parking.
 * -------------------------------------------------------------------- */

static void RumbleSleepMs(long ms) {
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

static void* RumblePumpThread(void* arg) {
    Controller* c = (Controller*)arg;

    while (c->running) {
        pthread_mutex_lock(&c->rumbleMutex);
        while (c->running && !c->rumblePumping)
            pthread_cond_wait(&c->rumbleCond, &c->rumbleMutex);
        pthread_mutex_unlock(&c->rumbleMutex);

        if (!c->running) break;

        while (c->running) {
            int hf, lf, stop;

            pthread_mutex_lock(&c->rumbleMutex);
            hf = c->currentHF;
            lf = c->currentLF;

            if (hf == 0 && lf == 0) {
                if (c->stopHoldRemaining > 0) {
                    c->stopHoldRemaining--;
                    stop = 0;
                } else {
                    c->rumblePumping = 0;
                    stop = 1;
                }
            } else {
                stop = 0;
            }
            pthread_mutex_unlock(&c->rumbleMutex);

            if (c->transport->type == TRANSPORT_USB)
                SendUsbRumble(c, hf, lf);
            else {
                uint8_t sample = (hf > 0 || lf > 0) ? 0x01 : 0x00;
                SendHapticsSample(c, sample);
            }

            if (stop) break;

            RumbleSleepMs(4);
        }
    }

    return NULL;
}

/* -----------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------- */

Controller* Controller_Create(Transport* transport, const char* deviceId) {
    Controller* c = (Controller*)calloc(1, sizeof(Controller));
    if (!c) return NULL;

    c->transport = transport;
    snprintf(c->deviceId, sizeof(c->deviceId), "%s", deviceId);

    c->virtualDeviceFd = VirtualDevice_CreateUinputDevice();
    if (c->virtualDeviceFd < 0) {
        free(c);
        return NULL;
    }

    pthread_mutex_init(&c->rumbleMutex, NULL);
    pthread_cond_init(&c->rumbleCond, NULL);
    c->running = 1;

    pthread_create(&c->rumbleThread, NULL, RumblePumpThread, c);

    printf("[%s] Controller created.\n", c->deviceId);
    return c;
}

void Controller_Destroy(Controller* c) {
    if (!c) return;
    c->running = 0;

    pthread_mutex_lock(&c->rumbleMutex);
    pthread_cond_signal(&c->rumbleCond);
    pthread_mutex_unlock(&c->rumbleMutex);
    pthread_join(c->rumbleThread, NULL);

    Transport_Destroy(c->transport);
    c->transport = NULL;

    pthread_cond_destroy(&c->rumbleCond);
    pthread_mutex_destroy(&c->rumbleMutex);

    if (c->virtualDeviceFd != -1) {
        ioctl(c->virtualDeviceFd, UI_DEV_DESTROY);
        close(c->virtualDeviceFd);
    }

    printf("[%s] Controller destroyed.\n", c->deviceId);
    free(c);
}

int Controller_ProcessPendingReport(Controller* c) {
    uint8_t buf[256];
    int n = c->transport->read_report(c->transport, buf, sizeof(buf));
    if (n < 0) return -1;
    if (n == 0) return  0;

    if (c->transport->type == TRANSPORT_USB)
        ProcessUsbReport(c, buf, n);
    else
        ProcessBleReport(c, buf, n);

    return 0;
}

/*
 * Drains all pending EV_FF/EV_UINPUT events from the uinput fd, scales
 * kernel 0-0xFFFF magnitudes to 0-15, and feeds the rumble pump thread.
 * Call from the main event loop whenever virtualDeviceFd is readable.
 */
void Controller_ProcessUhidEvents(Controller* c) {
    RumbleState rs;
    int ret;

    while ((ret = VirtualDevice_PollRumble(c->virtualDeviceFd, &rs)) > 0) {
        /* strong_magnitude → LF (low-freq / left) motor
         * weak_magnitude   → HF (high-freq / right) motor
         * >> 12 maps 0x0000-0xFFFF → 0-15                */
        int lf = (int)(rs.strong_magnitude >> 12);
        int hf = (int)(rs.weak_magnitude   >> 12);
        Controller_SetRumble(c, hf, lf);
    }
}

/* -----------------------------------------------------------------------
 * Calibration loading
 * -------------------------------------------------------------------- */

void Controller_LoadStickCalL(Controller* c, const uint8_t* spi_11bytes) {
    memcpy(c->calRaw.stick_l_raw, spi_11bytes, 11);
    c->calRaw.stick_l_loaded = 1;
}

void Controller_LoadStickCalR(Controller* c, const uint8_t* spi_11bytes) {
    memcpy(c->calRaw.stick_r_raw, spi_11bytes, 11);
    c->calRaw.stick_r_loaded = 1;
}

void Controller_LoadMotionCal(Controller* c, const uint8_t* spi_24bytes) {
    memcpy(c->calRaw.motion_raw, spi_24bytes, 24);
    c->calRaw.motion_loaded = 1;
}

void Controller_LoadUserCal(Controller* c,
                             const uint8_t* spi_l_11bytes,
                             const uint8_t* spi_r_11bytes,
                             uint16_t magic) {
    c->calRaw.user_magic = magic;
    memcpy(c->calRaw.user_stick_l_raw, spi_l_11bytes, 11);
    memcpy(c->calRaw.user_stick_r_raw, spi_r_11bytes, 11);
    c->calRaw.user_loaded = 1;
}

void Controller_FinalizeCalibration(Controller* c) {
    Calibration_Parse(&c->calRaw, &c->cal);
    printf("[%s] Calibration finalized (valid=%d)\n", c->deviceId, c->cal.valid);
}

/* -----------------------------------------------------------------------
 * Controller_SetRumble
 * -------------------------------------------------------------------- */

void Controller_SetRumble(Controller* c, int hf, int lf) {
    pthread_mutex_lock(&c->rumbleMutex);

    c->currentHF = hf;
    c->currentLF = lf;

    if (hf > 0 || lf > 0) {
        c->stopHoldRemaining = RUMBLE_STOP_HOLD_PACKETS;
        if (!c->rumblePumping && c->running) {
            c->rumblePumping = 1;
            pthread_cond_signal(&c->rumbleCond);
        }
    } else {
        if (c->stopHoldRemaining <= 0)
            c->stopHoldRemaining = RUMBLE_STOP_HOLD_PACKETS;
    }

    pthread_mutex_unlock(&c->rumbleMutex);
}