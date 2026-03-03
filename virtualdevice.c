#include "virtualdevice.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <linux/uinput.h>
#include <linux/input.h>

/* -------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------- */

static int ui_set_evbit(int fd, int bit) {
    return ioctl(fd, UI_SET_EVBIT, bit);
}
static int ui_set_keybit(int fd, int bit) {
    return ioctl(fd, UI_SET_KEYBIT, bit);
}
static int ui_set_absbit(int fd, int bit) {
    return ioctl(fd, UI_SET_ABSBIT, bit);
}
static int ui_set_ffbit(int fd, int bit) {
    return ioctl(fd, UI_SET_FFBIT, bit);
}

/* Configure a single absolute axis. */
static int set_abs(int fd, int axis, int min, int max, int flat, int fuzz) {
    struct uinput_abs_setup s = {0};
    s.code        = axis;
    s.absinfo.minimum   = min;
    s.absinfo.maximum   = max;
    s.absinfo.flat      = flat;
    s.absinfo.fuzz      = fuzz;
    return ioctl(fd, UI_ABS_SETUP, &s);
}

/* -------------------------------------------------------------------------
 * VirtualDevice_CreateUinputDevice
 * ---------------------------------------------------------------------- */

int VirtualDevice_CreateUinputDevice(void) {
    int fd = open("/dev/uinput", O_RDWR | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[-] Failed to open /dev/uinput: %s\n", strerror(errno));
        return -1;
    }

#define CHK(expr) do { if ((expr) < 0) { \
    fprintf(stderr, "[-] uinput setup failed (%s): %s\n", #expr, strerror(errno)); \
    goto err; } } while (0)

    /* --- EV_KEY: face buttons, bumpers, triggers-as-buttons, system --- */
    CHK(ui_set_evbit(fd, EV_KEY));
    CHK(ui_set_keybit(fd, BTN_SOUTH));       /* B  */
    CHK(ui_set_keybit(fd, BTN_EAST));        /* A  */
    CHK(ui_set_keybit(fd, BTN_NORTH));       /* Y  */
    CHK(ui_set_keybit(fd, BTN_WEST));        /* X  */
    CHK(ui_set_keybit(fd, BTN_TL));          /* L  */
    CHK(ui_set_keybit(fd, BTN_TR));          /* R  */
    CHK(ui_set_keybit(fd, BTN_TL2));         /* ZL */
    CHK(ui_set_keybit(fd, BTN_TR2));         /* ZR */
    CHK(ui_set_keybit(fd, BTN_SELECT));      /* -  */
    CHK(ui_set_keybit(fd, BTN_START));       /* +  */
    CHK(ui_set_keybit(fd, BTN_THUMBL));      /* Left stick click  */
    CHK(ui_set_keybit(fd, BTN_THUMBR));      /* Right stick click */
    CHK(ui_set_keybit(fd, BTN_MODE));              /* Home    */
    CHK(ui_set_keybit(fd, BTN_Z));                 /* Capture */
    /* Paddles — BTN_TRIGGER_HAPPY range (0x2c0-0x2c3): inside EV_KEY but
     * clearly not keyboard keys; SDL/Chrome ignore these for classification
     * but still forward them as extra buttons. */
    CHK(ui_set_keybit(fd, BTN_TRIGGER_HAPPY1));    /* Paddle GL */
    CHK(ui_set_keybit(fd, BTN_TRIGGER_HAPPY2));    /* Paddle GR */
    CHK(ui_set_keybit(fd, BTN_TRIGGER_HAPPY3));    /* Chat/C    */
    /* NOTE: KEY_CHAT (0xD8) was removed — it is outside the BTN range and
     * caused SDL and Chrome to misclassify the device as a misc/keyboard
     * input rather than a gamepad. BTN_TRIGGER_HAPPY* are the correct home
     * for extra gamepad buttons with no standard mapping. */

    /* --- EV_ABS: sticks + hat only ------------------------------------- */
    /*
     * IMU axes (ABS_TILT_X, ABS_WHEEL, ABS_GAS, ABS_BRAKE, ABS_RUDDER) have
     * been intentionally removed from this device.  Reason: SDL2, Steam Input,
     * and the Chrome Gamepad API all use heuristics that inspect the full set
     * of declared ABS axes to classify the device.  Unusual axes like ABS_GAS
     * and ABS_BRAKE with signed 16-bit ranges look wrong (they expect 0-255
     * for analog triggers) and can cause the device to be rejected or mapped
     * incorrectly.  IMU data should be forwarded via a separate uinput node
     * created with INPUT_PROP_ACCELEROMETER if needed by the application.
     */
    CHK(ui_set_evbit(fd, EV_ABS));

    /* Stick axes: calibrated signed 16-bit output from Calibration_Normalize*.
     * flat = ~3% of range as hardware deadzone; fuzz = noise floor. */
    CHK(ui_set_absbit(fd, ABS_X));
    CHK(ui_set_absbit(fd, ABS_Y));
    CHK(ui_set_absbit(fd, ABS_RX));
    CHK(ui_set_absbit(fd, ABS_RY));
    CHK(set_abs(fd, ABS_X,  -32768, 32767, 1000, 32));
    CHK(set_abs(fd, ABS_Y,  -32768, 32767, 1000, 32));
    CHK(set_abs(fd, ABS_RX, -32768, 32767, 1000, 32));
    CHK(set_abs(fd, ABS_RY, -32768, 32767, 1000, 32));

    /* Hat / D-pad: -1, 0, +1 per axis */
    CHK(ui_set_absbit(fd, ABS_HAT0X));
    CHK(ui_set_absbit(fd, ABS_HAT0Y));
    CHK(set_abs(fd, ABS_HAT0X, -1, 1, 0, 0));
    CHK(set_abs(fd, ABS_HAT0Y, -1, 1, 0, 0));

    /* --- EV_FF: rumble ------------------------------------------------- */
    CHK(ui_set_evbit(fd, EV_FF));
    CHK(ui_set_ffbit(fd, FF_RUMBLE));
    /*
     * Do NOT set any INPUT_PROP_* bits here.
     * INPUT_PROP_DIRECT (0x01) is for touchscreens and drawing tablets —
     * setting it on a gamepad causes SDL and Chrome to skip the device
     * entirely during their joystick/gamepad enumeration pass.
     * Real gamepads (Xbox, DualSense) have zero input props set.
     */

    /* --- EV_MSC: timestamp --------------------------------------------- */
    CHK(ui_set_evbit(fd, EV_MSC));
    CHK(ioctl(fd, UI_SET_MSCBIT, MSC_TIMESTAMP));

    /* --- Device info --------------------------------------------------- */
    struct uinput_setup us = {0};
    us.id.bustype = BUS_BLUETOOTH;
    us.id.vendor  = 0x057E;
    us.id.product = 0x2069;
    us.id.version = 2;
    us.ff_effects_max = 4;
    strncpy(us.name, "Nintendo Switch 2 Pro Controller", UINPUT_MAX_NAME_SIZE - 1);

    CHK(ioctl(fd, UI_DEV_SETUP, &us));
    CHK(ioctl(fd, UI_DEV_CREATE));

    printf("[+] uinput device created (fd=%d)\n", fd);
    return fd;

err:
    close(fd);
    return -1;
#undef CHK
}

/* -------------------------------------------------------------------------
 * VirtualDevice_PollRumble
 *
 * The kernel communicates FF_RUMBLE play/stop via EV_UINPUT events:
 *   UI_FF_UPLOAD  — host app must ack the effect upload
 *   UI_FF_ERASE   — effect being removed (set motors to 0)
 * Actual play/stop arrives as EV_FF with value=1 (play) or value=0 (stop).
 * ---------------------------------------------------------------------- */

/*
 * Effect cache — maps effect id (0..FF_MAX_EFFECTS) to the rumble magnitudes
 * uploaded by the caller (game/SDL).  The kernel sends UI_FF_UPLOAD before
 * the first EV_FF play event, so we always have the magnitudes ready.
 * FF_MAX_EFFECTS is 96; this array is tiny.
 */
#ifndef FF_MAX_EFFECTS
#define FF_MAX_EFFECTS 96
#endif
static RumbleState s_effect_cache[FF_MAX_EFFECTS];

int VirtualDevice_PollRumble(int fd, RumbleState *out) {
    struct input_event ev;

    while (1) {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            return -1;
        }
        if ((size_t)n < sizeof(ev)) return 0;

        if (ev.type == EV_UINPUT) {
            if (ev.code == UI_FF_UPLOAD) {
                struct uinput_ff_upload upload = {0};
                upload.request_id = ev.value;
                if (ioctl(fd, UI_BEGIN_FF_UPLOAD, &upload) == 0) {
                    int id = upload.effect.id;
                    if (id >= 0 && id < FF_MAX_EFFECTS &&
                        upload.effect.type == FF_RUMBLE) {
                        s_effect_cache[id].strong_magnitude =
                            upload.effect.u.rumble.strong_magnitude;
                        s_effect_cache[id].weak_magnitude =
                            upload.effect.u.rumble.weak_magnitude;
                        printf("[rumble] FF_UPLOAD  effect_id=%d  strong=0x%04X (%u)  weak=0x%04X (%u)\n",
                            id,
                            s_effect_cache[id].strong_magnitude,
                            s_effect_cache[id].strong_magnitude,
                            s_effect_cache[id].weak_magnitude,
                            s_effect_cache[id].weak_magnitude);
                    } else {
                        printf("[rumble] FF_UPLOAD  effect_id=%d  type=0x%04X (not FF_RUMBLE=0x%04X, ignoring)\n",
                            id, upload.effect.type, FF_RUMBLE);
                    }
                    ioctl(fd, UI_END_FF_UPLOAD, &upload);
                } else {
                    printf("[rumble] FF_UPLOAD  UI_BEGIN_FF_UPLOAD failed: %s\n", strerror(errno));
                }
            } else if (ev.code == UI_FF_ERASE) {
                struct uinput_ff_erase erase = {0};
                erase.request_id = ev.value;
                if (ioctl(fd, UI_BEGIN_FF_ERASE, &erase) == 0) {
                    int id = (int)erase.effect_id;
                    printf("[rumble] FF_ERASE   effect_id=%d\n", id);
                    if (id >= 0 && id < FF_MAX_EFFECTS)
                        s_effect_cache[id].strong_magnitude =
                        s_effect_cache[id].weak_magnitude = 0;
                    ioctl(fd, UI_END_FF_ERASE, &erase);
                } else {
                    printf("[rumble] FF_ERASE   UI_BEGIN_FF_ERASE failed: %s\n", strerror(errno));
                }
                out->strong_magnitude = 0;
                out->weak_magnitude   = 0;
                return 1;
            } else {
                printf("[rumble] EV_UINPUT  unknown code=%u\n", ev.code);
            }
            continue;
        }

        if (ev.type == EV_FF) {
            int id = ev.code;
            if (ev.value == 0) {
                printf("[rumble] EV_FF STOP  effect_id=%d\n", id);
                out->strong_magnitude = 0;
                out->weak_magnitude   = 0;
            } else {
                if (id >= 0 && id < FF_MAX_EFFECTS) {
                    *out = s_effect_cache[id];
                } else {
                    out->strong_magnitude = 0xFFFF;
                    out->weak_magnitude   = 0xFFFF;
                }
                printf("[rumble] EV_FF PLAY  effect_id=%d  strong=0x%04X (%u)  weak=0x%04X (%u)\n",
                    id,
                    out->strong_magnitude, out->strong_magnitude,
                    out->weak_magnitude,   out->weak_magnitude);
            }
            return 1;
        }
    }
}