#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * virtualdevice.h — uinput-based virtual gamepad
 *
 * Creates a /dev/input/eventN node that exposes:
 *   - EV_KEY  : face buttons, bumpers, triggers, system buttons, stick clicks
 *   - EV_ABS  : 4 stick axes (ABS_X/Y, ABS_RX/RY) + hat (ABS_HAT0X/Y)
 *   - EV_MSC  : timestamp (MSC_TIMESTAMP)
 *   - EV_FF   : FF_RUMBLE (weak + strong motor magnitudes)
 *
 * Rumble: the kernel writes EV_FF play/stop events to the fd.  The caller
 * must drain these with VirtualDevice_PollRumble() and forward the motor
 * values to the physical controller.
 */

typedef struct {
    uint16_t strong_magnitude;   /* low-freq  motor, 0-0xFFFF */
    uint16_t weak_magnitude;     /* high-freq motor, 0-0xFFFF */
} RumbleState;

/* Returns fd >= 0 on success, -1 on error. */
int VirtualDevice_CreateUinputDevice(void);

/*
 * Non-blocking poll for pending FF events on fd.
 * Returns 1 and fills *out if rumble state changed, 0 if nothing pending,
 * -1 on error / device gone.
 */
int VirtualDevice_PollRumble(int fd, RumbleState *out);

#ifdef __cplusplus
}
#endif