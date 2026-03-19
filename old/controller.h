#pragma once

#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include "transport.h"
#include "calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

// USB Input Report
#pragma pack(push, 1)
typedef struct {
    uint8_t  reportId;          // Packet ID
    uint16_t counter;           // Sequence Counter
    uint8_t  buttons1;          // B A Y X R ZR + rClick
    uint8_t  buttons2;          // down right left up L ZL - lClick
    uint8_t  buttons3;          // home capture GR GL chat
    uint8_t  sticksLeft[3];     // Left Stick 12-bit
    uint8_t  sticksRight[3];    // Right Stick 12-bit
    uint8_t  padding[8];        // 
    uint8_t  imuData[44];   /* 6 × int32: gyroX/Y/Z accelX/Y/Z */
} PhysicalProReport;
#pragma pack(pop)

/* -----------------------------------------------------------------------
 * Physical input report — BLE (handle 0x000E, 63 bytes)
 * -------------------------------------------------------------------- */
#pragma pack(push, 1)
typedef struct {
    uint8_t  packetId;
    uint8_t  status;
    uint8_t  buttons_lo;    /* B A Y X R ZR + rClick */
    uint8_t  buttons_hi;    /* down right left up L ZL - lClick */
    uint8_t  unk4;
    uint8_t  stickLeft[3];
    uint8_t  vibrationCode;
    int16_t  mouseX;
    int16_t  mouseY;
    uint8_t  mouseDist;
    uint8_t  unk14;
    uint8_t  imuLength;
    /* imuData follows at byte 16: imuLength bytes,
     * each 12-byte sample = accel xyz (int16) + gyro xyz (int16) */
} BleInputReport;
#pragma pack(pop)

/* -----------------------------------------------------------------------
 * Controller
 * -------------------------------------------------------------------- */

/* Number of 4ms packets to keep sending after rumble drops to zero.
 * Mirrors HapticSequencer.StopHoldPackets — gives the motor time to
 * actually spin up before being cut, and ensures the stop packet is
 * delivered even if the game's release phase is very short. */
#define RUMBLE_STOP_HOLD_PACKETS 8

typedef struct {
    Transport*      transport;
    char            deviceId[32];

    volatile int    running;
    int             virtualDeviceFd;    /* /dev/uinput fd */

    /* Calibration */
    Sw2CalibrationRaw calRaw;
    Sw2Calibration    cal;

    /* Rumble — protected by rumbleMutex */
    pthread_mutex_t rumbleMutex;
    pthread_cond_t  rumbleCond;         /* signalled when new rumble arrives  */
    int             currentHF;          /* 0-15, set from EV_FF events        */
    int             currentLF;          /* 0-15, set from EV_FF events        */
    volatile int    rumblePumping;      /* 1 while pump thread is active      */
    int             stopHoldRemaining;  /* packets left in stop-hold tail     */
    uint8_t         rumblePacketCount;
    pthread_t       rumbleThread;
} Controller;

/* transport is adopted (and freed on destroy) by the controller. */
Controller* Controller_Create(Transport* transport, const char* deviceId);
void        Controller_Destroy(Controller* c);

/* Call from the main event loop when the transport fd is readable.
 * Returns -1 on disconnect, 0 otherwise. */
int  Controller_ProcessPendingReport(Controller* c);

/* Call from the main event loop when virtualDeviceFd is readable
 * (EV_FF / EV_UINPUT rumble events from the kernel). */
void Controller_ProcessUhidEvents(Controller* c);   /* name kept for ABI compat */

/* Called by the initialiser after each SPI read completes. */
void Controller_LoadStickCalL      (Controller* c, const uint8_t* spi_11bytes);
void Controller_LoadStickCalR      (Controller* c, const uint8_t* spi_11bytes);
void Controller_LoadMotionCal      (Controller* c, const uint8_t* spi_24bytes);
void Controller_LoadUserCal        (Controller* c,
                                    const uint8_t* spi_l_11bytes,
                                    const uint8_t* spi_r_11bytes,
                                    uint16_t magic);
void Controller_FinalizeCalibration(Controller* c);

void Controller_SetRumble(Controller* c, int hf, int lf);

#ifdef __cplusplus
}
#endif