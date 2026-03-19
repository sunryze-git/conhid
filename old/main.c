#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <libusb-1.0/libusb.h>

#include "controller.h"
#include "initializer.h"
#include "transport.h"
#include "structs.h"

#define MAX_CONTROLLERS 4

typedef struct {
    char        id[32];
    Controller* controller;
} ActiveDevice;

static ActiveDevice active_devices[MAX_CONTROLLERS];
static int          active_device_count = 0;

static ActiveDevice* FindDevice(const char* id) {
    for (int i = 0; i < active_device_count; i++)
        if (strcmp(active_devices[i].id, id) == 0)
            return &active_devices[i];
    return NULL;
}

static void AddDevice(const char* id, Controller* c) {
    if (active_device_count >= MAX_CONTROLLERS) {
        fprintf(stderr, "[!] Max controllers reached\n");
        return;
    }
    ActiveDevice* slot = &active_devices[active_device_count++];
    snprintf(slot->id, sizeof(slot->id), "%s", id);
    slot->controller = c;
}

static void RemoveDevice(const char* id) {
    for (int i = 0; i < active_device_count; i++) {
        if (strcmp(active_devices[i].id, id) == 0) {
            Controller_Destroy(active_devices[i].controller);
            for (int j = i; j < active_device_count - 1; j++)
                active_devices[j] = active_devices[j + 1];
            active_device_count--;
            return;
        }
    }
}

static void RemoveAllDevices(void) {
    for (int i = 0; i < active_device_count; i++)
        Controller_Destroy(active_devices[i].controller);
    active_device_count = 0;
}

static const DeviceId allowed_devices[] = {
    { 0x057E, 0x2069 },  // Pro Controller 2
    { 0x057E, 0x2066 },  // Joy-Con 2 Right
    { 0x057E, 0x2067 },  // Joy-Con 2 Left
    { 0x057E, 0x2073 },  // GC Controller 2
};

static volatile int keep_running = 1;
static void sig_handler(int s) { (void)s; keep_running = 0; }

static int hotplug_callback(libusb_context* ctx, libusb_device* dev,
                            libusb_hotplug_event event, void* user_data) {
    (void)ctx; (void)user_data;

    char id[32];
    Initializer_GetDeviceId(dev, id, sizeof(id));

    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        if (!Initializer_IsCompatible(dev, allowed_devices,
                (int)(sizeof(allowed_devices) / sizeof(allowed_devices[0]))))
            return 0;

        printf("[+] USB device arrived: %s\n", id);

        Sw2CalibrationRaw cal_raw;
        libusb_device_handle* handle = Initializer_PrepareDevice(dev, &cal_raw);
        if (!handle) { fprintf(stderr, "[-] Prepare failed: %s\n", id); return 0; }

        Transport* t = Transport_CreateUSB(dev, handle);
        if (!t) { libusb_close(handle); return 0; }

        Controller* c = Controller_Create(t, id);
        if (!c) { Transport_Destroy(t); return 0; }

        if (cal_raw.stick_l_loaded) Controller_LoadStickCalL(c, cal_raw.stick_l_raw);
        if (cal_raw.stick_r_loaded) Controller_LoadStickCalR(c, cal_raw.stick_r_raw);
        if (cal_raw.motion_loaded)  Controller_LoadMotionCal(c, cal_raw.motion_raw);
        if (cal_raw.user_loaded)
            Controller_LoadUserCal(c, cal_raw.user_stick_l_raw,
                                      cal_raw.user_stick_r_raw,
                                      cal_raw.user_magic);
        Controller_FinalizeCalibration(c);

        AddDevice(id, c);
        printf("[*] Controller %s ready.\n", id);

    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        if (FindDevice(id)) {
            printf("[-] USB device left: %s\n", id);
            RemoveDevice(id);
        }
    }
    return 0;
}

int main(void) {
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    libusb_context* ctx = NULL;
    if (libusb_init(&ctx) < 0) { fprintf(stderr, "[-] libusb_init failed\n"); return 1; }

    if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        fprintf(stderr, "[-] Hotplug not supported\n");
        libusb_exit(ctx); return 1;
    }

    libusb_hotplug_callback_handle cb_handle;
    int rc = libusb_hotplug_register_callback(ctx,
        (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED |
                               LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
        LIBUSB_HOTPLUG_ENUMERATE,
        LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
        LIBUSB_HOTPLUG_MATCH_ANY,
        hotplug_callback, NULL, &cb_handle);

    if (rc != LIBUSB_SUCCESS) {
        fprintf(stderr, "[-] Hotplug register failed\n");
        libusb_exit(ctx); return 1;
    }

    printf("sw2d started. Waiting for controllers...\n");

    while (keep_running) {
        // Build poll set: one transport fd + one uhid fd per active controller
        struct pollfd pfds[MAX_CONTROLLERS * 2];
        int           nfds = 0;
        int           transport_base[MAX_CONTROLLERS];
        int           uhid_base[MAX_CONTROLLERS];

        for (int i = 0; i < active_device_count; i++) {
            Controller* c = active_devices[i].controller;

            transport_base[i]    = nfds;
            pfds[nfds].fd        = c->transport->get_poll_fd(c->transport);
            pfds[nfds].events    = POLLIN;
            nfds++;

            uhid_base[i]         = nfds;
            pfds[nfds].fd        = c->virtualDeviceFd;
            pfds[nfds].events    = POLLIN;
            nfds++;
        }

        // Pump libusb (drives USB async transfers), then poll other fds
        struct timeval tv_zero = {0, 0};
        libusb_handle_events_timeout_completed(ctx, &tv_zero, NULL);

        if (nfds > 0) poll(pfds, nfds, 10 /* ms */);

        for (int i = 0; i < active_device_count; i++) {
            Controller* c = active_devices[i].controller;

            if (pfds[transport_base[i]].revents & POLLIN) {
                if (Controller_ProcessPendingReport(c) < 0) {
                    char id[32];
                    snprintf(id, sizeof(id), "%s", c->deviceId);
                    printf("[-] Controller %s disconnected\n", id);
                    RemoveDevice(id);
                    break;
                }
            }

            if (pfds[uhid_base[i]].revents & POLLIN)
                Controller_ProcessUhidEvents(c);
        }
    }

    printf("\nShutting down.\n");
    RemoveAllDevices();
    libusb_hotplug_deregister_callback(ctx, cb_handle);
    libusb_exit(ctx);
    return 0;
}