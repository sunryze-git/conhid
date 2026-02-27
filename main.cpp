#include <csignal>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <sys/types.h>
#include <unordered_map>
#include <vector>
#include "controller.h"
#include "initializer.h"

bool keep_running = true;
void sig_handler(int s) { keep_running = false; }

std::unordered_map<std::string, std::unique_ptr<Controller>> active_devices;

// represents supported devices. right now only contains PC2
const std::vector<DeviceId> allowed_devices = {
    { 0x057E, 0x2069 }
};

int hotplug_callback(libusb_context* ctx, libusb_device* dev, libusb_hotplug_event event, void* user_data) {
    uint8_t bus = libusb_get_bus_number(dev);
    uint8_t addr = libusb_get_device_address(dev);
    std::string id = std::to_string(bus) + ":" + std::to_string(addr);

    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        if (Initializer::IsCompatible(dev, allowed_devices)) {
            std::cout << "[+] Connected: " << id << std::endl;
            
            libusb_device_handle* handle = Initializer::PrepareDevice(dev);
            if (handle) {
                active_devices[id] = std::make_unique<Controller>(dev, handle, id);
                std::cout << "[*] Device " << id << " initialized." << std::endl;
            }
        }
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        auto it = active_devices.find(id);
        if (it != active_devices.end()) {
            std::cout << "[-] Disconnected: " << id << std::endl;
            active_devices.erase(it);
        }
    }

    return 0;
}

int main() {
    signal(SIGINT, sig_handler);
    
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) < 0) return 1;
    
    if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        std::cerr << "hotplug not supported" << std::endl;
        libusb_exit(ctx);
        return 1;
    }

    libusb_hotplug_callback_handle handle;
    int rc = libusb_hotplug_register_callback(ctx,
    (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
    LIBUSB_HOTPLUG_ENUMERATE,
    LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
    hotplug_callback, nullptr, &handle);
    
    if (rc != LIBUSB_SUCCESS) {
        std::cerr << "Failed to register hotplug callback" << std::endl;
        libusb_exit(ctx);
        return 1;
    }

    std::cout << "service started, listening for usb changes. " << std::endl;

    while (keep_running) {
        struct timeval tv = {0, 100000};
        libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
    }

    std::cout << "\n Shutting Down" << std::endl;
    active_devices.clear();
    libusb_exit(ctx);
    return 0;
}