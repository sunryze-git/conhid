#pragma once

#include <linux/uhid.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <cstdint>

class VirtualDevice {
public:
    static int CreateUhidDevice();
};