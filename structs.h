#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t vid;
    uint16_t pid;
} DeviceId;

#ifdef __cplusplus
}
#endif