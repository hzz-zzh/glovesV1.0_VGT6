#ifndef HI04_DRIVER_H
#define HI04_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "hi04_can.h"
#include "hi04_j1939.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*hi04_send_fn)(void *ctx, const hi04_can_frame_t *frame);

typedef struct {
    hi04_send_fn send;
    void *ctx;
} hi04_bus_t;

typedef struct {
    hi04_bus_t bus;
    uint8_t node_id;
    hi04_sample_t latest;
    uint32_t seen_mask;
} hi04_device_t;

enum {
    HI04_SEEN_ACCEL = 1u << 0,
    HI04_SEEN_GYRO = 1u << 1,
    HI04_SEEN_MAG = 1u << 2,
    HI04_SEEN_QUAT = 1u << 3,
    HI04_SEEN_PITCH_ROLL = 1u << 4,
    HI04_SEEN_YAW = 1u << 5,
    HI04_SEEN_ENV = 1u << 6,
    HI04_SEEN_TIME = 1u << 7,
    HI04_SEEN_CANFD0 = 1u << 8
};

void hi04_device_init(hi04_device_t *dev,
                      hi04_bus_t bus,
                      uint8_t node_id);
bool hi04_device_send_frame(hi04_device_t *dev, const hi04_can_frame_t *frame);
bool hi04_device_process_frame(hi04_device_t *dev,
                               const hi04_can_frame_t *frame,
                               hi04_msg_type_t *out_type);

#ifdef __cplusplus
}
#endif

#endif
