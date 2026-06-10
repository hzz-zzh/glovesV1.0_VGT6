#ifndef HI04_FDCAN_STM32H563_H
#define HI04_FDCAN_STM32H563_H

#include <stdbool.h>
#include <stdint.h>

#include "hi04_can.h"
#include "stm32h5xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint64_t (*time_us)(void);
} hi04_fdcan_stm32h563_t;

bool hi04_fdcan_stm32h563_send(void *ctx, const hi04_can_frame_t *frame);
bool hi04_fdcan_stm32h563_read_fifo0(hi04_fdcan_stm32h563_t *port,
                                     hi04_can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
