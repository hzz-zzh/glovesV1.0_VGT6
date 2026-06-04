#ifndef SYSTEM_MANAGER_TASK_H
#define SYSTEM_MANAGER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

typedef struct
{
    uint8_t valid;
    uint8_t soc_percent;
    uint16_t soc_centi_percent;
    uint16_t voltage_mv;
    uint16_t reserved;
    uint32_t sample_seq;
    uint32_t timestamp_ms;
    uint32_t consecutive_failures;
    GloveStatus_t last_status;
} GloveBatteryStatus_t;

void SystemManagerTask(void *argument);
void SystemManagerTask_GetBatteryStatus(GloveBatteryStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_MANAGER_TASK_H */
