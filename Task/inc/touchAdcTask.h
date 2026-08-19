#ifndef TOUCH_ADC_TASK_H
#define TOUCH_ADC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void TouchAdcTask(void *argument);
void TouchAdcTask_SetAcquisitionEnabled(uint8_t enabled);
uint8_t TouchAdcTask_IsAcquisitionPaused(void);
uint8_t TouchAdcTask_IsRecoveryReady(void);

#ifdef __cplusplus
}
#endif

#endif /* TOUCH_ADC_TASK_H */
