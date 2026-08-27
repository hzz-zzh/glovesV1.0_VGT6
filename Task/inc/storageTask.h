#ifndef STORAGE_TASK_H
#define STORAGE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

GloveStatus_t StorageTask_ControlInit(void);
GloveStatus_t StorageTask_RequestStart(uint32_t timeout_ms);
GloveStatus_t StorageTask_RequestStop(uint32_t timeout_ms);
GloveStatus_t StorageTask_RequestStartAsync(void);
GloveStatus_t StorageTask_RequestStopAsync(void);
void StorageTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_TASK_H */
