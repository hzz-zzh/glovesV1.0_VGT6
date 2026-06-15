#ifndef __MODBUS_TIME_SYNC_H__
#define __MODBUS_TIME_SYNC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

HAL_StatusTypeDef ModbusTimeSync_Init(void);
void ModbusTimeSync_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);
void ModbusTimeSync_OnPpsEdge(uint16_t gpio_pin);

uint64_t ModbusTimeSync_GetLocalUptimeUs(void);
uint64_t ModbusTimeSync_GetUtcTimestampUs(void);
uint64_t ModbusTimeSync_GetLastSyncUtcUs(void);
uint64_t ModbusTimeSync_GetLastSyncEdgeLocalUs(void);
uint64_t ModbusTimeSync_GetLastLocalIntervalUs(void);
uint64_t ModbusTimeSync_GetPredictedEdgeUtcUs(void);
int64_t ModbusTimeSync_GetLastSyncErrorUs(void);
int32_t ModbusTimeSync_GetFreqCorrPpb(void);
uint8_t ModbusTimeSync_IsSynced(void);
uint8_t ModbusTimeSync_IsWaitingUtc(void);

void ModbusTimeSync_SetUtcFromMaster(uint64_t utc_us);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_TIME_SYNC_H__ */
