#ifndef BQ25622_H
#define BQ25622_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"
#include "i2c_bus.h"

#define BQ25622_DEFAULT_TIMEOUT_MS        (20U)

typedef struct
{
    I2cBusId_t bus_id;
    uint32_t timeout_ms;
} Bq25622Handle_t;

GloveStatus_t Bq25622_Init(Bq25622Handle_t *handle,
                           I2cBusId_t bus_id,
                           uint32_t timeout_ms);
GloveStatus_t Bq25622_DisableWatchdog(const Bq25622Handle_t *handle);
GloveStatus_t Bq25622_EnableExternalIlim(const Bq25622Handle_t *handle);
GloveStatus_t Bq25622_DisableExternalIlim(const Bq25622Handle_t *handle);
GloveStatus_t Bq25622_SetInputCurrentLimitMa(const Bq25622Handle_t *handle,
                                             uint16_t current_ma);
GloveStatus_t Bq25622_SetChargeVoltageLimitMv(const Bq25622Handle_t *handle,
                                              uint16_t voltage_mv);
GloveStatus_t Bq25622_SetChargeCurrentLimitMa(const Bq25622Handle_t *handle,
                                              uint16_t current_ma);
GloveStatus_t Bq25622_ReadChargeCurrentLimitMa(const Bq25622Handle_t *handle,
                                               uint16_t *current_ma);
GloveStatus_t Bq25622_PrintChargeStatus(const Bq25622Handle_t *handle);
GloveStatus_t Bq25622_DumpDebugRegisters(const Bq25622Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* BQ25622_H */
