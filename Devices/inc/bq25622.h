#ifndef BQ25622_H
#define BQ25622_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"
#include "i2c_bus.h"

#define BQ25622_DEFAULT_TIMEOUT_MS        (20U)

/* Charger_Flag_0，读取寄存器后由芯片自动清除。 */
#define BQ25622_EVENT_WATCHDOG            (1U << 0)
#define BQ25622_EVENT_SAFETY_TIMER        (1U << 1)
#define BQ25622_EVENT_VINDPM              (1U << 2)
#define BQ25622_EVENT_IINDPM              (1U << 3)
#define BQ25622_EVENT_VSYS                (1U << 4)
#define BQ25622_EVENT_THERMAL_REGULATION  (1U << 5)
#define BQ25622_EVENT_ADC_DONE            (1U << 6)

/* Charger_Flag_1。 */
#define BQ25622_EVENT_VBUS_CHANGED        (1U << 0)
#define BQ25622_EVENT_CHARGE_CHANGED      (1U << 3)

/* Fault_Flag_0。 */
#define BQ25622_EVENT_TS_CHANGED          (1U << 0)
#define BQ25622_EVENT_THERMAL_SHUTDOWN    (1U << 3)
#define BQ25622_EVENT_OTG_FAULT           (1U << 4)
#define BQ25622_EVENT_SYSTEM_FAULT        (1U << 5)
#define BQ25622_EVENT_BATTERY_FAULT       (1U << 6)
#define BQ25622_EVENT_VBUS_FAULT          (1U << 7)

typedef struct
{
    I2cBusId_t bus_id;
    uint32_t timeout_ms;
} Bq25622Handle_t;

typedef struct
{
    uint8_t charge_status;
    uint8_t vbus_status;
    uint8_t temperature_status;
    uint8_t charger_status0;
    uint8_t fault_status0;
    uint8_t vbus_present;
    uint8_t current_valid;
    uint8_t reserved;
    uint16_t battery_voltage_mv;
    uint16_t vbus_voltage_mv;
    int16_t battery_current_ma;
    int16_t input_current_ma;
} Bq25622StatusSnapshot_t;

typedef struct
{
    uint8_t charger_flag0;
    uint8_t charger_flag1;
    uint8_t fault_flag0;
    uint8_t reserved;
} Bq25622InterruptFlags_t;

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
GloveStatus_t Bq25622_SetTerminationCurrentMa(const Bq25622Handle_t *handle,
                                              uint16_t current_ma);
GloveStatus_t Bq25622_EnableAdc(const Bq25622Handle_t *handle);
GloveStatus_t Bq25622_EnableChargeSafety(const Bq25622Handle_t *handle);
GloveStatus_t Bq25622_ConfigureInterrupts(const Bq25622Handle_t *handle);
GloveStatus_t Bq25622_ReadInterruptFlags(const Bq25622Handle_t *handle,
                                         Bq25622InterruptFlags_t *flags);
GloveStatus_t Bq25622_ReadStatusSnapshot(const Bq25622Handle_t *handle,
                                         Bq25622StatusSnapshot_t *snapshot);
GloveStatus_t Bq25622_ReadChargeCurrentLimitMa(const Bq25622Handle_t *handle,
                                               uint16_t *current_ma);
GloveStatus_t Bq25622_PrintChargeStatus(const Bq25622Handle_t *handle);
GloveStatus_t Bq25622_DumpDebugRegisters(const Bq25622Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* BQ25622_H */
