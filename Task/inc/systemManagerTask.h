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

typedef enum
{
    GLOVE_POWER_STATE_INIT = 0,
    GLOVE_POWER_STATE_ON_NORMAL = 1,
    GLOVE_POWER_STATE_ON_LOW = 2,
    GLOVE_POWER_STATE_USER_OFF = 3,
    GLOVE_POWER_STATE_LOW_BAT_LOCKOUT = 4,
    GLOVE_POWER_STATE_STOPPING = 5,
    GLOVE_POWER_STATE_RECOVERING = 6,
    GLOVE_POWER_STATE_RECOVERY_FAULT = 7
} GlovePowerState_t;

typedef enum
{
    GLOVE_CHARGE_STATE_UNKNOWN = 0,
    GLOVE_CHARGE_STATE_NO_INPUT = 1,
    GLOVE_CHARGE_STATE_IDLE = 2,
    GLOVE_CHARGE_STATE_CC = 3,
    GLOVE_CHARGE_STATE_CV = 4,
    GLOVE_CHARGE_STATE_TOPOFF = 5,
    GLOVE_CHARGE_STATE_FULL = 6,
    GLOVE_CHARGE_STATE_SUSPENDED = 7,
    GLOVE_CHARGE_STATE_FAULT = 8
} GloveChargeState_t;

typedef enum
{
    GLOVE_BATTERY_LEVEL_UNKNOWN = 0,
    GLOVE_BATTERY_LEVEL_NORMAL = 1,
    GLOVE_BATTERY_LEVEL_LOW = 2,
    GLOVE_BATTERY_LEVEL_CRITICAL = 3
} GloveBatteryLevel_t;

typedef enum
{
    GLOVE_BQ_DIAG_NONE = 0,
    GLOVE_BQ_DIAG_INIT = 1,
    GLOVE_BQ_DIAG_WATCHDOG = 2,
    GLOVE_BQ_DIAG_INPUT_CURRENT = 3,
    GLOVE_BQ_DIAG_EXTERNAL_ILIM = 4,
    GLOVE_BQ_DIAG_CHARGE_VOLTAGE = 5,
    GLOVE_BQ_DIAG_CHARGE_CURRENT = 6,
    GLOVE_BQ_DIAG_TERMINATION_CURRENT = 7,
    GLOVE_BQ_DIAG_CHARGE_SAFETY = 8,
    GLOVE_BQ_DIAG_ADC = 9,
    GLOVE_BQ_DIAG_STATUS_READ = 10,
    GLOVE_BQ_DIAG_INTERRUPT_CONFIG = 11,
    GLOVE_BQ_DIAG_INTERRUPT_READ = 12
} GloveBqDiagnosticStage_t;

#define GLOVE_POWER_FLAG_VOLTAGE_VALID       (1U << 0)
#define GLOVE_POWER_FLAG_SOC_VALID           (1U << 1)
#define GLOVE_POWER_FLAG_CURRENT_VALID       (1U << 2)
#define GLOVE_POWER_FLAG_VBUS_PRESENT        (1U << 3)
#define GLOVE_POWER_FLAG_CHARGING            (1U << 4)
#define GLOVE_POWER_FLAG_LOW                 (1U << 5)
#define GLOVE_POWER_FLAG_CRITICAL            (1U << 6)
#define GLOVE_POWER_FLAG_LOCKOUT             (1U << 7)
#define GLOVE_POWER_FLAG_PERIPHERAL_ON       (1U << 8)
#define GLOVE_POWER_FLAG_BQ_COMM_FAULT       (1U << 9)
#define GLOVE_POWER_FLAG_GAUGE_COMM_FAULT    (1U << 10)
#define GLOVE_POWER_FLAG_VOLTAGE_MISMATCH    (1U << 11)
#define GLOVE_POWER_FLAG_TEMP_LIMITED        (1U << 12)
#define GLOVE_POWER_FLAG_CHARGE_FAULT        (1U << 13)
#define GLOVE_POWER_FLAG_SAFETY_TIMER         (1U << 14)
#define GLOVE_POWER_FLAG_CHARGE_FULL          (1U << 15)

typedef struct
{
    uint32_t sample_seq;
    uint32_t timestamp_ms;
    uint16_t battery_voltage_mv;
    int16_t battery_current_ma;
    uint16_t vbus_voltage_mv;
    int16_t input_current_ma;
    uint16_t soc_centi_percent;
    uint16_t flags;
    uint16_t fault_code;
    uint16_t bq_charger_events;
    uint16_t bq_fault_events;
    uint16_t bq_interrupt_count;
    uint8_t soc_percent;
    uint8_t system_state;
    uint8_t charge_state;
    uint8_t battery_level;
    uint8_t bq_vbus_type;
    uint8_t bq_temperature_status;
    uint8_t bq_diagnostic_stage;
    uint8_t bq_last_status;
} GlovePowerStatus_t;

void SystemManagerTask(void *argument);
void SystemManagerTask_GetBatteryStatus(GloveBatteryStatus_t *status);
void SystemManagerTask_GetPowerStatus(GlovePowerStatus_t *status);
void SystemManagerTask_OnPowerKeyEdgeFromIsr(void);
void SystemManagerTask_OnChargeStatusEdgeFromIsr(void);
void SystemManagerTask_OnBqInterruptFromIsr(void);
uint8_t SystemManagerTask_IsPeripheralPowerEnabled(void);
uint8_t SystemManagerTask_RequestPeripheralRecovery(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_MANAGER_TASK_H */
