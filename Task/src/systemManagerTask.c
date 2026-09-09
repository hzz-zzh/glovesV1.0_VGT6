#include "systemManagerTask.h"

#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/*
 * 当前产品由外部电源持续供电。
 * 电池检测、充电管理、低电保护和按键控制外设电源均停用。
 */
#define SYSTEM_MANAGER_STATUS_PERIOD_MS    (1000U)

static GloveBatteryStatus_t s_battery_status;
static GlovePowerStatus_t s_power_status;
static volatile uint8_t s_periph_power_enabled = 1U;

static uint32_t SystemManager_GetTimestampMs(void)
{
    uint32_t frequency = osKernelGetTickFreq();

    return (frequency == 0U) ? 0U :
           (uint32_t)(((uint64_t)osKernelGetTickCount() * 1000ULL) / frequency);
}

static void SystemManager_ForceExternalPowerMode(void)
{
    /* DISABLE_CHARGE低电平允许充电控制，同时始终使能传感器外设电源。 */
    HAL_GPIO_WritePin(DISABLE_CHARGE_GPIO_Port, DISABLE_CHARGE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin, GPIO_PIN_SET);
    s_periph_power_enabled = 1U;
}

static void SystemManager_UpdateFixedStatus(void)
{
    uint32_t timestamp_ms = SystemManager_GetTimestampMs();

    taskENTER_CRITICAL();
    s_power_status.timestamp_ms = timestamp_ms;
    s_power_status.battery_voltage_mv = 0U;
    s_power_status.battery_current_ma = 0;
    s_power_status.vbus_voltage_mv = 0U;
    s_power_status.input_current_ma = 0;
    s_power_status.soc_centi_percent = 0U;
    s_power_status.flags = GLOVE_POWER_FLAG_PERIPHERAL_ON;
    s_power_status.fault_code = 0U;
    s_power_status.bq_charger_events = 0U;
    s_power_status.bq_fault_events = 0U;
    s_power_status.bq_interrupt_count = 0U;
    s_power_status.soc_percent = 0U;
    s_power_status.system_state = GLOVE_POWER_STATE_ON_NORMAL;
    s_power_status.charge_state = GLOVE_CHARGE_STATE_UNKNOWN;
    s_power_status.battery_level = GLOVE_BATTERY_LEVEL_UNKNOWN;
    s_power_status.bq_vbus_type = 0U;
    s_power_status.bq_temperature_status = 0U;
    s_power_status.bq_diagnostic_stage = GLOVE_BQ_DIAG_NONE;
    s_power_status.bq_last_status = GLOVE_STATUS_OK;
    taskEXIT_CRITICAL();
}

uint8_t SystemManagerTask_IsPeripheralPowerEnabled(void)
{
    return s_periph_power_enabled;
}

void SystemManagerTask_GetPowerStatus(GlovePowerStatus_t *status)
{
    if (status == NULL) return;

    taskENTER_CRITICAL();
    *status = s_power_status;
    taskEXIT_CRITICAL();
}

void SystemManagerTask_GetBatteryStatus(GloveBatteryStatus_t *status)
{
    if (status == NULL) return;

    taskENTER_CRITICAL();
    *status = s_battery_status;
    taskEXIT_CRITICAL();
}

void SystemManagerTask(void *argument)
{
    (void)argument;
    (void)memset(&s_battery_status, 0, sizeof(s_battery_status));
    (void)memset(&s_power_status, 0, sizeof(s_power_status));

    s_battery_status.last_status = GLOVE_STATUS_NOT_READY;
    SystemManager_ForceExternalPowerMode();
    SystemManager_UpdateFixedStatus();

    for (;;)
    {
        /* 周期性重申固定电平，避免其他初始化路径意外修改。 */
        SystemManager_ForceExternalPowerMode();
        SystemManager_UpdateFixedStatus();
        osDelay(SYSTEM_MANAGER_STATUS_PERIOD_MS);
    }
}
