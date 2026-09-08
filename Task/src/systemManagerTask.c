#include "systemManagerTask.h"

#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system_health.h"
#include "system_watchdog.h"

#define SYSTEM_MANAGER_LOOP_PERIOD_MS (10U)

/*
 * 新板由外部电源直接供电，不再安装BQ25622、MAX17043和外设电源开关。
 * 保留电源状态结构及任务接口，避免485寄存器布局和其他任务接口发生变化。
 */
static GloveBatteryStatus_t s_battery_status;
static GlovePowerStatus_t s_power_status = {
    .flags = GLOVE_POWER_FLAG_PERIPHERAL_ON,
    .system_state = GLOVE_POWER_STATE_ON_NORMAL,
    .charge_state = GLOVE_CHARGE_STATE_UNKNOWN,
    .battery_level = GLOVE_BATTERY_LEVEL_UNKNOWN
};

static uint32_t SystemManager_MsToTicks(uint32_t timeout_ms)
{
    uint64_t ticks = ((uint64_t)timeout_ms * osKernelGetTickFreq() + 999ULL) / 1000ULL;

    if ((timeout_ms > 0U) && (ticks == 0ULL))
    {
        ticks = 1ULL;
    }
    return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

static uint32_t SystemManager_GetTimestampMs(void)
{
    uint32_t frequency = osKernelGetTickFreq();

    return (frequency == 0U) ? 0U :
           (uint32_t)(((uint64_t)osKernelGetTickCount() * 1000ULL) / frequency);
}

static void SystemManager_ResetUnavailablePowerStatus(void)
{
    taskENTER_CRITICAL();
    (void)memset(&s_battery_status, 0, sizeof(s_battery_status));
    (void)memset(&s_power_status, 0, sizeof(s_power_status));

    /* 电池和充电数据在新板上不存在，保持无效且不报告通信故障。 */
    s_battery_status.last_status = GLOVE_STATUS_NOT_READY;
    s_power_status.flags = GLOVE_POWER_FLAG_PERIPHERAL_ON;
    s_power_status.system_state = GLOVE_POWER_STATE_ON_NORMAL;
    s_power_status.charge_state = GLOVE_CHARGE_STATE_UNKNOWN;
    s_power_status.battery_level = GLOVE_BATTERY_LEVEL_UNKNOWN;
    taskEXIT_CRITICAL();
}

static void SystemManager_ClearRemovedHardwareFaults(void)
{
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_LOW_BATTERY,
                          SYSTEM_ERROR_BATTERY_LOW,
                          SYSTEM_HEALTH_SOURCE_BATTERY,
                          0U,
                          0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_CRITICAL_BATTERY,
                          SYSTEM_ERROR_BATTERY_CRITICAL,
                          SYSTEM_HEALTH_SOURCE_BATTERY,
                          0U,
                          0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_BQ_COMM,
                          SYSTEM_ERROR_BQ_COMM,
                          SYSTEM_HEALTH_SOURCE_CHARGER,
                          0U,
                          0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_GAUGE_COMM,
                          SYSTEM_ERROR_GAUGE_COMM,
                          SYSTEM_HEALTH_SOURCE_BATTERY,
                          0U,
                          0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_VOLTAGE_MISMATCH,
                          SYSTEM_ERROR_VOLTAGE_MISMATCH,
                          SYSTEM_HEALTH_SOURCE_BATTERY,
                          0U,
                          0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_TEMP_LIMIT,
                          SYSTEM_ERROR_TEMPERATURE_LIMIT,
                          SYSTEM_HEALTH_SOURCE_CHARGER,
                          0U,
                          0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_CHARGE_FAULT,
                          SYSTEM_ERROR_CHARGE_FAULT,
                          SYSTEM_HEALTH_SOURCE_CHARGER,
                          0U,
                          0U);
}

static void SystemManager_UpdateFixedPowerStatus(void)
{
    taskENTER_CRITICAL();
    s_power_status.timestamp_ms = SystemManager_GetTimestampMs();
    s_power_status.flags = GLOVE_POWER_FLAG_PERIPHERAL_ON;
    s_power_status.system_state = GLOVE_POWER_STATE_ON_NORMAL;
    taskEXIT_CRITICAL();
}

void SystemManagerTask_GetPowerStatus(GlovePowerStatus_t *status)
{
    if (status == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *status = s_power_status;
    taskEXIT_CRITICAL();
}

void SystemManagerTask_GetBatteryStatus(GloveBatteryStatus_t *status)
{
    if (status == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *status = s_battery_status;
    taskEXIT_CRITICAL();
}

void SystemManagerTask(void *argument)
{
    uint32_t next_wake;
    SystemWatchdogStatus_t watchdog_status;

    (void)argument;
    SystemManager_ResetUnavailablePowerStatus();
    SystemManager_ClearRemovedHardwareFaults();
    SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_NONE);
    next_wake = osKernelGetTickCount();

    for (;;)
    {
        SystemManager_UpdateFixedPowerStatus();
        SystemHealth_SetPowerState(GLOVE_POWER_STATE_ON_NORMAL);
        SystemHealth_Service();

        SystemWatchdog_GetStatus(&watchdog_status);
        SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_WATCHDOG_WARNING,
                              SYSTEM_ERROR_WATCHDOG_CONFIG,
                              SYSTEM_HEALTH_SOURCE_WATCHDOG,
                              0U,
                              ((watchdog_status.status_flags &
                                SYSTEM_WATCHDOG_STATUS_CONFIG_WARNING) != 0U) ? 1U : 0U);

        next_wake += SystemManager_MsToTicks(SYSTEM_MANAGER_LOOP_PERIOD_MS);
        (void)osDelayUntil(next_wake);
    }
}
