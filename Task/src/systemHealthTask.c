#include "systemHealthTask.h"

#include <stdint.h>

#include "acq_sync.h"
#include "cmsis_os2.h"
#include "system_health.h"
#include "system_watchdog.h"

#define SYSTEM_HEALTH_TASK_PERIOD_MS (10U)

static uint32_t SystemHealthTask_MsToTicks(uint32_t timeout_ms)
{
    uint64_t ticks = ((uint64_t)timeout_ms * osKernelGetTickFreq() + 999ULL) / 1000ULL;

    if ((timeout_ms > 0U) && (ticks == 0ULL))
    {
        ticks = 1ULL;
    }
    return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

void SystemHealthTask(void *argument)
{
    uint32_t next_wake;
    SystemWatchdogStatus_t watchdog_status;

    (void)argument;
    next_wake = osKernelGetTickCount();

    for (;;)
    {
        /* PPS超时检测放在独立周期任务中，不依赖485是否正在收发。 */
        AcqSync_Service();
        SystemHealth_Service();

        /* 看门狗刷新由独立高优先级任务负责，此处只汇总配置诊断状态。 */
        SystemWatchdog_GetStatus(&watchdog_status);
        SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_WATCHDOG_WARNING,
                              SYSTEM_ERROR_WATCHDOG_CONFIG,
                              SYSTEM_HEALTH_SOURCE_WATCHDOG,
                              0U,
                              ((watchdog_status.status_flags &
                                SYSTEM_WATCHDOG_STATUS_CONFIG_WARNING) != 0U) ? 1U : 0U);

        next_wake += SystemHealthTask_MsToTicks(SYSTEM_HEALTH_TASK_PERIOD_MS);
        (void)osDelayUntil(next_wake);
    }
}
