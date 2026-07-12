#include "watchdogTask.h"

#include "cmsis_os2.h"
#include "system_watchdog.h"

#define WATCHDOG_TASK_START_DELAY_MS    (3000U)
#define WATCHDOG_TASK_REFRESH_MS        (100U)

void WatchdogTask(void *argument)
{
    (void)argument;

    /* 先让外设初始化和采集任务进入稳定状态，再启动无法关闭的硬件IWDG。 */
    osDelay(WATCHDOG_TASK_START_DELAY_MS);
    SystemWatchdog_Start();

    for (;;)
    {
        /* 看门狗任务禁止执行日志和外设访问，只保留硬件刷新与延时。 */
        SystemWatchdog_Refresh();
        osDelay(WATCHDOG_TASK_REFRESH_MS);
    }
}
