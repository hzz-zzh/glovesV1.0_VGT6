#include "uartDebugTask.h"

#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "data_manager.h"
#include "uart_redirect.h"

#define UART_DEBUG_PRINT_PERIOD_MS    (1000U)

/**
 * @brief 串口调试线程
 *
 * 低频心跳打印
 */
void UartDebugTask(void *argument)
{
    (void)argument;

    uint32_t tick_count = 0U;

    printf("\r\n[UART] debug task started\r\n");

    for (;;)
    {
        DataManagerStats_t stats;
        DataManager_GetStats(&stats);

        printf("[UART] tick=%lu imu_pub=%lu touch_pub=%lu raw_pub=%lu full_pub=%lu alloc_fail=%lu queue_fail=%lu\r\n",
               (unsigned long)tick_count,
               (unsigned long)stats.data.imu_sensor_published,
               (unsigned long)stats.data.touch_sensor_published,
               (unsigned long)stats.data.raw_frames_published,
               (unsigned long)stats.data.full_frames_published,
               (unsigned long)stats.data.pool_alloc_failures,
               (unsigned long)stats.data.queue_send_failures);

        tick_count++;
        osDelay(UART_DEBUG_PRINT_PERIOD_MS);
    }
}