#include "uartDebugTask.h"

#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "data_manager.h"
#include "imuCanTask.h"
#include "uart_redirect.h"

#define UART_DEBUG_PRINT_PERIOD_MS    (1000U)

static void UartDebugTask_PrintImuNode(uint32_t node_id)
{
    ImuCanTaskDebugSnapshot_t imu;

    ImuCanTask_GetDebugSnapshotForNode(node_id, &imu);
    if (imu.first_valid_node_id != node_id)
    {
        printf("[IMU%02lu] no_data\r\n", (unsigned long)node_id);
        return;
    }

    printf("[IMU%02lu] seen=0x%08lx acc_mg=(%ld,%ld,%ld) gyro_mdps=(%ld,%ld,%ld) q1e4=(%ld,%ld,%ld,%ld) qsrc=%lu last=0x%08lx dlc=%lu\r\n",
           (unsigned long)node_id,
           (unsigned long)imu.first_valid_seen_mask,
           (long)imu.accel_x_mg,
           (long)imu.accel_y_mg,
           (long)imu.accel_z_mg,
           (long)imu.gyro_x_mdps,
           (long)imu.gyro_y_mdps,
           (long)imu.gyro_z_mdps,
           (long)imu.quat_w_1e4,
           (long)imu.quat_x_1e4,
           (long)imu.quat_y_1e4,
           (long)imu.quat_z_1e4,
           (unsigned long)imu.quat_source,
           (unsigned long)imu.node_last_rx_id,
           (unsigned long)imu.node_last_rx_dlc);
}

static void UartDebugTask_PrintImuDump(void)
{
    ImuCanTaskDebugSnapshot_t imu_stats;
    uint32_t first_node_id = ImuCanTask_GetFirstNodeId();
    uint32_t node_count = ImuCanTask_GetNodeCount();

    ImuCanTask_GetDebugSnapshot(&imu_stats);
    printf("[IMU_CAN] irq=%lu rx=%lu parsed=%lu unparsed=%lu rejected=%lu pub=%lu drop=%lu init_err=%lu err=%lu last=0x%08lx ext=%lu dlc=%lu cfg_tx=%lu cfg_reply=%lu\r\n",
           (unsigned long)imu_stats.rx_irq_count,
           (unsigned long)imu_stats.rx_frame_count,
           (unsigned long)imu_stats.parsed_frame_count,
           (unsigned long)imu_stats.unparsed_frame_count,
           (unsigned long)imu_stats.rejected_node_count,
           (unsigned long)imu_stats.published_count,
           (unsigned long)imu_stats.publish_drop_count,
           (unsigned long)imu_stats.init_error_count,
           (unsigned long)imu_stats.last_error,
           (unsigned long)imu_stats.last_rx_id,
           (unsigned long)imu_stats.last_rx_is_extended,
           (unsigned long)imu_stats.last_rx_dlc,
           (unsigned long)imu_stats.cfg_tx_count,
           (unsigned long)imu_stats.cfg_reply_count);

    printf("[IMU] latest synced dump nodes=%lu first=%lu\r\n",
           (unsigned long)node_count,
           (unsigned long)first_node_id);

    for (uint32_t i = 0U; i < node_count; i++)
    {
        UartDebugTask_PrintImuNode(first_node_id + i);
    }
}

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
        UartDebugTask_PrintImuDump();

        tick_count++;
        osDelay(UART_DEBUG_PRINT_PERIOD_MS);
    }
}
