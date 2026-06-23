#include "uartDebugTask.h"

#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "dataProcessTask.h"
#include "data_manager.h"
#include "frameAssemblerTask.h"
#include "imuCanTask.h"
#include "RS485_uasrt.h"
#include "rs485Task.h"
#include "uart_redirect.h"

#define UART_DEBUG_PRINT_PERIOD_MS    (1000U)
#define UART_DEBUG_PRINT_IMU_DUMP     (0U)

#if (UART_DEBUG_PRINT_IMU_DUMP != 0U)
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
#endif

static void UartDebugTask_PrintChainStatus(uint32_t tick_count)
{
    DataManagerStats_t dm_stats;
    FrameAssemblerStats_t frame_stats;
    DataProcessStats_t process_stats;
    ImuCanTaskDebugSnapshot_t imu_stats;
    RS485_StatusTypeDef rs485_status;
    Rs485TaskStats_t rs485_task_stats;

    DataManager_GetStats(&dm_stats);
    FrameAssemblerTask_GetStats(&frame_stats);
    DataProcessTask_GetStats(&process_stats);
    ImuCanTask_GetDebugSnapshot(&imu_stats);
    RS485_GetStatus(&rs485_status);
    RS485_TaskGetStats(&rs485_task_stats);

    printf("[CHAIN] tick=%lu imu_pub=%lu touch_pub=%lu raw_pub=%lu full_pub=%lu processed=%lu raw_recv=%lu alg_fail=%lu drops(i=%lu,t=%lu,r=%lu,f=%lu) alloc_fail=%lu queue_fail=%lu last_status(frame=%u,proc=%u)\r\n",
           (unsigned long)tick_count,
           (unsigned long)dm_stats.data.imu_sensor_published,
           (unsigned long)dm_stats.data.touch_sensor_published,
           (unsigned long)dm_stats.data.raw_frames_published,
           (unsigned long)dm_stats.data.full_frames_published,
           (unsigned long)process_stats.processed_frames,
           (unsigned long)process_stats.raw_frames_received,
           (unsigned long)process_stats.joint_solve_failures,
           (unsigned long)dm_stats.data.imu_sensor_dropped,
           (unsigned long)dm_stats.data.touch_sensor_dropped,
           (unsigned long)dm_stats.data.raw_frames_dropped,
           (unsigned long)dm_stats.data.full_frames_dropped,
           (unsigned long)dm_stats.data.pool_alloc_failures,
           (unsigned long)dm_stats.data.queue_send_failures,
           (unsigned int)frame_stats.last_status,
           (unsigned int)process_stats.last_status);

    printf("[FRAME] assembled=%lu imu_wait=%lu touch_wait=%lu imu_stale=%lu touch_stale=%lu mismatch=%lu raw_alloc_fail=%lu raw_pub_fail=%lu dt_us=%lu last_id=%lu\r\n",
           (unsigned long)frame_stats.assembled_frames,
           (unsigned long)frame_stats.imu_wait_timeouts,
           (unsigned long)frame_stats.touch_wait_timeouts,
           (unsigned long)frame_stats.imu_stale_drops,
           (unsigned long)frame_stats.touch_stale_drops,
           (unsigned long)frame_stats.timestamp_mismatch_drops,
           (unsigned long)frame_stats.raw_alloc_failures,
           (unsigned long)frame_stats.raw_publish_failures,
           (unsigned long)frame_stats.last_time_diff_us,
           (unsigned long)frame_stats.last_frame_id);

    printf("[IMU_CAN] irq=%lu rx=%lu parsed=%lu unparsed=%lu rejected=%lu pub=%lu drop=%lu init_err=%lu err=%lu last=0x%08lx ext=%lu dlc=%lu cfg_tx=%lu cfg_reply=%lu first_node=%lu seen=0x%08lx qsrc=%lu q1e4=(%ld,%ld,%ld,%ld)\r\n",
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
           (unsigned long)imu_stats.cfg_reply_count,
           (unsigned long)imu_stats.first_valid_node_id,
           (unsigned long)imu_stats.first_valid_seen_mask,
           (unsigned long)imu_stats.quat_source,
           (long)imu_stats.quat_w_1e4,
           (long)imu_stats.quat_x_1e4,
           (long)imu_stats.quat_y_1e4,
           (long)imu_stats.quat_z_1e4);

    printf("[IMU_BUS] started=0x%lx state=(%lu,%lu) act=(%lu,%lu) lec=(%lu,%lu) dlec=(%lu,%lu) txerr=(%lu,%lu) rxerr=(%lu,%lu) warn=(%lu,%lu) ep=(%lu,%lu) bo=(%lu,%lu)\r\n",
           (unsigned long)imu_stats.bus_started_mask,
           (unsigned long)imu_stats.bus_state[0],
           (unsigned long)imu_stats.bus_state[1],
           (unsigned long)imu_stats.bus_activity[0],
           (unsigned long)imu_stats.bus_activity[1],
           (unsigned long)imu_stats.bus_last_error_code[0],
           (unsigned long)imu_stats.bus_last_error_code[1],
           (unsigned long)imu_stats.bus_data_last_error_code[0],
           (unsigned long)imu_stats.bus_data_last_error_code[1],
           (unsigned long)imu_stats.bus_tx_error_count[0],
           (unsigned long)imu_stats.bus_tx_error_count[1],
           (unsigned long)imu_stats.bus_rx_error_count[0],
           (unsigned long)imu_stats.bus_rx_error_count[1],
           (unsigned long)imu_stats.bus_warning[0],
           (unsigned long)imu_stats.bus_warning[1],
           (unsigned long)imu_stats.bus_error_passive[0],
           (unsigned long)imu_stats.bus_error_passive[1],
           (unsigned long)imu_stats.bus_off[0],
           (unsigned long)imu_stats.bus_off[1]);

    printf("[RS485] rx_events=%lu rx_taken=%lu resp=%lu no_resp=%lu frame_err=%lu tx_done=%lu tx_fail=%lu full_snap=%lu tx_busy=%u err=%lu task_rx=%lu task_tx=%lu\r\n",
           (unsigned long)rs485_status.rx_events,
           (unsigned long)rs485_status.rx_taken,
           (unsigned long)rs485_status.modbus_response_ready,
           (unsigned long)rs485_status.modbus_no_response,
           (unsigned long)rs485_status.modbus_frame_error,
           (unsigned long)rs485_status.tx_done,
           (unsigned long)rs485_status.tx_send_fail,
           (unsigned long)rs485_task_stats.full_frame_count,
           (unsigned int)rs485_status.tx_busy,
           (unsigned long)rs485_status.errors,
           (unsigned long)rs485_task_stats.rx_event_count,
           (unsigned long)rs485_task_stats.tx_event_count);

    printf("[RS485_IO] rx_len=%u rx=%02x %02x %02x %02x %02x %02x %02x %02x tx_len=%u tx=%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
           (unsigned int)rs485_status.last_rx_size,
           (unsigned int)rs485_status.last_rx_head[0],
           (unsigned int)rs485_status.last_rx_head[1],
           (unsigned int)rs485_status.last_rx_head[2],
           (unsigned int)rs485_status.last_rx_head[3],
           (unsigned int)rs485_status.last_rx_head[4],
           (unsigned int)rs485_status.last_rx_head[5],
           (unsigned int)rs485_status.last_rx_head[6],
           (unsigned int)rs485_status.last_rx_head[7],
           (unsigned int)rs485_status.last_tx_size,
           (unsigned int)rs485_status.last_tx_head[0],
           (unsigned int)rs485_status.last_tx_head[1],
           (unsigned int)rs485_status.last_tx_head[2],
           (unsigned int)rs485_status.last_tx_head[3],
           (unsigned int)rs485_status.last_tx_head[4],
           (unsigned int)rs485_status.last_tx_head[5],
           (unsigned int)rs485_status.last_tx_head[6],
           (unsigned int)rs485_status.last_tx_head[7]);
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
        UartDebugTask_PrintChainStatus(tick_count);

#if (UART_DEBUG_PRINT_IMU_DUMP != 0U)
        UartDebugTask_PrintImuDump();
#endif

        tick_count++;
        osDelay(UART_DEBUG_PRINT_PERIOD_MS);
    }
}
