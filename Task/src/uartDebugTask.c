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
#define UART_DEBUG_FLUSH_MAX_BYTES    (3072U)

static uint32_t UartDebugTask_RateHzX10(uint32_t delta_count,
                                        uint32_t elapsed_ticks)
{
    uint32_t tick_freq = osKernelGetTickFreq();

    if ((elapsed_ticks == 0U) || (tick_freq == 0U))
    {
        return 0U;
    }

    return (uint32_t)(((uint64_t)delta_count * (uint64_t)tick_freq * 10ULL) /
                      (uint64_t)elapsed_ticks);
}

static void UartDebugTask_PrintHz10(uint32_t hz_x10)
{
    printf("%lu.%lu", (unsigned long)(hz_x10 / 10U), (unsigned long)(hz_x10 % 10U));
}

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

static void UartDebugTask_PrintChainStatus(uint32_t sample_count)
{
    DataManagerStats_t dm_stats;
    FrameAssemblerStats_t frame_stats;
    DataProcessStats_t process_stats;
    ImuCanTaskDebugSnapshot_t imu_stats;
    static uint8_t initialized = 0U;
    static uint32_t last_tick = 0U;
    static uint32_t last_imu_pub = 0U;
    static uint32_t last_touch_pub = 0U;
    static uint32_t last_raw_pub = 0U;
    static uint32_t last_full_pub = 0U;
    static uint32_t last_processed = 0U;
    static uint32_t last_imu_rx = 0U;
    uint32_t now_tick;
    uint32_t elapsed_ticks;
    uint32_t imu_delta;
    uint32_t touch_delta;
    uint32_t raw_delta;
    uint32_t full_delta;
    uint32_t processed_delta;
    uint32_t imu_rx_delta;

    DataManager_GetStats(&dm_stats);
    FrameAssemblerTask_GetStats(&frame_stats);
    DataProcessTask_GetStats(&process_stats);
    ImuCanTask_GetDebugSnapshot(&imu_stats);

    now_tick = osKernelGetTickCount();
    if (initialized == 0U)
    {
        initialized = 1U;
        last_tick = now_tick;
        last_imu_pub = dm_stats.data.imu_sensor_published;
        last_touch_pub = dm_stats.data.touch_sensor_published;
        last_raw_pub = dm_stats.data.raw_frames_published;
        last_full_pub = dm_stats.data.full_frames_published;
        last_processed = process_stats.processed_frames;
        last_imu_rx = imu_stats.rx_frame_count;
        return;
    }

    elapsed_ticks = now_tick - last_tick;
    imu_delta = dm_stats.data.imu_sensor_published - last_imu_pub;
    touch_delta = dm_stats.data.touch_sensor_published - last_touch_pub;
    raw_delta = dm_stats.data.raw_frames_published - last_raw_pub;
    full_delta = dm_stats.data.full_frames_published - last_full_pub;
    processed_delta = process_stats.processed_frames - last_processed;
    imu_rx_delta = imu_stats.rx_frame_count - last_imu_rx;

    printf("[RATE] sample=%lu imu_pub=%lu hz=",
           (unsigned long)sample_count,
           (unsigned long)imu_delta);
    UartDebugTask_PrintHz10(UartDebugTask_RateHzX10(imu_delta, elapsed_ticks));
    printf(" touch_pub=%lu hz=", (unsigned long)touch_delta);
    UartDebugTask_PrintHz10(UartDebugTask_RateHzX10(touch_delta, elapsed_ticks));
    printf(" raw_pub=%lu hz=", (unsigned long)raw_delta);
    UartDebugTask_PrintHz10(UartDebugTask_RateHzX10(raw_delta, elapsed_ticks));
    printf(" full_pub=%lu hz=", (unsigned long)full_delta);
    UartDebugTask_PrintHz10(UartDebugTask_RateHzX10(full_delta, elapsed_ticks));
    printf(" processed=%lu hz=", (unsigned long)processed_delta);
    UartDebugTask_PrintHz10(UartDebugTask_RateHzX10(processed_delta, elapsed_ticks));
    printf(" imu_rx=%lu hz=", (unsigned long)imu_rx_delta);
    UartDebugTask_PrintHz10(UartDebugTask_RateHzX10(imu_rx_delta, elapsed_ticks));
    printf("\r\n");

    printf("[COUNT] imu=%lu touch=%lu raw=%lu full=%lu processed=%lu raw_recv=%lu drops(i=%lu,t=%lu,r=%lu,f=%lu) alloc_fail=%lu queue_fail=%lu\r\n",
           (unsigned long)dm_stats.data.imu_sensor_published,
           (unsigned long)dm_stats.data.touch_sensor_published,
           (unsigned long)dm_stats.data.raw_frames_published,
           (unsigned long)dm_stats.data.full_frames_published,
           (unsigned long)process_stats.processed_frames,
           (unsigned long)process_stats.raw_frames_received,
           (unsigned long)dm_stats.data.imu_sensor_dropped,
           (unsigned long)dm_stats.data.touch_sensor_dropped,
           (unsigned long)dm_stats.data.raw_frames_dropped,
           (unsigned long)dm_stats.data.full_frames_dropped,
           (unsigned long)dm_stats.data.pool_alloc_failures,
           (unsigned long)dm_stats.data.queue_send_failures);

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

    printf("[IMU_CAN] irq=%lu rx=%lu parsed=%lu unparsed=%lu rejected=%lu pub=%lu drop=%lu init_err=%lu err=%lu last=0x%08lx ext=%lu dlc=%lu cfg_tx=%lu cfg_reply=%lu cfg_ok=0x%04lx cfg_fail=0x%04lx cfg_retry=%lu fresh=0x%04x first_node=%lu seen=0x%08lx\r\n",
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
           (unsigned long)imu_stats.cfg_verified_node_mask,
           (unsigned long)imu_stats.cfg_failed_node_mask,
           (unsigned long)imu_stats.cfg_retry_count,
           (unsigned int)ImuCanTask_GetFreshMask(),
           (unsigned long)imu_stats.first_valid_node_id,
           (unsigned long)imu_stats.first_valid_seen_mask);

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

    last_tick = now_tick;
    last_imu_pub = dm_stats.data.imu_sensor_published;
    last_touch_pub = dm_stats.data.touch_sensor_published;
    last_raw_pub = dm_stats.data.raw_frames_published;
    last_full_pub = dm_stats.data.full_frames_published;
    last_processed = process_stats.processed_frames;
    last_imu_rx = imu_stats.rx_frame_count;
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

        UartRedirect_Flush(UART_DEBUG_FLUSH_MAX_BYTES);
        tick_count++;
        osDelay(UART_DEBUG_PRINT_PERIOD_MS);
    }
}
