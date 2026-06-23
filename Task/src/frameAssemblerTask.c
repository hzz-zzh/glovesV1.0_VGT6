#include "frameAssemblerTask.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "acq_sync.h"
#include "data_manager.h"

#define FRAME_ASSEMBLER_GET_TIMEOUT_MS          (10U)
#define FRAME_ASSEMBLER_IDLE_DELAY_MS           (1U)
#define FRAME_ASSEMBLER_MAX_TIME_DIFF_US        (5000ULL)
#define FRAME_ASSEMBLER_PENDING_TIMEOUT_MS      (100U)
#define FRAME_ASSEMBLER_DEBUG_PRINT_ENABLE      (1U)
#define FRAME_ASSEMBLER_DEBUG_PRINT_PERIOD      (50U)
#define FRAME_ASSEMBLER_DEBUG_STATUS_PERIOD_MS  (1000U)
#define FRAME_ASSEMBLER_DEBUG_PRINT_FULL        (0U)
#define FRAME_ASSEMBLER_DEBUG_IMU_PRINT_COUNT   (2U)
#define FRAME_ASSEMBLER_DEBUG_TOUCH_PRINT_COUNT (16U)

static FrameAssemblerStats_t s_frame_assembler_stats;
static uint32_t s_next_frame_id;

#if (FRAME_ASSEMBLER_DEBUG_PRINT_ENABLE != 0U)
static int32_t FrameAssembler_FloatToMilli(float value)
{
    return (int32_t)(value * 1000.0f);
}

static int32_t FrameAssembler_FloatTo1e4(float value)
{
    return (int32_t)(value * 10000.0f);
}

static uint32_t FrameAssembler_GetImuValidMask(uint32_t valid_flags)
{
    return (valid_flags & GLOVE_FRAME_VALID_IMU_ALL_MASK) >>
           GLOVE_FRAME_VALID_IMU_BIT_SHIFT;
}

static uint32_t FrameAssembler_DebugImuCount(void)
{
#if (FRAME_ASSEMBLER_DEBUG_PRINT_FULL != 0U)
    return GLOVE_IMU_COUNT;
#else
    return (FRAME_ASSEMBLER_DEBUG_IMU_PRINT_COUNT > GLOVE_IMU_COUNT) ?
           GLOVE_IMU_COUNT :
           FRAME_ASSEMBLER_DEBUG_IMU_PRINT_COUNT;
#endif
}

static uint32_t FrameAssembler_DebugTouchCount(void)
{
#if (FRAME_ASSEMBLER_DEBUG_PRINT_FULL != 0U)
    return GLOVE_TOUCH_COUNT;
#else
    return (FRAME_ASSEMBLER_DEBUG_TOUCH_PRINT_COUNT > GLOVE_TOUCH_COUNT) ?
           GLOVE_TOUCH_COUNT :
           FRAME_ASSEMBLER_DEBUG_TOUCH_PRINT_COUNT;
#endif
}

static void FrameAssembler_PrintRawFrame(const GloveRawFrame_t *raw,
                                         const GloveImuSensorData_t *imu,
                                         const GloveTouchSensorData_t *touch,
                                         uint64_t time_diff_us)
{
    uint32_t ts_hi;
    uint32_t ts_lo;
    uint32_t imu_count;
    uint32_t touch_count;

    if ((raw == NULL) || (imu == NULL) || (touch == NULL))
    {
        return;
    }

    if ((FRAME_ASSEMBLER_DEBUG_PRINT_PERIOD > 1U) &&
        ((raw->frame_id % FRAME_ASSEMBLER_DEBUG_PRINT_PERIOD) != 0U))
    {
        return;
    }

    ts_hi = (uint32_t)(raw->timestamp_us >> 32);
    ts_lo = (uint32_t)(raw->timestamp_us & 0xFFFFFFFFULL);
    imu_count = FrameAssembler_DebugImuCount();
    touch_count = FrameAssembler_DebugTouchCount();

    printf("[FRAME] id=%lu ts=0x%08lX%08lX flags=0x%08lX imu_seq=%lu touch_seq=%lu dt_us=%lu imu_mask=0x%04lX\r\n",
           (unsigned long)raw->frame_id,
           (unsigned long)ts_hi,
           (unsigned long)ts_lo,
           (unsigned long)raw->valid_flags,
           (unsigned long)imu->sensor_seq,
           (unsigned long)touch->sensor_seq,
           (unsigned long)time_diff_us,
           (unsigned long)FrameAssembler_GetImuValidMask(raw->valid_flags));

    for (uint32_t i = 0U; i < imu_count; i++)
    {
        printf("[FRAME_IMU] i=%lu acc_mg=(%ld,%ld,%ld) gyr_mradps=(%ld,%ld,%ld) quat_1e4=(%ld,%ld,%ld,%ld)\r\n",
               (unsigned long)i,
               (long)FrameAssembler_FloatToMilli(raw->imu[i].accel_mps2.x),
               (long)FrameAssembler_FloatToMilli(raw->imu[i].accel_mps2.y),
               (long)FrameAssembler_FloatToMilli(raw->imu[i].accel_mps2.z),
               (long)FrameAssembler_FloatToMilli(raw->imu[i].gyro_radps.x),
               (long)FrameAssembler_FloatToMilli(raw->imu[i].gyro_radps.y),
               (long)FrameAssembler_FloatToMilli(raw->imu[i].gyro_radps.z),
               (long)FrameAssembler_FloatTo1e4(raw->quat[i].w),
               (long)FrameAssembler_FloatTo1e4(raw->quat[i].x),
               (long)FrameAssembler_FloatTo1e4(raw->quat[i].y),
               (long)FrameAssembler_FloatTo1e4(raw->quat[i].z));
    }

    printf("[FRAME_TOUCH] count=%lu values=", (unsigned long)touch_count);
    for (uint32_t i = 0U; i < touch_count; i++)
    {
        printf("%u", (unsigned int)raw->touch[i].value);
        if ((i + 1U) < touch_count)
        {
            printf(",");
        }
    }
    printf("\r\n");
}
#endif

/* 将毫秒转换为 RTOS tick 用于判断 pending 数据是否等待过久 */
static uint32_t FrameAssembler_MsToTicks(uint32_t timeout_ms)
{
    uint64_t ticks = ((uint64_t)timeout_ms * (uint64_t)osKernelGetTickFreq() + 999ULL) / 1000ULL;

    if ((timeout_ms > 0U) && (ticks == 0ULL))
    {
        ticks = 1ULL;
    }

    return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

/* 64 位微秒时间戳  直接按单调递增时间计算绝对差值 */
#if (FRAME_ASSEMBLER_DEBUG_PRINT_ENABLE != 0U)
static void FrameAssembler_PrintStatusIfDue(const GloveImuSensorBlock_t *pending_imu,
                                            const GloveTouchSensorBlock_t *pending_touch)
{
    static uint32_t s_last_status_print_tick = 0U;
    uint32_t now_tick = osKernelGetTickCount();
    uint32_t period_ticks = FrameAssembler_MsToTicks(FRAME_ASSEMBLER_DEBUG_STATUS_PERIOD_MS);
    DataManagerStats_t dm_stats;
    AcqSyncSnapshot_t sync;
    uint8_t sync_valid;
    uint32_t sync_ts_hi = 0U;
    uint32_t sync_ts_lo = 0U;
    uint32_t pending_imu_seq = 0xFFFFFFFFUL;
    uint32_t pending_touch_seq = 0xFFFFFFFFUL;

    if ((s_last_status_print_tick != 0U) &&
        ((uint32_t)(now_tick - s_last_status_print_tick) < period_ticks))
    {
        return;
    }
    s_last_status_print_tick = now_tick;

    DataManager_GetStats(&dm_stats);
    sync_valid = AcqSync_GetLatest(&sync);
    if (sync_valid != 0U)
    {
        sync_ts_hi = (uint32_t)(sync.timestamp_us >> 32);
        sync_ts_lo = (uint32_t)(sync.timestamp_us & 0xFFFFFFFFULL);
    }
    if (pending_imu != NULL)
    {
        pending_imu_seq = pending_imu->data.sensor_seq;
    }
    if (pending_touch != NULL)
    {
        pending_touch_seq = pending_touch->data.sensor_seq;
    }

    printf("[FRAME_STAT] sync_valid=%u sync_seq=%lu sync_ts=0x%08lX%08lX imu_pub=%lu touch_pub=%lu raw_pub=%lu imu_drop=%lu touch_drop=%lu raw_drop=%lu alloc_fail=%lu qfail=%lu asm=%lu imu_wait=%lu touch_wait=%lu mismatch=%lu imu_stale=%lu touch_stale=%lu pending_imu=%lu pending_touch=%lu last_dt=%lu status=%u\r\n",
           (unsigned int)sync_valid,
           (unsigned long)((sync_valid != 0U) ? sync.seq : 0U),
           (unsigned long)sync_ts_hi,
           (unsigned long)sync_ts_lo,
           (unsigned long)dm_stats.data.imu_sensor_published,
           (unsigned long)dm_stats.data.touch_sensor_published,
           (unsigned long)dm_stats.data.raw_frames_published,
           (unsigned long)dm_stats.data.imu_sensor_dropped,
           (unsigned long)dm_stats.data.touch_sensor_dropped,
           (unsigned long)dm_stats.data.raw_frames_dropped,
           (unsigned long)dm_stats.data.pool_alloc_failures,
           (unsigned long)dm_stats.data.queue_send_failures,
           (unsigned long)s_frame_assembler_stats.assembled_frames,
           (unsigned long)s_frame_assembler_stats.imu_wait_timeouts,
           (unsigned long)s_frame_assembler_stats.touch_wait_timeouts,
           (unsigned long)s_frame_assembler_stats.timestamp_mismatch_drops,
           (unsigned long)s_frame_assembler_stats.imu_stale_drops,
           (unsigned long)s_frame_assembler_stats.touch_stale_drops,
           (unsigned long)pending_imu_seq,
           (unsigned long)pending_touch_seq,
           (unsigned long)s_frame_assembler_stats.last_time_diff_us,
           (unsigned int)s_frame_assembler_stats.last_status);
}
#endif

static uint64_t FrameAssembler_TimeDiffAbsUs(GloveTimestampUs_t a, GloveTimestampUs_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static uint8_t FrameAssembler_IsFirstOlder(GloveTimestampUs_t first_timestamp_us,
                                           GloveTimestampUs_t second_timestamp_us)
{
    return (first_timestamp_us < second_timestamp_us) ? 1U : 0U;
}

static uint8_t FrameAssembler_IsPendingTimeout(uint32_t start_tick)
{
    uint32_t elapsed_ticks = osKernelGetTickCount() - start_tick;
    uint32_t timeout_ticks = FrameAssembler_MsToTicks(FRAME_ASSEMBLER_PENDING_TIMEOUT_MS);

    return (elapsed_ticks >= timeout_ticks) ? 1U : 0U;
}

/* 当前以 IMU 时间戳作为 RawFrame 时间戳 因为姿态算法主要依赖 IMU 数据 */
static GloveTimestampUs_t FrameAssembler_SelectFrameTimestamp(const GloveImuSensorData_t *imu,
                                                              const GloveTouchSensorData_t *touch)
{
    (void)touch;
    return imu->timestamp_us;
}

static void FrameAssembler_SetStatus(GloveStatus_t status)
{
    s_frame_assembler_stats.last_status = status;
}

static void FrameAssembler_ReleaseImu(GloveImuSensorBlock_t **imu)
{
    if ((imu != NULL) && (*imu != NULL))
    {
        (void)DataManager_ReleaseImuSensor(*imu);
        *imu = NULL;
    }
}

static void FrameAssembler_ReleaseTouch(GloveTouchSensorBlock_t **touch)
{
    if ((touch != NULL) && (*touch != NULL))
    {
        (void)DataManager_ReleaseTouchSensor(*touch);
        *touch = NULL;
    }
}

static GloveStatus_t FrameAssembler_PublishRawFrame(const GloveImuSensorBlock_t *imu,
                                                    const GloveTouchSensorBlock_t *touch,
                                                    uint64_t time_diff_us)
{
    GloveRawFrameBlock_t *raw;
    GloveStatus_t status;
    GloveTimestampUs_t frame_timestamp_us;

    raw = DataManager_AllocRawFrame();
    if (raw == NULL)
    {
        s_frame_assembler_stats.raw_alloc_failures++;
        return GLOVE_STATUS_NO_MEMORY;
    }

    frame_timestamp_us = FrameAssembler_SelectFrameTimestamp(&imu->data, &touch->data);

    AppData_BuildRawFrameFromSensors(&raw->frame,
                                     s_next_frame_id,
                                     frame_timestamp_us,
                                     &imu->data,
                                     &touch->data);

#if (FRAME_ASSEMBLER_DEBUG_PRINT_ENABLE != 0U)
    FrameAssembler_PrintRawFrame(&raw->frame,
                                 &imu->data,
                                 &touch->data,
                                 time_diff_us);
#endif

    status = DataManager_PublishRawFrame(raw, 0U);
    if (status != GLOVE_STATUS_OK)
    {
        s_frame_assembler_stats.raw_publish_failures++;
        return status;
    }

    s_frame_assembler_stats.assembled_frames++;
    s_frame_assembler_stats.last_frame_id = s_next_frame_id;
    s_next_frame_id++;

    return GLOVE_STATUS_OK;
}

/* 两类 Sensor 数据都已到达时 判断时间戳是否允许合帧 */
static GloveStatus_t FrameAssembler_TryAssemble(GloveImuSensorBlock_t **imu,
                                                GloveTouchSensorBlock_t **touch)
{
    uint64_t time_diff_us;
    GloveStatus_t status;

    if ((imu == NULL) || (touch == NULL) || (*imu == NULL) || (*touch == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    time_diff_us = FrameAssembler_TimeDiffAbsUs((*imu)->data.timestamp_us, (*touch)->data.timestamp_us);
    s_frame_assembler_stats.last_time_diff_us = time_diff_us;

    if (time_diff_us > FRAME_ASSEMBLER_MAX_TIME_DIFF_US)
    {
        s_frame_assembler_stats.timestamp_mismatch_drops++;

        /* 时间差过大时丢弃更旧的一包 保留较新的等待下一包匹配 */
        if (FrameAssembler_IsFirstOlder((*imu)->data.timestamp_us, (*touch)->data.timestamp_us) != 0U)
        {
            s_frame_assembler_stats.imu_stale_drops++;
            FrameAssembler_ReleaseImu(imu);
        }
        else
        {
            s_frame_assembler_stats.touch_stale_drops++;
            FrameAssembler_ReleaseTouch(touch);
        }

        return GLOVE_STATUS_TIMEOUT;
    }

    status = FrameAssembler_PublishRawFrame(*imu, *touch, time_diff_us);

    FrameAssembler_ReleaseImu(imu);
    FrameAssembler_ReleaseTouch(touch);

    return status;
}

void FrameAssemblerTask_GetStats(FrameAssemblerStats_t *stats)
{
    if (stats != NULL)
    {
        taskENTER_CRITICAL();
        *stats = s_frame_assembler_stats;
        taskEXIT_CRITICAL();
    }
}

void FrameAssemblerTask(void *argument)
{
    GloveImuSensorBlock_t *pending_imu = NULL;
    GloveTouchSensorBlock_t *pending_touch = NULL;
    uint32_t pending_imu_tick = 0U;
    uint32_t pending_touch_tick = 0U;
    GloveStatus_t status;

    (void)argument;
    (void)memset(&s_frame_assembler_stats, 0, sizeof(s_frame_assembler_stats));

    for (;;)
    {
        if (pending_imu == NULL)
        {
            status = DataManager_GetImuSensor(&pending_imu, FRAME_ASSEMBLER_GET_TIMEOUT_MS);
            if (status == GLOVE_STATUS_OK)
            {
                pending_imu_tick = osKernelGetTickCount();
            }
            else if (status == GLOVE_STATUS_TIMEOUT)
            {
                s_frame_assembler_stats.imu_wait_timeouts++;
            }
            else if (status != GLOVE_STATUS_QUEUE_EMPTY)
            {
                FrameAssembler_SetStatus(status);
            }
        }

        if (pending_touch == NULL)
        {
            status = DataManager_GetTouchSensor(&pending_touch, FRAME_ASSEMBLER_GET_TIMEOUT_MS);
            if (status == GLOVE_STATUS_OK)
            {
                pending_touch_tick = osKernelGetTickCount();
            }
            else if (status == GLOVE_STATUS_TIMEOUT)
            {
                s_frame_assembler_stats.touch_wait_timeouts++;
            }
            else if (status != GLOVE_STATUS_QUEUE_EMPTY)
            {
                FrameAssembler_SetStatus(status);
            }
        }

        if ((pending_imu != NULL) && (pending_touch != NULL))
        {
            status = FrameAssembler_TryAssemble(&pending_imu, &pending_touch);
            FrameAssembler_SetStatus(status);
        }
        else
        {
            if ((pending_imu != NULL) && (FrameAssembler_IsPendingTimeout(pending_imu_tick) != 0U))
            {
                s_frame_assembler_stats.imu_stale_drops++;
                FrameAssembler_ReleaseImu(&pending_imu);
            }

            if ((pending_touch != NULL) && (FrameAssembler_IsPendingTimeout(pending_touch_tick) != 0U))
            {
                s_frame_assembler_stats.touch_stale_drops++;
                FrameAssembler_ReleaseTouch(&pending_touch);
            }

            osDelay(FRAME_ASSEMBLER_IDLE_DELAY_MS);
        }

#if (FRAME_ASSEMBLER_DEBUG_PRINT_ENABLE != 0U)
        FrameAssembler_PrintStatusIfDue(pending_imu, pending_touch);
#endif
    }
}
