#include "frameAssemblerTask.h"

#include <stddef.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "data_manager.h"

#define FRAME_ASSEMBLER_GET_TIMEOUT_MS          (10U)
#define FRAME_ASSEMBLER_IDLE_DELAY_MS           (1U)
#define FRAME_ASSEMBLER_MAX_TIME_DIFF_US        (5000ULL)
#define FRAME_ASSEMBLER_PENDING_TIMEOUT_MS      (100U)

static FrameAssemblerStats_t s_frame_assembler_stats;
static uint32_t s_next_frame_id;

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
                                                    const GloveTouchSensorBlock_t *touch)
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

    status = FrameAssembler_PublishRawFrame(*imu, *touch);

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
    }
}