#include "app_data.h"

#include <stddef.h>
#include <string.h>

void AppData_ClearImuSensorData(GloveImuSensorData_t *data)
{
    if (data != NULL)
    {
        (void)memset(data, 0, sizeof(*data));
    }
}

void AppData_ClearTouchSensorData(GloveTouchSensorData_t *data)
{
    if (data != NULL)
    {
        (void)memset(data, 0, sizeof(*data));
    }
}

void AppData_ClearRawFrame(GloveRawFrame_t *frame)
{
    if (frame != NULL)
    {
        (void)memset(frame, 0, sizeof(*frame));
    }
}

void AppData_ClearProcessedFrame(GloveProcessedFrame_t *frame)
{
    if (frame != NULL)
    {
        (void)memset(frame, 0, sizeof(*frame));
    }
}

void AppData_ClearFullFrame(GloveFullFrame_t *frame)
{
    if (frame != NULL)
    {
        (void)memset(frame, 0, sizeof(*frame));
    }
}

/*
 * 将两个采集任务产生的 Sensor 数据合成为 RawFrame
 * 时间同步策略由 FrameAssemblerTask 决定；这里仅负责结构体拷贝和帧头填充
 */
void AppData_BuildRawFrameFromSensors(GloveRawFrame_t *raw,
                                      uint32_t frame_id,
                                      uint32_t timestamp_us,
                                      const GloveImuSensorData_t *imu,
                                      const GloveTouchSensorData_t *touch)
{
    if ((raw == NULL) || (imu == NULL) || (touch == NULL))
    {
        return;
    }

    raw->frame_id = frame_id;
    raw->timestamp_us = timestamp_us;
    raw->valid_flags = imu->valid_flags | touch->valid_flags;

    (void)memcpy(raw->imu, imu->imu, sizeof(raw->imu));
    (void)memcpy(raw->quat, imu->quat, sizeof(raw->quat));
    (void)memcpy(raw->touch, touch->touch, sizeof(raw->touch));
}

/* 将 RawFrame 和算法输出合成为 FullFrame */
void AppData_BuildFullFrame(GloveFullFrame_t *full,
                            const GloveRawFrame_t *raw,
                            const GloveProcessedFrame_t *processed)
{
    if ((full == NULL) || (raw == NULL) || (processed == NULL))
    {
        return;
    }

    full->frame_id = raw->frame_id;
    full->timestamp_us = raw->timestamp_us;
    full->valid_flags = raw->valid_flags | processed->valid_flags;
    full->raw = *raw;
    full->processed = *processed;
}
