#include "test_task.h"

#include <stddef.h>

#include "app_data.h"
#include "cmsis_os2.h"
#include "data_manager.h"
#include "uart_redirect.h"

typedef struct
{
    volatile uint32_t pass_count;
    volatile uint32_t fail_count;
    volatile GloveStatus_t last_status;
    volatile uint32_t last_frame_id;
    volatile uint32_t last_storage_frame_id;
    volatile uint32_t last_rs485_frame_id;
} TestTaskStats_t;

static TestTaskStats_t s_test_stats;

static void Test_SetError(GloveStatus_t status)
{
    s_test_stats.last_status = status;
    s_test_stats.fail_count++;
}

static GloveTimestampUs_t Test_MakeTimestampUs(uint32_t seq)
{
    return (GloveTimestampUs_t)seq * 10000ULL;
}

static void Test_FillImuSensor(GloveImuSensorBlock_t *block, uint32_t seq)
{
    if (block == NULL)
    {
        return;
    }

    block->data.sensor_seq = seq;
    block->data.timestamp_us = Test_MakeTimestampUs(seq);
    block->data.valid_flags = GLOVE_FRAME_FLAG_IMU_VALID |
                              GLOVE_FRAME_FLAG_QUAT_VALID |
                              GLOVE_FRAME_VALID_IMU_ALL_MASK;

    for (uint32_t i = 0U; i < GLOVE_IMU_COUNT; i++)
    {
        block->data.imu[i].accel_mps2.x = (float)i + 0.10f;
        block->data.imu[i].accel_mps2.y = (float)i + 0.20f;
        block->data.imu[i].accel_mps2.z = (float)i + 0.30f;

        block->data.imu[i].gyro_radps.x = (float)i + 1.10f;
        block->data.imu[i].gyro_radps.y = (float)i + 1.20f;
        block->data.imu[i].gyro_radps.z = (float)i + 1.30f;

        block->data.quat[i].w = 1.0f;
        block->data.quat[i].x = (float)i * 0.01f;
        block->data.quat[i].y = (float)i * 0.02f;
        block->data.quat[i].z = (float)i * 0.03f;
    }
}

static void Test_FillTouchSensor(GloveTouchSensorBlock_t *block, uint32_t seq)
{
    if (block == NULL)
    {
        return;
    }

    block->data.sensor_seq = seq;
    block->data.timestamp_us = Test_MakeTimestampUs(seq) + 100U;
    block->data.valid_flags = GLOVE_FRAME_FLAG_TOUCH_VALID;

    for (uint32_t i = 0U; i < GLOVE_TOUCH_COUNT; i++)
    {
        block->data.touch[i].value = (uint16_t)(1000U + i + seq);
        block->data.touch[i].baseline = (uint16_t)(900U + i);
    }
}

static void Test_FillProcessedFrame(GloveProcessedFrame_t *processed, const GloveRawFrame_t *raw)
{
    if ((processed == NULL) || (raw == NULL))
    {
        return;
    }

    AppData_ClearProcessedFrame(processed);
    processed->frame_id = raw->frame_id;
    processed->timestamp_us = raw->timestamp_us;
    processed->valid_flags = GLOVE_FRAME_FLAG_ALGORITHM_VALID;

    for (uint32_t i = 0U; i < GLOVE_IMU_COUNT; i++)
    {
        processed->imu_attitude[i] = raw->quat[i];
    }

    for (uint32_t i = 0U; i < GLOVE_JOINT_DOF_COUNT; i++)
    {
        processed->joint_angle_deg[i] = (float)i;
    }
}

static GloveStatus_t Test_ProduceSensors(uint32_t seq)
{
    GloveImuSensorBlock_t *imu = DataManager_AllocImuSensor();
    GloveTouchSensorBlock_t *touch = DataManager_AllocTouchSensor();
    GloveStatus_t status;

    if ((imu == NULL) || (touch == NULL))
    {
        if (imu != NULL)
        {
            (void)DataManager_ReleaseImuSensor(imu);
        }
        if (touch != NULL)
        {
            (void)DataManager_ReleaseTouchSensor(touch);
        }
        return GLOVE_STATUS_NO_MEMORY;
    }

    Test_FillImuSensor(imu, seq);
    Test_FillTouchSensor(touch, seq);

    status = DataManager_PublishImuSensor(imu, 0U);
    if (status != GLOVE_STATUS_OK)
    {
        (void)DataManager_ReleaseTouchSensor(touch);
        return status;
    }

    status = DataManager_PublishTouchSensor(touch, 0U);
    return status;
}

static GloveStatus_t Test_AssembleRawFrame(uint32_t frame_id)
{
    GloveImuSensorBlock_t *imu = NULL;
    GloveTouchSensorBlock_t *touch = NULL;
    GloveRawFrameBlock_t *raw = NULL;
    GloveStatus_t status;

    status = DataManager_GetImuSensor(&imu, 10U);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = DataManager_GetTouchSensor(&touch, 10U);
    if (status != GLOVE_STATUS_OK)
    {
        (void)DataManager_ReleaseImuSensor(imu);
        return status;
    }

    raw = DataManager_AllocRawFrame();
    if (raw == NULL)
    {
        (void)DataManager_ReleaseImuSensor(imu);
        (void)DataManager_ReleaseTouchSensor(touch);
        return GLOVE_STATUS_NO_MEMORY;
    }

    AppData_BuildRawFrameFromSensors(&raw->frame,
                                     frame_id,
                                     imu->data.timestamp_us,
                                     &imu->data,
                                     &touch->data);

    (void)DataManager_ReleaseImuSensor(imu);
    (void)DataManager_ReleaseTouchSensor(touch);

    return DataManager_PublishRawFrame(raw, 0U);
}

static GloveStatus_t Test_RunAlgorithm(void)
{
    GloveRawFrameBlock_t *raw = NULL;
    GloveFullFrameBlock_t *full = NULL;
    GloveProcessedFrame_t processed;
    GloveStatus_t status;

    status = DataManager_GetRawFrame(DATA_CONSUMER_ALGORITHM, &raw, 10U);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    full = DataManager_AllocFullFrame();
    if (full == NULL)
    {
        (void)DataManager_ReleaseRawFrame(raw);
        return GLOVE_STATUS_NO_MEMORY;
    }

    Test_FillProcessedFrame(&processed, &raw->frame);
    AppData_BuildFullFrame(&full->frame, &raw->frame, &processed);

    (void)DataManager_ReleaseRawFrame(raw);

    return DataManager_PublishFullFrame(full, 0U);
}

static GloveStatus_t Test_ConsumeFullFrames(void)
{
    GloveFullFrameBlock_t *storage_frame = NULL;
    GloveFullFrameBlock_t *rs485_frame = NULL;
    GloveStatus_t status;

    status = DataManager_GetFullFrame(DATA_CONSUMER_STORAGE, &storage_frame, 10U);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    s_test_stats.last_storage_frame_id = storage_frame->frame.frame_id;
    (void)DataManager_ReleaseFullFrame(storage_frame);

    status = DataManager_GetFullFrame(DATA_CONSUMER_RS485, &rs485_frame, 10U);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    s_test_stats.last_rs485_frame_id = rs485_frame->frame.frame_id;
    (void)DataManager_ReleaseFullFrame(rs485_frame);

    return GLOVE_STATUS_OK;
}

static GloveStatus_t Test_RunOneFrame(uint32_t frame_id)
{
    GloveStatus_t status;

    status = Test_ProduceSensors(frame_id);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Test_AssembleRawFrame(frame_id);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Test_RunAlgorithm();
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Test_ConsumeFullFrames();
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    s_test_stats.last_frame_id = frame_id;
    return GLOVE_STATUS_OK;
}

void StartTestTask(void *argument)
{
    uint32_t frame_id = 0U;
    GloveStatus_t status;

    (void)argument;

    for (;;)
    {
        // status = Test_RunOneFrame(frame_id);
        // s_test_stats.last_status = status;

        // if (status == GLOVE_STATUS_OK)
        // {
        //     s_test_stats.pass_count++;
        //     frame_id++;
        // }
        // else
        // {
        //     Test_SetError(status);
        // }

        // printf("TestTask: frame_id=%lu, status=%d, pass_count=%lu, fail_count=%lu\r\n",
        //        s_test_stats.last_frame_id,
        //        s_test_stats.last_status,
        //        s_test_stats.pass_count,
        //        s_test_stats.fail_count);

        osDelay(1000U);
    }
}
