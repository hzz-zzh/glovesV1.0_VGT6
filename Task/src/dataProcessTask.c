#include "dataProcessTask.h"

#include <stddef.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "data_manager.h"
#include "hand_solve.h"

#define DATA_PROCESS_GET_RAW_TIMEOUT_MS         (10U)
#define DATA_PROCESS_IDLE_DELAY_MS              (1U)
#define DATA_PROCESS_FULL_PUBLISH_TIMEOUT_MS    (0U)

typedef struct
{
    HandSolveLayout_t layout;
    GloveQuaternion_t c_calib[GLOVE_IMU_COUNT];
    GloveQuaternion_t m_calib[GLOVE_IMU_COUNT];
} DataProcessAlgorithmConfig_t;

static const GloveQuaternion_t s_identity_quat = {1.0f, 0.0f, 0.0f, 0.0f};

static DataProcessStats_t s_data_process_stats;
static DataProcessAlgorithmConfig_t s_algorithm_config = {
    {
        1U,
        {
            {2U, 3U, 4U},
            {5U, 6U, 7U},
            {8U, 9U, 10U},
            {11U, 12U, 13U},
            {14U, 15U, 16U}
        },
        GLOVE_HAND_LEFT
    },
    {
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}
    },
    {
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f}
    }
};

static uint8_t DataProcess_IsValidHandSide(GloveHandSide_t hand_side)
{
    return ((hand_side == GLOVE_HAND_LEFT) || (hand_side == GLOVE_HAND_RIGHT)) ? 1U : 0U;
}

static uint8_t DataProcess_IsValidImuId(uint8_t imu_id)
{
    return ((imu_id >= 1U) && (imu_id <= GLOVE_IMU_COUNT)) ? 1U : 0U;
}

static uint32_t DataProcess_GetRawImuValidMask(const GloveRawFrame_t *raw)
{
    if (raw == NULL)
    {
        return 0UL;
    }

    return (raw->valid_flags & GLOVE_FRAME_VALID_IMU_ALL_MASK) >>
           GLOVE_FRAME_VALID_IMU_BIT_SHIFT;
}

static uint8_t DataProcess_HasValidImuInput(const GloveRawFrame_t *raw)
{
    const uint32_t required_flags = GLOVE_FRAME_FLAG_IMU_VALID | GLOVE_FRAME_FLAG_QUAT_VALID;

    if (raw == NULL)
    {
        return 0U;
    }

    if ((raw->valid_flags & required_flags) != required_flags)
    {
        return 0U;
    }

    return (DataProcess_GetRawImuValidMask(raw) == GLOVE_IMU_VALID_ALL_MASK) ? 1U : 0U;
}

static void DataProcess_CopyAlgorithmConfig(DataProcessAlgorithmConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *config = s_algorithm_config;
    taskEXIT_CRITICAL();
}

static GloveTimestampUs_t DataProcess_GetKernelTimeUs(void)
{
    uint32_t tick_freq = osKernelGetTickFreq();

    if (tick_freq == 0U)
    {
        return 0ULL;
    }

    return (GloveTimestampUs_t)(((uint64_t)osKernelGetTickCount() * 1000000ULL) /
                                (uint64_t)tick_freq);
}

static GloveStatus_t DataProcess_SolveJointAnglesDeg(const GloveRawFrame_t *raw,
                                                     float joint_angle_deg[GLOVE_JOINT_DOF_COUNT])
{
    DataProcessAlgorithmConfig_t config;

    if ((raw == NULL) || (joint_angle_deg == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (DataProcess_HasValidImuInput(raw) == 0U)
    {
        (void)memset(joint_angle_deg, 0, sizeof(float) * GLOVE_JOINT_DOF_COUNT);
        return GLOVE_STATUS_NOT_READY;
    }

    DataProcess_CopyAlgorithmConfig(&config);

    return HandSolve_SolveAnglesDeg(raw->quat,
                                    DataProcess_GetRawImuValidMask(raw),
                                    &config.layout,
                                    config.c_calib,
                                    config.m_calib,
                                    joint_angle_deg);
}

static GloveStatus_t DataProcess_BuildProcessedFrame(const GloveRawFrame_t *raw,
                                                     GloveProcessedFrame_t *processed)
{
    GloveStatus_t status;

    if ((raw == NULL) || (processed == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    AppData_ClearProcessedFrame(processed);
    processed->frame_id = raw->frame_id;
    processed->timestamp_us = raw->timestamp_us;

    if ((raw->valid_flags & GLOVE_FRAME_FLAG_QUAT_VALID) != 0U)
    {
        (void)memcpy(processed->imu_attitude,
                     raw->quat,
                     sizeof(processed->imu_attitude));
    }

    status = DataProcess_SolveJointAnglesDeg(raw, processed->joint_angle_deg);
    if (status != GLOVE_STATUS_OK)
    {
        s_data_process_stats.invalid_input_frames++;
        return status;
    }

    processed->valid_flags = GLOVE_FRAME_FLAG_ALGORITHM_VALID;

    return GLOVE_STATUS_OK;
}

static GloveStatus_t DataProcess_PublishFullFrame(const GloveRawFrame_t *raw,
                                                  const GloveProcessedFrame_t *processed)
{
    GloveFullFrameBlock_t *full;
    GloveStatus_t status;

    if ((raw == NULL) || (processed == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    full = DataManager_AllocFullFrame();
    if (full == NULL)
    {
        s_data_process_stats.full_alloc_failures++;
        return GLOVE_STATUS_NO_MEMORY;
    }

    AppData_BuildFullFrame(&full->frame, raw, processed);

    status = DataManager_PublishFullFrame(full, DATA_PROCESS_FULL_PUBLISH_TIMEOUT_MS);
    if (status != GLOVE_STATUS_OK)
    {
        s_data_process_stats.full_publish_failures++;
        return status;
    }

    s_data_process_stats.full_frames_published++;
    return GLOVE_STATUS_OK;
}

static void DataProcess_SetLastStatus(GloveStatus_t status)
{
    s_data_process_stats.last_status = status;
}

GloveStatus_t DataProcessTask_SetHandSide(GloveHandSide_t hand_side)
{
    if (DataProcess_IsValidHandSide(hand_side) == 0U)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    taskENTER_CRITICAL();
    s_algorithm_config.layout.hand_side = hand_side;
    taskEXIT_CRITICAL();

    return GLOVE_STATUS_OK;
}

GloveStatus_t DataProcessTask_SetCalibration(uint8_t imu_id,
                                             const GloveQuaternion_t *c_calib,
                                             const GloveQuaternion_t *m_calib)
{
    uint32_t index;

    if ((DataProcess_IsValidImuId(imu_id) == 0U) ||
        ((c_calib == NULL) && (m_calib == NULL)))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    index = (uint32_t)imu_id - 1U;

    taskENTER_CRITICAL();
    if (c_calib != NULL)
    {
        s_algorithm_config.c_calib[index] = *c_calib;
    }
    if (m_calib != NULL)
    {
        s_algorithm_config.m_calib[index] = *m_calib;
    }
    taskEXIT_CRITICAL();

    return GLOVE_STATUS_OK;
}

GloveStatus_t DataProcessTask_SetCalibrationTable(const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                                  const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT])
{
    if ((c_calib == NULL) || (m_calib == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    taskENTER_CRITICAL();
    (void)memcpy(s_algorithm_config.c_calib,
                 c_calib,
                 sizeof(s_algorithm_config.c_calib));
    (void)memcpy(s_algorithm_config.m_calib,
                 m_calib,
                 sizeof(s_algorithm_config.m_calib));
    taskEXIT_CRITICAL();

    return GLOVE_STATUS_OK;
}

GloveStatus_t DataProcessTask_ResetCalibration(void)
{
    taskENTER_CRITICAL();
    for (uint32_t i = 0U; i < GLOVE_IMU_COUNT; i++)
    {
        s_algorithm_config.c_calib[i] = s_identity_quat;
        s_algorithm_config.m_calib[i] = s_identity_quat;
    }
    taskEXIT_CRITICAL();

    return GLOVE_STATUS_OK;
}

void DataProcessTask_GetStats(DataProcessStats_t *stats)
{
    if (stats != NULL)
    {
        taskENTER_CRITICAL();
        *stats = s_data_process_stats;
        taskEXIT_CRITICAL();
    }
}

void DataProcessTask(void *argument)
{
    GloveRawFrameBlock_t *raw = NULL;
    GloveProcessedFrame_t processed;
    GloveStatus_t raw_status;
    GloveStatus_t process_status;
    GloveStatus_t publish_status;
    GloveStatus_t release_status;
    GloveTimestampUs_t start_us;
    GloveTimestampUs_t end_us;

    (void)argument;
    (void)memset(&s_data_process_stats, 0, sizeof(s_data_process_stats));

    for (;;)
    {
        raw_status = DataManager_GetRawFrame(DATA_CONSUMER_ALGORITHM,
                                             &raw,
                                             DATA_PROCESS_GET_RAW_TIMEOUT_MS);
        if (raw_status == GLOVE_STATUS_OK)
        {
            start_us = DataProcess_GetKernelTimeUs();
            s_data_process_stats.raw_frames_received++;
            s_data_process_stats.last_frame_id = raw->frame.frame_id;

            process_status = DataProcess_BuildProcessedFrame(&raw->frame, &processed);
            if (process_status == GLOVE_STATUS_OK)
            {
                s_data_process_stats.processed_frames++;
            }
            else
            {
                s_data_process_stats.joint_solve_failures++;
            }

            publish_status = DataProcess_PublishFullFrame(&raw->frame, &processed);

            release_status = DataManager_ReleaseRawFrame(raw);
            raw = NULL;
            if (release_status != GLOVE_STATUS_OK)
            {
                s_data_process_stats.raw_release_failures++;
                DataProcess_SetLastStatus(release_status);
            }
            else if (publish_status != GLOVE_STATUS_OK)
            {
                DataProcess_SetLastStatus(publish_status);
            }
            else
            {
                DataProcess_SetLastStatus(process_status);
            }

            end_us = DataProcess_GetKernelTimeUs();
            if (end_us >= start_us)
            {
                s_data_process_stats.last_process_time_us = (uint32_t)(end_us - start_us);
            }
        }
        else if (raw_status == GLOVE_STATUS_TIMEOUT)
        {
            s_data_process_stats.raw_wait_timeouts++;
            osDelay(DATA_PROCESS_IDLE_DELAY_MS);
        }
        else if (raw_status == GLOVE_STATUS_QUEUE_EMPTY)
        {
            osDelay(DATA_PROCESS_IDLE_DELAY_MS);
        }
        else
        {
            DataProcess_SetLastStatus(raw_status);
            osDelay(DATA_PROCESS_IDLE_DELAY_MS);
        }
    }
}
