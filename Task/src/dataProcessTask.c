#include "dataProcessTask.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "cmsis_compiler.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "data_manager.h"

#define DATA_PROCESS_GET_RAW_TIMEOUT_MS         (10U)
#define DATA_PROCESS_IDLE_DELAY_MS              (1U)
#define DATA_PROCESS_FULL_PUBLISH_TIMEOUT_MS    (0U)
#define DATA_PROCESS_QUAT_EPSILON               (1.0e-12f)
#define DATA_PROCESS_US_PER_SECOND_F            (1000000.0f)

typedef struct
{
    uint8_t parent_imu;
    uint8_t child_imu;
} DataProcessJointPair_t;

static const DataProcessJointPair_t s_default_joint_pairs[GLOVE_JOINT_DOF_COUNT] = {
    {0U, 1U},  {1U, 2U},   {2U, 3U},   {0U, 2U},
    {0U, 4U},  {4U, 5U},   {5U, 6U},   {0U, 5U},
    {0U, 7U},  {7U, 8U},   {8U, 9U},   {0U, 8U},
    {0U, 10U}, {10U, 11U}, {11U, 12U}, {0U, 11U},
    {0U, 13U}, {13U, 14U}, {14U, 15U}, {0U, 14U},
    {0U, 0U}
};

static DataProcessStats_t s_data_process_stats;
static float s_last_joint_angle_rad[GLOVE_JOINT_DOF_COUNT];
static GloveTimestampUs_t s_last_processed_timestamp_us;
static uint8_t s_last_joint_angle_valid;

static uint8_t DataProcess_HasValidImuInput(const GloveRawFrame_t *raw)
{
    const uint32_t required_flags = GLOVE_FRAME_FLAG_IMU_VALID | GLOVE_FRAME_FLAG_QUAT_VALID;

    if (raw == NULL)
    {
        return 0U;
    }

    return ((raw->valid_flags & required_flags) == required_flags) ? 1U : 0U;
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

static GloveQuaternion_t DataProcess_NormalizeQuaternion(const GloveQuaternion_t *quat)
{
    GloveQuaternion_t normalized = {1.0f, 0.0f, 0.0f, 0.0f};
    float norm_sq;
    float inv_norm;

    if (quat == NULL)
    {
        return normalized;
    }

    norm_sq = (quat->w * quat->w) +
              (quat->x * quat->x) +
              (quat->y * quat->y) +
              (quat->z * quat->z);

    if (norm_sq <= DATA_PROCESS_QUAT_EPSILON)
    {
        return normalized;
    }

    inv_norm = 1.0f / sqrtf(norm_sq);
    normalized.w = quat->w * inv_norm;
    normalized.x = quat->x * inv_norm;
    normalized.y = quat->y * inv_norm;
    normalized.z = quat->z * inv_norm;

    return normalized;
}

static GloveQuaternion_t DataProcess_QuaternionConjugate(const GloveQuaternion_t *quat)
{
    GloveQuaternion_t conjugate = {1.0f, 0.0f, 0.0f, 0.0f};

    if (quat == NULL)
    {
        return conjugate;
    }

    conjugate.w = quat->w;
    conjugate.x = -quat->x;
    conjugate.y = -quat->y;
    conjugate.z = -quat->z;

    return conjugate;
}

static GloveQuaternion_t DataProcess_QuaternionMultiply(const GloveQuaternion_t *left,
                                                        const GloveQuaternion_t *right)
{
    GloveQuaternion_t product = {1.0f, 0.0f, 0.0f, 0.0f};

    if ((left == NULL) || (right == NULL))
    {
        return product;
    }

    product.w = (left->w * right->w) -
                (left->x * right->x) -
                (left->y * right->y) -
                (left->z * right->z);
    product.x = (left->w * right->x) +
                (left->x * right->w) +
                (left->y * right->z) -
                (left->z * right->y);
    product.y = (left->w * right->y) -
                (left->x * right->z) +
                (left->y * right->w) +
                (left->z * right->x);
    product.z = (left->w * right->z) +
                (left->x * right->y) -
                (left->y * right->x) +
                (left->z * right->w);

    return product;
}

static float DataProcess_GetRelativeAngleRad(const GloveQuaternion_t *parent,
                                             const GloveQuaternion_t *child)
{
    GloveQuaternion_t normalized_parent;
    GloveQuaternion_t normalized_child;
    GloveQuaternion_t parent_inverse;
    GloveQuaternion_t relative;
    float vector_norm;

    normalized_parent = DataProcess_NormalizeQuaternion(parent);
    normalized_child = DataProcess_NormalizeQuaternion(child);
    parent_inverse = DataProcess_QuaternionConjugate(&normalized_parent);
    relative = DataProcess_QuaternionMultiply(&parent_inverse, &normalized_child);

    if (relative.w < 0.0f)
    {
        relative.w = -relative.w;
        relative.x = -relative.x;
        relative.y = -relative.y;
        relative.z = -relative.z;
    }

    vector_norm = sqrtf((relative.x * relative.x) +
                        (relative.y * relative.y) +
                        (relative.z * relative.z));

    return 2.0f * atan2f(vector_norm, relative.w);
}

__WEAK GloveStatus_t DataProcess_SolveJointAngles(const GloveRawFrame_t *raw,
                                                  float joint_angle_rad[GLOVE_JOINT_DOF_COUNT])
{
    if ((raw == NULL) || (joint_angle_rad == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (DataProcess_HasValidImuInput(raw) == 0U)
    {
        return GLOVE_STATUS_NOT_READY;
    }

    for (uint32_t i = 0U; i < GLOVE_JOINT_DOF_COUNT; i++)
    {
        const DataProcessJointPair_t *pair = &s_default_joint_pairs[i];

        joint_angle_rad[i] =
            DataProcess_GetRelativeAngleRad(&raw->quat[pair->parent_imu],
                                            &raw->quat[pair->child_imu]);
    }

    return GLOVE_STATUS_OK;
}

static void DataProcess_UpdateJointVelocity(GloveProcessedFrame_t *processed)
{
    GloveTimestampUs_t diff_us;
    float dt_s;

    if (processed == NULL)
    {
        return;
    }

    if ((s_last_joint_angle_valid != 0U) &&
        (processed->timestamp_us > s_last_processed_timestamp_us))
    {
        diff_us = processed->timestamp_us - s_last_processed_timestamp_us;
        dt_s = (float)diff_us / DATA_PROCESS_US_PER_SECOND_F;

        if (dt_s > 0.0f)
        {
            for (uint32_t i = 0U; i < GLOVE_JOINT_DOF_COUNT; i++)
            {
                processed->joint_velocity_radps[i] =
                    (processed->joint_angle_rad[i] - s_last_joint_angle_rad[i]) / dt_s;
            }
        }
    }

    (void)memcpy(s_last_joint_angle_rad,
                 processed->joint_angle_rad,
                 sizeof(s_last_joint_angle_rad));
    s_last_processed_timestamp_us = processed->timestamp_us;
    s_last_joint_angle_valid = 1U;
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

    status = DataProcess_SolveJointAngles(raw, processed->joint_angle_rad);
    if (status != GLOVE_STATUS_OK)
    {
        if (DataProcess_HasValidImuInput(raw) == 0U)
        {
            s_data_process_stats.invalid_input_frames++;
        }
        return status;
    }

    DataProcess_UpdateJointVelocity(processed);
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
    (void)memset(s_last_joint_angle_rad, 0, sizeof(s_last_joint_angle_rad));
    s_last_processed_timestamp_us = 0ULL;
    s_last_joint_angle_valid = 0U;

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
