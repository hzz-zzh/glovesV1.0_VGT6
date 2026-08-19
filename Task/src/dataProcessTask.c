#include "dataProcessTask.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "data_manager.h"
#include "glove_hand_config.h"
#include "hand_solve.h"
#include "system_health.h"

#define DATA_PROCESS_GET_RAW_TIMEOUT_MS         (10U)
#define DATA_PROCESS_IDLE_DELAY_MS              (1U)
#define DATA_PROCESS_FULL_PUBLISH_TIMEOUT_MS    (0U)
#define DATA_PROCESS_FULL_DEBUG_PRINT_ENABLE    (0U)
#define DATA_PROCESS_FULL_DEBUG_PRINT_PERIOD    (50U)
#define DATA_PROCESS_FULL_DEBUG_IMU_PRINT_COUNT (2U)
#define DATA_PROCESS_FULL_DEBUG_TOUCH_COUNT     (16U)
#define DATA_PROCESS_HEALTH_FAILURE_LIMIT       (3U)
#define DATA_PROCESS_HEALTH_RECOVERY_FRAMES     (3U)

typedef struct
{
    HandSolveLayout_t layout;
    GloveQuaternion_t c_calib[GLOVE_IMU_COUNT];
    GloveQuaternion_t m_calib[GLOVE_IMU_COUNT];
    uint8_t calibration_applied;
    uint16_t calibration_seq;
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
    },
    0U,
    0U
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
    uint32_t imu_valid_mask;

    if (raw == NULL)
    {
        return 0U;
    }

    if ((raw->valid_flags & required_flags) != required_flags)
    {
        return 0U;
    }

    imu_valid_mask = DataProcess_GetRawImuValidMask(raw);

    return (imu_valid_mask != 0UL) ? 1U : 0U;
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

    config->layout.hand_side = GloveHandConfig_GetHandSide();
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

#if (DATA_PROCESS_FULL_DEBUG_PRINT_ENABLE != 0U)
static int32_t DataProcess_FloatToMilli(float value)
{
    return (int32_t)(value * 1000.0f);
}

static int32_t DataProcess_FloatTo1e4(float value)
{
    return (int32_t)(value * 10000.0f);
}

static uint32_t DataProcess_GetImuValidMask(uint32_t valid_flags)
{
    return (valid_flags & GLOVE_FRAME_VALID_IMU_ALL_MASK) >>
           GLOVE_FRAME_VALID_IMU_BIT_SHIFT;
}

static uint32_t DataProcess_DebugImuCount(void)
{
    return (DATA_PROCESS_FULL_DEBUG_IMU_PRINT_COUNT > GLOVE_IMU_COUNT) ?
           GLOVE_IMU_COUNT :
           DATA_PROCESS_FULL_DEBUG_IMU_PRINT_COUNT;
}

static uint32_t DataProcess_DebugTouchCount(void)
{
    return (DATA_PROCESS_FULL_DEBUG_TOUCH_COUNT > GLOVE_TOUCH_COUNT) ?
           GLOVE_TOUCH_COUNT :
           DATA_PROCESS_FULL_DEBUG_TOUCH_COUNT;
}

static uint8_t DataProcess_IsMissingJoint(float value)
{
    return ((value >= (HAND_SOLVE_MISSING_VALUE * 0.5f)) ||
            (value <= (-HAND_SOLVE_MISSING_VALUE * 0.5f))) ? 1U : 0U;
}

static uint8_t DataProcess_ShouldPrintFullFrame(void)
{
    static uint32_t s_debug_print_counter;

    s_debug_print_counter++;
    if (s_debug_print_counter == 1UL)
    {
        return 1U;
    }

    if (DATA_PROCESS_FULL_DEBUG_PRINT_PERIOD <= 1U)
    {
        return 1U;
    }

    return ((s_debug_print_counter % DATA_PROCESS_FULL_DEBUG_PRINT_PERIOD) == 0UL) ? 1U : 0U;
}

static void DataProcess_PrintFullFrame(const GloveFullFrame_t *full,
                                       GloveStatus_t process_status)
{
    uint32_t ts_hi;
    uint32_t ts_lo;
    uint32_t imu_count;
    uint32_t touch_count;
    uint32_t imu_mask;

    if (full == NULL)
    {
        return;
    }

    if (DataProcess_ShouldPrintFullFrame() == 0U)
    {
        return;
    }

    ts_hi = (uint32_t)(full->timestamp_us >> 32);
    ts_lo = (uint32_t)(full->timestamp_us & 0xFFFFFFFFULL);
    imu_count = DataProcess_DebugImuCount();
    touch_count = DataProcess_DebugTouchCount();
    imu_mask = DataProcess_GetImuValidMask(full->raw.valid_flags);

    printf("[FULL] id=%lu ts=0x%08lX%08lX flags=0x%08lX raw_flags=0x%08lX proc_flags=0x%08lX process_status=%u imu_mask=0x%04lX joints=%lu\r\n",
           (unsigned long)full->frame_id,
           (unsigned long)ts_hi,
           (unsigned long)ts_lo,
           (unsigned long)full->valid_flags,
           (unsigned long)full->raw.valid_flags,
           (unsigned long)full->processed.valid_flags,
           (unsigned int)process_status,
           (unsigned long)imu_mask,
           (unsigned long)GLOVE_JOINT_DOF_COUNT);

    printf("[FULL_JOINT] count=%lu mdeg=", (unsigned long)GLOVE_JOINT_DOF_COUNT);
    for (uint32_t i = 0U; i < GLOVE_JOINT_DOF_COUNT; i++)
    {
        if (DataProcess_IsMissingJoint(full->processed.joint_angle_deg[i]) != 0U)
        {
            printf("MISS");
        }
        else
        {
            printf("%ld", (long)DataProcess_FloatToMilli(full->processed.joint_angle_deg[i]));
        }

        if ((i + 1U) < GLOVE_JOINT_DOF_COUNT)
        {
            printf(",");
        }
    }
    printf("\r\n");

    for (uint32_t i = 0U; i < imu_count; i++)
    {
        if ((imu_mask & (1UL << i)) == 0UL)
        {
            continue;
        }

        printf("[FULL_IMU] i=%lu acc_mps2_1e3=(%ld,%ld,%ld) gyr_radps_1e3=(%ld,%ld,%ld) quat_1e4=(%ld,%ld,%ld,%ld)\r\n",
               (unsigned long)i,
               (long)DataProcess_FloatToMilli(full->raw.imu[i].accel_mps2.x),
               (long)DataProcess_FloatToMilli(full->raw.imu[i].accel_mps2.y),
               (long)DataProcess_FloatToMilli(full->raw.imu[i].accel_mps2.z),
               (long)DataProcess_FloatToMilli(full->raw.imu[i].gyro_radps.x),
               (long)DataProcess_FloatToMilli(full->raw.imu[i].gyro_radps.y),
               (long)DataProcess_FloatToMilli(full->raw.imu[i].gyro_radps.z),
               (long)DataProcess_FloatTo1e4(full->raw.quat[i].w),
               (long)DataProcess_FloatTo1e4(full->raw.quat[i].x),
               (long)DataProcess_FloatTo1e4(full->raw.quat[i].y),
               (long)DataProcess_FloatTo1e4(full->raw.quat[i].z));
    }

    printf("[FULL_TOUCH] count=%lu values=", (unsigned long)touch_count);
    for (uint32_t i = 0U; i < touch_count; i++)
    {
        printf("%u", (unsigned int)full->raw.touch[i].value);
        if ((i + 1U) < touch_count)
        {
            printf(",");
        }
    }
    printf("\r\n");
}
#endif

static GloveStatus_t DataProcess_SolveJointAnglesDeg(const GloveRawFrame_t *raw,
                                                     float joint_angle_deg[GLOVE_JOINT_DOF_COUNT],
                                                     uint8_t *calibration_applied)
{
    DataProcessAlgorithmConfig_t config;

    if ((raw == NULL) || (joint_angle_deg == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (calibration_applied != NULL)
    {
        *calibration_applied = 0U;
    }

    if (DataProcess_HasValidImuInput(raw) == 0U)
    {
        for (uint32_t i = 0U; i < GLOVE_JOINT_DOF_COUNT; i++)
        {
            joint_angle_deg[i] = HAND_SOLVE_MISSING_VALUE;
        }
        return GLOVE_STATUS_NOT_READY;
    }

    DataProcess_CopyAlgorithmConfig(&config);
    if (calibration_applied != NULL)
    {
        *calibration_applied = config.calibration_applied;
    }

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
    uint8_t calibration_applied = 0U;

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

    status = DataProcess_SolveJointAnglesDeg(raw,
                                             processed->joint_angle_deg,
                                             &calibration_applied);
    if (status != GLOVE_STATUS_OK)
    {
        s_data_process_stats.invalid_input_frames++;
        return status;
    }

    processed->valid_flags = GLOVE_FRAME_FLAG_ALGORITHM_VALID;
    if (calibration_applied != 0U)
    {
        processed->valid_flags |= GLOVE_FRAME_FLAG_IMU_CALIB_APPLIED;
    }

    return GLOVE_STATUS_OK;
}

static GloveStatus_t DataProcess_PublishFullFrame(const GloveRawFrame_t *raw,
                                                  const GloveProcessedFrame_t *processed,
                                                  GloveStatus_t process_status)
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

#if (DATA_PROCESS_FULL_DEBUG_PRINT_ENABLE != 0U)
    DataProcess_PrintFullFrame(&full->frame, process_status);
#else
    (void)process_status;
#endif

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
    GloveStatus_t status;

    if (DataProcess_IsValidHandSide(hand_side) == 0U)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = GloveHandConfig_SetHandSide(hand_side);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
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
    s_algorithm_config.calibration_applied = 1U;
    s_algorithm_config.calibration_seq = 0U;
    taskEXIT_CRITICAL();

    return GLOVE_STATUS_OK;
}

GloveStatus_t DataProcessTask_SetCalibrationTable(const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                                  const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT])
{
    return DataProcessTask_SetCalibrationTableWithSeq(c_calib, m_calib, 0U);
}

GloveStatus_t DataProcessTask_SetCalibrationTableWithSeq(const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                                         const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                                         uint16_t calibration_seq)
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
    s_algorithm_config.calibration_applied = 1U;
    s_algorithm_config.calibration_seq = calibration_seq;
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
    s_algorithm_config.calibration_applied = 0U;
    s_algorithm_config.calibration_seq = 0U;
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
    uint8_t publish_failure_count = 0U;
    uint8_t publish_success_count = 0U;

    (void)argument;
    (void)memset(&s_data_process_stats, 0, sizeof(s_data_process_stats));
    (void)DataProcessTask_SetHandSide(GloveHandConfig_GetHandSide());

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

            publish_status = DataProcess_PublishFullFrame(&raw->frame,
                                                          &processed,
                                                          process_status);
            if (publish_status == GLOVE_STATUS_OK)
            {
                publish_failure_count = 0U;
                if (publish_success_count < DATA_PROCESS_HEALTH_RECOVERY_FRAMES)
                {
                    publish_success_count++;
                }
                SystemHealth_MarkFullFrame((process_status == GLOVE_STATUS_OK) ? 1U : 0U);
                if (publish_success_count >= DATA_PROCESS_HEALTH_RECOVERY_FRAMES)
                {
                    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_POOL_EXHAUSTED,
                                          SYSTEM_ERROR_NONE,
                                          SYSTEM_HEALTH_SOURCE_PIPELINE,
                                          0U,
                                          0U);
                    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_QUEUE_PRESSURE,
                                          SYSTEM_ERROR_NONE,
                                          SYSTEM_HEALTH_SOURCE_PIPELINE,
                                          0U,
                                          0U);
                }
            }
            else
            {
                publish_success_count = 0U;
                if (publish_failure_count < DATA_PROCESS_HEALTH_FAILURE_LIMIT)
                {
                    publish_failure_count++;
                }
                if (publish_failure_count >= DATA_PROCESS_HEALTH_FAILURE_LIMIT)
                {
                    SystemHealth_SetFault((publish_status == GLOVE_STATUS_NO_MEMORY) ?
                                          SYSTEM_HEALTH_FLAG_POOL_EXHAUSTED :
                                          SYSTEM_HEALTH_FLAG_QUEUE_PRESSURE,
                                          (publish_status == GLOVE_STATUS_NO_MEMORY) ?
                                          SYSTEM_ERROR_POOL_EXHAUSTED :
                                          SYSTEM_ERROR_QUEUE_FULL,
                                          SYSTEM_HEALTH_SOURCE_PIPELINE,
                                          0U,
                                          1U);
                }
            }

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
