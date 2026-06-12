#ifndef DATA_PROCESS_TASK_H
#define DATA_PROCESS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

typedef struct
{
    uint32_t raw_frames_received;
    uint32_t processed_frames;
    uint32_t full_frames_published;
    uint32_t raw_wait_timeouts;
    uint32_t invalid_input_frames;
    uint32_t joint_solve_failures;
    uint32_t full_alloc_failures;
    uint32_t full_publish_failures;
    uint32_t raw_release_failures;
    uint32_t last_frame_id;
    uint32_t last_process_time_us;
    GloveStatus_t last_status;
} DataProcessStats_t;

void DataProcessTask(void *argument);
void DataProcessTask_GetStats(DataProcessStats_t *stats);

GloveStatus_t DataProcessTask_SetHandSide(GloveHandSide_t hand_side);
GloveStatus_t DataProcessTask_SetCalibration(uint8_t imu_id,
                                             const GloveQuaternion_t *c_calib,
                                             const GloveQuaternion_t *m_calib);
GloveStatus_t DataProcessTask_SetCalibrationTable(const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                                  const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT]);
GloveStatus_t DataProcessTask_ResetCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* DATA_PROCESS_TASK_H */
