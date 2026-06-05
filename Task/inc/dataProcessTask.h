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

/*
 * Override this weak function in the algorithm module when the calibrated
 * hand kinematic model is ready. The task keeps the DataManager ownership
 * flow unchanged either way.
 */
GloveStatus_t DataProcess_SolveJointAngles(const GloveRawFrame_t *raw,
                                           float joint_angle_rad[GLOVE_JOINT_DOF_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* DATA_PROCESS_TASK_H */
