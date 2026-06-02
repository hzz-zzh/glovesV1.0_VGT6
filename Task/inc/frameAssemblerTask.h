#ifndef FRAME_ASSEMBLER_TASK_H
#define FRAME_ASSEMBLER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

/* 合帧任务运行统计 用于调试时间同步 丢帧和内存压力 */
typedef struct
{
    uint32_t assembled_frames;
    uint32_t imu_wait_timeouts;
    uint32_t touch_wait_timeouts;
    uint32_t imu_stale_drops;
    uint32_t touch_stale_drops;
    uint32_t timestamp_mismatch_drops;
    uint32_t raw_alloc_failures;
    uint32_t raw_publish_failures;
    uint32_t last_frame_id;
    uint32_t last_time_diff_us;
    GloveStatus_t last_status;
} FrameAssemblerStats_t;

void FrameAssemblerTask(void *argument);
void FrameAssemblerTask_GetStats(FrameAssemblerStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_ASSEMBLER_TASK_H */
