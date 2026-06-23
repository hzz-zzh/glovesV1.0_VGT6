#ifndef HAND_SOLVE_H
#define HAND_SOLVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

#define HAND_SOLVE_FINGER_COUNT                 (5U)
#define HAND_SOLVE_SEGMENT_COUNT_PER_FINGER     (3U)
#define HAND_SOLVE_INVALID_IMU_ID               (0U)
#define HAND_SOLVE_MISSING_VALUE                (1.0e9f)

typedef struct
{
    uint8_t palm_imu_id;
    uint8_t finger_imu_id[HAND_SOLVE_FINGER_COUNT][HAND_SOLVE_SEGMENT_COUNT_PER_FINGER];
    GloveHandSide_t hand_side;
} HandSolveLayout_t;

void HandSolve_FillIdentityCalibration(GloveQuaternion_t calibration[GLOVE_IMU_COUNT]);

GloveStatus_t HandSolve_InitDefaultLayout(HandSolveLayout_t *layout,
                                          GloveHandSide_t hand_side);

/*
 * imu_valid_mask uses bit 0 for IMU id 1, bit 1 for IMU id 2, ... bit 15 for
 * IMU id 16. raw_quat is the project native array: raw_quat[imu_id - 1].
 *
 * Output order follows hand_solve_src_6.22:
 * [0..15]  index/middle/ring/little: MCP flex, MCP swing, PIP flex, DIP flex
 * [16..18] thumb: MCP flex, MCP swing, IP flex
 * [19..22] thumb CMC quaternion w,x,y,z
 * [23..26] palm absolute quaternion w,x,y,z
 * Values that cannot be solved because an IMU is missing are set to
 * HAND_SOLVE_MISSING_VALUE.
 */
GloveStatus_t HandSolve_SolveAnglesDeg(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                       uint32_t imu_valid_mask,
                                       const HandSolveLayout_t *layout,
                                       const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                       const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                       float joint_angle_deg[GLOVE_JOINT_DOF_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* HAND_SOLVE_H */
