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
 * imu_valid_mask 的 bit0 对应 1 号 IMU，依次到 bit15 对应 16 号 IMU；
 * raw_quat 使用工程原生下标，即 raw_quat[imu_id - 1]。
 *
 * 输出顺序与 hand_solve_src_6.22 保持一致：
 * [0..15]  食指/中指/无名指/小拇指，每指依次为 MCP 屈伸、MCP 侧摆、PIP 屈伸、DIP 屈伸
 * [16..18] 大拇指 MCP 屈伸、MCP 侧摆、IP 屈伸
 * [19..22] 大拇指 CMC 四元数 w、x、y、z
 * [23..26] 手掌绝对姿态四元数 w、x、y、z
 * 缺少必要 IMU 时，对应输出设置为 HAND_SOLVE_MISSING_VALUE。
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
