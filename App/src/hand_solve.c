#include "hand_solve.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if GLOVE_IMU_COUNT != 16U
#error "hand_solve expects GLOVE_IMU_COUNT to be 16"
#endif

#if GLOVE_JOINT_DOF_COUNT != 27U
#error "hand_solve expects GLOVE_JOINT_DOF_COUNT to be 27"
#endif

#define HAND_SOLVE_EPSILON                      (1.0e-12f)
#define HAND_SOLVE_PI_F                         (3.14159265358979323846f)
#define HAND_SOLVE_RAD_TO_DEG_F                 (180.0f / HAND_SOLVE_PI_F)
#define HAND_SOLVE_THUMB_INDEX                  (0U)
#define HAND_SOLVE_OUTPUT_FINGER_FIRST          (1U)
#define HAND_SOLVE_FINGER_OUTPUT_COUNT          (4U)

typedef struct
{
    float x;
    float y;
    float z;
} HandSolveVec3_t;

typedef struct
{
    float m[3][3];
} HandSolveMat3_t;

typedef struct
{
    HandSolveVec3_t hinge_axis;
    float sign;
    float neutral_offset_deg;
    float flex_min_deg;
    float flex_max_deg;

    uint8_t matrix_method;
    uint8_t has_swing;
    HandSolveVec3_t swing_axis;
    float swing_sign;
    float swing_neutral_offset_deg;
    float swing_min_deg;
    float swing_max_deg;
} HandSolveJointConfig_t;

typedef struct
{
    uint8_t proximal_imu_id;
    uint8_t distal_imu_id;
    HandSolveJointConfig_t config;
} HandSolveJoint_t;

typedef struct
{
    float flex_deg;
    float swing_deg;
} HandSolveJointResult_t;

typedef struct
{
    float flex_deg;
    float swing_deg;
} HandSolveLuAngles_t;

static const GloveQuaternion_t s_identity_quat = {1.0f, 0.0f, 0.0f, 0.0f};
static const HandSolveVec3_t s_unit_y = {0.0f, 1.0f, 0.0f};
static const HandSolveVec3_t s_unit_z = {0.0f, 0.0f, 1.0f};

static float HandSolve_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float HandSolve_WrapDeg(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg <= -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static float HandSolve_Vec3Dot(HandSolveVec3_t left, HandSolveVec3_t right)
{
    return (left.x * right.x) + (left.y * right.y) + (left.z * right.z);
}

static HandSolveVec3_t HandSolve_Vec3Scale(HandSolveVec3_t vec, float scale)
{
    HandSolveVec3_t out;

    out.x = vec.x * scale;
    out.y = vec.y * scale;
    out.z = vec.z * scale;

    return out;
}

static HandSolveVec3_t HandSolve_Vec3Subtract(HandSolveVec3_t left,
                                              HandSolveVec3_t right)
{
    HandSolveVec3_t out;

    out.x = left.x - right.x;
    out.y = left.y - right.y;
    out.z = left.z - right.z;

    return out;
}

static HandSolveVec3_t HandSolve_Vec3Add(HandSolveVec3_t left,
                                         HandSolveVec3_t right)
{
    HandSolveVec3_t out;

    out.x = left.x + right.x;
    out.y = left.y + right.y;
    out.z = left.z + right.z;

    return out;
}

static HandSolveVec3_t HandSolve_Vec3Negate(HandSolveVec3_t vec)
{
    HandSolveVec3_t out;

    out.x = -vec.x;
    out.y = -vec.y;
    out.z = -vec.z;

    return out;
}

static HandSolveVec3_t HandSolve_Vec3Normalize(HandSolveVec3_t vec)
{
    HandSolveVec3_t out = {0.0f, 0.0f, 0.0f};
    float norm_sq;
    float inv_norm;

    norm_sq = HandSolve_Vec3Dot(vec, vec);
    if (norm_sq <= HAND_SOLVE_EPSILON)
    {
        return out;
    }

    inv_norm = 1.0f / sqrtf(norm_sq);
    out.x = vec.x * inv_norm;
    out.y = vec.y * inv_norm;
    out.z = vec.z * inv_norm;

    return out;
}

static uint8_t HandSolve_IsVec3Valid(HandSolveVec3_t vec)
{
    return (HandSolve_Vec3Dot(vec, vec) > HAND_SOLVE_EPSILON) ? 1U : 0U;
}

static GloveQuaternion_t HandSolve_NormalizeQuat(const GloveQuaternion_t *quat)
{
    GloveQuaternion_t out = s_identity_quat;
    float norm_sq;
    float inv_norm;

    if (quat == NULL)
    {
        return out;
    }

    norm_sq = (quat->w * quat->w) +
              (quat->x * quat->x) +
              (quat->y * quat->y) +
              (quat->z * quat->z);
    if (norm_sq <= HAND_SOLVE_EPSILON)
    {
        return out;
    }

    inv_norm = 1.0f / sqrtf(norm_sq);
    out.w = quat->w * inv_norm;
    out.x = quat->x * inv_norm;
    out.y = quat->y * inv_norm;
    out.z = quat->z * inv_norm;

    return out;
}

static GloveQuaternion_t HandSolve_QuatConjugate(GloveQuaternion_t quat)
{
    GloveQuaternion_t out;

    out.w = quat.w;
    out.x = -quat.x;
    out.y = -quat.y;
    out.z = -quat.z;

    return out;
}

static GloveQuaternion_t HandSolve_QuatMultiply(GloveQuaternion_t left,
                                                GloveQuaternion_t right)
{
    GloveQuaternion_t out;

    out.w = (left.w * right.w) -
            (left.x * right.x) -
            (left.y * right.y) -
            (left.z * right.z);
    out.x = (left.w * right.x) +
            (left.x * right.w) +
            (left.y * right.z) -
            (left.z * right.y);
    out.y = (left.w * right.y) -
            (left.x * right.z) +
            (left.y * right.w) +
            (left.z * right.x);
    out.z = (left.w * right.z) +
            (left.x * right.y) -
            (left.y * right.x) +
            (left.z * right.w);

    return out;
}

static HandSolveVec3_t HandSolve_QuatVec(GloveQuaternion_t quat)
{
    HandSolveVec3_t out;

    out.x = quat.x;
    out.y = quat.y;
    out.z = quat.z;

    return out;
}

static HandSolveMat3_t HandSolve_QuatToRotationMatrix(GloveQuaternion_t quat)
{
    GloveQuaternion_t q;
    HandSolveMat3_t r;
    float xx;
    float yy;
    float zz;
    float xy;
    float xz;
    float yz;
    float wx;
    float wy;
    float wz;

    q = HandSolve_NormalizeQuat(&quat);

    xx = q.x * q.x;
    yy = q.y * q.y;
    zz = q.z * q.z;
    xy = q.x * q.y;
    xz = q.x * q.z;
    yz = q.y * q.z;
    wx = q.w * q.x;
    wy = q.w * q.y;
    wz = q.w * q.z;

    r.m[0][0] = 1.0f - (2.0f * (yy + zz));
    r.m[0][1] = 2.0f * (xy - wz);
    r.m[0][2] = 2.0f * (xz + wy);

    r.m[1][0] = 2.0f * (xy + wz);
    r.m[1][1] = 1.0f - (2.0f * (xx + zz));
    r.m[1][2] = 2.0f * (yz - wx);

    r.m[2][0] = 2.0f * (xz - wy);
    r.m[2][1] = 2.0f * (yz + wx);
    r.m[2][2] = 1.0f - (2.0f * (xx + yy));

    return r;
}

static HandSolveVec3_t HandSolve_Mat3Column(const HandSolveMat3_t *mat,
                                            uint32_t column)
{
    HandSolveVec3_t out = {0.0f, 0.0f, 0.0f};

    if ((mat == NULL) || (column >= 3U))
    {
        return out;
    }

    out.x = mat->m[0][column];
    out.y = mat->m[1][column];
    out.z = mat->m[2][column];

    return out;
}

static GloveQuaternion_t HandSolve_CalibrationOf(const GloveQuaternion_t table[GLOVE_IMU_COUNT],
                                                 uint8_t imu_id)
{
    if ((table == NULL) ||
        (imu_id == HAND_SOLVE_INVALID_IMU_ID) ||
        (imu_id > GLOVE_IMU_COUNT))
    {
        return s_identity_quat;
    }

    return HandSolve_NormalizeQuat(&table[(uint32_t)imu_id - 1U]);
}

static GloveQuaternion_t HandSolve_RawQuatOf(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                             uint8_t imu_id)
{
    if ((raw_quat == NULL) ||
        (imu_id == HAND_SOLVE_INVALID_IMU_ID) ||
        (imu_id > GLOVE_IMU_COUNT))
    {
        return s_identity_quat;
    }

    return HandSolve_NormalizeQuat(&raw_quat[(uint32_t)imu_id - 1U]);
}

static uint8_t HandSolve_IsImuValid(uint32_t imu_valid_mask, uint8_t imu_id)
{
    if ((imu_id == HAND_SOLVE_INVALID_IMU_ID) || (imu_id > GLOVE_IMU_COUNT))
    {
        return 0U;
    }

    return ((imu_valid_mask & (1UL << ((uint32_t)imu_id - 1U))) != 0UL) ? 1U : 0U;
}

static GloveQuaternion_t HandSolve_ApplyCalibration(const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                                    const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                                    uint8_t imu_id,
                                                    GloveQuaternion_t raw)
{
    GloveQuaternion_t c;
    GloveQuaternion_t m;
    GloveQuaternion_t calibrated;

    c = HandSolve_CalibrationOf(c_calib, imu_id);
    m = HandSolve_CalibrationOf(m_calib, imu_id);
    raw = HandSolve_NormalizeQuat(&raw);

    calibrated = HandSolve_QuatMultiply(HandSolve_QuatMultiply(c, raw), m);

    return HandSolve_NormalizeQuat(&calibrated);
}

static float HandSolve_TwistAngleDeg(GloveQuaternion_t quat,
                                     HandSolveVec3_t axis)
{
    GloveQuaternion_t q;
    GloveQuaternion_t twist;
    HandSolveVec3_t normalized_axis;
    float projection;
    float signed_vec;
    float angle_deg;

    normalized_axis = HandSolve_Vec3Normalize(axis);
    if (HandSolve_IsVec3Valid(normalized_axis) == 0U)
    {
        return 0.0f;
    }

    q = HandSolve_NormalizeQuat(&quat);
    projection = HandSolve_Vec3Dot(HandSolve_QuatVec(q), normalized_axis);
    twist.w = q.w;
    twist.x = normalized_axis.x * projection;
    twist.y = normalized_axis.y * projection;
    twist.z = normalized_axis.z * projection;
    twist = HandSolve_NormalizeQuat(&twist);

    signed_vec = HandSolve_Vec3Dot(HandSolve_QuatVec(twist), normalized_axis);
    angle_deg = 2.0f * atan2f(signed_vec, twist.w) * HAND_SOLVE_RAD_TO_DEG_F;

    return HandSolve_WrapDeg(angle_deg);
}

static float HandSolve_SwingAngleDeg(GloveQuaternion_t quat,
                                     HandSolveVec3_t flex_axis,
                                     HandSolveVec3_t swing_axis)
{
    GloveQuaternion_t q;
    GloveQuaternion_t twist;
    GloveQuaternion_t swing;
    HandSolveVec3_t flex;
    HandSolveVec3_t swing_direction;
    float projection;
    float axis_on_flex;
    float signed_vec;
    float angle_deg;

    flex = HandSolve_Vec3Normalize(flex_axis);
    swing_direction = HandSolve_Vec3Normalize(swing_axis);
    if ((HandSolve_IsVec3Valid(flex) == 0U) ||
        (HandSolve_IsVec3Valid(swing_direction) == 0U))
    {
        return 0.0f;
    }

    q = HandSolve_NormalizeQuat(&quat);
    projection = HandSolve_Vec3Dot(HandSolve_QuatVec(q), flex);
    twist.w = q.w;
    twist.x = flex.x * projection;
    twist.y = flex.y * projection;
    twist.z = flex.z * projection;
    twist = HandSolve_NormalizeQuat(&twist);

    swing = HandSolve_QuatMultiply(q, HandSolve_QuatConjugate(twist));
    swing = HandSolve_NormalizeQuat(&swing);

    axis_on_flex = HandSolve_Vec3Dot(swing_direction, flex);
    swing_direction = HandSolve_Vec3Subtract(swing_direction,
                                             HandSolve_Vec3Scale(flex, axis_on_flex));
    swing_direction = HandSolve_Vec3Normalize(swing_direction);
    if (HandSolve_IsVec3Valid(swing_direction) == 0U)
    {
        return 0.0f;
    }

    signed_vec = HandSolve_Vec3Dot(HandSolve_QuatVec(swing), swing_direction);
    angle_deg = 2.0f * atan2f(signed_vec, swing.w) * HAND_SOLVE_RAD_TO_DEG_F;

    return HandSolve_WrapDeg(angle_deg);
}

static HandSolveLuAngles_t HandSolve_LuAnglesDeg(GloveQuaternion_t quat)
{
    HandSolveMat3_t r;
    HandSolveLuAngles_t out;
    float r02;

    r = HandSolve_QuatToRotationMatrix(quat);
    r02 = HandSolve_ClampFloat(r.m[0][2], -1.0f, 1.0f);

    out.flex_deg = HandSolve_WrapDeg(asinf(r02) * HAND_SOLVE_RAD_TO_DEG_F);
    out.swing_deg = HandSolve_WrapDeg(atan2f(r.m[1][2], r.m[2][2]) *
                                      HAND_SOLVE_RAD_TO_DEG_F);

    return out;
}

static HandSolveJointConfig_t HandSolve_MakeDefaultJointConfig(uint8_t is_thumb,
                                                               uint8_t has_swing,
                                                               GloveHandSide_t hand_side)
{
    HandSolveJointConfig_t config;

    config.hinge_axis = s_unit_y;
    config.sign = 1.0f;
    config.neutral_offset_deg = 0.0f;
    config.flex_min_deg = -180.0f;
    config.flex_max_deg = 180.0f;

    config.matrix_method = ((is_thumb != 0U) && (has_swing != 0U)) ? 1U : 0U;
    config.has_swing = has_swing;
    config.swing_axis = s_unit_z;
    config.swing_sign = (hand_side == GLOVE_HAND_LEFT) ? -1.0f : 1.0f;
    config.swing_neutral_offset_deg = 0.0f;
    if ((is_thumb != 0U) && (has_swing != 0U))
    {
        config.swing_min_deg = -90.0f;
        config.swing_max_deg = 90.0f;
    }
    else
    {
        config.swing_min_deg = -30.0f;
        config.swing_max_deg = 30.0f;
    }

    return config;
}

static HandSolveJointResult_t HandSolve_SolveJoint(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                                   const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                                   const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                                   const HandSolveJoint_t *joint)
{
    HandSolveJointResult_t out = {0.0f, 0.0f};
    GloveQuaternion_t raw_proximal;
    GloveQuaternion_t raw_distal;
    GloveQuaternion_t seg_proximal;
    GloveQuaternion_t seg_distal;
    GloveQuaternion_t relative;
    HandSolveLuAngles_t lu;
    float raw_flex;
    float raw_swing;

    if (joint == NULL)
    {
        return out;
    }

    raw_proximal = HandSolve_RawQuatOf(raw_quat, joint->proximal_imu_id);
    raw_distal = HandSolve_RawQuatOf(raw_quat, joint->distal_imu_id);
    seg_proximal = HandSolve_ApplyCalibration(c_calib,
                                              m_calib,
                                              joint->proximal_imu_id,
                                              raw_proximal);
    seg_distal = HandSolve_ApplyCalibration(c_calib,
                                            m_calib,
                                            joint->distal_imu_id,
                                            raw_distal);
    relative = HandSolve_QuatMultiply(HandSolve_QuatConjugate(seg_proximal),
                                      seg_distal);
    relative = HandSolve_NormalizeQuat(&relative);

    if (joint->config.matrix_method != 0U)
    {
        lu = HandSolve_LuAnglesDeg(relative);
        raw_flex = lu.flex_deg;
        raw_swing = lu.swing_deg;
    }
    else
    {
        raw_flex = HandSolve_TwistAngleDeg(relative, joint->config.hinge_axis);
        raw_swing = (joint->config.has_swing != 0U) ?
                    HandSolve_SwingAngleDeg(relative,
                                            joint->config.hinge_axis,
                                            joint->config.swing_axis) :
                    0.0f;
    }

    out.flex_deg = HandSolve_WrapDeg(joint->config.sign *
                                     HandSolve_WrapDeg(raw_flex -
                                                       joint->config.neutral_offset_deg));
    out.flex_deg = HandSolve_ClampFloat(out.flex_deg,
                                        joint->config.flex_min_deg,
                                        joint->config.flex_max_deg);

    if (joint->config.has_swing != 0U)
    {
        out.swing_deg = HandSolve_WrapDeg(joint->config.swing_sign *
                                          HandSolve_WrapDeg(raw_swing -
                                                            joint->config.swing_neutral_offset_deg));
        out.swing_deg = HandSolve_ClampFloat(out.swing_deg,
                                             joint->config.swing_min_deg,
                                             joint->config.swing_max_deg);
    }

    return out;
}

static void HandSolve_FillMissingOutput(float joint_angle_deg[GLOVE_JOINT_DOF_COUNT])
{
    if (joint_angle_deg == NULL)
    {
        return;
    }

    for (uint32_t i = 0U; i < GLOVE_JOINT_DOF_COUNT; i++)
    {
        joint_angle_deg[i] = HAND_SOLVE_MISSING_VALUE;
    }
}

static uint8_t HandSolve_HasAnyValidImu(const HandSolveLayout_t *layout,
                                        uint32_t imu_valid_mask)
{
    if (layout == NULL)
    {
        return 0U;
    }

    if (HandSolve_IsImuValid(imu_valid_mask, layout->palm_imu_id) != 0U)
    {
        return 1U;
    }

    for (uint32_t finger = 0U; finger < HAND_SOLVE_FINGER_COUNT; finger++)
    {
        for (uint32_t segment = 0U;
             segment < HAND_SOLVE_SEGMENT_COUNT_PER_FINGER;
             segment++)
        {
            if (HandSolve_IsImuValid(imu_valid_mask,
                                     layout->finger_imu_id[finger][segment]) != 0U)
            {
                return 1U;
            }
        }
    }

    return 0U;
}

static uint8_t HandSolve_SolveJointIfPresent(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                             uint32_t imu_valid_mask,
                                             const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                             const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                             uint8_t proximal_imu_id,
                                             uint8_t distal_imu_id,
                                             HandSolveJointConfig_t config,
                                             HandSolveJointResult_t *result)
{
    HandSolveJoint_t joint;

    if ((result == NULL) ||
        (HandSolve_IsImuValid(imu_valid_mask, proximal_imu_id) == 0U) ||
        (HandSolve_IsImuValid(imu_valid_mask, distal_imu_id) == 0U))
    {
        return 0U;
    }

    joint.proximal_imu_id = proximal_imu_id;
    joint.distal_imu_id = distal_imu_id;
    joint.config = config;
    *result = HandSolve_SolveJoint(raw_quat, c_calib, m_calib, &joint);

    return 1U;
}

static uint8_t HandSolve_FingerAbductionRawDeg(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                               uint32_t imu_valid_mask,
                                               const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                               const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                               uint8_t palm_imu_id,
                                               const uint8_t segment_imu_id[HAND_SOLVE_SEGMENT_COUNT_PER_FINGER],
                                               float *raw_deg)
{
    GloveQuaternion_t palm_cal;
    HandSolveVec3_t y_accum = {0.0f, 0.0f, 0.0f};
    uint32_t used_count = 0U;

    if ((raw_quat == NULL) || (segment_imu_id == NULL) || (raw_deg == NULL) ||
        (HandSolve_IsImuValid(imu_valid_mask, palm_imu_id) == 0U))
    {
        return 0U;
    }

    palm_cal = HandSolve_ApplyCalibration(c_calib,
                                          m_calib,
                                          palm_imu_id,
                                          raw_quat[(uint32_t)palm_imu_id - 1U]);

    for (uint32_t segment = 0U;
         segment < HAND_SOLVE_SEGMENT_COUNT_PER_FINGER;
         segment++)
    {
        uint8_t imu_id = segment_imu_id[segment];
        GloveQuaternion_t seg_cal;
        GloveQuaternion_t relative;
        HandSolveMat3_t rotation;
        HandSolveVec3_t y_axis;

        if (HandSolve_IsImuValid(imu_valid_mask, imu_id) == 0U)
        {
            continue;
        }

        seg_cal = HandSolve_ApplyCalibration(c_calib,
                                             m_calib,
                                             imu_id,
                                             raw_quat[(uint32_t)imu_id - 1U]);
        relative = HandSolve_QuatMultiply(HandSolve_QuatConjugate(palm_cal),
                                          seg_cal);
        relative = HandSolve_NormalizeQuat(&relative);
        rotation = HandSolve_QuatToRotationMatrix(relative);
        y_axis = HandSolve_Mat3Column(&rotation, 1U);

        if ((used_count > 0U) && (HandSolve_Vec3Dot(y_axis, y_accum) < 0.0f))
        {
            y_axis = HandSolve_Vec3Negate(y_axis);
        }

        y_accum = HandSolve_Vec3Add(y_accum, y_axis);
        used_count++;
    }

    if (used_count == 0U)
    {
        return 0U;
    }

    y_accum = HandSolve_Vec3Normalize(y_accum);
    if (HandSolve_IsVec3Valid(y_accum) == 0U)
    {
        return 0U;
    }

    *raw_deg = atan2f(-y_accum.x, y_accum.y) * HAND_SOLVE_RAD_TO_DEG_F;
    return 1U;
}

static void HandSolve_SolveLongFingerState(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                           uint32_t imu_valid_mask,
                                           const HandSolveLayout_t *layout,
                                           const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                           const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                           uint32_t finger,
                                           float joint_angle_deg[GLOVE_JOINT_DOF_COUNT])
{
    const uint8_t *segments;
    uint32_t out_base;
    HandSolveJointConfig_t config;
    HandSolveJointResult_t result;
    float raw_abduction_deg;

    if ((raw_quat == NULL) || (layout == NULL) || (joint_angle_deg == NULL) ||
        (finger >= HAND_SOLVE_FINGER_COUNT) || (finger == HAND_SOLVE_THUMB_INDEX))
    {
        return;
    }

    segments = layout->finger_imu_id[finger];
    out_base = (finger - HAND_SOLVE_OUTPUT_FINGER_FIRST) * HAND_SOLVE_FINGER_OUTPUT_COUNT;

    config = HandSolve_MakeDefaultJointConfig(0U, 1U, layout->hand_side);
    if (HandSolve_SolveJointIfPresent(raw_quat,
                                      imu_valid_mask,
                                      c_calib,
                                      m_calib,
                                      layout->palm_imu_id,
                                      segments[0],
                                      config,
                                      &result) != 0U)
    {
        joint_angle_deg[out_base] = result.flex_deg;
        joint_angle_deg[out_base + 1U] = result.swing_deg;
    }

    if (HandSolve_FingerAbductionRawDeg(raw_quat,
                                        imu_valid_mask,
                                        c_calib,
                                        m_calib,
                                        layout->palm_imu_id,
                                        segments,
                                        &raw_abduction_deg) != 0U)
    {
        float swing_deg = HandSolve_WrapDeg(config.swing_sign *
                                            HandSolve_WrapDeg(raw_abduction_deg -
                                                              config.swing_neutral_offset_deg));
        joint_angle_deg[out_base + 1U] = HandSolve_ClampFloat(swing_deg,
                                                              config.swing_min_deg,
                                                              config.swing_max_deg);
    }

    config = HandSolve_MakeDefaultJointConfig(0U, 0U, layout->hand_side);
    if (HandSolve_SolveJointIfPresent(raw_quat,
                                      imu_valid_mask,
                                      c_calib,
                                      m_calib,
                                      segments[0],
                                      segments[1],
                                      config,
                                      &result) != 0U)
    {
        joint_angle_deg[out_base + 2U] = result.flex_deg;
    }

    if (HandSolve_SolveJointIfPresent(raw_quat,
                                      imu_valid_mask,
                                      c_calib,
                                      m_calib,
                                      segments[1],
                                      segments[2],
                                      config,
                                      &result) != 0U)
    {
        joint_angle_deg[out_base + 3U] = result.flex_deg;
    }
}

static void HandSolve_SolveThumbState(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                      uint32_t imu_valid_mask,
                                      const HandSolveLayout_t *layout,
                                      const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                      const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                      float joint_angle_deg[GLOVE_JOINT_DOF_COUNT])
{
    const uint8_t *segments;
    const uint8_t palm_imu_id = (layout != NULL) ? layout->palm_imu_id : HAND_SOLVE_INVALID_IMU_ID;
    uint8_t meta_imu_id;
    uint8_t prox_imu_id;
    uint8_t dist_imu_id;
    uint8_t has_palm;
    uint8_t has_meta;
    uint8_t has_prox;
    uint8_t has_dist;
    GloveQuaternion_t palm_cal = s_identity_quat;
    GloveQuaternion_t meta_cal = s_identity_quat;
    GloveQuaternion_t prox_cal = s_identity_quat;
    GloveQuaternion_t dist_cal = s_identity_quat;

    if ((raw_quat == NULL) || (layout == NULL) || (joint_angle_deg == NULL))
    {
        return;
    }

    segments = layout->finger_imu_id[HAND_SOLVE_THUMB_INDEX];
    meta_imu_id = segments[0];
    prox_imu_id = segments[1];
    dist_imu_id = segments[2];

    has_palm = HandSolve_IsImuValid(imu_valid_mask, palm_imu_id);
    has_meta = HandSolve_IsImuValid(imu_valid_mask, meta_imu_id);
    has_prox = HandSolve_IsImuValid(imu_valid_mask, prox_imu_id);
    has_dist = HandSolve_IsImuValid(imu_valid_mask, dist_imu_id);

    if (has_palm != 0U)
    {
        palm_cal = HandSolve_ApplyCalibration(c_calib,
                                              m_calib,
                                              palm_imu_id,
                                              raw_quat[(uint32_t)palm_imu_id - 1U]);
        joint_angle_deg[23] = palm_cal.w;
        joint_angle_deg[24] = palm_cal.x;
        joint_angle_deg[25] = palm_cal.y;
        joint_angle_deg[26] = palm_cal.z;
    }

    if (has_meta != 0U)
    {
        meta_cal = HandSolve_ApplyCalibration(c_calib,
                                              m_calib,
                                              meta_imu_id,
                                              raw_quat[(uint32_t)meta_imu_id - 1U]);
    }
    if (has_prox != 0U)
    {
        prox_cal = HandSolve_ApplyCalibration(c_calib,
                                              m_calib,
                                              prox_imu_id,
                                              raw_quat[(uint32_t)prox_imu_id - 1U]);
    }
    if (has_dist != 0U)
    {
        dist_cal = HandSolve_ApplyCalibration(c_calib,
                                              m_calib,
                                              dist_imu_id,
                                              raw_quat[(uint32_t)dist_imu_id - 1U]);
    }

    if ((has_meta != 0U) && (has_prox != 0U))
    {
        GloveQuaternion_t relative;
        HandSolveMat3_t rotation;
        HandSolveVec3_t direction;
        float clamped_z;

        relative = HandSolve_QuatMultiply(HandSolve_QuatConjugate(meta_cal),
                                          prox_cal);
        relative = HandSolve_NormalizeQuat(&relative);
        rotation = HandSolve_QuatToRotationMatrix(relative);
        direction = HandSolve_Mat3Column(&rotation, 0U);
        clamped_z = HandSolve_ClampFloat(direction.z, -1.0f, 1.0f);

        joint_angle_deg[16] = asinf(clamped_z) * HAND_SOLVE_RAD_TO_DEG_F;
        joint_angle_deg[17] = -atan2f(direction.y, direction.x) *
                              HAND_SOLVE_RAD_TO_DEG_F;
        if (layout->hand_side == GLOVE_HAND_LEFT)
        {
            joint_angle_deg[17] = -joint_angle_deg[17];
        }
    }

    if ((has_prox != 0U) && (has_dist != 0U))
    {
        GloveQuaternion_t relative;
        HandSolveMat3_t rotation;
        HandSolveVec3_t direction;

        relative = HandSolve_QuatMultiply(HandSolve_QuatConjugate(prox_cal),
                                          dist_cal);
        relative = HandSolve_NormalizeQuat(&relative);
        rotation = HandSolve_QuatToRotationMatrix(relative);
        direction = HandSolve_Mat3Column(&rotation, 0U);

        joint_angle_deg[18] = atan2f(direction.z, direction.x) *
                              HAND_SOLVE_RAD_TO_DEG_F;
    }

    if ((has_meta != 0U) && (has_palm != 0U))
    {
        GloveQuaternion_t cmc;

        cmc = HandSolve_QuatMultiply(HandSolve_QuatConjugate(palm_cal),
                                     meta_cal);
        cmc = HandSolve_NormalizeQuat(&cmc);
        joint_angle_deg[19] = cmc.w;
        joint_angle_deg[20] = cmc.x;
        joint_angle_deg[21] = cmc.y;
        joint_angle_deg[22] = cmc.z;
    }
}

void HandSolve_FillIdentityCalibration(GloveQuaternion_t calibration[GLOVE_IMU_COUNT])
{
    if (calibration == NULL)
    {
        return;
    }

    for (uint32_t i = 0U; i < GLOVE_IMU_COUNT; i++)
    {
        calibration[i] = s_identity_quat;
    }
}

GloveStatus_t HandSolve_InitDefaultLayout(HandSolveLayout_t *layout,
                                          GloveHandSide_t hand_side)
{
    uint8_t imu_id;

    if (layout == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if ((hand_side != GLOVE_HAND_LEFT) && (hand_side != GLOVE_HAND_RIGHT))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    (void)memset(layout, 0, sizeof(*layout));
    layout->palm_imu_id = 1U;
    layout->hand_side = hand_side;

    imu_id = 2U;
    for (uint32_t finger = 0U; finger < HAND_SOLVE_FINGER_COUNT; finger++)
    {
        for (uint32_t segment = 0U;
             segment < HAND_SOLVE_SEGMENT_COUNT_PER_FINGER;
             segment++)
        {
            layout->finger_imu_id[finger][segment] = imu_id;
            imu_id++;
        }
    }

    return GLOVE_STATUS_OK;
}

GloveStatus_t HandSolve_SolveAnglesDeg(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                       uint32_t imu_valid_mask,
                                       const HandSolveLayout_t *layout,
                                       const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                       const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                       float joint_angle_deg[GLOVE_JOINT_DOF_COUNT])
{
    if ((raw_quat == NULL) || (layout == NULL) || (joint_angle_deg == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if ((layout->hand_side != GLOVE_HAND_LEFT) &&
        (layout->hand_side != GLOVE_HAND_RIGHT))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    HandSolve_FillMissingOutput(joint_angle_deg);

    if (HandSolve_HasAnyValidImu(layout, imu_valid_mask) == 0U)
    {
        return GLOVE_STATUS_NOT_READY;
    }

    for (uint32_t finger = HAND_SOLVE_OUTPUT_FINGER_FIRST;
         finger < HAND_SOLVE_FINGER_COUNT;
         finger++)
    {
        HandSolve_SolveLongFingerState(raw_quat,
                                       imu_valid_mask,
                                       layout,
                                       c_calib,
                                       m_calib,
                                       finger,
                                       joint_angle_deg);
    }

    HandSolve_SolveThumbState(raw_quat,
                              imu_valid_mask,
                              layout,
                              c_calib,
                              m_calib,
                              joint_angle_deg);

    return GLOVE_STATUS_OK;
}
