#include "hand_solve.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if GLOVE_IMU_COUNT != 16U
#error "hand_solve expects GLOVE_IMU_COUNT to be 16"
#endif

#if GLOVE_JOINT_DOF_COUNT != 21U
#error "hand_solve expects GLOVE_JOINT_DOF_COUNT to be 21"
#endif

#define HAND_SOLVE_EPSILON                      (1.0e-12f)
#define HAND_SOLVE_PI_F                         (3.14159265358979323846f)
#define HAND_SOLVE_RAD_TO_DEG_F                 (180.0f / HAND_SOLVE_PI_F)
#define HAND_SOLVE_THUMB_INDEX                  (0U)

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
    config.swing_min_deg = -30.0f;
    config.swing_max_deg = 30.0f;

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

static uint8_t HandSolve_IsLayoutComplete(const HandSolveLayout_t *layout,
                                          uint32_t imu_valid_mask)
{
    if (layout == NULL)
    {
        return 0U;
    }

    if (HandSolve_IsImuValid(imu_valid_mask, layout->palm_imu_id) == 0U)
    {
        return 0U;
    }

    for (uint32_t finger = 0U; finger < HAND_SOLVE_FINGER_COUNT; finger++)
    {
        for (uint32_t segment = 0U;
             segment < HAND_SOLVE_SEGMENT_COUNT_PER_FINGER;
             segment++)
        {
            if (HandSolve_IsImuValid(imu_valid_mask,
                                     layout->finger_imu_id[finger][segment]) == 0U)
            {
                return 0U;
            }
        }
    }

    return 1U;
}

static GloveStatus_t HandSolve_AppendJointAngles(const GloveQuaternion_t raw_quat[GLOVE_IMU_COUNT],
                                                 const GloveQuaternion_t c_calib[GLOVE_IMU_COUNT],
                                                 const GloveQuaternion_t m_calib[GLOVE_IMU_COUNT],
                                                 const HandSolveJoint_t *joint,
                                                 float joint_angle_deg[GLOVE_JOINT_DOF_COUNT],
                                                 uint32_t *out_index)
{
    HandSolveJointResult_t result;

    if ((joint == NULL) || (joint_angle_deg == NULL) || (out_index == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (*out_index >= GLOVE_JOINT_DOF_COUNT)
    {
        return GLOVE_STATUS_ERROR;
    }

    result = HandSolve_SolveJoint(raw_quat, c_calib, m_calib, joint);
    joint_angle_deg[*out_index] = result.flex_deg;
    (*out_index)++;

    if (joint->config.has_swing != 0U)
    {
        if (*out_index >= GLOVE_JOINT_DOF_COUNT)
        {
            return GLOVE_STATUS_ERROR;
        }
        joint_angle_deg[*out_index] = result.swing_deg;
        (*out_index)++;
    }

    return GLOVE_STATUS_OK;
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
    uint32_t out_index = 0U;
    GloveStatus_t status;

    if ((raw_quat == NULL) || (layout == NULL) || (joint_angle_deg == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if ((layout->hand_side != GLOVE_HAND_LEFT) &&
        (layout->hand_side != GLOVE_HAND_RIGHT))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    (void)memset(joint_angle_deg, 0, sizeof(float) * GLOVE_JOINT_DOF_COUNT);

    if (HandSolve_IsLayoutComplete(layout, imu_valid_mask) == 0U)
    {
        return GLOVE_STATUS_NOT_READY;
    }

    for (uint32_t finger = 0U; finger < HAND_SOLVE_FINGER_COUNT; finger++)
    {
        const uint8_t is_thumb = (finger == HAND_SOLVE_THUMB_INDEX) ? 1U : 0U;
        HandSolveJoint_t joint;

        joint.proximal_imu_id = layout->palm_imu_id;
        joint.distal_imu_id = layout->finger_imu_id[finger][0];
        joint.config = HandSolve_MakeDefaultJointConfig(is_thumb,
                                                        1U,
                                                        layout->hand_side);
        status = HandSolve_AppendJointAngles(raw_quat,
                                             c_calib,
                                             m_calib,
                                             &joint,
                                             joint_angle_deg,
                                             &out_index);
        if (status != GLOVE_STATUS_OK)
        {
            return status;
        }

        for (uint32_t segment = 1U;
             segment < HAND_SOLVE_SEGMENT_COUNT_PER_FINGER;
             segment++)
        {
            const uint8_t has_swing =
                ((is_thumb != 0U) && (segment == 1U)) ? 1U : 0U;

            joint.proximal_imu_id = layout->finger_imu_id[finger][segment - 1U];
            joint.distal_imu_id = layout->finger_imu_id[finger][segment];
            joint.config = HandSolve_MakeDefaultJointConfig(is_thumb,
                                                            has_swing,
                                                            layout->hand_side);
            status = HandSolve_AppendJointAngles(raw_quat,
                                                 c_calib,
                                                 m_calib,
                                                 &joint,
                                                 joint_angle_deg,
                                                 &out_index);
            if (status != GLOVE_STATUS_OK)
            {
                return status;
            }
        }
    }

    return (out_index == GLOVE_JOINT_DOF_COUNT) ? GLOVE_STATUS_OK : GLOVE_STATUS_ERROR;
}
