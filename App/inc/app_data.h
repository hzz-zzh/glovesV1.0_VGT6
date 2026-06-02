#ifndef APP_DATA_H
#define APP_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_config.h"

/* 系统通用返回状态 */
typedef enum
{
    GLOVE_STATUS_OK = 0,
    GLOVE_STATUS_ERROR = 1,
    GLOVE_STATUS_TIMEOUT = 2,
    GLOVE_STATUS_NO_MEMORY = 3,
    GLOVE_STATUS_INVALID_PARAM = 4,
    GLOVE_STATUS_QUEUE_FULL = 5,
    GLOVE_STATUS_QUEUE_EMPTY = 6,
    GLOVE_STATUS_NOT_READY = 7
} GloveStatus_t;

/* 手套左右手标识，后续做标定、协议上报时用 */
typedef enum
{
    GLOVE_HAND_UNKNOWN = 0,
    GLOVE_HAND_LEFT = 1,
    GLOVE_HAND_RIGHT = 2
} GloveHandSide_t;

/* 三维向量，统一用于加速度、角速度等三轴物理量 */
typedef struct
{
    float x;
    float y;
    float z;
} GloveVector3f_t;

/* 四元数，表示 IMU 姿态 */
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} GloveQuaternion_t;

/* 单个 IMU 的原始采样值 */
typedef struct
{
    GloveVector3f_t accel_mps2;       /* 加速度，单位 m/s^2 */
    GloveVector3f_t gyro_radps;       /* 角速度，单位 rad/s */
    int16_t temperature_cdeg;         /* 温度，单位 0.01 摄氏度 */
    uint16_t status;                  /* 传感器状态位 */
} GloveImuSample_t;

/* 单个触觉阵列点的采样值 */
typedef struct
{
    uint16_t value;                   /* 当前触觉采样值 */
    uint16_t baseline;                /* 基线值，用于压力/触觉变化量计算 */
} GloveTouchSample_t;

/*由 IMU_CAN_Task 产生，包含一次 16 路 IMU 采集结果 */
typedef struct
{
    uint32_t sensor_seq;              /* IMU 采集序号 */
    uint32_t timestamp_us;            /* IMU 数据时间戳，单位 us */
    uint32_t valid_flags;            

    GloveImuSample_t imu[GLOVE_IMU_COUNT];
    GloveQuaternion_t quat[GLOVE_IMU_COUNT];
} GloveImuSensorData_t;

/*由 Touch_ADC_Task 产生，包含一次 81 点触觉阵列采集结果 */
typedef struct
{
    uint32_t sensor_seq;              /* 触觉采集序号 */
    uint32_t timestamp_us;            /* 触觉数据时间戳，单位 us */
    uint32_t valid_flags;             /* 有效标志 */

    GloveTouchSample_t touch[GLOVE_TOUCH_COUNT];
} GloveTouchSensorData_t;

/* 由 FrameAssemblerTask 根据 ImuSensorData + TouchSensorData 组装生成 */
typedef struct
{
    uint32_t frame_id;                /* 单调递增帧号，用于日志和通讯对齐 */
    uint32_t timestamp_us;            /* 合帧时间戳，单位 us */
    uint32_t valid_flags;             /* 本帧有效数据标志 */

    GloveImuSample_t imu[GLOVE_IMU_COUNT];
    GloveQuaternion_t quat[GLOVE_IMU_COUNT];
    GloveTouchSample_t touch[GLOVE_TOUCH_COUNT];
} GloveRawFrame_t;

/*
 * 算法结果帧
 *
 * 由算法任务根据 RawFrame 计算得到，包含姿态和关节运动学结果
 */
typedef struct
{
    uint32_t frame_id;                /* 与输入 RawFrame 保持一致 */
    uint32_t timestamp_us;            /* 与输入 RawFrame 保持一致 */
    uint32_t valid_flags;             /* 算法结果有效标志 */

    GloveQuaternion_t imu_attitude[GLOVE_IMU_COUNT];
    float joint_angle_rad[GLOVE_JOINT_DOF_COUNT];
    float joint_velocity_radps[GLOVE_JOINT_DOF_COUNT];
} GloveProcessedFrame_t;

/* SD 卡和 RS485 都只消费 FullFrame，确保原始数据和对应算法结果绑定在一起 */
typedef struct
{
    uint32_t frame_id;
    uint32_t timestamp_us;
    uint32_t valid_flags;

    GloveRawFrame_t raw;
    GloveProcessedFrame_t processed;
} GloveFullFrame_t;

/* 数据管理模块运行统计 */
typedef struct
{
    uint32_t imu_sensor_published;
    uint32_t touch_sensor_published;
    uint32_t raw_frames_published;
    uint32_t full_frames_published;

    uint32_t imu_sensor_dropped;
    uint32_t touch_sensor_dropped;
    uint32_t raw_frames_dropped;
    uint32_t full_frames_dropped;

    uint32_t pool_alloc_failures;
    uint32_t queue_send_failures;
} GloveDataStats_t;

void AppData_ClearImuSensorData(GloveImuSensorData_t *data);
void AppData_ClearTouchSensorData(GloveTouchSensorData_t *data);
void AppData_ClearRawFrame(GloveRawFrame_t *frame);
void AppData_ClearProcessedFrame(GloveProcessedFrame_t *frame);
void AppData_ClearFullFrame(GloveFullFrame_t *frame);

void AppData_BuildRawFrameFromSensors(GloveRawFrame_t *raw,
                                      uint32_t frame_id,
                                      uint32_t timestamp_us,
                                      const GloveImuSensorData_t *imu,
                                      const GloveTouchSensorData_t *touch);

void AppData_BuildFullFrame(GloveFullFrame_t *full,
                            const GloveRawFrame_t *raw,
                            const GloveProcessedFrame_t *processed);

#ifdef __cplusplus
}
#endif

#endif /* APP_DATA_H */
