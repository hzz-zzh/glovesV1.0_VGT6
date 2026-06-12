#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 手套应用层配置文件
 * 和具体引脚、外设实例相关的配置仍然放在 BSP 层
 * 这里主要放系统规模、队列深度、内存池大小、数据流策略等业务参数
 */

#define GLOVE_IMU_COUNT                         (16U)
#define GLOVE_TOUCH_COUNT                       (81U)
#define GLOVE_JOINT_DOF_COUNT                   (21U)

#define GLOVE_IMU_SENSOR_POOL_SIZE              (6U)
#define GLOVE_TOUCH_SENSOR_POOL_SIZE            (6U)
#define GLOVE_IMU_SENSOR_QUEUE_DEPTH            (4U)
#define GLOVE_TOUCH_SENSOR_QUEUE_DEPTH          (4U)

#define GLOVE_RAW_FRAME_POOL_SIZE               (8U)
#define GLOVE_RAW_FRAME_QUEUE_DEPTH             (4U)

#define GLOVE_FULL_FRAME_POOL_SIZE              (8U)
#define GLOVE_FULL_FRAME_QUEUE_DEPTH            (4U)

#define GLOVE_RAW_FRAME_CONSUMER_COUNT          (1U)
#define GLOVE_FULL_FRAME_CONSUMER_COUNT         (2U)

/* 数据有效标志，用于描述当前数据块包含哪些有效内容。 */
#define GLOVE_FRAME_FLAG_NONE                   (0x00000000UL)
#define GLOVE_FRAME_FLAG_IMU_VALID              (0x00000001UL)
#define GLOVE_FRAME_FLAG_QUAT_VALID             (0x00000002UL)
#define GLOVE_FRAME_FLAG_TOUCH_VALID            (0x00000004UL)
#define GLOVE_FRAME_FLAG_ALGORITHM_VALID        (0x00000008UL)

#define GLOVE_FRAME_VALID_IMU_BIT_SHIFT         (16U)
#define GLOVE_FRAME_VALID_IMU_BIT(index)        (1UL << (GLOVE_FRAME_VALID_IMU_BIT_SHIFT + (index)))
#define GLOVE_FRAME_VALID_IMU_ALL_MASK          (((1UL << GLOVE_IMU_COUNT) - 1UL) << \
                                                 GLOVE_FRAME_VALID_IMU_BIT_SHIFT)
#define GLOVE_IMU_VALID_ALL_MASK                ((1UL << GLOVE_IMU_COUNT) - 1UL)

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
