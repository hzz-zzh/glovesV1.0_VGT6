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

/* 量产构建关闭调试串口、诊断任务和触觉数据流输出。 */
#define APP_BUILD_PRODUCTION                    (1U)
#define APP_ENABLE_DEBUG_UART_OUTPUT            (0U)
#define APP_ENABLE_UART_DEBUG_TASK              (0U)
#define APP_ENABLE_GENERAL_DEBUG_OUTPUT          (0U)
#define APP_ENABLE_ACQUISITION_DEBUG             (0U)
#define APP_ENABLE_TOUCH_1_30_STREAM             (0U)
#define APP_ENABLE_TEST_TASK                    (0U)
#define APP_ENABLE_TEST_DATA_INJECTION          (0U)
#define APP_ENABLE_TOUCH_ADC_TEST_ONLY          (0U)
#define APP_ENABLE_SD_LOG_TEST_ONLY             (0U)
#define APP_ENABLE_STORAGE_SIM_DATA             (0U)

#if (APP_BUILD_PRODUCTION != 0U) && \
    ((APP_ENABLE_DEBUG_UART_OUTPUT != 0U) || \
     (APP_ENABLE_UART_DEBUG_TASK != 0U) || \
     (APP_ENABLE_GENERAL_DEBUG_OUTPUT != 0U) || \
     (APP_ENABLE_ACQUISITION_DEBUG != 0U) || \
     (APP_ENABLE_TOUCH_1_30_STREAM != 0U) || \
     (APP_ENABLE_TEST_TASK != 0U) || \
     (APP_ENABLE_TEST_DATA_INJECTION != 0U) || \
     (APP_ENABLE_TOUCH_ADC_TEST_ONLY != 0U) || \
     (APP_ENABLE_SD_LOG_TEST_ONLY != 0U) || \
     (APP_ENABLE_STORAGE_SIM_DATA != 0U))
#error "Production build must not enable debug or test features"
#endif

#if (APP_ENABLE_UART_DEBUG_TASK != 0U) && (APP_ENABLE_DEBUG_UART_OUTPUT == 0U)
#error "UART debug task requires debug UART output"
#endif

#if (APP_ENABLE_GENERAL_DEBUG_OUTPUT != 0U) && (APP_ENABLE_DEBUG_UART_OUTPUT == 0U)
#error "General debug output requires debug UART output"
#endif

#if (APP_ENABLE_ACQUISITION_DEBUG != 0U) && \
    ((APP_ENABLE_DEBUG_UART_OUTPUT == 0U) || (APP_ENABLE_UART_DEBUG_TASK == 0U))
#error "Acquisition debug requires UART debug output and task"
#endif

#if (APP_ENABLE_TOUCH_1_30_STREAM != 0U) && \
    ((APP_ENABLE_DEBUG_UART_OUTPUT == 0U) || (APP_ENABLE_UART_DEBUG_TASK == 0U))
#error "Touch point stream requires UART debug output and task"
#endif

#if (APP_ENABLE_TEST_DATA_INJECTION != 0U) && (APP_ENABLE_TEST_TASK == 0U)
#error "Test data injection requires the test task"
#endif

#if (APP_ENABLE_STORAGE_SIM_DATA != 0U) && (APP_ENABLE_SD_LOG_TEST_ONLY == 0U)
#error "Storage simulated data is only allowed in SD log test mode"
#endif

#if (APP_ENABLE_TOUCH_ADC_TEST_ONLY != 0U) && (APP_ENABLE_SD_LOG_TEST_ONLY != 0U)
#error "Only one exclusive hardware test mode may be enabled"
#endif

#define GLOVE_IMU_COUNT                         (16U)
#define GLOVE_TOUCH_COUNT                       (68U)
#define GLOVE_JOINT_DOF_COUNT                   (27U)

/* PPS标称周期为1秒，每个采样窗口包含125点，标称采样间隔为8ms。 */
#define GLOVE_SENSOR_SAMPLE_RATE_HZ             (125U)
#define GLOVE_SENSOR_SAMPLE_PERIOD_US           (1000000ULL / GLOVE_SENSOR_SAMPLE_RATE_HZ)

/* 采样窗口计数使用uint16_t，频率必须非零且不能超过计数容量。 */
#if (GLOVE_SENSOR_SAMPLE_RATE_HZ == 0U) || (GLOVE_SENSOR_SAMPLE_RATE_HZ > 65535U)
#error "GLOVE_SENSOR_SAMPLE_RATE_HZ must be within 1..65535"
#endif

#define GLOVE_IMU_SENSOR_POOL_SIZE              (6U)
#define GLOVE_TOUCH_SENSOR_POOL_SIZE            (6U)
#define GLOVE_IMU_SENSOR_QUEUE_DEPTH            (4U)
#define GLOVE_TOUCH_SENSOR_QUEUE_DEPTH          (4U)

#define GLOVE_RAW_FRAME_POOL_SIZE               (8U)
#define GLOVE_RAW_FRAME_QUEUE_DEPTH             (4U)

#define GLOVE_FULL_FRAME_POOL_SIZE              (136U)
#define GLOVE_FULL_FRAME_STORAGE_QUEUE_DEPTH    (128U)
#define GLOVE_FULL_FRAME_RS485_QUEUE_DEPTH      (4U)

/* Storage、RS485和正在发布的帧必须都有独立的数据块余量。 */
#if GLOVE_FULL_FRAME_POOL_SIZE < (GLOVE_FULL_FRAME_STORAGE_QUEUE_DEPTH + \
                                  GLOVE_FULL_FRAME_RS485_QUEUE_DEPTH + 4U)
#error "GLOVE_FULL_FRAME_POOL_SIZE is too small for the configured consumers"
#endif

#define GLOVE_RAW_FRAME_CONSUMER_COUNT          (1U)
#define GLOVE_FULL_FRAME_CONSUMER_COUNT         (2U)

/* 数据有效标志，用于描述当前数据块包含哪些有效内容。 */
#define GLOVE_FRAME_FLAG_NONE                   (0x00000000UL)
#define GLOVE_FRAME_FLAG_IMU_VALID              (0x00000001UL)
#define GLOVE_FRAME_FLAG_QUAT_VALID             (0x00000002UL)
#define GLOVE_FRAME_FLAG_TOUCH_VALID            (0x00000004UL)
#define GLOVE_FRAME_FLAG_ALGORITHM_VALID        (0x00000008UL)
#define GLOVE_FRAME_FLAG_IMU_CALIB_APPLIED      (0x00000010UL)
/* 采集时UTC可信，不受后续写UTC或消费者处理延迟影响。 */
#define GLOVE_FRAME_FLAG_UTC_VALID              (0x00000020UL)

#define GLOVE_FRAME_VALID_IMU_BIT_SHIFT         (16U)
#define GLOVE_FRAME_VALID_IMU_BIT(index)        (1UL << (GLOVE_FRAME_VALID_IMU_BIT_SHIFT + (index)))
#define GLOVE_FRAME_VALID_IMU_ALL_MASK          (((1UL << GLOVE_IMU_COUNT) - 1UL) << \
                                                 GLOVE_FRAME_VALID_IMU_BIT_SHIFT)
#define GLOVE_IMU_VALID_ALL_MASK                ((1UL << GLOVE_IMU_COUNT) - 1UL)

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
