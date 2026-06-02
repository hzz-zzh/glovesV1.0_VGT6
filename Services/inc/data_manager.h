#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"
#include "frame_pool.h"

/* 数据消费者类型 用于选择对应的订阅队列 */
typedef enum
{
    DATA_CONSUMER_ALGORITHM = 0,
    DATA_CONSUMER_STORAGE = 1,
    DATA_CONSUMER_RS485 = 2
} DataConsumer_t;

/*
 * 由 IMU_CAN_Task 生产 由 FrameAssemblerTask 消费
 */
typedef struct
{
    GloveImuSensorData_t data;
    volatile uint8_t ref_count;
    uint8_t reserved[3];
} GloveImuSensorBlock_t;

/*
 * 由 Touch_ADC_Task 生产 由 FrameAssemblerTask 消费
 */
typedef struct
{
    GloveTouchSensorData_t data;
    volatile uint8_t ref_count;
    uint8_t reserved[3];
} GloveTouchSensorBlock_t;

/*
 * 由 FrameAssemblerTask 生产 由 AlgorithmTask 消费
 */
typedef struct
{
    GloveRawFrame_t frame;
    volatile uint8_t ref_count;
    uint8_t reserved[3];
} GloveRawFrameBlock_t;

/*
 * 由 AlgorithmTask 生产 由 StorageTask 和 RS485Task 消费
 */
typedef struct
{
    GloveFullFrame_t frame;
    volatile uint8_t ref_count;
    uint8_t reserved[3];
} GloveFullFrameBlock_t;

/* DataManager 统计信 */
typedef struct
{
    GloveDataStats_t data;
    FramePoolStats_t imu_sensor_pool;
    FramePoolStats_t touch_sensor_pool;
    FramePoolStats_t raw_pool;
    FramePoolStats_t full_pool;
} DataManagerStats_t;

GloveStatus_t DataManager_Init(void);

/*
 * 总体数据流
 *
 *   IMU_CAN_Task   -> ImuSensorData   -> FrameAssemblerTask
 *   Touch_ADC_Task -> TouchSensorData -> FrameAssemblerTask
 *   FrameAssembler -> RawFrame        -> AlgorithmTask
 *   AlgorithmTask  -> FullFrame       -> StorageTask / RS485Task
 *
 * Alloc 返回生产者拥有的数据块
 * Publish 返回后生产者不再拥有该数据块
 * Get 成功后消费者必须在使用完后调用对应 Release
 */

GloveImuSensorBlock_t *DataManager_AllocImuSensor(void);
GloveTouchSensorBlock_t *DataManager_AllocTouchSensor(void);
GloveRawFrameBlock_t *DataManager_AllocRawFrame(void);
GloveFullFrameBlock_t *DataManager_AllocFullFrame(void);

GloveStatus_t DataManager_PublishImuSensor(GloveImuSensorBlock_t *block, uint32_t timeout_ms);
GloveStatus_t DataManager_PublishTouchSensor(GloveTouchSensorBlock_t *block, uint32_t timeout_ms);
GloveStatus_t DataManager_PublishRawFrame(GloveRawFrameBlock_t *block, uint32_t timeout_ms);
GloveStatus_t DataManager_PublishFullFrame(GloveFullFrameBlock_t *block, uint32_t timeout_ms);

GloveStatus_t DataManager_GetImuSensor(GloveImuSensorBlock_t **block, uint32_t timeout_ms);
GloveStatus_t DataManager_GetTouchSensor(GloveTouchSensorBlock_t **block, uint32_t timeout_ms);
GloveStatus_t DataManager_GetRawFrame(DataConsumer_t consumer,
                                      GloveRawFrameBlock_t **block,
                                      uint32_t timeout_ms);
GloveStatus_t DataManager_GetFullFrame(DataConsumer_t consumer,
                                       GloveFullFrameBlock_t **block,
                                       uint32_t timeout_ms);

GloveStatus_t DataManager_ReleaseImuSensor(GloveImuSensorBlock_t *block);
GloveStatus_t DataManager_ReleaseTouchSensor(GloveTouchSensorBlock_t *block);
GloveStatus_t DataManager_ReleaseRawFrame(GloveRawFrameBlock_t *block);
GloveStatus_t DataManager_ReleaseFullFrame(GloveFullFrameBlock_t *block);

void DataManager_GetStats(DataManagerStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* DATA_MANAGER_H */
