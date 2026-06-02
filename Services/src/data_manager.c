#include "data_manager.h"

#include <stddef.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"


typedef struct
{
    osMessageQueueId_t imu_sensor_for_assembler;
    osMessageQueueId_t touch_sensor_for_assembler;
    osMessageQueueId_t raw_for_algorithm;
    osMessageQueueId_t full_for_storage;
    osMessageQueueId_t full_for_rs485;
} DataManagerQueues_t;

/* 四类数据各自使用独立内存池 互不影响生命周期 */
static FramePool_t s_imu_sensor_pool;
static FramePool_t s_touch_sensor_pool;
static FramePool_t s_raw_pool;
static FramePool_t s_full_pool;

static DataManagerQueues_t s_queues;
static GloveDataStats_t s_stats;
static uint8_t s_initialized;

/* 静态数据块存储区 运行期不使用 malloc */
static GloveImuSensorBlock_t s_imu_sensor_blocks[GLOVE_IMU_SENSOR_POOL_SIZE];
static GloveTouchSensorBlock_t s_touch_sensor_blocks[GLOVE_TOUCH_SENSOR_POOL_SIZE];
static GloveRawFrameBlock_t s_raw_blocks[GLOVE_RAW_FRAME_POOL_SIZE];
static GloveFullFrameBlock_t s_full_blocks[GLOVE_FULL_FRAME_POOL_SIZE];

/* 各内存池对应的空闲索引栈 */
static uint16_t s_imu_sensor_free_stack[GLOVE_IMU_SENSOR_POOL_SIZE];
static uint16_t s_touch_sensor_free_stack[GLOVE_TOUCH_SENSOR_POOL_SIZE];
static uint16_t s_raw_free_stack[GLOVE_RAW_FRAME_POOL_SIZE];
static uint16_t s_full_free_stack[GLOVE_FULL_FRAME_POOL_SIZE];

/* 创建只传指针的消息队列 避免拷贝大结构体 */
static osMessageQueueId_t CreatePointerQueue(uint32_t depth, const char *name)
{
    const osMessageQueueAttr_t attr = {
        .name = name,
        .attr_bits = 0U,
        .cb_mem = NULL,
        .cb_size = 0U,
        .mq_mem = NULL,
        .mq_size = 0U
    };

    return osMessageQueueNew(depth, sizeof(void *), &attr);
}

static uint32_t TimeoutMsToTicks(uint32_t timeout_ms)
{
    uint64_t ticks;

    if (timeout_ms == osWaitForever)
    {
        return osWaitForever;
    }

    ticks = ((uint64_t)timeout_ms * (uint64_t)osKernelGetTickFreq() + 999ULL) / 1000ULL;
    if ((timeout_ms > 0U) && (ticks == 0ULL))
    {
        ticks = 1ULL;
    }

    return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

static void StatsIncrement(uint32_t *value)
{
    taskENTER_CRITICAL();
    (*value)++;
    taskEXIT_CRITICAL();
}

static GloveStatus_t SendPointer(osMessageQueueId_t queue, void *ptr, uint32_t timeout_ms)
{
    if ((queue == NULL) || (ptr == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (osMessageQueuePut(queue, &ptr, 0U, TimeoutMsToTicks(timeout_ms)) != osOK)
    {
        StatsIncrement(&s_stats.queue_send_failures);
        return GLOVE_STATUS_QUEUE_FULL;
    }

    return GLOVE_STATUS_OK;
}

static GloveStatus_t ReceivePointer(osMessageQueueId_t queue, void **ptr, uint32_t timeout_ms)
{
    void *local_ptr = NULL;
    osStatus_t status;

    if ((queue == NULL) || (ptr == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = osMessageQueueGet(queue, &local_ptr, NULL, TimeoutMsToTicks(timeout_ms));
    if (status == osOK)
    {
        *ptr = local_ptr;
        return GLOVE_STATUS_OK;
    }

    return (status == osErrorTimeout) ? GLOVE_STATUS_TIMEOUT : GLOVE_STATUS_QUEUE_EMPTY;
}

static void AddRef(volatile uint8_t *ref_count)
{
    taskENTER_CRITICAL();
    (*ref_count)++;
    taskEXIT_CRITICAL();
}

/*
 * 释放一个数据块引用
 * ref_count 变为 0 时才归还到对应内存池
 */
static GloveStatus_t ReleaseRef(FramePool_t *pool, void *block, volatile uint8_t *ref_count)
{
    uint8_t should_free = 0U;

    if ((pool == NULL) || (block == NULL) || (ref_count == NULL) ||
        (FramePool_Owns(pool, block) == 0U))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    taskENTER_CRITICAL();
    if (*ref_count > 0U)
    {
        (*ref_count)--;
        should_free = (*ref_count == 0U) ? 1U : 0U;
    }
    taskEXIT_CRITICAL();

    return (should_free != 0U) ? FramePool_Free(pool, block) : GLOVE_STATUS_OK;
}

/*
 * 发布给单个消费者
 * Sensor 数据和 RawFrame 都只有一个下游消费者
 */
static GloveStatus_t PublishSingleConsumer(void *block,
                                           volatile uint8_t *ref_count,
                                           osMessageQueueId_t queue,
                                           FramePool_t *pool,
                                           uint32_t *published_counter,
                                           uint32_t *dropped_counter,
                                           uint32_t timeout_ms)
{
    GloveStatus_t status;

    if ((s_initialized == 0U) || (block == NULL) || (ref_count == NULL) || (pool == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    /*
     * Alloc 已经让发布者持有一个引用
     * 队列发送成功后消费者再持有一个引用
     */
    AddRef(ref_count);

    status = SendPointer(queue, block, timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        (void)ReleaseRef(pool, block, ref_count);
        (void)ReleaseRef(pool, block, ref_count);
        StatsIncrement(dropped_counter);
        return status;
    }

    /* 发布结束后释放发布者临时引用 */
    (void)ReleaseRef(pool, block, ref_count);
    StatsIncrement(published_counter);
    return GLOVE_STATUS_OK;
}

GloveStatus_t DataManager_Init(void)
{
    GloveStatus_t status;

    (void)memset(&s_queues, 0, sizeof(s_queues));
    (void)memset(&s_stats, 0, sizeof(s_stats));

    /* 初始化 IMU Sensor 数据池 */
    status = FramePool_Init(&s_imu_sensor_pool,
                            s_imu_sensor_blocks,
                            sizeof(s_imu_sensor_blocks[0]),
                            GLOVE_IMU_SENSOR_POOL_SIZE,
                            s_imu_sensor_free_stack);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    /* 初始化 Touch Sensor 数据池 */
    status = FramePool_Init(&s_touch_sensor_pool,
                            s_touch_sensor_blocks,
                            sizeof(s_touch_sensor_blocks[0]),
                            GLOVE_TOUCH_SENSOR_POOL_SIZE,
                            s_touch_sensor_free_stack);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    /* 初始化 RawFrame 数据池 */
    status = FramePool_Init(&s_raw_pool,
                            s_raw_blocks,
                            sizeof(s_raw_blocks[0]),
                            GLOVE_RAW_FRAME_POOL_SIZE,
                            s_raw_free_stack);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    /* 初始化 FullFrame 数据池 */
    status = FramePool_Init(&s_full_pool,
                            s_full_blocks,
                            sizeof(s_full_blocks[0]),
                            GLOVE_FULL_FRAME_POOL_SIZE,
                            s_full_free_stack);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    /* 创建各数据通道的指针队列 */
    s_queues.imu_sensor_for_assembler =
        CreatePointerQueue(GLOVE_IMU_SENSOR_QUEUE_DEPTH, "sensor.imu");
    s_queues.touch_sensor_for_assembler =
        CreatePointerQueue(GLOVE_TOUCH_SENSOR_QUEUE_DEPTH, "sensor.touch");
    s_queues.raw_for_algorithm =
        CreatePointerQueue(GLOVE_RAW_FRAME_QUEUE_DEPTH, "raw.alg");
    s_queues.full_for_storage =
        CreatePointerQueue(GLOVE_FULL_FRAME_QUEUE_DEPTH, "full.store");
    s_queues.full_for_rs485 =
        CreatePointerQueue(GLOVE_FULL_FRAME_QUEUE_DEPTH, "full.rs485");

    if ((s_queues.imu_sensor_for_assembler == NULL) ||
        (s_queues.touch_sensor_for_assembler == NULL) ||
        (s_queues.raw_for_algorithm == NULL) ||
        (s_queues.full_for_storage == NULL) ||
        (s_queues.full_for_rs485 == NULL))
    {
        return GLOVE_STATUS_NO_MEMORY;
    }

    s_initialized = 1U;
    return GLOVE_STATUS_OK;
}

GloveImuSensorBlock_t *DataManager_AllocImuSensor(void)
{
    GloveImuSensorBlock_t *block;

    if (s_initialized == 0U)
    {
        return NULL;
    }

    block = (GloveImuSensorBlock_t *)FramePool_Alloc(&s_imu_sensor_pool);
    if (block == NULL)
    {
        StatsIncrement(&s_stats.pool_alloc_failures);
        return NULL;
    }

    AppData_ClearImuSensorData(&block->data);
    block->ref_count = 1U;
    return block;
}

GloveTouchSensorBlock_t *DataManager_AllocTouchSensor(void)
{
    GloveTouchSensorBlock_t *block;

    if (s_initialized == 0U)
    {
        return NULL;
    }

    block = (GloveTouchSensorBlock_t *)FramePool_Alloc(&s_touch_sensor_pool);
    if (block == NULL)
    {
        StatsIncrement(&s_stats.pool_alloc_failures);
        return NULL;
    }

    AppData_ClearTouchSensorData(&block->data);
    block->ref_count = 1U;
    return block;
}

GloveRawFrameBlock_t *DataManager_AllocRawFrame(void)
{
    GloveRawFrameBlock_t *block;

    if (s_initialized == 0U)
    {
        return NULL;
    }

    block = (GloveRawFrameBlock_t *)FramePool_Alloc(&s_raw_pool);
    if (block == NULL)
    {
        StatsIncrement(&s_stats.pool_alloc_failures);
        return NULL;
    }

    AppData_ClearRawFrame(&block->frame);
    block->ref_count = 1U;
    return block;
}

GloveFullFrameBlock_t *DataManager_AllocFullFrame(void)
{
    GloveFullFrameBlock_t *block;

    if (s_initialized == 0U)
    {
        return NULL;
    }

    block = (GloveFullFrameBlock_t *)FramePool_Alloc(&s_full_pool);
    if (block == NULL)
    {
        StatsIncrement(&s_stats.pool_alloc_failures);
        return NULL;
    }

    AppData_ClearFullFrame(&block->frame);
    block->ref_count = 1U;
    return block;
}

GloveStatus_t DataManager_PublishImuSensor(GloveImuSensorBlock_t *block, uint32_t timeout_ms)
{
    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return PublishSingleConsumer(block,
                                 &block->ref_count,
                                 s_queues.imu_sensor_for_assembler,
                                 &s_imu_sensor_pool,
                                 &s_stats.imu_sensor_published,
                                 &s_stats.imu_sensor_dropped,
                                 timeout_ms);
}

GloveStatus_t DataManager_PublishTouchSensor(GloveTouchSensorBlock_t *block, uint32_t timeout_ms)
{
    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return PublishSingleConsumer(block,
                                 &block->ref_count,
                                 s_queues.touch_sensor_for_assembler,
                                 &s_touch_sensor_pool,
                                 &s_stats.touch_sensor_published,
                                 &s_stats.touch_sensor_dropped,
                                 timeout_ms);
}

GloveStatus_t DataManager_PublishRawFrame(GloveRawFrameBlock_t *block, uint32_t timeout_ms)
{
    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return PublishSingleConsumer(block,
                                 &block->ref_count,
                                 s_queues.raw_for_algorithm,
                                 &s_raw_pool,
                                 &s_stats.raw_frames_published,
                                 &s_stats.raw_frames_dropped,
                                 timeout_ms);
}

GloveStatus_t DataManager_PublishFullFrame(GloveFullFrameBlock_t *block, uint32_t timeout_ms)
{
    GloveStatus_t status;
    GloveStatus_t final_status = GLOVE_STATUS_OK;
    uint8_t delivered_count = 0U;

    if ((s_initialized == 0U) || (block == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    /*
     * FullFrame 有两个消费者
     * Alloc 已经让发布者持有一个引用
     * Storage 引用和 RS485 引用共同管理后续生命周期
     */
    AddRef(&block->ref_count);
    status = SendPointer(s_queues.full_for_storage, block, timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        (void)DataManager_ReleaseFullFrame(block);
        final_status = status;
    }
    else
    {
        delivered_count++;
    }

    AddRef(&block->ref_count);
    status = SendPointer(s_queues.full_for_rs485, block, timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        (void)DataManager_ReleaseFullFrame(block);
        final_status = status;
    }
    else
    {
        delivered_count++;
    }

    /* 发布结束后释放发布者临时引用 */
    (void)DataManager_ReleaseFullFrame(block);

    if (delivered_count == 0U)
    {
        StatsIncrement(&s_stats.full_frames_dropped);
        return GLOVE_STATUS_QUEUE_FULL;
    }

    StatsIncrement(&s_stats.full_frames_published);
    return final_status;
}

GloveStatus_t DataManager_GetImuSensor(GloveImuSensorBlock_t **block, uint32_t timeout_ms)
{
    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReceivePointer(s_queues.imu_sensor_for_assembler, (void **)block, timeout_ms);
}

GloveStatus_t DataManager_GetTouchSensor(GloveTouchSensorBlock_t **block, uint32_t timeout_ms)
{
    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReceivePointer(s_queues.touch_sensor_for_assembler, (void **)block, timeout_ms);
}

GloveStatus_t DataManager_GetRawFrame(DataConsumer_t consumer,
                                      GloveRawFrameBlock_t **block,
                                      uint32_t timeout_ms)
{
    osMessageQueueId_t queue = NULL;

    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    switch (consumer)
    {
        case DATA_CONSUMER_ALGORITHM:
            queue = s_queues.raw_for_algorithm;
            break;
        default:
            return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReceivePointer(queue, (void **)block, timeout_ms);
}

GloveStatus_t DataManager_GetFullFrame(DataConsumer_t consumer,
                                       GloveFullFrameBlock_t **block,
                                       uint32_t timeout_ms)
{
    osMessageQueueId_t queue = NULL;

    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    switch (consumer)
    {
        case DATA_CONSUMER_STORAGE:
            queue = s_queues.full_for_storage;
            break;
        case DATA_CONSUMER_RS485:
            queue = s_queues.full_for_rs485;
            break;
        default:
            return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReceivePointer(queue, (void **)block, timeout_ms);
}

GloveStatus_t DataManager_ReleaseImuSensor(GloveImuSensorBlock_t *block)
{
    if ((s_initialized == 0U) || (block == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReleaseRef(&s_imu_sensor_pool, block, &block->ref_count);
}

GloveStatus_t DataManager_ReleaseTouchSensor(GloveTouchSensorBlock_t *block)
{
    if ((s_initialized == 0U) || (block == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReleaseRef(&s_touch_sensor_pool, block, &block->ref_count);
}

GloveStatus_t DataManager_ReleaseRawFrame(GloveRawFrameBlock_t *block)
{
    if ((s_initialized == 0U) || (block == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReleaseRef(&s_raw_pool, block, &block->ref_count);
}

GloveStatus_t DataManager_ReleaseFullFrame(GloveFullFrameBlock_t *block)
{
    if ((s_initialized == 0U) || (block == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReleaseRef(&s_full_pool, block, &block->ref_count);
}

void DataManager_GetStats(DataManagerStats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    stats->data = s_stats;
    taskEXIT_CRITICAL();

    FramePool_GetStats(&s_imu_sensor_pool, &stats->imu_sensor_pool);
    FramePool_GetStats(&s_touch_sensor_pool, &stats->touch_sensor_pool);
    FramePool_GetStats(&s_raw_pool, &stats->raw_pool);
    FramePool_GetStats(&s_full_pool, &stats->full_pool);
}
