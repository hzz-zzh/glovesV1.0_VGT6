#ifndef FRAME_POOL_H
#define FRAME_POOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "app_data.h"

/*
 * 1. 启动时一次性准备好固定数量的数据块
 * 2. 运行过程中不使用 malloc/free，避免堆碎片和不可预测耗时
 * 3. 队列里只传指针，大数据帧本体由内存池管理
 */
typedef struct
{
    uint8_t *storage;               /* 数据块连续存储区起始地址 */
    uint16_t *free_stack;           /* 空闲块索引栈 */
    uint16_t block_size;            /* 单个数据块大小，单位 byte */
    uint16_t capacity;              /* 数据块总数 */
    uint16_t free_count;            /* 当前空闲块数量 */
    uint16_t min_free_count;        /* 历史最低空闲块数量，用于评估池容量是否足够 */
} FramePool_t;

/* 内存池运行统计 */
typedef struct
{
    uint16_t capacity;              /* 总块数 */
    uint16_t free_count;            /* 当前空闲块数 */
    uint16_t min_free_count;        /* 历史最低空闲块数 */
    uint16_t used_count;            /* 当前已使用块数 */
} FramePoolStats_t;

GloveStatus_t FramePool_Init(FramePool_t *pool,
                             void *storage,
                             uint16_t block_size,
                             uint16_t capacity,
                             uint16_t *free_stack);

void *FramePool_Alloc(FramePool_t *pool);

GloveStatus_t FramePool_Free(FramePool_t *pool, void *block);

/* 判断某个指针是否落在该内存池管理范围内 */
uint8_t FramePool_Owns(const FramePool_t *pool, const void *block);

void FramePool_GetStats(const FramePool_t *pool, FramePoolStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_POOL_H */
