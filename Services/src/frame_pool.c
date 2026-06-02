#include "frame_pool.h"

#include "FreeRTOS.h"
#include "task.h"

/* 检查内存地址是否按 32 bit 对齐，避免后续访问结构体时出现非对齐访问风险 */
static uint8_t FramePool_IsAligned(const void *ptr)
{
    return ((((uintptr_t)ptr) % sizeof(uint32_t)) == 0U) ? 1U : 0U;
}

/*
 * storage 是上层提供的连续数据块内存；
 * free_stack 是空闲块索引栈；
 */
GloveStatus_t FramePool_Init(FramePool_t *pool,
                             void *storage,
                             uint16_t block_size,
                             uint16_t capacity,
                             uint16_t *free_stack)
{
    if ((pool == NULL) || (storage == NULL) || (free_stack == NULL) ||
        (block_size == 0U) || (capacity == 0U) ||
        (FramePool_IsAligned(storage) == 0U))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    pool->storage = (uint8_t *)storage;
    pool->free_stack = free_stack;
    pool->block_size = block_size;
    pool->capacity = capacity;
    pool->free_count = capacity;
    pool->min_free_count = capacity;

    /*
     * 初始化空闲索引栈
     * 分配时从栈顶弹出索引，释放时再压回索引
     */
    for (uint16_t i = 0U; i < capacity; i++)
    {
        pool->free_stack[i] = (uint16_t)(capacity - 1U - i);
    }

    return GLOVE_STATUS_OK;
}


void *FramePool_Alloc(FramePool_t *pool)
{
    void *block = NULL;

    if (pool == NULL)
    {
        return NULL;
    }

    /* 多任务环境下，空闲栈操作进入临界区 */
    taskENTER_CRITICAL();
    if (pool->free_count > 0U)
    {
        const uint16_t index = pool->free_stack[pool->free_count - 1U];
        pool->free_count--;
        if (pool->free_count < pool->min_free_count)
        {
            /* 记录历史最低空闲数，用于判断池容量是否需要加大 */
            pool->min_free_count = pool->free_count;
        }
        block = &pool->storage[(uint32_t)index * pool->block_size];
    }
    taskEXIT_CRITICAL();

    return block;
}

GloveStatus_t FramePool_Free(FramePool_t *pool, void *block)
{
    uint32_t offset;
    uint16_t index;

    /* 只允许释放本内存池管理范围内的块 */
    if ((pool == NULL) || (block == NULL) || (FramePool_Owns(pool, block) == 0U))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    offset = (uint32_t)((uint8_t *)block - pool->storage);
    /* 必须正好指向某个块的起始地址，不能释放块内部地址 */
    if ((offset % pool->block_size) != 0U)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    index = (uint16_t)(offset / pool->block_size);

    taskENTER_CRITICAL();
    if (pool->free_count >= pool->capacity)
    {
        /* 空闲数量已满，说明可能发生了重复释放 */
        taskEXIT_CRITICAL();
        return GLOVE_STATUS_ERROR;
    }

    pool->free_stack[pool->free_count] = index;
    pool->free_count++;
    taskEXIT_CRITICAL();

    return GLOVE_STATUS_OK;
}

/* 判断 block 是否落在 pool 管理的连续内存范围内 */
uint8_t FramePool_Owns(const FramePool_t *pool, const void *block)
{
    const uint8_t *first;
    const uint8_t *last;
    const uint8_t *addr;

    if ((pool == NULL) || (block == NULL) || (pool->storage == NULL))
    {
        return 0U;
    }

    first = pool->storage;
    last = &pool->storage[(uint32_t)pool->capacity * pool->block_size];
    addr = (const uint8_t *)block;

    return ((addr >= first) && (addr < last)) ? 1U : 0U;
}

void FramePool_GetStats(const FramePool_t *pool, FramePoolStats_t *stats)
{
    if ((pool == NULL) || (stats == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    stats->capacity = pool->capacity;
    stats->free_count = pool->free_count;
    stats->min_free_count = pool->min_free_count;
    stats->used_count = (uint16_t)(pool->capacity - pool->free_count);
    taskEXIT_CRITICAL();
}
