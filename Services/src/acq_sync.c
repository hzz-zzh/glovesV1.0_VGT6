#include "acq_sync.h"

#include <string.h>

#include "main.h"

#define ACQ_SYNC_TOUCH_THREAD_FLAG      (1UL << 8)
#define ACQ_SYNC_TIM2_PERIOD_US         (10000ULL)

static volatile AcqSyncSnapshot_t s_acq_sync_latest;
static osThreadId_t s_acq_sync_touch_task_id;

static void AcqSync_CopyLatest(AcqSyncSnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    __disable_irq();
    *snapshot = s_acq_sync_latest;
    __enable_irq();
}

void AcqSync_Reset(void)
{
    __disable_irq();
    (void)memset((void *)&s_acq_sync_latest, 0, sizeof(s_acq_sync_latest));
    s_acq_sync_touch_task_id = NULL;
    __enable_irq();
}

void AcqSync_RegisterTouchTask(osThreadId_t thread_id)
{
    __disable_irq();
    s_acq_sync_touch_task_id = thread_id;
    __enable_irq();
}

void AcqSync_OnTim2PeriodElapsedFromIsr(void)
{
    uint32_t next_seq = s_acq_sync_latest.seq + 1U;

    if (s_acq_sync_latest.valid == 0U)
    {
        s_acq_sync_latest.timestamp_us = (GloveTimestampUs_t)HAL_GetTick() * 1000ULL;
    }
    else
    {
        s_acq_sync_latest.timestamp_us += ACQ_SYNC_TIM2_PERIOD_US;
    }

    s_acq_sync_latest.seq = next_seq;
    s_acq_sync_latest.valid = 1U;

    if (s_acq_sync_touch_task_id != NULL)
    {
        (void)osThreadFlagsSet(s_acq_sync_touch_task_id, ACQ_SYNC_TOUCH_THREAD_FLAG);
    }
}

uint8_t AcqSync_GetLatest(AcqSyncSnapshot_t *snapshot)
{
    AcqSync_CopyLatest(snapshot);

    return ((snapshot != NULL) && (snapshot->valid != 0U)) ? 1U : 0U;
}

osStatus_t AcqSync_WaitForTouchSync(AcqSyncSnapshot_t *snapshot, uint32_t timeout_ms)
{
    osStatus_t status;
    uint32_t flags;

    (void)osThreadFlagsClear(ACQ_SYNC_TOUCH_THREAD_FLAG);

    flags = osThreadFlagsWait(ACQ_SYNC_TOUCH_THREAD_FLAG,
                              osFlagsWaitAny,
                              timeout_ms);
    if ((flags & osFlagsError) != 0U)
    {
        status = (flags == (uint32_t)osFlagsErrorTimeout) ? osErrorTimeout : osError;
    }
    else
    {
        AcqSync_CopyLatest(snapshot);
        status = osOK;
    }

    return status;
}
