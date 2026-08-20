#include "acq_sync.h"

#include <string.h>

#include "main.h"
#include "modbus_time_sync.h"

#define ACQ_SYNC_TOUCH_THREAD_FLAG      (1UL << 8)

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

    s_acq_sync_latest.timestamp_us =
        (GloveTimestampUs_t)ModbusTimeSync_GetUtcTimestampUsFromIsr();
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
    AcqSyncSnapshot_t latest;
    uint32_t last_seq;
    uint32_t flags;

    if (snapshot == NULL)
    {
        return osErrorParameter;
    }

    last_seq = (snapshot->valid != 0U) ? snapshot->seq : 0U;
    AcqSync_CopyLatest(&latest);
    if ((latest.valid != 0U) &&
        ((snapshot->valid == 0U) || (latest.seq != last_seq)))
    {
        *snapshot = latest;
        return osOK;
    }

    /* 清标志后再次核对序号，避免同步中断恰好发生在清标志前而丢帧。 */
    (void)osThreadFlagsClear(ACQ_SYNC_TOUCH_THREAD_FLAG);
    AcqSync_CopyLatest(&latest);
    if ((latest.valid != 0U) &&
        ((snapshot->valid == 0U) || (latest.seq != last_seq)))
    {
        *snapshot = latest;
        return osOK;
    }

    flags = osThreadFlagsWait(ACQ_SYNC_TOUCH_THREAD_FLAG,
                              osFlagsWaitAny,
                              timeout_ms);
    if ((flags & osFlagsError) != 0U)
    {
        return (flags == (uint32_t)osFlagsErrorTimeout) ? osErrorTimeout : osError;
    }

    AcqSync_CopyLatest(snapshot);
    return (snapshot->valid != 0U) ? osOK : osError;
}
