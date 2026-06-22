#ifndef ACQ_SYNC_H
#define ACQ_SYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"
#include "cmsis_os2.h"

typedef struct
{
    uint32_t seq;
    GloveTimestampUs_t timestamp_us;
    uint8_t valid;
} AcqSyncSnapshot_t;

void AcqSync_Reset(void);
void AcqSync_RegisterTouchTask(osThreadId_t thread_id);
void AcqSync_OnTim2PeriodElapsedFromIsr(void);
uint8_t AcqSync_GetLatest(AcqSyncSnapshot_t *snapshot);
osStatus_t AcqSync_WaitForTouchSync(AcqSyncSnapshot_t *snapshot, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ACQ_SYNC_H */
