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
    uint8_t debug_mode;               /* 独立调试使用本地采样时间，禁止声明有效UTC。 */
    uint8_t utc_valid;                /* 采样触发时的UTC有效性，与传感器数据有效性分开。 */
} AcqSyncSnapshot_t;

typedef enum
{
    ACQ_SYNC_STATE_WAIT_FIRST_PPS = 0,
    ACQ_SYNC_STATE_SAMPLING = 1,
    ACQ_SYNC_STATE_WAIT_NEXT_PPS = 2,
    ACQ_SYNC_STATE_PPS_LOST = 3,
    ACQ_SYNC_STATE_DEBUG_SAMPLING = 4,
    ACQ_SYNC_STATE_MODE_SWITCH = 5
} AcqSyncState_t;

typedef enum
{
    ACQ_MODE_NORMAL = 0,
    ACQ_MODE_DEBUG = 1,
    ACQ_MODE_SWITCHING = 2
} AcqSyncMode_t;

typedef struct
{
    AcqSyncState_t state;
    AcqSyncMode_t mode;
    uint32_t generation;
    uint32_t latest_sample_seq;
    uint64_t latest_sample_local_us;
    uint32_t mode_age_ms;
    uint16_t debug_lease_remaining_ms;
    uint8_t sampling_allowed;
    uint32_t last_pps_age_ms;
    uint32_t expected_pps_interval_us;
    uint16_t samples_in_window;
    uint8_t pps_present;
    uint8_t window_active;
    uint8_t has_seen_pps;
} AcqSyncStatus_t;

void AcqSync_Reset(void);
void AcqSync_Service(void);
void AcqSync_RegisterTouchTask(osThreadId_t thread_id);
void AcqSync_OnTim2PeriodElapsedFromIsr(void);
void AcqSync_OnTim2PpsTriggerFromIsr(void);
void AcqSync_OnTim2PulseFinishedFromIsr(void);
uint8_t AcqSync_GetLatest(AcqSyncSnapshot_t *snapshot);
void AcqSync_GetStatus(AcqSyncStatus_t *status);
/* 命令仅申请切换，周期服务等待脉冲下降沿和20ms排空时间后完成。 */
GloveStatus_t AcqSync_RequestDebugMode(uint8_t enabled);
GloveStatus_t AcqSync_RenewDebugLease(void);
uint8_t AcqSync_IsDebugMode(void);
uint8_t AcqSync_IsNormalMode(void);
uint8_t AcqSync_IsSamplingAllowed(void);
uint8_t AcqSync_IsSequenceCurrent(uint32_t seq);
void AcqSync_OnDebugPpsEdgeFromIsr(void);
uint8_t AcqSync_IsPpsPresent(void);
uint8_t AcqSync_IsSamplingActive(void);
osStatus_t AcqSync_WaitForTouchSync(AcqSyncSnapshot_t *snapshot, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ACQ_SYNC_H */
