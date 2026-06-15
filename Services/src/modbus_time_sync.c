#include "modbus_time_sync.h"

#include "main.h"

#define TIME_SYNC_PPB_SCALE             1000000000LL
#define TIME_SYNC_ERROR_LIMIT_US        500000LL
#define TIME_SYNC_MAX_CORR_STEP_PPB     100000LL
#define TIME_SYNC_MAX_CORR_PPB          1000000LL

static volatile uint32_t time_sync_tim5_overflow = 0U;
static volatile uint64_t time_sync_utc_base_us = 0U;
static volatile uint64_t time_sync_last_sync_utc_us = 0U;
static volatile uint64_t time_sync_last_edge_local_us = 0U;
static volatile uint64_t time_sync_last_local_interval_us = 0U;
static volatile uint64_t time_sync_predicted_edge_utc_us = 0U;
static volatile int64_t time_sync_last_error_us = 0;
static volatile int32_t time_sync_freq_corr_ppb = 0;
static volatile uint32_t time_sync_edge_count = 0U;
static volatile uint8_t time_sync_wait_utc_frame = 0U;
static volatile uint8_t time_sync_synced = 0U;
static volatile uint8_t time_sync_has_utc_base = 0U;
static volatile uint8_t time_sync_has_prediction = 0U;
static volatile uint8_t time_sync_timer_running = 0U;

static uint64_t ModbusTimeSync_GetLocalUptimeUsIrqUnsafe(void)
{
  return (((uint64_t)time_sync_tim5_overflow) << 32) | (uint64_t)__HAL_TIM_GET_COUNTER(&htim5);
}

static void ModbusTimeSync_ResetLocalTimerIrqUnsafe(void)
{
  __HAL_TIM_SET_COUNTER(&htim5, 0U);
  time_sync_tim5_overflow = 0U;
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
}

static uint64_t ModbusTimeSync_ApplyFreqCorr(uint64_t elapsed_us)
{
  int64_t corr_ppb = (int64_t)time_sync_freq_corr_ppb;
  int64_t corr_us = (int64_t)((((int64_t)elapsed_us) * corr_ppb) / TIME_SYNC_PPB_SCALE);

  if ((corr_us < 0) && ((uint64_t)(-corr_us) > elapsed_us))
  {
    return 0U;
  }

  return (uint64_t)((int64_t)elapsed_us + corr_us);
}

static int32_t ModbusTimeSync_ClampCorrPpb(int64_t corr_ppb)
{
  if (corr_ppb > TIME_SYNC_MAX_CORR_PPB)
  {
    return (int32_t)TIME_SYNC_MAX_CORR_PPB;
  }

  if (corr_ppb < -TIME_SYNC_MAX_CORR_PPB)
  {
    return (int32_t)(-TIME_SYNC_MAX_CORR_PPB);
  }

  return (int32_t)corr_ppb;
}

static int32_t ModbusTimeSync_ClampCorrStepPpb(int64_t corr_step_ppb)
{
  if (corr_step_ppb > TIME_SYNC_MAX_CORR_STEP_PPB)
  {
    return (int32_t)TIME_SYNC_MAX_CORR_STEP_PPB;
  }

  if (corr_step_ppb < -TIME_SYNC_MAX_CORR_STEP_PPB)
  {
    return (int32_t)(-TIME_SYNC_MAX_CORR_STEP_PPB);
  }

  return (int32_t)corr_step_ppb;
}

HAL_StatusTypeDef ModbusTimeSync_Init(void)
{
  time_sync_tim5_overflow = 0U;
  time_sync_utc_base_us = 0U;
  time_sync_last_sync_utc_us = 0U;
  time_sync_last_edge_local_us = 0U;
  time_sync_last_local_interval_us = 0U;
  time_sync_predicted_edge_utc_us = 0U;
  time_sync_last_error_us = 0;
  time_sync_freq_corr_ppb = 0;
  time_sync_edge_count = 0U;
  time_sync_wait_utc_frame = 0U;
  time_sync_synced = 0U;
  time_sync_has_utc_base = 0U;
  time_sync_has_prediction = 0U;
  time_sync_timer_running = 0U;

  __HAL_TIM_SET_COUNTER(&htim5, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);

  return HAL_OK;
}

void ModbusTimeSync_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
  if ((htim != NULL) && (htim->Instance == TIM5))
  {
    time_sync_tim5_overflow++;
  }
}

void ModbusTimeSync_OnPpsEdge(uint16_t gpio_pin)
{
  if (gpio_pin == PPS_IN_Pin)
  {
    uint64_t elapsed_us = ModbusTimeSync_GetLocalUptimeUsIrqUnsafe();
    uint64_t corrected_elapsed_us;

    time_sync_last_edge_local_us = elapsed_us;
    time_sync_last_local_interval_us = elapsed_us;
    time_sync_edge_count++;

    if (time_sync_has_utc_base != 0U)
    {
      corrected_elapsed_us = ModbusTimeSync_ApplyFreqCorr(elapsed_us);
      time_sync_predicted_edge_utc_us = time_sync_utc_base_us + corrected_elapsed_us;
      time_sync_has_prediction = 1U;
    }
    else
    {
      time_sync_predicted_edge_utc_us = 0U;
      time_sync_has_prediction = 0U;
    }

    ModbusTimeSync_ResetLocalTimerIrqUnsafe();
    if (time_sync_timer_running == 0U)
    {
      __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
      __HAL_TIM_ENABLE(&htim5);
      time_sync_timer_running = 1U;
    }

    time_sync_wait_utc_frame = 1U;
    time_sync_synced = 0U;
  }
}

uint64_t ModbusTimeSync_GetLocalUptimeUs(void)
{
  uint64_t local_us;

  __disable_irq();
  local_us = ModbusTimeSync_GetLocalUptimeUsIrqUnsafe();
  __enable_irq();

  return local_us;
}

uint64_t ModbusTimeSync_GetUtcTimestampUs(void)
{
  uint64_t utc_us = 0U;
  uint64_t local_now_us;
  uint64_t utc_base_us;
  uint64_t corrected_elapsed_us;
  uint8_t synced;

  __disable_irq();
  synced = time_sync_synced;
  local_now_us = ModbusTimeSync_GetLocalUptimeUsIrqUnsafe();
  utc_base_us = time_sync_utc_base_us;
  corrected_elapsed_us = ModbusTimeSync_ApplyFreqCorr(local_now_us);
  __enable_irq();

  if (synced != 0U)
  {
    utc_us = utc_base_us + corrected_elapsed_us;
  }

  return utc_us;
}

uint64_t ModbusTimeSync_GetLastSyncUtcUs(void)
{
  uint64_t utc_us;

  __disable_irq();
  utc_us = time_sync_last_sync_utc_us;
  __enable_irq();

  return utc_us;
}

uint64_t ModbusTimeSync_GetLastSyncEdgeLocalUs(void)
{
  uint64_t local_us;

  __disable_irq();
  local_us = time_sync_last_edge_local_us;
  __enable_irq();

  return local_us;
}

uint64_t ModbusTimeSync_GetLastLocalIntervalUs(void)
{
  uint64_t interval_us;

  __disable_irq();
  interval_us = time_sync_last_local_interval_us;
  __enable_irq();

  return interval_us;
}

uint64_t ModbusTimeSync_GetPredictedEdgeUtcUs(void)
{
  uint64_t utc_us;

  __disable_irq();
  utc_us = time_sync_predicted_edge_utc_us;
  __enable_irq();

  return utc_us;
}

int64_t ModbusTimeSync_GetLastSyncErrorUs(void)
{
  int64_t error_us;

  __disable_irq();
  error_us = time_sync_last_error_us;
  __enable_irq();

  return error_us;
}

int32_t ModbusTimeSync_GetFreqCorrPpb(void)
{
  int32_t corr_ppb;

  __disable_irq();
  corr_ppb = time_sync_freq_corr_ppb;
  __enable_irq();

  return corr_ppb;
}

uint8_t ModbusTimeSync_IsSynced(void)
{
  return time_sync_synced;
}

uint8_t ModbusTimeSync_IsWaitingUtc(void)
{
  return time_sync_wait_utc_frame;
}

void ModbusTimeSync_SetUtcFromMaster(uint64_t utc_us)
{
  __disable_irq();
  time_sync_last_sync_utc_us = utc_us;

  if (time_sync_wait_utc_frame == 0U)
  {
    time_sync_utc_base_us = utc_us;
    time_sync_has_utc_base = 1U;
    time_sync_has_prediction = 0U;
    ModbusTimeSync_ResetLocalTimerIrqUnsafe();
    if (time_sync_timer_running == 0U)
    {
      __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
      __HAL_TIM_ENABLE(&htim5);
      time_sync_timer_running = 1U;
    }
  }
  else if (time_sync_has_prediction == 0U)
  {
    time_sync_utc_base_us = utc_us;
    time_sync_freq_corr_ppb = 0;
    time_sync_last_error_us = 0;
    time_sync_has_utc_base = 1U;
  }
  else
  {
    int64_t error_us = (int64_t)utc_us - (int64_t)time_sync_predicted_edge_utc_us;

    time_sync_last_error_us = error_us;
    if ((error_us > TIME_SYNC_ERROR_LIMIT_US) || (error_us < -TIME_SYNC_ERROR_LIMIT_US))
    {
      time_sync_freq_corr_ppb = 0;
    }
    else if (time_sync_last_local_interval_us > 0U)
    {
      int64_t corr_step_ppb = (error_us * TIME_SYNC_PPB_SCALE) / (int64_t)time_sync_last_local_interval_us;
      int64_t next_corr_ppb = (int64_t)time_sync_freq_corr_ppb + (int64_t)ModbusTimeSync_ClampCorrStepPpb(corr_step_ppb);

      time_sync_freq_corr_ppb = ModbusTimeSync_ClampCorrPpb(next_corr_ppb);
    }

    time_sync_utc_base_us = utc_us;
    time_sync_has_utc_base = 1U;
  }

  time_sync_wait_utc_frame = 0U;
  time_sync_has_prediction = 0U;
  time_sync_synced = 1U;
  __enable_irq();
}
