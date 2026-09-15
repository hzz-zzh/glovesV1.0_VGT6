#include "modbus_time_sync.h"

#include "main.h"

#define TIME_SYNC_PPB_SCALE             1000000000LL
#define TIME_SYNC_ERROR_LIMIT_US        500000LL
#define TIME_SYNC_MAX_CORR_STEP_PPB     100000LL
#define TIME_SYNC_MAX_CORR_PPB          1000000LL

static volatile uint32_t time_sync_tim5_overflow = 0U;
static volatile uint64_t time_sync_utc_base_us = 0U;
static volatile uint64_t time_sync_local_base_us = 0U;
static volatile uint64_t time_sync_last_sync_utc_us = 0U;
static volatile uint64_t time_sync_last_edge_local_us = 0U;
static volatile uint64_t time_sync_last_local_interval_us = 0U;
static volatile uint64_t time_sync_predicted_edge_utc_us = 0U;
static volatile int64_t time_sync_last_error_us = 0;
static volatile int32_t time_sync_freq_corr_ppb = 0;
static volatile uint32_t time_sync_edge_count = 0U;
static volatile uint8_t time_sync_wait_utc_frame = 0U;
static volatile uint8_t time_sync_synced = 0U;
static volatile uint8_t time_sync_pps_present = 0U;
static volatile uint8_t time_sync_has_utc_base = 0U;
static volatile uint8_t time_sync_has_prediction = 0U;
static uint32_t time_sync_tim5_counter_hz = 1000000U;

static uint32_t ModbusTimeSync_GetApb1TimerClockHz(void)
{
  RCC_ClkInitTypeDef clock_config = {0};
  uint32_t flash_latency = 0U;
  uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  if ((RCC->CFGR1 & RCC_CFGR1_TIMPRE) == 0U)
  {
    /* TIMPRE=0时，APB分频为1或2均使用HCLK，其余使用2倍PCLK。 */
    timer_clock_hz = ((clock_config.APB1CLKDivider == RCC_HCLK_DIV1) ||
                      (clock_config.APB1CLKDivider == RCC_HCLK_DIV2)) ?
                     hclk_hz : (timer_clock_hz * 2U);
  }
  else
  {
    /* TIMPRE=1时，APB分频不超过4均使用HCLK，其余使用4倍PCLK。 */
    timer_clock_hz = ((clock_config.APB1CLKDivider == RCC_HCLK_DIV1) ||
                      (clock_config.APB1CLKDivider == RCC_HCLK_DIV2) ||
                      (clock_config.APB1CLKDivider == RCC_HCLK_DIV4)) ?
                     hclk_hz : (timer_clock_hz * 4U);
  }
  return timer_clock_hz;
}

static uint64_t ModbusTimeSync_GetLocalTicksIrqUnsafe(void)
{
  uint32_t overflow = time_sync_tim5_overflow;
  uint32_t counter = __HAL_TIM_GET_COUNTER(&htim5);

  /* CNT回卷后、更新中断尚未执行时补偿高32位，避免偶发时间倒退。 */
  if ((__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) &&
      (counter < 0x80000000UL))
  {
    overflow++;
  }
  return (((uint64_t)overflow) << 32) | (uint64_t)counter;
}

static uint64_t ModbusTimeSync_TicksToUs(uint64_t ticks)
{
  uint64_t whole_seconds;
  uint64_t remainder_ticks;

  if (time_sync_tim5_counter_hz == 0U)
  {
    return 0U;
  }
  whole_seconds = ticks / time_sync_tim5_counter_hz;
  remainder_ticks = ticks % time_sync_tim5_counter_hz;
  return (whole_seconds * 1000000ULL) +
         ((remainder_ticks * 1000000ULL) / time_sync_tim5_counter_hz);
}

static uint64_t ModbusTimeSync_GetLocalUptimeUsIrqUnsafe(void)
{
  return ModbusTimeSync_TicksToUs(ModbusTimeSync_GetLocalTicksIrqUnsafe());
}

static uint64_t ModbusTimeSync_ApplyFreqCorr(uint64_t elapsed_us)
{
  int64_t corr_ppb = (int64_t)time_sync_freq_corr_ppb;
  int64_t corr_us =
      ((int64_t)(elapsed_us / (uint64_t)TIME_SYNC_PPB_SCALE) * corr_ppb) +
      (((int64_t)(elapsed_us % (uint64_t)TIME_SYNC_PPB_SCALE) * corr_ppb) /
       TIME_SYNC_PPB_SCALE);

  if ((corr_us < 0) && ((uint64_t)(-corr_us) > elapsed_us))
  {
    return 0U;
  }

  return (uint64_t)((int64_t)elapsed_us + corr_us);
}

static uint64_t ModbusTimeSync_GetUtcTimestampUsIrqUnsafe(void)
{
  uint64_t utc_us = 0U;
  uint64_t local_now_us;
  uint64_t elapsed_us;
  uint64_t corrected_elapsed_us;

  local_now_us = ModbusTimeSync_GetLocalUptimeUsIrqUnsafe();
  if (time_sync_has_utc_base != 0U)
  {
    elapsed_us = local_now_us - time_sync_local_base_us;
    corrected_elapsed_us = ModbusTimeSync_ApplyFreqCorr(elapsed_us);
    utc_us = time_sync_utc_base_us + corrected_elapsed_us;
  }

  return utc_us;
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
  uint32_t timer_clock_hz = ModbusTimeSync_GetApb1TimerClockHz();

  time_sync_tim5_overflow = 0U;
  time_sync_utc_base_us = 0U;
  time_sync_local_base_us = 0U;
  time_sync_last_sync_utc_us = 0U;
  time_sync_last_edge_local_us = 0U;
  time_sync_last_local_interval_us = 0U;
  time_sync_predicted_edge_utc_us = 0U;
  time_sync_last_error_us = 0;
  time_sync_freq_corr_ppb = 0;
  time_sync_edge_count = 0U;
  time_sync_wait_utc_frame = 0U;
  time_sync_synced = 0U;
  time_sync_pps_present = 0U;
  time_sync_has_utc_base = 0U;
  time_sync_has_prediction = 0U;
  time_sync_tim5_counter_hz = timer_clock_hz / (htim5.Init.Prescaler + 1U);

  __HAL_TIM_SET_COUNTER(&htim5, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);

  /* TIM5始终自由运行，主机写UTC或PPS到来都只更新基准，不再清零本地时钟。 */
  return HAL_TIM_Base_Start_IT(&htim5);
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
    uint64_t previous_edge_local_us = time_sync_last_edge_local_us;
    uint64_t corrected_elapsed_us;

    time_sync_last_local_interval_us = (time_sync_edge_count > 0U) ?
                                        (elapsed_us - previous_edge_local_us) : 0U;
    time_sync_last_edge_local_us = elapsed_us;
    time_sync_edge_count++;

    if (time_sync_has_utc_base != 0U)
    {
      corrected_elapsed_us = ModbusTimeSync_ApplyFreqCorr(
          elapsed_us - time_sync_local_base_us);
      time_sync_predicted_edge_utc_us = time_sync_utc_base_us + corrected_elapsed_us;
      time_sync_has_prediction = 1U;
    }
    else
    {
      time_sync_predicted_edge_utc_us = 0U;
      time_sync_has_prediction = 0U;
    }

    time_sync_wait_utc_frame = 1U;
    time_sync_pps_present = 1U;
    time_sync_synced = 0U;
  }
}

void ModbusTimeSync_OnPpsLost(void)
{
  time_sync_wait_utc_frame = 0U;
  time_sync_has_prediction = 0U;
  time_sync_pps_present = 0U;
  time_sync_synced = 0U;
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
  uint64_t utc_us;

  __disable_irq();
  utc_us = ModbusTimeSync_GetUtcTimestampUsIrqUnsafe();
  __enable_irq();

  return utc_us;
}

uint64_t ModbusTimeSync_GetUtcTimestampUsFromIsr(void)
{
  return ModbusTimeSync_GetUtcTimestampUsIrqUnsafe();
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

uint64_t ModbusTimeSync_GetLastLocalIntervalUsFromIsr(void)
{
  return time_sync_last_local_interval_us;
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
  uint64_t local_now_us;

  __disable_irq();
  local_now_us = ModbusTimeSync_GetLocalUptimeUsIrqUnsafe();
  time_sync_last_sync_utc_us = utc_us;

  if (time_sync_wait_utc_frame == 0U)
  {
    time_sync_utc_base_us = utc_us;
    time_sync_local_base_us = local_now_us;
    time_sync_has_utc_base = 1U;
    time_sync_has_prediction = 0U;
  }
  else if (time_sync_has_prediction == 0U)
  {
    time_sync_utc_base_us = utc_us;
    time_sync_local_base_us = time_sync_last_edge_local_us;
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
    time_sync_local_base_us = time_sync_last_edge_local_us;
    time_sync_has_utc_base = 1U;
  }

  time_sync_wait_utc_frame = 0U;
  time_sync_has_prediction = 0U;
  /* 只有当前PPS存在时，主机时间写入才构成有效的PPS对时。 */
  time_sync_synced = time_sync_pps_present;
  __enable_irq();
}
