#include "modbus_time_sync.h"

#include "main.h"
#include "acq_sync.h"

#define TIME_SYNC_PPB_SCALE             1000000000LL
#define TIME_SYNC_SECOND_US             1000000ULL
#define TIME_SYNC_PPS_INTERVAL_MIN_US   900000ULL
#define TIME_SYNC_PPS_INTERVAL_MAX_US   1100000ULL
#define TIME_SYNC_MAX_CORR_STEP_PPB     100000LL
#define TIME_SYNC_MAX_CORR_PPB          1000000LL

static volatile uint32_t time_sync_tim5_overflow = 0U;
static volatile uint64_t time_sync_utc_base_us = 0U;
static volatile uint64_t time_sync_local_base_us = 0U;
static volatile uint64_t time_sync_last_sync_utc_us = 0U;
static volatile uint64_t time_sync_last_edge_local_us = 0U;
static volatile uint64_t time_sync_last_edge_utc_us = 0U;
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

static void ModbusTimeSync_UpdateFreqFromPps(uint64_t interval_us)
{
  int64_t measured_corr_ppb;
  int64_t corr_step_ppb;

  /* 一个正常PPS周期对应准确1秒，频率校正不依赖主机是否重复写UTC。 */
  measured_corr_ppb = (((int64_t)TIME_SYNC_SECOND_US - (int64_t)interval_us) *
                       TIME_SYNC_PPB_SCALE) / (int64_t)interval_us;
  measured_corr_ppb = ModbusTimeSync_ClampCorrPpb(measured_corr_ppb);
  corr_step_ppb = measured_corr_ppb - (int64_t)time_sync_freq_corr_ppb;
  time_sync_freq_corr_ppb = ModbusTimeSync_ClampCorrPpb(
      (int64_t)time_sync_freq_corr_ppb + ModbusTimeSync_ClampCorrStepPpb(corr_step_ppb));
}

HAL_StatusTypeDef ModbusTimeSync_Init(void)
{
  uint32_t timer_clock_hz = ModbusTimeSync_GetApb1TimerClockHz();

  time_sync_tim5_overflow = 0U;
  time_sync_utc_base_us = 0U;
  time_sync_local_base_us = 0U;
  time_sync_last_sync_utc_us = 0U;
  time_sync_last_edge_local_us = 0U;
  time_sync_last_edge_utc_us = 0U;
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
  if ((gpio_pin == PPS_IN_Pin) && (AcqSync_IsNormalMode() != 0U))
  {
    uint64_t elapsed_us = ModbusTimeSync_GetLocalUptimeUsIrqUnsafe();
    uint64_t previous_edge_local_us = time_sync_last_edge_local_us;
    uint64_t corrected_elapsed_us;
    uint8_t continuous_synced_pps;

    time_sync_last_local_interval_us = (time_sync_edge_count > 0U) ?
                                        (elapsed_us - previous_edge_local_us) : 0U;
    continuous_synced_pps = ((time_sync_pps_present != 0U) &&
                             (time_sync_synced != 0U) &&
                             (time_sync_last_local_interval_us >= TIME_SYNC_PPS_INTERVAL_MIN_US) &&
                             (time_sync_last_local_interval_us <= TIME_SYNC_PPS_INTERVAL_MAX_US)) ?
                            1U : 0U;
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

    if (continuous_synced_pps != 0U)
    {
      /* 已知绝对秒数后，每个连续PPS自动推进1秒，不再等待UTC报文才使时间有效。 */
      time_sync_last_edge_utc_us += TIME_SYNC_SECOND_US;
      ModbusTimeSync_UpdateFreqFromPps(time_sync_last_local_interval_us);
      time_sync_utc_base_us = time_sync_last_edge_utc_us;
      time_sync_local_base_us = elapsed_us;
    }
    else
    {
      /* 首次PPS、丢失恢复或周期异常不能猜测绝对秒数，等待主机重新确认。 */
      time_sync_last_edge_utc_us = time_sync_predicted_edge_utc_us;
      time_sync_synced = 0U;
    }

    /* 仅首次或失步后的PPS需要主机确认UTC，连续正常PPS不进入周期性等待。 */
    time_sync_wait_utc_frame = (time_sync_synced == 0U) ? 1U : 0U;
    time_sync_pps_present = 1U;
  }
}

void ModbusTimeSync_OnAcquisitionModeChanged(void)
{
  /* 模式边界必须重新建立PPS关联，不能把调试前的一秒继续当成连续PPS。 */
  ModbusTimeSync_OnPpsLost();
  time_sync_edge_count = 0U;
  time_sync_last_local_interval_us = 0ULL;
  time_sync_last_edge_local_us = 0ULL;
}

void ModbusTimeSync_OnPpsLost(void)
{
  /* 丢失期间没有可绑定的PPS，下一次PPS到来后才等待对应的主机UTC。 */
  time_sync_wait_utc_frame = 0U;
  time_sync_has_prediction = 0U;
  time_sync_pps_present = 0U;
  time_sync_synced = 0U;
}

uint64_t ModbusTimeSync_GetLocalUptimeUs(void)
{
  uint64_t local_us;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  local_us = ModbusTimeSync_GetLocalUptimeUsIrqUnsafe();
  __set_PRIMASK(irq_mask);

  return local_us;
}

uint64_t ModbusTimeSync_GetUtcTimestampUs(void)
{
  uint64_t utc_us;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  utc_us = ModbusTimeSync_GetUtcTimestampUsIrqUnsafe();
  __set_PRIMASK(irq_mask);

  return utc_us;
}

uint64_t ModbusTimeSync_GetUtcTimestampUsFromIsr(void)
{
  return ModbusTimeSync_GetUtcTimestampUsIrqUnsafe();
}

uint64_t ModbusTimeSync_GetPpsEdgeUtcUsFromIsr(void)
{
  /* 第0帧直接使用边沿UTC，避免把中断处理耗时当成采集时刻的微秒部分。 */
  return time_sync_last_edge_utc_us;
}

uint64_t ModbusTimeSync_GetLastSyncUtcUs(void)
{
  uint64_t utc_us;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  utc_us = time_sync_last_sync_utc_us;
  __set_PRIMASK(irq_mask);

  return utc_us;
}

uint64_t ModbusTimeSync_GetLastSyncEdgeLocalUs(void)
{
  uint64_t local_us;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  local_us = time_sync_last_edge_local_us;
  __set_PRIMASK(irq_mask);

  return local_us;
}

uint64_t ModbusTimeSync_GetLastLocalIntervalUs(void)
{
  uint64_t interval_us;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  interval_us = time_sync_last_local_interval_us;
  __set_PRIMASK(irq_mask);

  return interval_us;
}

uint64_t ModbusTimeSync_GetLastLocalIntervalUsFromIsr(void)
{
  return time_sync_last_local_interval_us;
}

uint64_t ModbusTimeSync_GetPredictedEdgeUtcUs(void)
{
  uint64_t utc_us;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  utc_us = time_sync_predicted_edge_utc_us;
  __set_PRIMASK(irq_mask);

  return utc_us;
}

int64_t ModbusTimeSync_GetLastSyncErrorUs(void)
{
  int64_t error_us;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  error_us = time_sync_last_error_us;
  __set_PRIMASK(irq_mask);

  return error_us;
}

int32_t ModbusTimeSync_GetFreqCorrPpb(void)
{
  int32_t corr_ppb;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  corr_ppb = time_sync_freq_corr_ppb;
  __set_PRIMASK(irq_mask);

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
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  local_now_us = ModbusTimeSync_GetLocalUptimeUsIrqUnsafe();
  if (AcqSync_IsNormalMode() == 0U)
  {
    __set_PRIMASK(irq_mask);
    return;
  }
  time_sync_last_sync_utc_us = utc_us;

  if (time_sync_pps_present == 0U)
  {
    /* 无PPS时只建立估算时间，不声明已经完成PPS对时。 */
    time_sync_utc_base_us = utc_us;
    time_sync_local_base_us = local_now_us;
    time_sync_has_utc_base = 1U;
    time_sync_has_prediction = 0U;
    time_sync_synced = 0U;
  }
  else
  {
    time_sync_last_error_us = (time_sync_has_prediction != 0U) ?
        ((int64_t)utc_us - (int64_t)time_sync_predicted_edge_utc_us) : 0;
    if (time_sync_synced == 0U)
    {
      time_sync_freq_corr_ppb = 0;
    }

    /* 通常只在首次或失步后写UTC，已同步时仍允许主机主动校正绝对秒数。 */
    /* 包括重复写入在内，UTC始终绑定最近的PPS边沿，不能改绑到报文接收时刻。 */
    time_sync_last_edge_utc_us = utc_us;
    time_sync_utc_base_us = utc_us;
    time_sync_local_base_us = time_sync_last_edge_local_us;
    time_sync_has_utc_base = 1U;
    time_sync_synced = 1U;
  }

  time_sync_wait_utc_frame = 0U;
  time_sync_has_prediction = 0U;
  __set_PRIMASK(irq_mask);
}
