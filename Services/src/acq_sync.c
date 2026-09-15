#include "acq_sync.h"

#include <string.h>

#include "main.h"
#include "modbus_time_sync.h"

#define ACQ_SYNC_TOUCH_THREAD_FLAG      (1UL << 8)
#define ACQ_SYNC_SAMPLES_PER_PPS        (200U)
#define ACQ_SYNC_PPS_INTERVAL_MIN_US    (900000ULL)
#define ACQ_SYNC_PPS_INTERVAL_MAX_US    (1100000ULL)
#define ACQ_SYNC_DEFAULT_PPS_INTERVAL_US (1000000ULL)
#define ACQ_SYNC_PPS_LOSS_GUARD_MS      (50U)
#define ACQ_SYNC_PERIOD_FILTER_WEIGHT   (7ULL)
#define ACQ_SYNC_PERIOD_FILTER_DIVISOR  (8ULL)

static volatile AcqSyncSnapshot_t s_acq_sync_latest;
static osThreadId_t s_acq_sync_touch_task_id;
static volatile uint16_t s_acq_sync_samples_in_window;
static volatile uint8_t s_acq_sync_window_active;
static volatile uint8_t s_acq_sync_stop_after_pulse;
static volatile uint8_t s_acq_sync_has_seen_pps;
static volatile uint8_t s_acq_sync_pps_present;
static volatile AcqSyncState_t s_acq_sync_state;
static volatile uint32_t s_acq_sync_last_pps_tick_ms;
static volatile uint32_t s_acq_sync_pps_edge_count;
static volatile uint64_t s_acq_sync_filtered_pps_interval_us;
static uint32_t s_acq_sync_tim2_counter_hz;

static uint32_t AcqSync_GetApb1TimerClockHz(void)
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

static void AcqSync_UpdateTim2PeriodFromPps(uint64_t interval_us)
{
    uint64_t period_ticks;

    if ((interval_us < ACQ_SYNC_PPS_INTERVAL_MIN_US) ||
        (interval_us > ACQ_SYNC_PPS_INTERVAL_MAX_US))
    {
        return;
    }

    if (s_acq_sync_filtered_pps_interval_us == 0ULL)
    {
        s_acq_sync_filtered_pps_interval_us = interval_us;
    }
    else
    {
        /* 对PPS周期做低通滤波，避免单次输入抖动直接改变全部采样间隔。 */
        s_acq_sync_filtered_pps_interval_us =
            ((s_acq_sync_filtered_pps_interval_us * ACQ_SYNC_PERIOD_FILTER_WEIGHT) +
             interval_us + (ACQ_SYNC_PERIOD_FILTER_DIVISOR / 2ULL)) /
            ACQ_SYNC_PERIOD_FILTER_DIVISOR;
    }

    /* 根据实际TIM2计数频率换算，修改系统时钟后不会继续沿用固定的250倍比例。 */
    period_ticks =
        ((s_acq_sync_filtered_pps_interval_us * (uint64_t)s_acq_sync_tim2_counter_hz) +
         ((uint64_t)ACQ_SYNC_SAMPLES_PER_PPS * 500000ULL)) /
        ((uint64_t)ACQ_SYNC_SAMPLES_PER_PPS * 1000000ULL);
    if ((period_ticks < 2ULL) || (period_ticks > 0xFFFFFFFFULL))
    {
        return;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)(period_ticks - 1ULL));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(period_ticks / 2ULL));
}

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
    uint32_t timer_clock_hz = AcqSync_GetApb1TimerClockHz();

    __disable_irq();
    (void)memset((void *)&s_acq_sync_latest, 0, sizeof(s_acq_sync_latest));
    s_acq_sync_touch_task_id = NULL;
    s_acq_sync_samples_in_window = 0U;
    s_acq_sync_window_active = 0U;
    s_acq_sync_stop_after_pulse = 0U;
    s_acq_sync_has_seen_pps = 0U;
    s_acq_sync_pps_present = 0U;
    s_acq_sync_state = ACQ_SYNC_STATE_WAIT_FIRST_PPS;
    s_acq_sync_last_pps_tick_ms = 0U;
    s_acq_sync_pps_edge_count = 0U;
    s_acq_sync_filtered_pps_interval_us = 0ULL;
    s_acq_sync_tim2_counter_hz = timer_clock_hz / (htim2.Init.Prescaler + 1U);
    __enable_irq();
}

void AcqSync_Service(void)
{
    uint32_t now_ms;
    uint32_t last_pps_ms;
    uint32_t timeout_ms;
    uint64_t expected_interval_us;
    uint8_t mark_lost = 0U;

    if (s_acq_sync_has_seen_pps == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    __disable_irq();
    last_pps_ms = s_acq_sync_last_pps_tick_ms;
    expected_interval_us = s_acq_sync_filtered_pps_interval_us;
    __enable_irq();

    if (expected_interval_us == 0ULL)
    {
        expected_interval_us = ACQ_SYNC_DEFAULT_PPS_INTERVAL_US;
    }
    timeout_ms = (uint32_t)((expected_interval_us + 999ULL) / 1000ULL) +
                 ACQ_SYNC_PPS_LOSS_GUARD_MS;

    if ((uint32_t)(now_ms - last_pps_ms) > timeout_ms)
    {
        __disable_irq();
        if ((s_acq_sync_has_seen_pps != 0U) &&
            ((uint32_t)(now_ms - s_acq_sync_last_pps_tick_ms) > timeout_ms) &&
            (s_acq_sync_state != ACQ_SYNC_STATE_PPS_LOST))
        {
            s_acq_sync_state = ACQ_SYNC_STATE_PPS_LOST;
            s_acq_sync_pps_present = 0U;
            s_acq_sync_window_active = 0U;
            s_acq_sync_stop_after_pulse = 0U;
            s_acq_sync_latest.valid = 0U;
            /* 与状态切换保持原子性，避免临界点的新PPS刚启动TIM2又被任务停掉。 */
            __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
            CLEAR_BIT(htim2.Instance->CR1, TIM_CR1_CEN);
            /* 即使异常路径在高电平阶段停表，也把PWM参考状态恢复为低电平。 */
            __HAL_TIM_SET_COUNTER(&htim2,
                                  __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));
            ModbusTimeSync_OnPpsLost();
            mark_lost = 1U;
        }
        __enable_irq();
    }

    if (mark_lost != 0U)
    {
        if (s_acq_sync_touch_task_id != NULL)
        {
            (void)osThreadFlagsSet(s_acq_sync_touch_task_id, ACQ_SYNC_TOUCH_THREAD_FLAG);
        }
    }
}

void AcqSync_RegisterTouchTask(osThreadId_t thread_id)
{
    __disable_irq();
    s_acq_sync_touch_task_id = thread_id;
    __enable_irq();
}

void AcqSync_OnTim2PeriodElapsedFromIsr(void)
{
    uint8_t pps_triggered;
    uint32_t next_seq = s_acq_sync_latest.seq + 1U;

    /* 组合复位触发模式会在PPS到来时同时置位TRG和更新标志。 */
    pps_triggered = (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_TRIGGER) != RESET) ? 1U : 0U;
    if (pps_triggered != 0U)
    {
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);
        s_acq_sync_stop_after_pulse = 0U;
        s_acq_sync_samples_in_window = 1U;
        s_acq_sync_window_active = 1U;
        s_acq_sync_has_seen_pps = 1U;
        s_acq_sync_pps_present = 1U;
        s_acq_sync_state = ACQ_SYNC_STATE_SAMPLING;
    }
    else
    {
        if ((s_acq_sync_window_active == 0U) ||
            (s_acq_sync_samples_in_window >= ACQ_SYNC_SAMPLES_PER_PPS))
        {
            return;
        }
        s_acq_sync_samples_in_window++;
    }

    s_acq_sync_latest.timestamp_us =
        (GloveTimestampUs_t)ModbusTimeSync_GetUtcTimestampUsFromIsr();
    s_acq_sync_latest.seq = next_seq;
    s_acq_sync_latest.valid = 1U;

    if (s_acq_sync_touch_task_id != NULL)
    {
        (void)osThreadFlagsSet(s_acq_sync_touch_task_id, ACQ_SYNC_TOUCH_THREAD_FLAG);
    }

    if (s_acq_sync_samples_in_window >= ACQ_SYNC_SAMPLES_PER_PPS)
    {
        /* 等本帧IMU同步脉冲回到低电平后停表，下一次PPS再由硬件重新启动。 */
        s_acq_sync_stop_after_pulse = 1U;
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);
        __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
    }
}

void AcqSync_OnTim2PpsTriggerFromIsr(void)
{
    s_acq_sync_last_pps_tick_ms = HAL_GetTick();
    s_acq_sync_has_seen_pps = 1U;
    s_acq_sync_pps_present = 1U;
    s_acq_sync_state = ACQ_SYNC_STATE_SAMPLING;
    ModbusTimeSync_OnPpsEdge(PPS_IN_Pin);
    s_acq_sync_pps_edge_count++;
    if (s_acq_sync_pps_edge_count >= 2U)
    {
        /* 第一个PPS只建立周期起点，从第二个PPS开始才有完整的1秒测量值。 */
        AcqSync_UpdateTim2PeriodFromPps(ModbusTimeSync_GetLastLocalIntervalUsFromIsr());
    }
}

void AcqSync_OnTim2PulseFinishedFromIsr(void)
{
    if (s_acq_sync_stop_after_pulse == 0U)
    {
        return;
    }

    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);

    /* CH2在比较点已经回到低电平，直接停计数器可保证下一次PPS产生新的上升沿。 */
    CLEAR_BIT(htim2.Instance->CR1, TIM_CR1_CEN);
    s_acq_sync_window_active = 0U;
    s_acq_sync_stop_after_pulse = 0U;
    if (s_acq_sync_pps_present != 0U)
    {
        s_acq_sync_state = ACQ_SYNC_STATE_WAIT_NEXT_PPS;
    }
}

uint8_t AcqSync_GetLatest(AcqSyncSnapshot_t *snapshot)
{
    AcqSync_CopyLatest(snapshot);

    return ((snapshot != NULL) && (snapshot->valid != 0U) &&
            (s_acq_sync_pps_present != 0U)) ? 1U : 0U;
}

void AcqSync_GetStatus(AcqSyncStatus_t *status)
{
    uint32_t now_ms;
    uint64_t expected_interval_us;

    if (status == NULL)
    {
        return;
    }

    now_ms = HAL_GetTick();
    __disable_irq();
    status->state = s_acq_sync_state;
    status->samples_in_window = s_acq_sync_samples_in_window;
    status->pps_present = s_acq_sync_pps_present;
    status->window_active = s_acq_sync_window_active;
    status->has_seen_pps = s_acq_sync_has_seen_pps;
    status->last_pps_age_ms = (s_acq_sync_has_seen_pps != 0U) ?
                              (uint32_t)(now_ms - s_acq_sync_last_pps_tick_ms) :
                              0xFFFFFFFFUL;
    expected_interval_us = s_acq_sync_filtered_pps_interval_us;
    __enable_irq();

    if (expected_interval_us == 0ULL)
    {
        expected_interval_us = ACQ_SYNC_DEFAULT_PPS_INTERVAL_US;
    }
    status->expected_pps_interval_us =
        (expected_interval_us > 0xFFFFFFFFULL) ?
        0xFFFFFFFFUL : (uint32_t)expected_interval_us;
}

uint8_t AcqSync_IsPpsPresent(void)
{
    return s_acq_sync_pps_present;
}

uint8_t AcqSync_IsSamplingActive(void)
{
    return (s_acq_sync_state == ACQ_SYNC_STATE_SAMPLING) ? 1U : 0U;
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
    if ((s_acq_sync_pps_present != 0U) &&
        (latest.valid != 0U) &&
        ((snapshot->valid == 0U) || (latest.seq != last_seq)))
    {
        *snapshot = latest;
        return osOK;
    }

    /* 清标志后再次核对序号，避免同步中断恰好发生在清标志前而丢帧。 */
    (void)osThreadFlagsClear(ACQ_SYNC_TOUCH_THREAD_FLAG);
    AcqSync_CopyLatest(&latest);
    if ((s_acq_sync_pps_present != 0U) &&
        (latest.valid != 0U) &&
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
    return ((snapshot->valid != 0U) && (s_acq_sync_pps_present != 0U)) ?
           osOK : osError;
}
