#include "acq_sync.h"

#include <string.h>

#include "app_config.h"
#include "main.h"
#include "modbus_time_sync.h"
#include "sd_log.h"

#define ACQ_SYNC_TOUCH_THREAD_FLAG      (1UL << 8)
/* 每个PPS触发第0点，TIM2补齐剩余124点后停止，等待下一次PPS。 */
#define ACQ_SYNC_SAMPLES_PER_PPS        GLOVE_SENSOR_SAMPLE_RATE_HZ
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
static volatile uint8_t s_acq_sync_pps_callback_pending;
static volatile uint8_t s_acq_sync_has_seen_pps;
static volatile uint8_t s_acq_sync_pps_present;
static volatile AcqSyncState_t s_acq_sync_state;
static volatile uint32_t s_acq_sync_last_pps_tick_ms;
static volatile uint32_t s_acq_sync_pps_edge_count;
static volatile uint64_t s_acq_sync_filtered_pps_interval_us;
static uint32_t s_acq_sync_tim2_counter_hz;
/* 调试入口不持久化；掉线后自动回到等待真实PPS的正常模式。 */
#define ACQ_SYNC_DEBUG_LEASE_MS         (10000U)
#define ACQ_SYNC_MODE_DRAIN_MS          (20U)
static volatile AcqSyncMode_t s_acq_sync_mode;
static volatile uint8_t s_acq_sync_requested_debug;
static volatile uint8_t s_acq_sync_normal_arm_pending;
static volatile uint32_t s_acq_sync_generation;
static volatile uint32_t s_acq_sync_first_seq;
static volatile uint32_t s_acq_sync_mode_tick_ms;
static volatile uint32_t s_acq_sync_stopped_tick_ms;
static volatile uint32_t s_acq_sync_debug_renew_tick_ms;
static volatile uint64_t s_acq_sync_latest_local_us;

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
    uint32_t irq_mask;

    if (snapshot == NULL)
    {
        return;
    }

    irq_mask = __get_PRIMASK();
    __disable_irq();
    *snapshot = s_acq_sync_latest;
    __set_PRIMASK(irq_mask);
}

static void AcqSync_NotifyTouch(void)
{
    if (s_acq_sync_touch_task_id != NULL)
    {
        (void)osThreadFlagsSet(s_acq_sync_touch_task_id, ACQ_SYNC_TOUCH_THREAD_FLAG);
    }
}

static void AcqSync_StopAtLowLevel(void)
{
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
    CLEAR_BIT(htim2.Instance->CR1, TIM_CR1_CEN);
    __HAL_TIM_SET_COUNTER(&htim2, __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));
    s_acq_sync_window_active = 0U;
    s_acq_sync_stop_after_pulse = 0U;
    s_acq_sync_stopped_tick_ms = HAL_GetTick();
}

GloveStatus_t AcqSync_RenewDebugLease(void)
{
    uint32_t irq_mask = __get_PRIMASK();
    uint32_t now_ms;
    GloveStatus_t result = GLOVE_STATUS_NOT_READY;

    __disable_irq();
    now_ms = HAL_GetTick();
    /* 周期服务尚未执行时，也不允许迟到的RENEW复活已到期会话。 */
    if (((s_acq_sync_mode == ACQ_MODE_DEBUG) ||
         ((s_acq_sync_mode == ACQ_MODE_SWITCHING) &&
          (s_acq_sync_requested_debug != 0U))) &&
        ((uint32_t)(now_ms - s_acq_sync_debug_renew_tick_ms) < ACQ_SYNC_DEBUG_LEASE_MS))
    {
        s_acq_sync_debug_renew_tick_ms = now_ms;
        result = GLOVE_STATUS_OK;
    }
    __set_PRIMASK(irq_mask);
    return result;
}

GloveStatus_t AcqSync_RequestDebugMode(uint8_t enabled)
{
    uint32_t irq_mask;
    SdLogStatusSnapshot_t sd_status;

    if (enabled != 0U)
    {
        SdLog_GetStatus(&sd_status);
        if ((sd_status.log_status == SD_LOG_RECORD_RECORDING) ||
            (sd_status.log_status == SD_LOG_RECORD_PREPARING) ||
            (sd_status.log_status == SD_LOG_RECORD_STOPPING))
        {
            return GLOVE_STATUS_NOT_READY;
        }
    }

    irq_mask = __get_PRIMASK();
    __disable_irq();
    if (s_acq_sync_mode == ACQ_MODE_SWITCHING)
    {
        /* STOP可取消尚未完成的START，其他重复申请不重新延长排空时间。 */
        if (enabled == 0U)
        {
            s_acq_sync_requested_debug = 0U;
            __set_PRIMASK(irq_mask);
            return GLOVE_STATUS_OK;
        }
        __set_PRIMASK(irq_mask);
        return GLOVE_STATUS_QUEUE_FULL;
    }
    if (((enabled != 0U) && (s_acq_sync_mode == ACQ_MODE_DEBUG)) ||
        ((enabled == 0U) && (s_acq_sync_mode == ACQ_MODE_NORMAL)))
    {
        if (enabled != 0U)
        {
            GloveStatus_t result = AcqSync_RenewDebugLease();
            __set_PRIMASK(irq_mask);
            return result;
        }
        __set_PRIMASK(irq_mask);
        return GLOVE_STATUS_OK;
    }

    s_acq_sync_requested_debug = (enabled != 0U) ? 1U : 0U;
    s_acq_sync_normal_arm_pending = 0U;
    s_acq_sync_mode = ACQ_MODE_SWITCHING;
    s_acq_sync_state = ACQ_SYNC_STATE_MODE_SWITCH;
    s_acq_sync_mode_tick_ms = HAL_GetTick();
    s_acq_sync_debug_renew_tick_ms = s_acq_sync_mode_tick_ms;
    s_acq_sync_generation++;
    s_acq_sync_first_seq = s_acq_sync_latest.seq + 1U;
    if (s_acq_sync_first_seq == 0U)
    {
        s_acq_sync_first_seq = 1U;
    }
    s_acq_sync_latest.valid = 0U;
    s_acq_sync_latest.utc_valid = 0U;
    s_acq_sync_pps_callback_pending = 0U;
    /* 先切断PPS硬件复位，切换期间不得打断正在输出的同步脉冲。 */
    CLEAR_BIT(htim2.Instance->SMCR, TIM_SMCR_SMS | TIM_SMCR_ECE);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_TRIGGER);
    HAL_NVIC_DisableIRQ(EXTI15_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(PPS_IN_Pin);
    ModbusTimeSync_OnAcquisitionModeChanged();

    if (((htim2.Instance->CR1 & TIM_CR1_CEN) != 0U) &&
        (__HAL_TIM_GET_COUNTER(&htim2) <
         __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2)))
    {
        s_acq_sync_stop_after_pulse = 1U;
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);
        __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
    }
    else
    {
        AcqSync_StopAtLowLevel();
    }
    __set_PRIMASK(irq_mask);
    AcqSync_NotifyTouch();
    return GLOVE_STATUS_OK;
}

static void AcqSync_ArmNormalIfLow(void)
{
    uint32_t irq_mask = __get_PRIMASK();

    __disable_irq();
    if ((s_acq_sync_mode == ACQ_MODE_NORMAL) &&
        (s_acq_sync_normal_arm_pending != 0U) &&
        (HAL_GPIO_ReadPin(PPS_IN_GPIO_Port, PPS_IN_Pin) == GPIO_PIN_RESET))
    {
        /* 先确认PPS为低并清除切换残留，避免AF接通时的高电平被误当成新PPS。 */
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_TRIGGER | TIM_FLAG_CC2);
        HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
        __HAL_TIM_SET_COUNTER(&htim2, __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));
        __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_TRIGGER);
        s_acq_sync_normal_arm_pending = 0U;
        MODIFY_REG(htim2.Instance->SMCR, TIM_SMCR_SMS,
                   TIM_SLAVEMODE_COMBINED_RESETTRIGGER);
    }
    __set_PRIMASK(irq_mask);
}

static void AcqSync_CompleteModeSwitch(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t irq_mask = __get_PRIMASK();
    uint32_t now_ms;
    uint32_t period_ticks;

    __disable_irq();
    now_ms = HAL_GetTick();
    if ((s_acq_sync_mode != ACQ_MODE_SWITCHING) ||
        ((htim2.Instance->CR1 & TIM_CR1_CEN) != 0U) ||
        ((uint32_t)(now_ms - s_acq_sync_stopped_tick_ms) < ACQ_SYNC_MODE_DRAIN_MS))
    {
        __set_PRIMASK(irq_mask);
        return;
    }
    /* 清掉停采前的更新/触发标志；排空阶段迟到的数据由各任务按代次丢弃。 */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_TRIGGER | TIM_FLAG_CC2);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(PPS_IN_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI15_IRQn);
    s_acq_sync_samples_in_window = 0U;
    s_acq_sync_has_seen_pps = 0U;
    s_acq_sync_pps_present = 0U;
    s_acq_sync_pps_edge_count = 0U;
    s_acq_sync_filtered_pps_interval_us = 0ULL;
    s_acq_sync_mode_tick_ms = now_ms;
    period_ticks = s_acq_sync_tim2_counter_hz / GLOVE_SENSOR_SAMPLE_RATE_HZ;
    /* 强制低电平后刷新CCR预装载，避免改周期或软件UG产生额外的IMU上升沿。 */
    MODIFY_REG(htim2.Instance->CCMR1, TIM_CCMR1_OC2M, TIM_CCMR1_OC2M_2);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period_ticks - 1U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period_ticks / 2U);
    htim2.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim2, period_ticks / 2U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_TRIGGER | TIM_FLAG_CC2);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    MODIFY_REG(htim2.Instance->CCMR1, TIM_CCMR1_OC2M,
               TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    gpio.Pin = PPS_IN_Pin;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    if ((s_acq_sync_requested_debug != 0U) &&
        ((uint32_t)(now_ms - s_acq_sync_debug_renew_tick_ms) < ACQ_SYNC_DEBUG_LEASE_MS))
    {
        /* PA15仅监测真实PPS，不再接入TIM2 ETR，外部脉冲不能抢占自由采样。 */
        gpio.Mode = GPIO_MODE_IT_RISING;
        HAL_GPIO_Init(PPS_IN_GPIO_Port, &gpio);
        __HAL_GPIO_EXTI_CLEAR_IT(PPS_IN_Pin);
        HAL_NVIC_SetPriority(EXTI15_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(EXTI15_IRQn);
        s_acq_sync_mode = ACQ_MODE_DEBUG;
        s_acq_sync_state = ACQ_SYNC_STATE_DEBUG_SAMPLING;
        s_acq_sync_window_active = 1U;
        SET_BIT(htim2.Instance->CR1, TIM_CR1_CEN);
    }
    else
    {
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Alternate = GPIO_AF14_TIM2;
        HAL_GPIO_Init(PPS_IN_GPIO_Port, &gpio);
        s_acq_sync_mode = ACQ_MODE_NORMAL;
        s_acq_sync_state = ACQ_SYNC_STATE_WAIT_FIRST_PPS;
        s_acq_sync_normal_arm_pending = 1U;
        AcqSync_ArmNormalIfLow();
        /* 正常模式不主动置CEN，必须由新的真实PPS启动。 */
    }
    __set_PRIMASK(irq_mask);
    AcqSync_NotifyTouch();
}

uint8_t AcqSync_IsDebugMode(void)
{
    return (s_acq_sync_mode == ACQ_MODE_DEBUG) ? 1U : 0U;
}

uint8_t AcqSync_IsNormalMode(void)
{
    return (s_acq_sync_mode == ACQ_MODE_NORMAL) ? 1U : 0U;
}

uint8_t AcqSync_IsSamplingAllowed(void)
{
    return ((s_acq_sync_mode == ACQ_MODE_DEBUG) ||
            ((s_acq_sync_mode == ACQ_MODE_NORMAL) &&
             (s_acq_sync_pps_present != 0U))) ? 1U : 0U;
}

uint8_t AcqSync_IsSequenceCurrent(uint32_t seq)
{
    uint8_t current;
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    current = ((seq != 0U) && (s_acq_sync_latest.valid != 0U) &&
               ((uint32_t)(seq - s_acq_sync_first_seq) <=
                (uint32_t)(s_acq_sync_latest.seq - s_acq_sync_first_seq)) &&
               (AcqSync_IsSamplingAllowed() != 0U)) ? 1U : 0U;
    __set_PRIMASK(irq_mask);
    return current;
}

void AcqSync_OnDebugPpsEdgeFromIsr(void)
{
    uint32_t now_ms = HAL_GetTick();
    if (s_acq_sync_mode != ACQ_MODE_DEBUG)
    {
        return;
    }
    /* 仅反映真实上升沿；短间隔尖峰不能续命PPS_PRESENT，更不能建立UTC。 */
    if ((s_acq_sync_has_seen_pps == 0U) ||
        ((uint32_t)(now_ms - s_acq_sync_last_pps_tick_ms) >= 900U))
    {
        s_acq_sync_has_seen_pps = 1U;
        s_acq_sync_pps_present = 1U;
        s_acq_sync_last_pps_tick_ms = now_ms;
    }
}

void AcqSync_Reset(void)
{
    uint32_t timer_clock_hz = AcqSync_GetApb1TimerClockHz();
    uint32_t irq_mask = __get_PRIMASK();

    __disable_irq();
    (void)memset((void *)&s_acq_sync_latest, 0, sizeof(s_acq_sync_latest));
    s_acq_sync_touch_task_id = NULL;
    s_acq_sync_samples_in_window = 0U;
    s_acq_sync_window_active = 0U;
    s_acq_sync_stop_after_pulse = 0U;
    s_acq_sync_pps_callback_pending = 0U;
    s_acq_sync_has_seen_pps = 0U;
    s_acq_sync_pps_present = 0U;
    s_acq_sync_state = ACQ_SYNC_STATE_WAIT_FIRST_PPS;
    s_acq_sync_last_pps_tick_ms = 0U;
    s_acq_sync_pps_edge_count = 0U;
    s_acq_sync_filtered_pps_interval_us = 0ULL;
    s_acq_sync_tim2_counter_hz = timer_clock_hz / (htim2.Init.Prescaler + 1U);
    s_acq_sync_mode = ACQ_MODE_NORMAL;
    s_acq_sync_requested_debug = 0U;
    s_acq_sync_normal_arm_pending = 0U;
    s_acq_sync_generation = 0U;
    s_acq_sync_first_seq = 1U;
    s_acq_sync_mode_tick_ms = HAL_GetTick();
    s_acq_sync_latest_local_us = 0ULL;
    __set_PRIMASK(irq_mask);
}

void AcqSync_Service(void)
{
    uint32_t now_ms;
    uint32_t last_pps_ms;
    uint32_t timeout_ms;
    uint64_t expected_interval_us;
    uint32_t irq_mask;
    uint8_t mark_lost = 0U;

    if (s_acq_sync_mode == ACQ_MODE_SWITCHING)
    {
        AcqSync_CompleteModeSwitch();
        return;
    }
    if (s_acq_sync_mode == ACQ_MODE_DEBUG)
    {
        irq_mask = __get_PRIMASK();
        __disable_irq();
        now_ms = HAL_GetTick();
        /* 续期、PPS中断和超时退出必须串行，不能用旧时间误判新会话。 */
        if (s_acq_sync_mode != ACQ_MODE_DEBUG)
        {
            __set_PRIMASK(irq_mask);
            return;
        }
        if ((uint32_t)(now_ms - s_acq_sync_debug_renew_tick_ms) >=
            ACQ_SYNC_DEBUG_LEASE_MS)
        {
            (void)AcqSync_RequestDebugMode(0U);
        }
        else if ((s_acq_sync_has_seen_pps != 0U) &&
                 ((uint32_t)(now_ms - s_acq_sync_last_pps_tick_ms) > 1050U))
        {
            s_acq_sync_pps_present = 0U;
        }
        __set_PRIMASK(irq_mask);
        return;
    }
    if (s_acq_sync_normal_arm_pending != 0U)
    {
        AcqSync_ArmNormalIfLow();
        return;
    }
    if (s_acq_sync_has_seen_pps == 0U)
    {
        return;
    }

    irq_mask = __get_PRIMASK();
    __disable_irq();
    /* 当前时间与上次PPS必须一起读取，避免中断抢占后无符号时间差下溢。 */
    now_ms = HAL_GetTick();
    last_pps_ms = s_acq_sync_last_pps_tick_ms;
    expected_interval_us = s_acq_sync_filtered_pps_interval_us;
    __set_PRIMASK(irq_mask);

    if (expected_interval_us == 0ULL)
    {
        expected_interval_us = ACQ_SYNC_DEFAULT_PPS_INTERVAL_US;
    }
    timeout_ms = (uint32_t)((expected_interval_us + 999ULL) / 1000ULL) +
                 ACQ_SYNC_PPS_LOSS_GUARD_MS;

    if ((uint32_t)(now_ms - last_pps_ms) > timeout_ms)
    {
        irq_mask = __get_PRIMASK();
        __disable_irq();
        /* 判定前重读当前时间，期间到来的新PPS不能被旧时间误判为丢失。 */
        now_ms = HAL_GetTick();
        if ((s_acq_sync_mode == ACQ_MODE_NORMAL) &&
            (s_acq_sync_has_seen_pps != 0U) &&
            ((uint32_t)(now_ms - s_acq_sync_last_pps_tick_ms) > timeout_ms) &&
            (s_acq_sync_state != ACQ_SYNC_STATE_PPS_LOST))
        {
            s_acq_sync_state = ACQ_SYNC_STATE_PPS_LOST;
            s_acq_sync_pps_present = 0U;
            s_acq_sync_window_active = 0U;
            s_acq_sync_stop_after_pulse = 0U;
            s_acq_sync_latest.valid = 0U;
            /* 恢复PPS后也不能补发丢失前尚未处理完的旧帧。 */
            s_acq_sync_generation++;
            s_acq_sync_first_seq = s_acq_sync_latest.seq + 1U;
            if (s_acq_sync_first_seq == 0U)
            {
                s_acq_sync_first_seq = 1U;
            }
            /* 与状态切换保持原子性，避免临界点的新PPS刚启动TIM2又被任务停掉。 */
            __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
            CLEAR_BIT(htim2.Instance->CR1, TIM_CR1_CEN);
            /* 即使异常路径在高电平阶段停表，也把PWM参考状态恢复为低电平。 */
            __HAL_TIM_SET_COUNTER(&htim2,
                                  __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));
            ModbusTimeSync_OnPpsLost();
            mark_lost = 1U;
        }
        __set_PRIMASK(irq_mask);
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
    uint32_t irq_mask = __get_PRIMASK();

    __disable_irq();
    s_acq_sync_touch_task_id = thread_id;
    __set_PRIMASK(irq_mask);
}

void AcqSync_OnTim2PeriodElapsedFromIsr(void)
{
    uint8_t pps_triggered;
    uint32_t next_seq = s_acq_sync_latest.seq + 1U;

    if (s_acq_sync_mode == ACQ_MODE_SWITCHING)
    {
        return;
    }
    if (next_seq == 0U)
    {
        next_seq = 1U;
    }
    s_acq_sync_latest_local_us = ModbusTimeSync_GetLocalUptimeUs();
    if (s_acq_sync_mode == ACQ_MODE_DEBUG)
    {
        s_acq_sync_latest.seq = next_seq;
        s_acq_sync_latest.timestamp_us = s_acq_sync_latest_local_us;
        s_acq_sync_latest.utc_valid = 0U;
        s_acq_sync_latest.debug_mode = 1U;
        s_acq_sync_latest.valid = 1U;
        AcqSync_NotifyTouch();
        return;
    }
    s_acq_sync_latest.debug_mode = 0U;
    /* 组合复位触发模式会在PPS到来时同时置位TRG和更新标志。 */
    pps_triggered = (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_TRIGGER) != RESET) ? 1U : 0U;
    if (pps_triggered != 0U)
    {
        /* HAL先回调更新、后回调触发；先推进UTC再给第0帧打时间戳。 */
        AcqSync_OnTim2PpsTriggerFromIsr();
        s_acq_sync_pps_callback_pending = 1U;
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

    s_acq_sync_latest.timestamp_us = (pps_triggered != 0U) ?
        (GloveTimestampUs_t)ModbusTimeSync_GetPpsEdgeUtcUsFromIsr() :
        (GloveTimestampUs_t)ModbusTimeSync_GetUtcTimestampUsFromIsr();
    s_acq_sync_latest.utc_valid = ModbusTimeSync_IsSynced();
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
    if (s_acq_sync_mode != ACQ_MODE_NORMAL)
    {
        return;
    }
    if (s_acq_sync_pps_callback_pending != 0U)
    {
        /* 同一个PPS已在更新回调中处理，触发回调只消除待处理标记。 */
        s_acq_sync_pps_callback_pending = 0U;
        return;
    }

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
    s_acq_sync_stopped_tick_ms = HAL_GetTick();
    if (s_acq_sync_mode == ACQ_MODE_SWITCHING)
    {
        return;
    }
    if (s_acq_sync_pps_present != 0U)
    {
        s_acq_sync_state = ACQ_SYNC_STATE_WAIT_NEXT_PPS;
    }
}

uint8_t AcqSync_GetLatest(AcqSyncSnapshot_t *snapshot)
{
    AcqSync_CopyLatest(snapshot);

    return ((snapshot != NULL) && (snapshot->valid != 0U) &&
            (AcqSync_IsSamplingAllowed() != 0U)) ? 1U : 0U;
}

void AcqSync_GetStatus(AcqSyncStatus_t *status)
{
    uint32_t now_ms;
    uint64_t expected_interval_us;
    uint32_t irq_mask;

    if (status == NULL)
    {
        return;
    }

    irq_mask = __get_PRIMASK();
    __disable_irq();
    now_ms = HAL_GetTick();
    status->state = s_acq_sync_state;
    status->mode = s_acq_sync_mode;
    status->generation = s_acq_sync_generation;
    status->latest_sample_seq = s_acq_sync_latest.seq;
    status->latest_sample_local_us = s_acq_sync_latest_local_us;
    status->mode_age_ms = (uint32_t)(now_ms - s_acq_sync_mode_tick_ms);
    status->sampling_allowed = AcqSync_IsSamplingAllowed();
    status->debug_lease_remaining_ms = 0U;
    if ((s_acq_sync_mode == ACQ_MODE_DEBUG) ||
        ((s_acq_sync_mode == ACQ_MODE_SWITCHING) && (s_acq_sync_requested_debug != 0U)))
    {
        uint32_t age_ms = (uint32_t)(now_ms - s_acq_sync_debug_renew_tick_ms);
        if (age_ms < ACQ_SYNC_DEBUG_LEASE_MS)
        {
            status->debug_lease_remaining_ms = (uint16_t)(ACQ_SYNC_DEBUG_LEASE_MS - age_ms);
        }
    }
    status->samples_in_window = s_acq_sync_samples_in_window;
    status->pps_present = s_acq_sync_pps_present;
    status->window_active = s_acq_sync_window_active;
    status->has_seen_pps = s_acq_sync_has_seen_pps;
    status->last_pps_age_ms = (s_acq_sync_has_seen_pps != 0U) ?
                              (uint32_t)(now_ms - s_acq_sync_last_pps_tick_ms) :
                              0xFFFFFFFFUL;
    expected_interval_us = s_acq_sync_filtered_pps_interval_us;
    __set_PRIMASK(irq_mask);

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
    return ((s_acq_sync_state == ACQ_SYNC_STATE_SAMPLING) ||
            (s_acq_sync_state == ACQ_SYNC_STATE_DEBUG_SAMPLING)) ? 1U : 0U;
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
    if ((AcqSync_IsSamplingAllowed() != 0U) &&
        (latest.valid != 0U) &&
        ((snapshot->valid == 0U) || (latest.seq != last_seq)))
    {
        *snapshot = latest;
        return osOK;
    }

    /* 清标志后再次核对序号，避免同步中断恰好发生在清标志前而丢帧。 */
    (void)osThreadFlagsClear(ACQ_SYNC_TOUCH_THREAD_FLAG);
    AcqSync_CopyLatest(&latest);
    if ((AcqSync_IsSamplingAllowed() != 0U) &&
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
    return ((snapshot->valid != 0U) && (AcqSync_IsSamplingAllowed() != 0U)) ?
           osOK : osError;
}
