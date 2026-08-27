#include "systemManagerTask.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "acq_sync.h"
#include "bq25622.h"
#include "data_manager.h"
#include "dataProcessTask.h"
#include "i2c_bus.h"
#include "imuCanTask.h"
#include "main.h"
#include "max17043.h"
#include "modbus_frame.h"
#include "storageTask.h"
#include "system_health.h"
#include "system_watchdog.h"
#include "touchAdcTask.h"

extern TIM_HandleTypeDef htim2;

#define SYSTEM_MANAGER_LOOP_PERIOD_MS                (10U)
#define SYSTEM_MANAGER_BQ_READ_PERIOD_MS             (250U)
#define SYSTEM_MANAGER_GAUGE_READ_PERIOD_MS          (1000U)
#define SYSTEM_MANAGER_BATTERY_STARTUP_DELAY_MS      (600U)
#define SYSTEM_MANAGER_BQ_RETRY_PERIOD_MS            (1000U)
#define SYSTEM_MANAGER_BQ_FAILURE_LIMIT              (3U)
#define SYSTEM_MANAGER_BQ25622_TIMEOUT_MS            (20U)
#define SYSTEM_MANAGER_BQ25622_INPUT_CURRENT_MA      (2500U)
#define SYSTEM_MANAGER_BQ25622_CHARGE_VOLTAGE_MV     (4400U)
/* BQ25622以80mA为步进，1440mA是不超过1.5A的最近可配置值。 */
#define SYSTEM_MANAGER_BQ25622_CHARGE_CURRENT_MA     (1440U)
#define SYSTEM_MANAGER_BQ25622_TERMINATION_CURRENT_MA (100U)
#define SYSTEM_MANAGER_STORAGE_STOP_TIMEOUT_MS        (6000U)
#define SYSTEM_MANAGER_MAX17043_TIMEOUT_MS           (20U)
#define SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS         (20U)
#define SYSTEM_MANAGER_POWER_KEY_LONG_PRESS_MS       (600U)
#define SYSTEM_MANAGER_POWER_KEY_PRESSED_LEVEL       GPIO_PIN_RESET
#define SYSTEM_MANAGER_POWER_STOP_TIMEOUT_MS         (500U)
#define SYSTEM_MANAGER_RECOVERY_TIMEOUT_MS           (30000U)
#define SYSTEM_MANAGER_AUTO_RECOVERY_OFF_HOLD_MS     (100U)
#define SYSTEM_MANAGER_LOW_VOLTAGE_MV                (3300U)
#define SYSTEM_MANAGER_LOW_RELEASE_VOLTAGE_MV        (3650U)
#define SYSTEM_MANAGER_CRITICAL_VOLTAGE_MV           (3200U)
#define SYSTEM_MANAGER_EMERGENCY_VOLTAGE_MV          (3050U)
#define SYSTEM_MANAGER_LOCKOUT_RELEASE_VOLTAGE_MV    (3500U)
#define SYSTEM_MANAGER_LOW_SOC_PERCENT               (20U)
#define SYSTEM_MANAGER_LOW_SOC_RELEASE_PERCENT       (25U)
/* MAX17043尚未加载本电芯模型，SOC仅显示，不参与低电状态判断。 */
#define SYSTEM_MANAGER_SOC_LOW_DETECTION_ENABLE      (0U)
#define SYSTEM_MANAGER_LOW_CONFIRM_SAMPLES           (5U)
#define SYSTEM_MANAGER_LOW_RELEASE_SAMPLES           (10U)
#define SYSTEM_MANAGER_CRITICAL_CONFIRM_SAMPLES      (3U)
#define SYSTEM_MANAGER_LOCKOUT_RELEASE_SAMPLES       (10U)
#define SYSTEM_MANAGER_VOLTAGE_MISMATCH_MV           (100U)
#define SYSTEM_MANAGER_FULL_MIN_VOLTAGE_MV            (4300U)
#define SYSTEM_MANAGER_FULL_RELEASE_VOLTAGE_MV        (4200U)
#define SYSTEM_MANAGER_FULL_CONFIRM_SAMPLES           (3U)
#define SYSTEM_MANAGER_FULL_CURRENT_MARGIN_MA         (40U)

static GloveBatteryStatus_t s_battery_status;
static GlovePowerStatus_t s_power_status;
static Bq25622Handle_t s_bq25622;
static Max17043Handle_t s_max17043;
static Bq25622StatusSnapshot_t s_bq_snapshot;
static Bq25622InterruptFlags_t s_bq_last_events;
static Max17043BatteryData_t s_gauge_data;
static volatile uint8_t s_periph_power_enabled = 1U;
static volatile uint8_t s_power_key_off_wake_pulse_seen;
static volatile uint8_t s_power_key_release_edge_seen;
static volatile uint8_t s_power_key_off_wake_armed = 1U;
static volatile uint8_t s_power_key_ignore_until_release;
static volatile uint8_t s_bq_interrupt_pending;
static volatile uint8_t s_charge_status_edge_pending;
static volatile uint32_t s_bq_interrupt_count;
static volatile uint8_t s_power_key_press_edge_valid;
static volatile uint32_t s_power_key_press_edge_ms;
static uint8_t s_power_key_last_raw_pressed;
static uint8_t s_power_key_debounced_pressed;
static uint8_t s_power_key_long_handled;
static uint32_t s_power_key_changed_ms;
static uint32_t s_power_key_pressed_ms;
static uint32_t s_power_key_last_trigger_elapsed_ms;
static uint32_t s_power_key_last_poweroff_elapsed_ms;
static uint8_t s_bq_ready;
static uint8_t s_bq_valid;
static uint8_t s_gauge_valid;
static uint8_t s_bq_failures;
static uint8_t s_gauge_failures;
static uint8_t s_charge_allowed;
static uint8_t s_charge_was_active;
static uint8_t s_charge_change_event_pending;
static uint8_t s_full_confirm_count;
static uint8_t s_full_latched;
static uint8_t s_bq_diagnostic_stage;
static GloveStatus_t s_bq_last_status = GLOVE_STATUS_OK;
static uint8_t s_low_voltage_count;
static uint8_t s_low_soc_count;
static uint8_t s_low_release_count;
static uint8_t s_critical_count;
static uint8_t s_critical_release_count;
static uint8_t s_lockout_release_count;
static uint8_t s_power_stop_pending;
static GlovePowerState_t s_power_stop_target_state;
static uint32_t s_recovery_started_ms;
static uint32_t s_recovery_full_frame_baseline;
static uint8_t s_recovery_producers_ready;
static volatile uint8_t s_auto_recovery_request_pending;
static volatile uint8_t s_auto_recovery_active;
static uint8_t s_auto_recovery_waiting_power_on;
static uint32_t s_auto_recovery_power_off_ms;

static uint32_t SystemManager_MsToTicks(uint32_t timeout_ms)
{
    uint64_t ticks = ((uint64_t)timeout_ms * osKernelGetTickFreq() + 999ULL) / 1000ULL;
    if ((timeout_ms > 0U) && (ticks == 0ULL)) ticks = 1ULL;
    return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

static uint32_t SystemManager_GetTimestampMs(void)
{
    uint32_t frequency = osKernelGetTickFreq();
    return (frequency == 0U) ? 0U :
           (uint32_t)(((uint64_t)osKernelGetTickCount() * 1000ULL) / frequency);
}

static uint16_t SystemManager_AbsDiffU16(uint16_t left, uint16_t right)
{
    return (left >= right) ? (uint16_t)(left - right) : (uint16_t)(right - left);
}

static void SystemManager_SetImuSyncPinAnalog(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = IMU_SYNC_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IMU_SYNC_GPIO_Port, &gpio);
}

static void SystemManager_StopAcquisitionSync(void)
{
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    SystemManager_SetImuSyncPinAnalog();
    AcqSync_Reset();
}

static uint8_t SystemManager_StartAcquisitionSync(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = IMU_SYNC_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(IMU_SYNC_GPIO_Port, &gpio);

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
    {
        SystemManager_SetImuSyncPinAnalog();
        return 0U;
    }
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    return 1U;
}

static void SystemManager_SetChargeAllowed(uint8_t allowed)
{
    s_charge_allowed = (allowed != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(DISABLE_CHARGE_GPIO_Port,
                      DISABLE_CHARGE_Pin,
                      (s_charge_allowed != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint8_t SystemManager_WaitAcquisitionPaused(uint32_t timeout_ms)
{
    uint32_t start = SystemManager_GetTimestampMs();
    while ((ImuCanTask_IsAcquisitionPaused() == 0U) ||
           (TouchAdcTask_IsAcquisitionPaused() == 0U))
    {
        if ((uint32_t)(SystemManager_GetTimestampMs() - start) >= timeout_ms)
        {
            return 0U;
        }
        osDelay(SystemManager_MsToTicks(5U));
    }
    return 1U;
}

static uint8_t SystemManager_TryFinalizePeripheralStop(void)
{
    GloveStatus_t storage_status;

    if (s_power_stop_pending == 0U)
    {
        return 1U;
    }
    if ((ImuCanTask_IsAcquisitionPaused() == 0U) ||
        (TouchAdcTask_IsAcquisitionPaused() == 0U))
    {
        return 0U;
    }

    /* 采集源已停止后先落盘并关闭文件，再清队列和断外设电源。 */
    storage_status = StorageTask_RequestStop(SYSTEM_MANAGER_STORAGE_STOP_TIMEOUT_MS);
    if (storage_status != GLOVE_STATUS_OK)
    {
        SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_SD_ERROR,
                              SYSTEM_ERROR_SD,
                              SYSTEM_HEALTH_SOURCE_STORAGE,
                              (uint16_t)storage_status,
                              1U);
    }

    SystemManager_StopAcquisitionSync();
    DataManager_FlushAcquisitionQueues();
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin, GPIO_PIN_RESET);
    s_periph_power_enabled = 0U;
    s_power_status.system_state = (uint8_t)s_power_stop_target_state;
    s_power_stop_pending = 0U;
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_POWER_RECOVERY_FAIL,
                          SYSTEM_ERROR_ACQ_PAUSE_TIMEOUT,
                          SYSTEM_HEALTH_SOURCE_POWER,
                          0U,
                          0U);
    SystemHealth_SetPowerRecovery((s_auto_recovery_active != 0U) ?
                                  SYSTEM_RECOVERY_POWER_OFF_HOLD :
                                  SYSTEM_RECOVERY_NONE);
    return 1U;
}

static uint8_t SystemManager_StopPeripheralPower(GlovePowerState_t target_state)
{
    if (s_power_stop_pending != 0U)
    {
        if (target_state == GLOVE_POWER_STATE_LOW_BAT_LOCKOUT)
        {
            s_power_stop_target_state = target_state;
        }
        return SystemManager_TryFinalizePeripheralStop();
    }

    if (s_periph_power_enabled != 0U)
    {
        s_power_stop_target_state = target_state;
        s_power_stop_pending = 1U;
        s_power_status.system_state = GLOVE_POWER_STATE_STOPPING;
        SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_SAFE_STOP);
        Modbus_InvalidateSensorSnapshots();
        ImuCanTask_SetAcquisitionEnabled(0U);
        TouchAdcTask_SetAcquisitionEnabled(0U);
        if (SystemManager_WaitAcquisitionPaused(SYSTEM_MANAGER_POWER_STOP_TIMEOUT_MS) == 0U)
        {
            /* 未安全暂停时保持供电，由主循环继续等待，禁止强制切断外设电源。 */
            SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_POWER_RECOVERY_FAIL,
                                  SYSTEM_ERROR_ACQ_PAUSE_TIMEOUT,
                                  SYSTEM_HEALTH_SOURCE_POWER,
                                  0U,
                                  1U);
            SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_FAILED);
            return 0U;
        }
        return SystemManager_TryFinalizePeripheralStop();
    }
    s_power_status.system_state = (uint8_t)target_state;
    return 1U;
}

static void SystemManager_StartPeripheralPower(void)
{
    if (s_periph_power_enabled == 0U)
    {
        SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_POWER_START);
        Modbus_InvalidateSensorSnapshots();
        HAL_GPIO_WritePin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
        osDelay(SystemManager_MsToTicks(10U));
        HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
        osDelay(SystemManager_MsToTicks(100U));
        AcqSync_Reset();
        if (SystemManager_StartAcquisitionSync() == 0U)
        {
            /* 同步时钟启动失败时重新关闭电源，禁止无同步条件下进入采集。 */
            HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin, GPIO_PIN_RESET);
            s_periph_power_enabled = 0U;
            s_power_status.system_state = GLOVE_POWER_STATE_RECOVERY_FAULT;
            SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_POWER_RECOVERY_FAIL,
                                  SYSTEM_ERROR_SYNC_START_FAILED,
                                  SYSTEM_HEALTH_SOURCE_POWER,
                                  0U,
                                  1U);
            SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_FAILED);
            printf("[Power] acquisition sync start failed\r\n");
            return;
        }
        TouchAdcTask_SetAcquisitionEnabled(1U);
        ImuCanTask_SetAcquisitionEnabled(1U);
        s_periph_power_enabled = 1U;
        s_recovery_started_ms = SystemManager_GetTimestampMs();
        s_recovery_producers_ready = 0U;
        s_power_status.system_state = GLOVE_POWER_STATE_RECOVERING;
        SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_SENSOR_WAIT);
        return;
    }
    s_power_status.system_state =
        (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_NORMAL) ?
        GLOVE_POWER_STATE_ON_NORMAL : GLOVE_POWER_STATE_ON_LOW;
}

static void SystemManager_ServicePeripheralRecovery(void)
{
    DataProcessStats_t stats;
    uint8_t imu_ready;
    uint8_t touch_ready;

    if ((s_periph_power_enabled == 0U) ||
        ((s_power_status.system_state != GLOVE_POWER_STATE_RECOVERING) &&
         (s_power_status.system_state != GLOVE_POWER_STATE_RECOVERY_FAULT)))
    {
        return;
    }

    imu_ready = ImuCanTask_IsRecoveryReady();
    touch_ready = TouchAdcTask_IsRecoveryReady();
    if ((imu_ready != 0U) && (touch_ready != 0U))
    {
        DataProcessTask_GetStats(&stats);
        if (s_recovery_producers_ready == 0U)
        {
            /* 从两个采集源均就绪之后开始等待新的FullFrame，排除旧队列数据。 */
            s_recovery_full_frame_baseline = stats.full_frames_published;
            s_recovery_producers_ready = 1U;
            SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_FRAME_VERIFY);
        }
        else if (stats.full_frames_published != s_recovery_full_frame_baseline)
        {
            s_power_status.system_state =
                (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_NORMAL) ?
                GLOVE_POWER_STATE_ON_NORMAL : GLOVE_POWER_STATE_ON_LOW;
            SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_POWER_RECOVERY_FAIL,
                                  SYSTEM_ERROR_POWER_RECOVERY_TIMEOUT,
                                  SYSTEM_HEALTH_SOURCE_POWER,
                                  0U,
                                  0U);
            SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_NONE);
            printf("[Power] peripheral recovery ready imu=0x%04X full=%lu\r\n",
                   (unsigned int)ImuCanTask_GetFreshMask(),
                   (unsigned long)stats.full_frames_published);
            return;
        }
    }
    else
    {
        s_recovery_producers_ready = 0U;
    }

    if (((uint32_t)(SystemManager_GetTimestampMs() - s_recovery_started_ms) >=
         SYSTEM_MANAGER_RECOVERY_TIMEOUT_MS) &&
        (s_power_status.system_state == GLOVE_POWER_STATE_RECOVERING))
    {
        s_power_status.system_state = GLOVE_POWER_STATE_RECOVERY_FAULT;
        SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_POWER_RECOVERY_FAIL,
                              SYSTEM_ERROR_POWER_RECOVERY_TIMEOUT,
                              SYSTEM_HEALTH_SOURCE_POWER,
                              0U,
                              1U);
        SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_FAILED);
        printf("[Power] peripheral recovery timeout imu_ready=%u touch_ready=%u mask=0x%04X\r\n",
               (unsigned int)imu_ready,
               (unsigned int)touch_ready,
               (unsigned int)ImuCanTask_GetFreshMask());
    }
}

uint8_t SystemManagerTask_IsPeripheralPowerEnabled(void)
{
    return s_periph_power_enabled;
}

void SystemManagerTask_OnPowerKeyEdgeFromIsr(void)
{
    if (s_periph_power_enabled != 0U)
    {
        if (HAL_GPIO_ReadPin(POWER_ON_OFF_GPIO_Port, POWER_ON_OFF_Pin) ==
            SYSTEM_MANAGER_POWER_KEY_PRESSED_LEVEL)
        {
            /* 新的按下沿先清除旧松手标志，避免长按关机后误开机。 */
            s_power_key_release_edge_seen = 0U;
            s_power_key_press_edge_ms = HAL_GetTick();
            s_power_key_press_edge_valid = 1U;
        }
        else if (s_power_key_ignore_until_release != 0U)
        {
            s_power_key_release_edge_seen = 1U;
        }
        return;
    }

    if (s_power_key_ignore_until_release != 0U)
    {
        s_power_key_release_edge_seen = 1U;
        return;
    }
    if (s_power_key_off_wake_armed != 0U)
    {
        /* 外设断电后保留边沿脉冲唤醒，兼容板上按键的实际波形。 */
        s_power_key_off_wake_pulse_seen = 1U;
        s_power_key_off_wake_armed = 0U;
    }
}

void SystemManagerTask_OnChargeStatusEdgeFromIsr(void)
{
    s_charge_status_edge_pending = 1U;
}

void SystemManagerTask_OnBqInterruptFromIsr(void)
{
    s_bq_interrupt_pending = 1U;
    s_bq_interrupt_count++;
}

static uint8_t SystemManager_TakeIsrFlag(volatile uint8_t *flag)
{
    uint8_t value;
    taskENTER_CRITICAL();
    value = *flag;
    *flag = 0U;
    taskEXIT_CRITICAL();
    return value;
}

uint8_t SystemManagerTask_RequestPeripheralRecovery(void)
{
    uint8_t accepted = 0U;

    taskENTER_CRITICAL();
    if ((s_auto_recovery_request_pending != 0U) ||
        (s_auto_recovery_active != 0U))
    {
        accepted = 1U;
    }
    else if ((s_periph_power_enabled != 0U) &&
             ((s_power_status.system_state == GLOVE_POWER_STATE_ON_NORMAL) ||
              (s_power_status.system_state == GLOVE_POWER_STATE_ON_LOW)))
    {
        s_auto_recovery_request_pending = 1U;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static void SystemManager_CancelAutomaticRecovery(void)
{
    taskENTER_CRITICAL();
    s_auto_recovery_request_pending = 0U;
    s_auto_recovery_active = 0U;
    taskEXIT_CRITICAL();
    s_auto_recovery_waiting_power_on = 0U;
    s_auto_recovery_power_off_ms = 0U;
    SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_NONE);
}

static void SystemManager_ServiceAutomaticRecovery(void)
{
    uint32_t now_ms = SystemManager_GetTimestampMs();

    if (SystemManager_TakeIsrFlag(&s_auto_recovery_request_pending) != 0U)
    {
        if ((s_periph_power_enabled != 0U) &&
            ((s_power_status.system_state == GLOVE_POWER_STATE_ON_NORMAL) ||
             (s_power_status.system_state == GLOVE_POWER_STATE_ON_LOW)))
        {
            s_auto_recovery_active = 1U;
            s_auto_recovery_waiting_power_on = 0U;
            s_auto_recovery_power_off_ms = 0U;
            SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_SAFE_STOP);
            printf("[Power] IMU recovery requests peripheral power cycle\r\n");
            (void)SystemManager_StopPeripheralPower(GLOVE_POWER_STATE_USER_OFF);
        }
    }

    if (s_auto_recovery_active == 0U)
    {
        return;
    }

    if ((s_power_status.battery_level == GLOVE_BATTERY_LEVEL_CRITICAL) &&
        ((s_power_status.flags & GLOVE_POWER_FLAG_VBUS_PRESENT) == 0U))
    {
        /* 自动恢复不能绕过低电保护，断电后保持锁定。 */
        SystemManager_CancelAutomaticRecovery();
        s_power_status.system_state = GLOVE_POWER_STATE_LOW_BAT_LOCKOUT;
        return;
    }

    if (s_periph_power_enabled != 0U)
    {
        if (s_power_status.system_state != GLOVE_POWER_STATE_STOPPING)
        {
            /* 按键等其他路径已经重新上电时，取消尚未完成的自动周期。 */
            SystemManager_CancelAutomaticRecovery();
        }
        return;
    }

    if (s_auto_recovery_waiting_power_on == 0U)
    {
        s_auto_recovery_waiting_power_on = 1U;
        s_auto_recovery_power_off_ms = now_ms;
        SystemHealth_SetPowerRecovery(SYSTEM_RECOVERY_POWER_OFF_HOLD);
        return;
    }
    if ((uint32_t)(now_ms - s_auto_recovery_power_off_ms) <
        SYSTEM_MANAGER_AUTO_RECOVERY_OFF_HOLD_MS)
    {
        return;
    }

    SystemManager_CancelAutomaticRecovery();
    SystemManager_StartPeripheralPower();
    printf("[Power] automatic peripheral power cycle started\r\n");
}

static uint8_t SystemManager_ReadPowerKeyPressed(void)
{
    return (HAL_GPIO_ReadPin(POWER_ON_OFF_GPIO_Port, POWER_ON_OFF_Pin) ==
            SYSTEM_MANAGER_POWER_KEY_PRESSED_LEVEL) ? 1U : 0U;
}

static void SystemManager_InitPowerKeyState(void)
{
    uint32_t now = HAL_GetTick();
    s_periph_power_enabled =
        (HAL_GPIO_ReadPin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    s_power_key_last_raw_pressed = SystemManager_ReadPowerKeyPressed();
    s_power_key_debounced_pressed = s_power_key_last_raw_pressed;
    s_power_key_off_wake_pulse_seen = 0U;
    s_power_key_release_edge_seen = 0U;
    s_power_key_off_wake_armed = 1U;
    s_power_key_changed_ms = now;
    s_power_key_pressed_ms = (s_power_key_debounced_pressed != 0U) ? now : 0U;
    s_power_key_press_edge_valid = 0U;
}

static void SystemManager_ServicePowerKey(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t raw_pressed = SystemManager_ReadPowerKeyPressed();

    if (raw_pressed != s_power_key_last_raw_pressed)
    {
        s_power_key_last_raw_pressed = raw_pressed;
        s_power_key_changed_ms = now;
    }

    if (s_periph_power_enabled == 0U)
    {
        if (s_power_key_ignore_until_release != 0U)
        {
            if ((SystemManager_TakeIsrFlag(&s_power_key_release_edge_seen) != 0U) ||
                ((raw_pressed == 0U) &&
                 ((uint32_t)(now - s_power_key_changed_ms) >= SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS)))
            {
                s_power_key_ignore_until_release = 0U;
                s_power_key_off_wake_armed = 1U;
                s_power_key_debounced_pressed = 0U;
            }
            return;
        }

        if (SystemManager_TakeIsrFlag(&s_power_key_off_wake_pulse_seen) != 0U)
        {
            if (s_power_status.system_state != GLOVE_POWER_STATE_LOW_BAT_LOCKOUT)
            {
                SystemManager_StartPeripheralPower();
                printf("[Power] peripheral power on by key pulse\r\n");
            }
            else
            {
                printf("[Power] key press ignored by low battery lockout\r\n");
            }
            s_power_key_ignore_until_release = 1U;
            s_power_key_off_wake_armed = 0U;
        }
        else if (raw_pressed == 0U)
        {
            s_power_key_off_wake_armed = 1U;
        }
        return;
    }

    if (s_power_key_ignore_until_release != 0U)
    {
        if ((SystemManager_TakeIsrFlag(&s_power_key_release_edge_seen) != 0U) ||
            ((raw_pressed == 0U) &&
             ((uint32_t)(now - s_power_key_changed_ms) >= SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS)))
        {
            /* 按键稳定释放后再解除锁定，避免松手抖动被误认为下一次开机按下。 */
            s_power_key_ignore_until_release = 0U;
            s_power_key_off_wake_armed = 1U;
            s_power_key_debounced_pressed = 0U;
            s_power_key_long_handled = 0U;
            s_power_key_pressed_ms = 0U;
            s_power_key_press_edge_valid = 0U;
        }
        else
        {
            return;
        }
    }

    if ((raw_pressed != s_power_key_debounced_pressed) &&
        ((uint32_t)(now - s_power_key_changed_ms) >= SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS))
    {
        s_power_key_debounced_pressed = raw_pressed;
        if (raw_pressed != 0U)
        {
            taskENTER_CRITICAL();
            s_power_key_pressed_ms = (s_power_key_press_edge_valid != 0U) ?
                                     s_power_key_press_edge_ms : now;
            s_power_key_press_edge_valid = 0U;
            taskEXIT_CRITICAL();
            s_power_key_long_handled = 0U;
        }
        else
        {
            s_power_key_pressed_ms = 0U;
            s_power_key_press_edge_valid = 0U;
            s_power_key_long_handled = 0U;
            s_power_key_ignore_until_release = 0U;
        }
    }

    if ((s_power_key_debounced_pressed != 0U) &&
        (s_power_key_ignore_until_release == 0U) &&
        (s_power_key_long_handled == 0U) &&
        ((uint32_t)(now - s_power_key_pressed_ms) >= SYSTEM_MANAGER_POWER_KEY_LONG_PRESS_MS))
    {
        uint32_t press_start_ms = s_power_key_pressed_ms;

        s_power_key_long_handled = 1U;
        taskENTER_CRITICAL();
        s_power_key_release_edge_seen = 0U;
        s_power_key_off_wake_pulse_seen = 0U;
        s_power_key_ignore_until_release = 1U;
        s_power_key_off_wake_armed = 0U;
        taskEXIT_CRITICAL();
        s_power_key_last_trigger_elapsed_ms = (uint32_t)(now - press_start_ms);
        SystemManager_CancelAutomaticRecovery();
        uint8_t power_off_complete =
            SystemManager_StopPeripheralPower(GLOVE_POWER_STATE_USER_OFF);
        s_power_key_last_poweroff_elapsed_ms =
            (uint32_t)(HAL_GetTick() - press_start_ms);
        printf("[Power] peripheral power off trigger=%lums elapsed=%lums complete=%u\r\n",
               (unsigned long)s_power_key_last_trigger_elapsed_ms,
               (unsigned long)s_power_key_last_poweroff_elapsed_ms,
               (unsigned int)power_off_complete);
    }
}

static const char *SystemManager_BqStageName(uint8_t stage)
{
    switch ((GloveBqDiagnosticStage_t)stage)
    {
        case GLOVE_BQ_DIAG_INIT: return "init";
        case GLOVE_BQ_DIAG_WATCHDOG: return "watchdog";
        case GLOVE_BQ_DIAG_INPUT_CURRENT: return "input_current";
        case GLOVE_BQ_DIAG_EXTERNAL_ILIM: return "external_ilim";
        case GLOVE_BQ_DIAG_CHARGE_VOLTAGE: return "charge_voltage";
        case GLOVE_BQ_DIAG_CHARGE_CURRENT: return "charge_current";
        case GLOVE_BQ_DIAG_TERMINATION_CURRENT: return "termination_current";
        case GLOVE_BQ_DIAG_CHARGE_SAFETY: return "charge_safety";
        case GLOVE_BQ_DIAG_ADC: return "adc";
        case GLOVE_BQ_DIAG_STATUS_READ: return "status_read";
        case GLOVE_BQ_DIAG_INTERRUPT_CONFIG: return "interrupt_config";
        case GLOVE_BQ_DIAG_INTERRUPT_READ: return "interrupt_read";
        default: return "none";
    }
}

static void SystemManager_SetBqDiagnostic(uint8_t stage, GloveStatus_t status)
{
    if ((stage != s_bq_diagnostic_stage) || (status != s_bq_last_status))
    {
        if (stage == GLOVE_BQ_DIAG_NONE)
        {
            printf("[Power][BQ] recovered\r\n");
        }
        else
        {
            printf("[Power][BQ] failed stage=%u(%s) status=%u\r\n",
                   (unsigned int)stage,
                   SystemManager_BqStageName(stage),
                   (unsigned int)status);
        }
    }
    s_bq_diagnostic_stage = stage;
    s_bq_last_status = status;
}

static GloveStatus_t SystemManager_ConfigureBq(void)
{
    GloveStatus_t status;
    uint8_t stage = GLOVE_BQ_DIAG_INIT;
    Bq25622InterruptFlags_t cleared_events;

    SystemManager_SetChargeAllowed(0U);
    s_full_latched = 0U;
    s_full_confirm_count = 0U;
    s_charge_was_active = 0U;
    s_charge_change_event_pending = 0U;
    status = Bq25622_Init(&s_bq25622, I2C_BUS_1, SYSTEM_MANAGER_BQ25622_TIMEOUT_MS);
    if (status != GLOVE_STATUS_OK) goto failed;

    stage = GLOVE_BQ_DIAG_WATCHDOG;
    status = Bq25622_DisableWatchdog(&s_bq25622);
    if (status != GLOVE_STATUS_OK) goto failed;

    stage = GLOVE_BQ_DIAG_INPUT_CURRENT;
    status = Bq25622_SetInputCurrentLimitMa(&s_bq25622,
                                            SYSTEM_MANAGER_BQ25622_INPUT_CURRENT_MA);
    if (status != GLOVE_STATUS_OK) goto failed;

    stage = GLOVE_BQ_DIAG_EXTERNAL_ILIM;
    status = Bq25622_EnableExternalIlim(&s_bq25622);
    if (status != GLOVE_STATUS_OK) goto failed;

    stage = GLOVE_BQ_DIAG_CHARGE_VOLTAGE;
    status = Bq25622_SetChargeVoltageLimitMv(&s_bq25622,
                                             SYSTEM_MANAGER_BQ25622_CHARGE_VOLTAGE_MV);
    if (status != GLOVE_STATUS_OK) goto failed;

    stage = GLOVE_BQ_DIAG_CHARGE_CURRENT;
    status = Bq25622_SetChargeCurrentLimitMa(&s_bq25622,
                                             SYSTEM_MANAGER_BQ25622_CHARGE_CURRENT_MA);
    if (status != GLOVE_STATUS_OK) goto failed;

    stage = GLOVE_BQ_DIAG_TERMINATION_CURRENT;
    status = Bq25622_SetTerminationCurrentMa(&s_bq25622,
                                             SYSTEM_MANAGER_BQ25622_TERMINATION_CURRENT_MA);
    if (status != GLOVE_STATUS_OK) goto failed;

    stage = GLOVE_BQ_DIAG_CHARGE_SAFETY;
    status = Bq25622_EnableChargeSafety(&s_bq25622);
    if (status != GLOVE_STATUS_OK) goto failed;

    stage = GLOVE_BQ_DIAG_INTERRUPT_CONFIG;
    status = Bq25622_ConfigureInterrupts(&s_bq25622);
    if (status != GLOVE_STATUS_OK) goto failed;
    /* 清除上电和重新配置前锁存的事件，之后只记录新的INT原因。 */
    status = Bq25622_ReadInterruptFlags(&s_bq25622, &cleared_events);
    if (status != GLOVE_STATUS_OK) goto failed;
    (void)memset(&s_bq_last_events, 0, sizeof(s_bq_last_events));

    stage = GLOVE_BQ_DIAG_ADC;
    status = Bq25622_EnableAdc(&s_bq25622);
    if (status != GLOVE_STATUS_OK) goto failed;

    s_bq_ready = 1U;
    s_bq_failures = 0U;
    SystemManager_SetBqDiagnostic(GLOVE_BQ_DIAG_NONE, GLOVE_STATUS_OK);
    SystemManager_SetChargeAllowed(1U);
    printf("[Power][BQ] configured vreg=%umV ichg=%umA iindpm=%umA iterm=%umA\r\n",
           (unsigned int)SYSTEM_MANAGER_BQ25622_CHARGE_VOLTAGE_MV,
           (unsigned int)SYSTEM_MANAGER_BQ25622_CHARGE_CURRENT_MA,
           (unsigned int)SYSTEM_MANAGER_BQ25622_INPUT_CURRENT_MA,
           (unsigned int)SYSTEM_MANAGER_BQ25622_TERMINATION_CURRENT_MA);
    return GLOVE_STATUS_OK;

failed:
    s_bq_ready = 0U;
    s_bq_valid = 0U;
    SystemManager_SetChargeAllowed(0U);
    SystemManager_SetBqDiagnostic(stage, status);
    return status;
}

static uint8_t SystemManager_IsRawChargeActive(void)
{
    return ((s_bq_snapshot.charge_status >= 1U) &&
            (s_bq_snapshot.charge_status <= 3U)) ? 1U : 0U;
}

static uint8_t SystemManager_IsFullElectricalCondition(void)
{
    int32_t current_ma;
    int32_t current_limit_ma =
        (int32_t)SYSTEM_MANAGER_BQ25622_TERMINATION_CURRENT_MA +
        (int32_t)SYSTEM_MANAGER_FULL_CURRENT_MARGIN_MA;

    if ((s_bq_snapshot.battery_voltage_mv < SYSTEM_MANAGER_FULL_MIN_VOLTAGE_MV) ||
        (s_bq_snapshot.current_valid == 0U))
    {
        return 0U;
    }

    current_ma = (int32_t)s_bq_snapshot.battery_current_ma;
    if (current_ma < 0) current_ma = -current_ma;
    return (current_ma <= current_limit_ma) ? 1U : 0U;
}

static void SystemManager_UpdateFullDetection(void)
{
    uint8_t raw_active;
    uint8_t full_candidate;

    if ((s_bq_valid == 0U) ||
        (s_bq_snapshot.vbus_present == 0U) ||
        (s_charge_allowed == 0U) ||
        ((s_bq_snapshot.fault_status0 & 0xF8U) != 0U) ||
        (s_bq_snapshot.temperature_status != 0U))
    {
        s_full_latched = 0U;
        s_full_confirm_count = 0U;
        s_charge_was_active = 0U;
        s_charge_change_event_pending = 0U;
        return;
    }

    raw_active = SystemManager_IsRawChargeActive();
    if (raw_active != 0U)
    {
        s_charge_was_active = 1U;
        s_charge_change_event_pending = 0U;
        s_full_latched = 0U;
        s_full_confirm_count = 0U;
        return;
    }

    if ((s_full_latched != 0U) &&
        (s_bq_snapshot.battery_voltage_mv >= SYSTEM_MANAGER_FULL_RELEASE_VOLTAGE_MV))
    {
        return;
    }

    full_candidate = ((s_charge_was_active != 0U) ||
                      (s_charge_change_event_pending != 0U) ||
                      (SystemManager_IsFullElectricalCondition() != 0U)) ? 1U : 0U;
    if ((full_candidate != 0U) &&
        (s_bq_snapshot.battery_voltage_mv >= SYSTEM_MANAGER_FULL_MIN_VOLTAGE_MV))
    {
        if (s_full_confirm_count < SYSTEM_MANAGER_FULL_CONFIRM_SAMPLES)
        {
            s_full_confirm_count++;
        }
        if (s_full_confirm_count >= SYSTEM_MANAGER_FULL_CONFIRM_SAMPLES)
        {
            s_full_latched = 1U;
            s_charge_was_active = 0U;
            s_charge_change_event_pending = 0U;
        }
    }
    else
    {
        s_full_latched = 0U;
        s_full_confirm_count = 0U;
    }
}

static GloveChargeState_t SystemManager_DecodeChargeState(void)
{
    if (s_bq_valid == 0U) return GLOVE_CHARGE_STATE_UNKNOWN;
    if (s_bq_snapshot.vbus_present == 0U) return GLOVE_CHARGE_STATE_NO_INPUT;
    if ((s_bq_snapshot.fault_status0 & 0xF8U) != 0U) return GLOVE_CHARGE_STATE_FAULT;
    if (s_bq_snapshot.temperature_status != 0U) return GLOVE_CHARGE_STATE_SUSPENDED;
    if (s_bq_snapshot.charge_status == 1U) return GLOVE_CHARGE_STATE_CC;
    if (s_bq_snapshot.charge_status == 2U) return GLOVE_CHARGE_STATE_CV;
    if (s_bq_snapshot.charge_status == 3U) return GLOVE_CHARGE_STATE_TOPOFF;
    if (s_full_latched != 0U) return GLOVE_CHARGE_STATE_FULL;
    return GLOVE_CHARGE_STATE_IDLE;
}

static uint8_t SystemManager_SelectBatteryVoltage(uint16_t *voltage_mv, uint8_t *mismatch)
{
    uint8_t bq_voltage_valid = ((s_bq_valid != 0U) &&
                                (s_bq_snapshot.battery_voltage_mv != 0U)) ? 1U : 0U;

    *mismatch = 0U;
    if ((bq_voltage_valid != 0U) && (s_gauge_valid != 0U))
    {
        *mismatch = (SystemManager_AbsDiffU16(s_bq_snapshot.battery_voltage_mv,
                                             s_gauge_data.voltage_mv) >
                     SYSTEM_MANAGER_VOLTAGE_MISMATCH_MV) ? 1U : 0U;
        *voltage_mv = (s_bq_snapshot.battery_voltage_mv < s_gauge_data.voltage_mv) ?
                      s_bq_snapshot.battery_voltage_mv : s_gauge_data.voltage_mv;
        return 1U;
    }
    if (bq_voltage_valid != 0U)
    {
        *voltage_mv = s_bq_snapshot.battery_voltage_mv;
        return 1U;
    }
    if (s_gauge_valid != 0U)
    {
        *voltage_mv = s_gauge_data.voltage_mv;
        return 1U;
    }
    return 0U;
}

static void SystemManager_ApplyCriticalProtection(void)
{
    if ((s_power_status.battery_level == GLOVE_BATTERY_LEVEL_CRITICAL) &&
        ((s_power_status.flags & GLOVE_POWER_FLAG_VBUS_PRESENT) == 0U))
    {
        SystemManager_CancelAutomaticRecovery();
        if (s_periph_power_enabled != 0U)
        {
            uint8_t power_off_complete =
                SystemManager_StopPeripheralPower(GLOVE_POWER_STATE_LOW_BAT_LOCKOUT);
            printf("[Power] peripheral power off by critical battery complete=%u\r\n",
                   (unsigned int)power_off_complete);
        }
        else
        {
            /* 手动关机期间进入严重低电后同样禁止按键重新启动。 */
            s_power_status.system_state = GLOVE_POWER_STATE_LOW_BAT_LOCKOUT;
        }
    }
}

static void SystemManager_EvaluateBatteryLevel(void)
{
    uint16_t voltage_mv;
    uint8_t mismatch;
    uint8_t voltage_valid = SystemManager_SelectBatteryVoltage(&voltage_mv, &mismatch);

    if (voltage_valid == 0U) return;

    if (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_UNKNOWN)
    {
        s_power_status.battery_level = GLOVE_BATTERY_LEVEL_NORMAL;
    }

    if (voltage_mv <= SYSTEM_MANAGER_EMERGENCY_VOLTAGE_MV)
    {
        s_power_status.battery_level = GLOVE_BATTERY_LEVEL_CRITICAL;
    }
    else
    {
        s_critical_count = (voltage_mv <= SYSTEM_MANAGER_CRITICAL_VOLTAGE_MV) ?
                           (uint8_t)(s_critical_count + 1U) : 0U;
        if (s_critical_count >= SYSTEM_MANAGER_CRITICAL_CONFIRM_SAMPLES)
        {
            s_power_status.battery_level = GLOVE_BATTERY_LEVEL_CRITICAL;
        }
    }

    if (s_power_status.battery_level != GLOVE_BATTERY_LEVEL_CRITICAL)
    {
        s_low_voltage_count = (voltage_mv <= SYSTEM_MANAGER_LOW_VOLTAGE_MV) ?
                              (uint8_t)(s_low_voltage_count + 1U) : 0U;
        s_low_soc_count = ((SYSTEM_MANAGER_SOC_LOW_DETECTION_ENABLE != 0U) &&
                           (s_gauge_valid != 0U) &&
                           (s_gauge_data.soc_percent <= SYSTEM_MANAGER_LOW_SOC_PERCENT)) ?
                          (uint8_t)(s_low_soc_count + 1U) : 0U;
        if ((s_low_voltage_count >= SYSTEM_MANAGER_LOW_CONFIRM_SAMPLES) ||
            ((SYSTEM_MANAGER_SOC_LOW_DETECTION_ENABLE != 0U) &&
             (s_low_soc_count >= SYSTEM_MANAGER_LOW_CONFIRM_SAMPLES)))
        {
            s_power_status.battery_level = GLOVE_BATTERY_LEVEL_LOW;
        }
    }

    if (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_CRITICAL)
    {
        s_critical_release_count = (voltage_mv >= SYSTEM_MANAGER_LOCKOUT_RELEASE_VOLTAGE_MV) ?
                                   (uint8_t)(s_critical_release_count + 1U) : 0U;
        if ((s_critical_release_count >= SYSTEM_MANAGER_LOCKOUT_RELEASE_SAMPLES) &&
            (s_power_status.system_state != GLOVE_POWER_STATE_LOW_BAT_LOCKOUT))
        {
            s_power_status.battery_level = GLOVE_BATTERY_LEVEL_LOW;
            s_critical_count = 0U;
            s_critical_release_count = 0U;
        }
    }

    if (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_LOW)
    {
        uint8_t soc_released = (SYSTEM_MANAGER_SOC_LOW_DETECTION_ENABLE == 0U) ||
            (s_gauge_valid == 0U) ||
            (s_gauge_data.soc_percent > SYSTEM_MANAGER_LOW_SOC_RELEASE_PERCENT);
        s_low_release_count = ((voltage_mv >= SYSTEM_MANAGER_LOW_RELEASE_VOLTAGE_MV) &&
                               (soc_released != 0U)) ?
                              (uint8_t)(s_low_release_count + 1U) : 0U;
        if (s_low_release_count >= SYSTEM_MANAGER_LOW_RELEASE_SAMPLES)
        {
            s_power_status.battery_level = GLOVE_BATTERY_LEVEL_NORMAL;
            s_low_voltage_count = 0U;
            s_low_soc_count = 0U;
        }
    }

    if (s_power_status.system_state == GLOVE_POWER_STATE_LOW_BAT_LOCKOUT)
    {
        s_lockout_release_count =
            (((s_power_status.flags & GLOVE_POWER_FLAG_VBUS_PRESENT) != 0U) &&
             (voltage_mv >= SYSTEM_MANAGER_LOCKOUT_RELEASE_VOLTAGE_MV)) ?
            (uint8_t)(s_lockout_release_count + 1U) : 0U;
        if (s_lockout_release_count >= SYSTEM_MANAGER_LOCKOUT_RELEASE_SAMPLES)
        {
            s_power_status.system_state = GLOVE_POWER_STATE_USER_OFF;
            s_power_status.battery_level =
                (voltage_mv >= SYSTEM_MANAGER_LOW_RELEASE_VOLTAGE_MV) ?
                GLOVE_BATTERY_LEVEL_NORMAL : GLOVE_BATTERY_LEVEL_LOW;
            s_lockout_release_count = 0U;
        }
    }

    if ((s_periph_power_enabled != 0U) &&
        (s_power_status.system_state != GLOVE_POWER_STATE_STOPPING) &&
        (s_power_status.system_state != GLOVE_POWER_STATE_RECOVERING) &&
        (s_power_status.system_state != GLOVE_POWER_STATE_RECOVERY_FAULT))
    {
        s_power_status.system_state =
            (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_NORMAL) ?
            GLOVE_POWER_STATE_ON_NORMAL : GLOVE_POWER_STATE_ON_LOW;
    }
    SystemManager_ApplyCriticalProtection();
}

static void SystemManager_UpdatePublicStatus(void)
{
    uint16_t voltage_mv = s_power_status.battery_voltage_mv;
    uint16_t flags = 0U;
    uint8_t mismatch = 0U;
    uint8_t voltage_valid = SystemManager_SelectBatteryVoltage(&voltage_mv, &mismatch);
    GloveChargeState_t charge_state = SystemManager_DecodeChargeState();

    if (voltage_valid != 0U) flags |= GLOVE_POWER_FLAG_VOLTAGE_VALID;
    if (s_gauge_valid != 0U) flags |= GLOVE_POWER_FLAG_SOC_VALID;
    if ((s_bq_valid != 0U) && (s_bq_snapshot.current_valid != 0U)) flags |= GLOVE_POWER_FLAG_CURRENT_VALID;
    if ((s_bq_valid != 0U) && (s_bq_snapshot.vbus_present != 0U)) flags |= GLOVE_POWER_FLAG_VBUS_PRESENT;
    if ((charge_state == GLOVE_CHARGE_STATE_CC) ||
        (charge_state == GLOVE_CHARGE_STATE_CV) ||
        (charge_state == GLOVE_CHARGE_STATE_TOPOFF)) flags |= GLOVE_POWER_FLAG_CHARGING;
    if (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_LOW) flags |= GLOVE_POWER_FLAG_LOW;
    if (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_CRITICAL) flags |= GLOVE_POWER_FLAG_CRITICAL;
    if (s_power_status.system_state == GLOVE_POWER_STATE_LOW_BAT_LOCKOUT) flags |= GLOVE_POWER_FLAG_LOCKOUT;
    if (s_periph_power_enabled != 0U) flags |= GLOVE_POWER_FLAG_PERIPHERAL_ON;
    if ((s_bq_ready == 0U) || (s_bq_failures != 0U)) flags |= GLOVE_POWER_FLAG_BQ_COMM_FAULT;
    if (s_gauge_failures != 0U) flags |= GLOVE_POWER_FLAG_GAUGE_COMM_FAULT;
    if (mismatch != 0U) flags |= GLOVE_POWER_FLAG_VOLTAGE_MISMATCH;
    if ((s_bq_valid != 0U) && (s_bq_snapshot.temperature_status != 0U)) flags |= GLOVE_POWER_FLAG_TEMP_LIMITED;
    if (charge_state == GLOVE_CHARGE_STATE_FAULT) flags |= GLOVE_POWER_FLAG_CHARGE_FAULT;
    if ((s_bq_valid != 0U) && ((s_bq_snapshot.charger_status0 & 0x02U) != 0U)) flags |= GLOVE_POWER_FLAG_SAFETY_TIMER;
    if (charge_state == GLOVE_CHARGE_STATE_FULL) flags |= GLOVE_POWER_FLAG_CHARGE_FULL;

    taskENTER_CRITICAL();
    s_power_status.timestamp_ms = SystemManager_GetTimestampMs();
    s_power_status.battery_voltage_mv = voltage_mv;
    s_power_status.battery_current_ma = (s_bq_valid != 0U) ? s_bq_snapshot.battery_current_ma : 0;
    s_power_status.vbus_voltage_mv = (s_bq_valid != 0U) ? s_bq_snapshot.vbus_voltage_mv : 0U;
    s_power_status.input_current_ma = (s_bq_valid != 0U) ? s_bq_snapshot.input_current_ma : 0;
    s_power_status.soc_centi_percent = (s_gauge_valid != 0U) ? s_gauge_data.soc_centi_percent : 0U;
    s_power_status.soc_percent = (s_gauge_valid != 0U) ? s_gauge_data.soc_percent : 0U;
    s_power_status.flags = flags;
    s_power_status.bq_charger_events =
        (uint16_t)((uint16_t)s_bq_last_events.charger_flag0 |
                   ((uint16_t)s_bq_last_events.charger_flag1 << 8));
    s_power_status.bq_fault_events = s_bq_last_events.fault_flag0;
    s_power_status.bq_interrupt_count = (uint16_t)s_bq_interrupt_count;
    s_power_status.charge_state = (uint8_t)charge_state;
    s_power_status.bq_vbus_type = (s_bq_valid != 0U) ? s_bq_snapshot.vbus_status : 0U;
    s_power_status.bq_temperature_status = (s_bq_valid != 0U) ? s_bq_snapshot.temperature_status : 0U;
    s_power_status.bq_diagnostic_stage = s_bq_diagnostic_stage;
    s_power_status.bq_last_status = (uint8_t)s_bq_last_status;
    s_power_status.fault_code = (uint16_t)((s_bq_valid != 0U) ? s_bq_snapshot.fault_status0 : 0U);
    if ((s_bq_ready == 0U) || (s_bq_failures != 0U)) s_power_status.fault_code |= 0x0100U;
    if (s_gauge_failures != 0U) s_power_status.fault_code |= 0x0200U;
    if (mismatch != 0U) s_power_status.fault_code |= 0x0400U;
    taskEXIT_CRITICAL();

    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_LOW_BATTERY,
                          SYSTEM_ERROR_BATTERY_LOW,
                          SYSTEM_HEALTH_SOURCE_BATTERY,
                          0U,
                          ((flags & GLOVE_POWER_FLAG_LOW) != 0U) ? 1U : 0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_CRITICAL_BATTERY,
                          SYSTEM_ERROR_BATTERY_CRITICAL,
                          SYSTEM_HEALTH_SOURCE_BATTERY,
                          0U,
                          ((flags & GLOVE_POWER_FLAG_CRITICAL) != 0U) ? 1U : 0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_BQ_COMM,
                          SYSTEM_ERROR_BQ_COMM,
                          SYSTEM_HEALTH_SOURCE_CHARGER,
                          s_bq_diagnostic_stage,
                          ((flags & GLOVE_POWER_FLAG_BQ_COMM_FAULT) != 0U) ? 1U : 0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_GAUGE_COMM,
                          SYSTEM_ERROR_GAUGE_COMM,
                          SYSTEM_HEALTH_SOURCE_BATTERY,
                          0U,
                          ((flags & GLOVE_POWER_FLAG_GAUGE_COMM_FAULT) != 0U) ? 1U : 0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_VOLTAGE_MISMATCH,
                          SYSTEM_ERROR_VOLTAGE_MISMATCH,
                          SYSTEM_HEALTH_SOURCE_BATTERY,
                          0U,
                          ((flags & GLOVE_POWER_FLAG_VOLTAGE_MISMATCH) != 0U) ? 1U : 0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_TEMP_LIMIT,
                          SYSTEM_ERROR_TEMPERATURE_LIMIT,
                          SYSTEM_HEALTH_SOURCE_CHARGER,
                          0U,
                          ((flags & GLOVE_POWER_FLAG_TEMP_LIMITED) != 0U) ? 1U : 0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_CHARGE_FAULT,
                          SYSTEM_ERROR_CHARGE_FAULT,
                          SYSTEM_HEALTH_SOURCE_CHARGER,
                          0U,
                          ((flags & GLOVE_POWER_FLAG_CHARGE_FAULT) != 0U) ? 1U : 0U);
}

void SystemManagerTask_GetPowerStatus(GlovePowerStatus_t *status)
{
    if (status == NULL) return;
    taskENTER_CRITICAL();
    *status = s_power_status;
    taskEXIT_CRITICAL();
}

void SystemManagerTask_GetBatteryStatus(GloveBatteryStatus_t *status)
{
    if (status == NULL) return;
    taskENTER_CRITICAL();
    *status = s_battery_status;
    taskEXIT_CRITICAL();
}

static void SystemManager_UpdateLegacyBatteryStatus(GloveStatus_t status)
{
    taskENTER_CRITICAL();
    s_battery_status.timestamp_ms = SystemManager_GetTimestampMs();
    s_battery_status.last_status = status;
    if (status == GLOVE_STATUS_OK)
    {
        s_battery_status.valid = 1U;
        s_battery_status.sample_seq++;
        s_battery_status.voltage_mv = s_gauge_data.voltage_mv;
        s_battery_status.soc_percent = s_gauge_data.soc_percent;
        s_battery_status.soc_centi_percent = s_gauge_data.soc_centi_percent;
        s_battery_status.consecutive_failures = 0U;
    }
    else
    {
        s_battery_status.consecutive_failures++;
    }
    taskEXIT_CRITICAL();
}

void SystemManagerTask(void *argument)
{
    uint32_t next_wake;
    uint32_t bq_elapsed = SYSTEM_MANAGER_BQ_READ_PERIOD_MS;
    uint32_t gauge_elapsed = 0U;
    uint32_t startup_elapsed = 0U;
    uint32_t retry_elapsed = SYSTEM_MANAGER_BQ_RETRY_PERIOD_MS;
    uint8_t bq_interrupt_pending;
    uint8_t charge_status_edge_pending;
    Bq25622InterruptFlags_t interrupt_events;
    GloveStatus_t status;
    SystemWatchdogStatus_t watchdog_status;

    (void)argument;
    (void)memset(&s_battery_status, 0, sizeof(s_battery_status));
    (void)memset(&s_power_status, 0, sizeof(s_power_status));
    s_power_status.system_state = GLOVE_POWER_STATE_INIT;
    s_power_status.charge_state = GLOVE_CHARGE_STATE_UNKNOWN;
    s_power_status.battery_level = GLOVE_BATTERY_LEVEL_UNKNOWN;
    SystemManager_SetChargeAllowed(0U);
    SystemManager_InitPowerKeyState();
    (void)Max17043_Init(&s_max17043, I2C_BUS_1, SYSTEM_MANAGER_MAX17043_TIMEOUT_MS);
    (void)SystemManager_ConfigureBq();
    s_power_status.system_state = (s_periph_power_enabled != 0U) ?
        GLOVE_POWER_STATE_INIT : GLOVE_POWER_STATE_USER_OFF;
    next_wake = osKernelGetTickCount();

    for (;;)
    {
        SystemManager_ServicePowerKey();
        (void)SystemManager_TryFinalizePeripheralStop();
        SystemManager_ServiceAutomaticRecovery();
        SystemManager_ServicePeripheralRecovery();
        SystemHealth_SetPowerState(s_power_status.system_state);
        SystemHealth_Service();
        SystemWatchdog_GetStatus(&watchdog_status);
        SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_WATCHDOG_WARNING,
                              SYSTEM_ERROR_WATCHDOG_CONFIG,
                              SYSTEM_HEALTH_SOURCE_WATCHDOG,
                              0U,
                              ((watchdog_status.status_flags &
                                SYSTEM_WATCHDOG_STATUS_CONFIG_WARNING) != 0U) ? 1U : 0U);

        if ((s_periph_power_enabled != 0U) &&
            (s_power_key_last_raw_pressed != 0U) &&
            (s_power_key_long_handled == 0U))
        {
            /* 按住关机键期间不启动阻塞式I2C事务，保证600ms判定不被电源采样拖延。 */
            next_wake += SystemManager_MsToTicks(SYSTEM_MANAGER_LOOP_PERIOD_MS);
            (void)osDelayUntil(next_wake);
            continue;
        }

        bq_interrupt_pending = SystemManager_TakeIsrFlag(&s_bq_interrupt_pending);
        charge_status_edge_pending =
            SystemManager_TakeIsrFlag(&s_charge_status_edge_pending);

        if ((bq_interrupt_pending != 0U) && (s_bq_ready != 0U))
        {
            status = Bq25622_ReadInterruptFlags(&s_bq25622, &interrupt_events);
            if (status == GLOVE_STATUS_OK)
            {
                /* 保存最近一次硬件事件，供状态机和485诊断使用。 */
                s_bq_last_events = interrupt_events;
                if ((interrupt_events.charger_flag1 &
                     BQ25622_EVENT_CHARGE_CHANGED) != 0U)
                {
                    s_charge_change_event_pending = 1U;
                }
            }
            else
            {
                SystemManager_SetBqDiagnostic(GLOVE_BQ_DIAG_INTERRUPT_READ, status);
            }
        }

        if ((bq_interrupt_pending != 0U) ||
            (charge_status_edge_pending != 0U))
        {
            /* INT和STAT只用于提前轮询，最终状态仍以I2C寄存器为准。 */
            bq_elapsed = SYSTEM_MANAGER_BQ_READ_PERIOD_MS;
        }

        if (s_bq_ready == 0U)
        {
            retry_elapsed += SYSTEM_MANAGER_LOOP_PERIOD_MS;
            if (retry_elapsed >= SYSTEM_MANAGER_BQ_RETRY_PERIOD_MS)
            {
                retry_elapsed = 0U;
                (void)SystemManager_ConfigureBq();
            }
        }
        else
        {
            bq_elapsed += SYSTEM_MANAGER_LOOP_PERIOD_MS;
            if (bq_elapsed >= SYSTEM_MANAGER_BQ_READ_PERIOD_MS)
            {
                bq_elapsed = 0U;
                status = Bq25622_ReadStatusSnapshot(&s_bq25622, &s_bq_snapshot);
                if (status == GLOVE_STATUS_OK)
                {
                    s_bq_valid = 1U;
                    s_bq_failures = 0U;
                    if ((s_bq_diagnostic_stage == GLOVE_BQ_DIAG_STATUS_READ) ||
                        (s_bq_diagnostic_stage == GLOVE_BQ_DIAG_INTERRUPT_READ))
                    {
                        SystemManager_SetBqDiagnostic(GLOVE_BQ_DIAG_NONE, GLOVE_STATUS_OK);
                    }
                    s_power_status.sample_seq++;
                    SystemManager_UpdateFullDetection();
                    SystemManager_UpdatePublicStatus();
                    if ((s_bq_snapshot.battery_voltage_mv <= SYSTEM_MANAGER_EMERGENCY_VOLTAGE_MV) &&
                        (s_bq_snapshot.battery_voltage_mv != 0U))
                    {
                        s_power_status.battery_level = GLOVE_BATTERY_LEVEL_CRITICAL;
                        SystemManager_UpdatePublicStatus();
                        SystemManager_ApplyCriticalProtection();
                    }
                }
                else
                {
                    s_bq_valid = 0U;
                    SystemManager_SetBqDiagnostic(GLOVE_BQ_DIAG_STATUS_READ, status);
                    if (s_bq_failures < 0xFFU) s_bq_failures++;
                    if (s_bq_failures >= SYSTEM_MANAGER_BQ_FAILURE_LIMIT)
                    {
                        s_bq_ready = 0U;
                        SystemManager_SetChargeAllowed(0U);
                    }
                    SystemManager_UpdatePublicStatus();
                }
            }
        }

        startup_elapsed += SYSTEM_MANAGER_LOOP_PERIOD_MS;
        if (startup_elapsed >= SYSTEM_MANAGER_BATTERY_STARTUP_DELAY_MS)
        {
            if ((s_periph_power_enabled != 0U) &&
                (s_power_status.system_state == GLOVE_POWER_STATE_INIT))
            {
                /* 电量芯片异常时也要结束INIT，并通过健康告警单独说明采样失败。 */
                s_power_status.system_state =
                    (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_LOW) ?
                    GLOVE_POWER_STATE_ON_LOW : GLOVE_POWER_STATE_ON_NORMAL;
            }
            gauge_elapsed += SYSTEM_MANAGER_LOOP_PERIOD_MS;
            if (gauge_elapsed >= SYSTEM_MANAGER_GAUGE_READ_PERIOD_MS)
            {
                gauge_elapsed = 0U;
                status = Max17043_ReadBattery(&s_max17043, &s_gauge_data);
                if (status == GLOVE_STATUS_OK)
                {
                    s_gauge_valid = 1U;
                    s_gauge_failures = 0U;
                    s_power_status.sample_seq++;
                }
                else
                {
                    s_gauge_valid = 0U;
                    if (s_gauge_failures < 0xFFU) s_gauge_failures++;
                }
                SystemManager_UpdateLegacyBatteryStatus(status);
                SystemManager_UpdatePublicStatus();
                SystemManager_EvaluateBatteryLevel();
                SystemManager_UpdatePublicStatus();
            }
        }

        next_wake += SystemManager_MsToTicks(SYSTEM_MANAGER_LOOP_PERIOD_MS);
        (void)osDelayUntil(next_wake);
    }
}
