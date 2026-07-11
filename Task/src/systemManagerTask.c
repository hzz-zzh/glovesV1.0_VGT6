#include "systemManagerTask.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "acq_sync.h"
#include "bq25622.h"
#include "data_manager.h"
#include "i2c_bus.h"
#include "imuCanTask.h"
#include "main.h"
#include "max17043.h"
#include "touchAdcTask.h"

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
#define SYSTEM_MANAGER_MAX17043_TIMEOUT_MS           (20U)
#define SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS         (20U)
#define SYSTEM_MANAGER_POWER_KEY_LONG_PRESS_MS       (1000U)
#define SYSTEM_MANAGER_POWER_KEY_PRESSED_LEVEL       GPIO_PIN_RESET
#define SYSTEM_MANAGER_POWER_STOP_TIMEOUT_MS         (100U)
#define SYSTEM_MANAGER_LOW_VOLTAGE_MV                (3500U)
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

static GloveBatteryStatus_t s_battery_status;
static GlovePowerStatus_t s_power_status;
static Bq25622Handle_t s_bq25622;
static Max17043Handle_t s_max17043;
static Bq25622StatusSnapshot_t s_bq_snapshot;
static Max17043BatteryData_t s_gauge_data;
static volatile uint8_t s_periph_power_enabled = 1U;
static volatile uint8_t s_power_key_off_wake_pulse_seen;
static volatile uint8_t s_power_key_release_edge_seen;
static volatile uint8_t s_power_key_off_wake_armed = 1U;
static volatile uint8_t s_power_key_ignore_until_release;
static volatile uint8_t s_power_status_edge_pending;
static uint8_t s_power_key_last_raw_pressed;
static uint8_t s_power_key_debounced_pressed;
static uint8_t s_power_key_long_handled;
static uint32_t s_power_key_changed_ms;
static uint32_t s_power_key_pressed_ms;
static uint8_t s_bq_ready;
static uint8_t s_bq_valid;
static uint8_t s_gauge_valid;
static uint8_t s_bq_failures;
static uint8_t s_gauge_failures;
static uint8_t s_bq_diagnostic_stage;
static GloveStatus_t s_bq_last_status = GLOVE_STATUS_OK;
static uint8_t s_low_voltage_count;
static uint8_t s_low_soc_count;
static uint8_t s_low_release_count;
static uint8_t s_critical_count;
static uint8_t s_critical_release_count;
static uint8_t s_lockout_release_count;

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

static void SystemManager_SetChargeAllowed(uint8_t allowed)
{
    HAL_GPIO_WritePin(DISABLE_CHARGE_GPIO_Port,
                      DISABLE_CHARGE_Pin,
                      (allowed != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
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

static void SystemManager_StopPeripheralPower(GlovePowerState_t target_state)
{
    if (s_periph_power_enabled != 0U)
    {
        ImuCanTask_SetAcquisitionEnabled(0U);
        TouchAdcTask_SetAcquisitionEnabled(0U);
        (void)SystemManager_WaitAcquisitionPaused(SYSTEM_MANAGER_POWER_STOP_TIMEOUT_MS);
        DataManager_FlushAcquisitionQueues();
        HAL_GPIO_WritePin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin, GPIO_PIN_RESET);
        s_periph_power_enabled = 0U;
    }
    s_power_status.system_state = (uint8_t)target_state;
}

static void SystemManager_StartPeripheralPower(void)
{
    if (s_periph_power_enabled == 0U)
    {
        HAL_GPIO_WritePin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
        osDelay(SystemManager_MsToTicks(10U));
        HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
        osDelay(SystemManager_MsToTicks(100U));
        AcqSync_Reset();
        TouchAdcTask_SetAcquisitionEnabled(1U);
        ImuCanTask_SetAcquisitionEnabled(1U);
        s_periph_power_enabled = 1U;
    }
    s_power_status.system_state =
        (s_power_status.battery_level == GLOVE_BATTERY_LEVEL_NORMAL) ?
        GLOVE_POWER_STATE_ON_NORMAL : GLOVE_POWER_STATE_ON_LOW;
}

uint8_t SystemManagerTask_IsPeripheralPowerEnabled(void)
{
    return s_periph_power_enabled;
}

void SystemManagerTask_OnPowerKeyEdgeFromIsr(void)
{
    if (s_periph_power_enabled != 0U) return;
    if (s_power_key_ignore_until_release != 0U)
    {
        s_power_key_release_edge_seen = 1U;
        return;
    }
    if (s_power_key_off_wake_armed != 0U)
    {
        s_power_key_off_wake_pulse_seen = 1U;
        s_power_key_off_wake_armed = 0U;
    }
}

void SystemManagerTask_OnPowerStatusEdgeFromIsr(void)
{
    s_power_status_edge_pending = 1U;
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

static uint8_t SystemManager_ReadPowerKeyPressed(void)
{
    return (HAL_GPIO_ReadPin(POWER_ON_OFF_GPIO_Port, POWER_ON_OFF_Pin) ==
            SYSTEM_MANAGER_POWER_KEY_PRESSED_LEVEL) ? 1U : 0U;
}

static void SystemManager_InitPowerKeyState(void)
{
    uint32_t now = SystemManager_GetTimestampMs();
    s_periph_power_enabled =
        (HAL_GPIO_ReadPin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    s_power_key_last_raw_pressed = SystemManager_ReadPowerKeyPressed();
    s_power_key_debounced_pressed = s_power_key_last_raw_pressed;
    s_power_key_off_wake_armed = 1U;
    s_power_key_changed_ms = now;
    s_power_key_pressed_ms = (s_power_key_debounced_pressed != 0U) ? now : 0U;
}

static void SystemManager_ServicePowerKey(void)
{
    uint32_t now = SystemManager_GetTimestampMs();
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
            s_power_key_ignore_until_release = 1U;
            s_power_key_off_wake_armed = 0U;
        }
        else if (raw_pressed == 0U)
        {
            s_power_key_off_wake_armed = 1U;
        }
        return;
    }

    if ((raw_pressed != s_power_key_debounced_pressed) &&
        ((uint32_t)(now - s_power_key_changed_ms) >= SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS))
    {
        s_power_key_debounced_pressed = raw_pressed;
        if (raw_pressed != 0U)
        {
            s_power_key_pressed_ms = now;
            s_power_key_long_handled = 0U;
        }
        else
        {
            s_power_key_pressed_ms = 0U;
            s_power_key_long_handled = 0U;
            s_power_key_ignore_until_release = 0U;
        }
    }

    if ((s_power_key_debounced_pressed != 0U) &&
        (s_power_key_ignore_until_release == 0U) &&
        (s_power_key_long_handled == 0U) &&
        ((uint32_t)(now - s_power_key_pressed_ms) >= SYSTEM_MANAGER_POWER_KEY_LONG_PRESS_MS))
    {
        s_power_key_long_handled = 1U;
        s_power_key_ignore_until_release = 1U;
        s_power_key_off_wake_armed = 0U;
        SystemManager_StopPeripheralPower(GLOVE_POWER_STATE_USER_OFF);
        printf("[Power] peripheral power off by long press\r\n");
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

    SystemManager_SetChargeAllowed(0U);
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

static GloveChargeState_t SystemManager_DecodeChargeState(void)
{
    if (s_bq_valid == 0U) return GLOVE_CHARGE_STATE_UNKNOWN;
    if (s_bq_snapshot.vbus_present == 0U) return GLOVE_CHARGE_STATE_NO_INPUT;
    if ((s_bq_snapshot.fault_status0 & 0xF8U) != 0U) return GLOVE_CHARGE_STATE_FAULT;
    if (s_bq_snapshot.temperature_status != 0U) return GLOVE_CHARGE_STATE_SUSPENDED;
    if (s_bq_snapshot.charge_status == 1U) return GLOVE_CHARGE_STATE_CC;
    if (s_bq_snapshot.charge_status == 2U) return GLOVE_CHARGE_STATE_CV;
    if (s_bq_snapshot.charge_status == 3U) return GLOVE_CHARGE_STATE_TOPOFF;
    if ((s_bq_snapshot.battery_voltage_mv >= 4300U) &&
        (s_bq_snapshot.current_valid != 0U) &&
        (s_bq_snapshot.battery_current_ma >= 0) &&
        (s_bq_snapshot.battery_current_ma <= (int16_t)SYSTEM_MANAGER_BQ25622_TERMINATION_CURRENT_MA))
    {
        return GLOVE_CHARGE_STATE_FULL;
    }
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
        if (s_periph_power_enabled != 0U)
        {
            SystemManager_StopPeripheralPower(GLOVE_POWER_STATE_LOW_BAT_LOCKOUT);
            printf("[Power] peripheral power off by critical battery\r\n");
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

    if (s_periph_power_enabled != 0U)
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

    taskENTER_CRITICAL();
    s_power_status.timestamp_ms = SystemManager_GetTimestampMs();
    s_power_status.battery_voltage_mv = voltage_mv;
    s_power_status.battery_current_ma = (s_bq_valid != 0U) ? s_bq_snapshot.battery_current_ma : 0;
    s_power_status.vbus_voltage_mv = (s_bq_valid != 0U) ? s_bq_snapshot.vbus_voltage_mv : 0U;
    s_power_status.input_current_ma = (s_bq_valid != 0U) ? s_bq_snapshot.input_current_ma : 0;
    s_power_status.soc_centi_percent = (s_gauge_valid != 0U) ? s_gauge_data.soc_centi_percent : 0U;
    s_power_status.soc_percent = (s_gauge_valid != 0U) ? s_gauge_data.soc_percent : 0U;
    s_power_status.flags = flags;
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
    GloveStatus_t status;

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

        if (SystemManager_TakeIsrFlag(&s_power_status_edge_pending) != 0U)
        {
            /* 中断只用于提前轮询，最终状态仍以I2C寄存器为准。 */
            bq_elapsed = SYSTEM_MANAGER_BQ_READ_PERIOD_MS;
            gauge_elapsed = SYSTEM_MANAGER_GAUGE_READ_PERIOD_MS;
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
                    if (s_bq_diagnostic_stage == GLOVE_BQ_DIAG_STATUS_READ)
                    {
                        SystemManager_SetBqDiagnostic(GLOVE_BQ_DIAG_NONE, GLOVE_STATUS_OK);
                    }
                    s_power_status.sample_seq++;
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
