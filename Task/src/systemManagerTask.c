#include "systemManagerTask.h"

#include <string.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bq25622.h"
#include "i2c_bus.h"
#include "main.h"
#include "max17043.h"
#include "uart_redirect.h"

#define SYSTEM_MANAGER_LOOP_PERIOD_MS              (10U)
#define SYSTEM_MANAGER_BATTERY_READ_PERIOD_MS      (1000U)
#define SYSTEM_MANAGER_BATTERY_STARTUP_DELAY_MS    (600U)
#define SYSTEM_MANAGER_POWER_DEBUG_ENABLE          (0U)
#define SYSTEM_MANAGER_POWER_DEBUG_PERIOD_MS       (5000U)
#define SYSTEM_MANAGER_BQ25622_TIMEOUT_MS          (20U)
#define SYSTEM_MANAGER_BQ25622_INPUT_CURRENT_MA    (2500U)
#define SYSTEM_MANAGER_BQ25622_CHARGE_VOLTAGE_MV   (4200U)
#define SYSTEM_MANAGER_BQ25622_CHARGE_CURRENT_MA   (2400U)
#define SYSTEM_MANAGER_MAX17043_TIMEOUT_MS         (20U)
#define SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS       (20U)
#define SYSTEM_MANAGER_POWER_KEY_LONG_PRESS_MS     (1000U)
#define SYSTEM_MANAGER_POWER_KEY_PRESSED_LEVEL     GPIO_PIN_RESET

static GloveBatteryStatus_t s_battery_status;
static Bq25622Handle_t s_bq25622;
static Max17043Handle_t s_max17043;
static volatile uint8_t s_periph_power_enabled = 1U;
static volatile uint8_t s_power_key_off_wake_pulse_seen = 0U;
static volatile uint8_t s_power_key_release_edge_seen = 0U;
static volatile uint8_t s_power_key_off_wake_armed = 1U;
static uint8_t s_power_key_last_raw_pressed = 0U;
static uint8_t s_power_key_debounced_pressed = 0U;
static volatile uint8_t s_power_key_ignore_until_release = 0U;
static uint8_t s_power_key_long_handled = 0U;
static uint32_t s_power_key_changed_ms = 0U;
static uint32_t s_power_key_pressed_ms = 0U;

static uint32_t SystemManager_MsToTicks(uint32_t timeout_ms)
{
    uint64_t ticks = ((uint64_t)timeout_ms * (uint64_t)osKernelGetTickFreq() + 999ULL) / 1000ULL;

    if ((timeout_ms > 0U) && (ticks == 0ULL))
    {
        ticks = 1ULL;
    }

    return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

static uint32_t SystemManager_GetTimestampMs(void)
{
    uint32_t tick_freq = osKernelGetTickFreq();

    if (tick_freq == 0U)
    {
        return 0U;
    }

    return (uint32_t)(((uint64_t)osKernelGetTickCount() * 1000ULL) / (uint64_t)tick_freq);
}

static void SystemManager_SetPeripheralPowerEnabled(uint8_t enabled)
{
    GPIO_PinState level = (enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin, level);
    s_periph_power_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t SystemManagerTask_IsPeripheralPowerEnabled(void)
{
    return s_periph_power_enabled;
}

void SystemManagerTask_OnPowerKeyEdgeFromIsr(void)
{
    if (s_periph_power_enabled != 0U)
    {
        return;
    }

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

static uint8_t SystemManager_TakePowerKeyWakePulse(void)
{
    uint8_t seen;

    taskENTER_CRITICAL();
    seen = s_power_key_off_wake_pulse_seen;
    s_power_key_off_wake_pulse_seen = 0U;
    taskEXIT_CRITICAL();

    return seen;
}

static uint8_t SystemManager_TakePowerKeyReleaseEdge(void)
{
    uint8_t seen;

    taskENTER_CRITICAL();
    seen = s_power_key_release_edge_seen;
    s_power_key_release_edge_seen = 0U;
    taskEXIT_CRITICAL();

    return seen;
}

static uint8_t SystemManager_ReadPowerKeyPressedWhenOn(void)
{
    return (HAL_GPIO_ReadPin(POWER_ON_OFF_GPIO_Port, POWER_ON_OFF_Pin) ==
            SYSTEM_MANAGER_POWER_KEY_PRESSED_LEVEL) ? 1U : 0U;
}

static void SystemManager_MarkPowerKeyWake(uint32_t now_ms)
{
    s_power_key_ignore_until_release = 1U;
    s_power_key_off_wake_armed = 0U;
    s_power_key_long_handled = 0U;
    s_power_key_last_raw_pressed = 1U;
    s_power_key_debounced_pressed = 1U;
    s_power_key_changed_ms = now_ms;
    s_power_key_pressed_ms = now_ms;
}

static void SystemManager_InitPowerKeyState(void)
{
    uint32_t now_ms = SystemManager_GetTimestampMs();

    s_periph_power_enabled =
        (HAL_GPIO_ReadPin(PERIPH_PWR_EN_GPIO_Port, PERIPH_PWR_EN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    s_power_key_last_raw_pressed = SystemManager_ReadPowerKeyPressedWhenOn();
    s_power_key_debounced_pressed = s_power_key_last_raw_pressed;
    s_power_key_ignore_until_release = 0U;
    s_power_key_off_wake_pulse_seen = 0U;
    s_power_key_release_edge_seen = 0U;
    s_power_key_off_wake_armed = 1U;
    s_power_key_long_handled = 0U;
    s_power_key_changed_ms = now_ms;
    s_power_key_pressed_ms = (s_power_key_debounced_pressed != 0U) ? now_ms : 0U;
}

static void SystemManager_ServicePowerKey(void)
{
    uint32_t now_ms = SystemManager_GetTimestampMs();
    uint8_t raw_pressed;

    if (s_periph_power_enabled == 0U)
    {
        raw_pressed = SystemManager_ReadPowerKeyPressedWhenOn();
        if (raw_pressed != s_power_key_last_raw_pressed)
        {
            s_power_key_last_raw_pressed = raw_pressed;
            s_power_key_changed_ms = now_ms;
        }

        if (s_power_key_ignore_until_release != 0U)
        {
            if ((SystemManager_TakePowerKeyReleaseEdge() != 0U) ||
                ((raw_pressed == 0U) &&
                 ((uint32_t)(now_ms - s_power_key_changed_ms) >= SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS)))
            {
                taskENTER_CRITICAL();
                s_power_key_ignore_until_release = 0U;
                s_power_key_off_wake_pulse_seen = 0U;
                s_power_key_release_edge_seen = 0U;
                s_power_key_off_wake_armed = 1U;
                taskEXIT_CRITICAL();
                s_power_key_debounced_pressed = 0U;
                s_power_key_pressed_ms = 0U;
                s_power_key_long_handled = 0U;
            }
            return;
        }

        if (SystemManager_TakePowerKeyWakePulse() != 0U)
        {
            SystemManager_SetPeripheralPowerEnabled(1U);
            SystemManager_MarkPowerKeyWake(now_ms);
            printf("[Power] peripheral power on by key pulse\r\n");
            return;
        }

        if (raw_pressed == 0U)
        {
            s_power_key_off_wake_armed = 1U;
        }
        return;
    }

    raw_pressed = SystemManager_ReadPowerKeyPressedWhenOn();
    if (raw_pressed != s_power_key_last_raw_pressed)
    {
        s_power_key_last_raw_pressed = raw_pressed;
        s_power_key_changed_ms = now_ms;
    }

    if ((raw_pressed != s_power_key_debounced_pressed) &&
        ((uint32_t)(now_ms - s_power_key_changed_ms) >= SYSTEM_MANAGER_POWER_KEY_DEBOUNCE_MS))
    {
        s_power_key_debounced_pressed = raw_pressed;
        if (s_power_key_debounced_pressed != 0U)
        {
            s_power_key_pressed_ms = now_ms;
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
        ((uint32_t)(now_ms - s_power_key_pressed_ms) >= SYSTEM_MANAGER_POWER_KEY_LONG_PRESS_MS))
    {
        s_power_key_long_handled = 1U;
        s_power_key_ignore_until_release = 1U;
        s_power_key_release_edge_seen = 0U;
        s_power_key_off_wake_pulse_seen = 0U;
        s_power_key_off_wake_armed = 0U;
        SystemManager_SetPeripheralPowerEnabled(0U);
        printf("[Power] peripheral power off by long press\r\n");
    }
}

static void SystemManager_UpdateBatterySuccess(const Max17043BatteryData_t *data)
{
    if (data == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_battery_status.valid = 1U;
    s_battery_status.sample_seq++;
    s_battery_status.timestamp_ms = SystemManager_GetTimestampMs();
    s_battery_status.voltage_mv = data->voltage_mv;
    s_battery_status.soc_percent = data->soc_percent;
    s_battery_status.soc_centi_percent = data->soc_centi_percent;
    s_battery_status.consecutive_failures = 0U;
    s_battery_status.last_status = GLOVE_STATUS_OK;
    taskEXIT_CRITICAL();
}

static void SystemManager_UpdateBatteryFailure(GloveStatus_t status)
{
    taskENTER_CRITICAL();
    s_battery_status.timestamp_ms = SystemManager_GetTimestampMs();
    s_battery_status.consecutive_failures++;
    s_battery_status.last_status = status;
    taskEXIT_CRITICAL();
}

#if (SYSTEM_MANAGER_POWER_DEBUG_ENABLE != 0U)
static void SystemManager_PrintBatteryDebug(GloveStatus_t status,
                                            const Max17043BatteryData_t *data)
{
    uint16_t soc_integer;
    uint16_t soc_fraction;

    if ((status != GLOVE_STATUS_OK) || (data == NULL))
    {
        printf("[MAX17043] battery read failed status=%u\r\n",
               (unsigned int)status);
        return;
    }

    soc_integer = (uint16_t)(data->soc_centi_percent / 100U);
    soc_fraction = (uint16_t)(data->soc_centi_percent % 100U);
    printf("[MAX17043] battery voltage=%u mV soc=%u.%02u%% raw_vcell=0x%04X raw_soc=0x%04X\r\n",
           (unsigned int)data->voltage_mv,
           (unsigned int)soc_integer,
           (unsigned int)soc_fraction,
           (unsigned int)data->raw_vcell,
           (unsigned int)data->raw_soc);
}
#endif

void SystemManagerTask_GetBatteryStatus(GloveBatteryStatus_t *status)
{
    if (status != NULL)
    {
        taskENTER_CRITICAL();
        *status = s_battery_status;
        taskEXIT_CRITICAL();
    }
}

void SystemManagerTask(void *argument)
{
    Max17043BatteryData_t battery_data;
    GloveStatus_t status;
    uint32_t period_ticks;
    uint32_t next_wake_tick;
    uint32_t battery_startup_elapsed_ms = 0U;
    uint32_t battery_read_elapsed_ms = 0U;
#if (SYSTEM_MANAGER_POWER_DEBUG_ENABLE != 0U)
    uint32_t power_debug_elapsed_ms = 0U;
#endif
    uint8_t battery_startup_done = 0U;

    (void)argument;
    (void)memset(&s_battery_status, 0, sizeof(s_battery_status));
    SystemManager_InitPowerKeyState();

    status = Bq25622_Init(&s_bq25622, I2C_BUS_1, SYSTEM_MANAGER_BQ25622_TIMEOUT_MS);
    if (status == GLOVE_STATUS_OK)
    {
        status = Bq25622_DisableWatchdog(&s_bq25622);
    }
    if (status == GLOVE_STATUS_OK)
    {
        status = Bq25622_SetInputCurrentLimitMa(&s_bq25622,
                                                SYSTEM_MANAGER_BQ25622_INPUT_CURRENT_MA);
    }
    if (status == GLOVE_STATUS_OK)
    {
        status = Bq25622_EnableExternalIlim(&s_bq25622);
    }
    if (status == GLOVE_STATUS_OK)
    {
        status = Bq25622_SetChargeVoltageLimitMv(&s_bq25622,
                                                 SYSTEM_MANAGER_BQ25622_CHARGE_VOLTAGE_MV);
    }
    if (status == GLOVE_STATUS_OK)
    {
        status = Bq25622_SetChargeCurrentLimitMa(&s_bq25622,
                                                 SYSTEM_MANAGER_BQ25622_CHARGE_CURRENT_MA);
    }
    if (status == GLOVE_STATUS_OK)
    {
#if (SYSTEM_MANAGER_POWER_DEBUG_ENABLE != 0U)
        (void)Bq25622_DumpDebugRegisters(&s_bq25622);
#endif
    }
#if (SYSTEM_MANAGER_POWER_DEBUG_ENABLE != 0U)
    printf("[System] BQ25622 charge config status=%u iindpm=%u mA vreg=%u mV ichg=%u mA\r\n",
           (unsigned int)status,
           (unsigned int)SYSTEM_MANAGER_BQ25622_INPUT_CURRENT_MA,
           (unsigned int)SYSTEM_MANAGER_BQ25622_CHARGE_VOLTAGE_MV,
           (unsigned int)SYSTEM_MANAGER_BQ25622_CHARGE_CURRENT_MA);
#endif
    if (status != GLOVE_STATUS_OK)
    {
        SystemManager_UpdateBatteryFailure(status);
    }

    status = Max17043_Init(&s_max17043, I2C_BUS_1, SYSTEM_MANAGER_MAX17043_TIMEOUT_MS);
    if (status != GLOVE_STATUS_OK)
    {
        SystemManager_UpdateBatteryFailure(status);
    }

    period_ticks = SystemManager_MsToTicks(SYSTEM_MANAGER_LOOP_PERIOD_MS);
    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        SystemManager_ServicePowerKey();

        if (battery_startup_done == 0U)
        {
            battery_startup_elapsed_ms += SYSTEM_MANAGER_LOOP_PERIOD_MS;
            if (battery_startup_elapsed_ms >= SYSTEM_MANAGER_BATTERY_STARTUP_DELAY_MS)
            {
                battery_startup_done = 1U;
                battery_read_elapsed_ms = SYSTEM_MANAGER_BATTERY_READ_PERIOD_MS;
            }
        }
        else
        {
            battery_read_elapsed_ms += SYSTEM_MANAGER_LOOP_PERIOD_MS;
            if (battery_read_elapsed_ms >= SYSTEM_MANAGER_BATTERY_READ_PERIOD_MS)
            {
                battery_read_elapsed_ms = 0U;
                status = Max17043_ReadBattery(&s_max17043, &battery_data);
                if (status == GLOVE_STATUS_OK)
                {
                    SystemManager_UpdateBatterySuccess(&battery_data);
                }
                else
                {
                    SystemManager_UpdateBatteryFailure(status);
                }

#if (SYSTEM_MANAGER_POWER_DEBUG_ENABLE != 0U)
                power_debug_elapsed_ms += SYSTEM_MANAGER_BATTERY_READ_PERIOD_MS;
                if (power_debug_elapsed_ms >= SYSTEM_MANAGER_POWER_DEBUG_PERIOD_MS)
                {
                    power_debug_elapsed_ms = 0U;
                    SystemManager_PrintBatteryDebug(status,
                                                    (status == GLOVE_STATUS_OK) ? &battery_data : NULL);
                    (void)Bq25622_DumpDebugRegisters(&s_bq25622);
                }
#endif
            }
        }

        next_wake_tick += period_ticks;
        (void)osDelayUntil(next_wake_tick);
    }
}
