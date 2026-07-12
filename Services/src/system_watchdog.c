#include "system_watchdog.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h5xx_hal.h"

/* 32kHz标称LSI，128分频、重装值1999，对应约8秒超时。 */
#define SYSTEM_WATCHDOG_RELOAD_VALUE    (1999U)

static IWDG_HandleTypeDef s_iwdg;
static volatile uint16_t s_reset_cause;
static volatile uint16_t s_status_flags;
static volatile uint32_t s_refresh_count;

static uint16_t SystemWatchdog_ReadResetCause(void)
{
    uint16_t cause = 0U;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U) cause |= SYSTEM_RESET_CAUSE_PIN;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != 0U) cause |= SYSTEM_RESET_CAUSE_POWER;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U) cause |= SYSTEM_RESET_CAUSE_SOFTWARE;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U) cause |= SYSTEM_RESET_CAUSE_IWDG;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != 0U) cause |= SYSTEM_RESET_CAUSE_WWDG;
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != 0U) cause |= SYSTEM_RESET_CAUSE_LOW_POWER;

    return cause;
}

void SystemWatchdog_CaptureResetCause(void)
{
    s_reset_cause = SystemWatchdog_ReadResetCause();
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void SystemWatchdog_Start(void)
{
    if ((s_status_flags & SYSTEM_WATCHDOG_STATUS_RUNNING) != 0U) return;

    s_iwdg.Instance = IWDG;
    s_iwdg.Init.Prescaler = IWDG_PRESCALER_128;
    s_iwdg.Init.Reload = SYSTEM_WATCHDOG_RELOAD_VALUE;
    s_iwdg.Init.Window = IWDG_WINDOW_DISABLE;
    s_iwdg.Init.EWI = IWDG_EWI_DISABLE;

    /* 直接配置IWDG，禁止最高优先级任务进入HAL内部的状态等待循环。 */
    __HAL_IWDG_START(&s_iwdg);
    IWDG_ENABLE_WRITE_ACCESS(&s_iwdg);
    WRITE_REG(s_iwdg.Instance->PR, s_iwdg.Init.Prescaler);
    WRITE_REG(s_iwdg.Instance->RLR, s_iwdg.Init.Reload);
    WRITE_REG(s_iwdg.Instance->EWCR, IWDG_EWCR_EWIC);
    WRITE_REG(s_iwdg.Instance->WINR, s_iwdg.Init.Window);
    s_status_flags |= SYSTEM_WATCHDOG_STATUS_RUNNING;
    SystemWatchdog_Refresh();
}

void SystemWatchdog_Refresh(void)
{
    if ((s_status_flags & SYSTEM_WATCHDOG_STATUS_RUNNING) == 0U) return;

    __HAL_IWDG_RELOAD_COUNTER(&s_iwdg);
    s_status_flags |= SYSTEM_WATCHDOG_STATUS_REFRESH_OK;
    s_refresh_count++;

    /* 异步更新通常很快完成，持续一秒仍未生效时再报告配置警告。 */
    if (s_refresh_count == 10U)
    {
        if (((READ_REG(s_iwdg.Instance->PR) & IWDG_PR_PR) != s_iwdg.Init.Prescaler) ||
            ((READ_REG(s_iwdg.Instance->RLR) & IWDG_RLR_RL) != s_iwdg.Init.Reload))
        {
            s_status_flags |= SYSTEM_WATCHDOG_STATUS_CONFIG_WARNING;
        }
    }
}

void SystemWatchdog_GetStatus(SystemWatchdogStatus_t *status)
{
    if (status == NULL) return;

    taskENTER_CRITICAL();
    status->reset_cause = s_reset_cause;
    status->status_flags = s_status_flags;
    status->refresh_count = s_refresh_count;
    taskEXIT_CRITICAL();
}
