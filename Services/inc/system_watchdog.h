#ifndef SYSTEM_WATCHDOG_H
#define SYSTEM_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SYSTEM_WATCHDOG_STATUS_RUNNING       (1U << 0)
#define SYSTEM_WATCHDOG_STATUS_REFRESH_OK    (1U << 1)
#define SYSTEM_WATCHDOG_STATUS_CONFIG_WARNING (1U << 2)

#define SYSTEM_RESET_CAUSE_PIN               (1U << 0)
#define SYSTEM_RESET_CAUSE_POWER             (1U << 1)
#define SYSTEM_RESET_CAUSE_SOFTWARE          (1U << 2)
#define SYSTEM_RESET_CAUSE_IWDG              (1U << 3)
#define SYSTEM_RESET_CAUSE_WWDG              (1U << 4)
#define SYSTEM_RESET_CAUSE_LOW_POWER         (1U << 5)

typedef struct
{
    uint16_t reset_cause;
    uint16_t status_flags;
    uint32_t refresh_count;
} SystemWatchdogStatus_t;

void SystemWatchdog_CaptureResetCause(void);
void SystemWatchdog_Start(void);
void SystemWatchdog_Refresh(void);
void SystemWatchdog_GetStatus(SystemWatchdogStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_WATCHDOG_H */
