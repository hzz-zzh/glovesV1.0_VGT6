#include "system_health.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

#define SYSTEM_HEALTH_FULL_FRAME_TIMEOUT_MS     (100U)
#define SYSTEM_HEALTH_POWER_INIT                (0U)
#define SYSTEM_HEALTH_POWER_ON_NORMAL           (1U)
#define SYSTEM_HEALTH_POWER_ON_LOW              (2U)
#define SYSTEM_HEALTH_POWER_USER_OFF            (3U)
#define SYSTEM_HEALTH_POWER_LOW_BAT_LOCKOUT     (4U)
#define SYSTEM_HEALTH_POWER_STOPPING            (5U)
#define SYSTEM_HEALTH_POWER_RECOVERING          (6U)
#define SYSTEM_HEALTH_POWER_RECOVERY_FAULT      (7U)

typedef struct
{
    uint16_t error;
    uint16_t source;
    uint16_t target;
} SystemHealthFaultInfo_t;

static SystemHealthSnapshot_t s_health;
static SystemHealthFaultInfo_t s_fault_info[32];
static uint8_t s_power_state;
static uint8_t s_full_frame_seen;
static uint32_t s_last_full_frame_ms;
static uint16_t s_imu_recovery_stage;
static uint16_t s_imu_recovery_target;
static uint16_t s_imu_recovery_attempt;
static uint16_t s_power_recovery_stage;

static const uint8_t s_fault_priority[] =
{
    13U, 11U, 10U, 6U, 8U, 9U, 1U, 2U, 3U, 4U, 0U,
    14U, 15U, 18U, 17U, 16U, 19U, 23U, 24U, 22U, 26U,
    25U, 27U, 12U, 5U, 7U, 20U, 21U
};

static uint8_t SystemHealth_FlagIndex(uint32_t flag, uint8_t *index)
{
    if ((flag == 0UL) || ((flag & (flag - 1UL)) != 0UL) || (index == NULL))
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < 32U; i++)
    {
        if (flag == (1UL << i))
        {
            *index = i;
            return 1U;
        }
    }
    return 0U;
}

static void SystemHealth_SelectCurrentError(void)
{
    s_health.current_error = SYSTEM_ERROR_NONE;
    s_health.current_source = SYSTEM_HEALTH_SOURCE_NONE;
    s_health.current_target = 0U;

    for (uint32_t i = 0U;
         i < (sizeof(s_fault_priority) / sizeof(s_fault_priority[0]));
         i++)
    {
        uint8_t index = s_fault_priority[i];
        if ((s_health.current_flags & (1UL << index)) != 0UL)
        {
            s_health.current_error = s_fault_info[index].error;
            s_health.current_source = s_fault_info[index].source;
            s_health.current_target = s_fault_info[index].target;
            return;
        }
    }
}

static void SystemHealth_ClearFaultMask(uint32_t mask)
{
    s_health.current_flags &= ~mask;
    for (uint8_t index = 0U; index < 32U; index++)
    {
        if ((mask & (1UL << index)) != 0UL)
        {
            (void)memset(&s_fault_info[index], 0, sizeof(s_fault_info[index]));
        }
    }
    SystemHealth_SelectCurrentError();
}

static void SystemHealth_UpdateRecovery(void)
{
    /* 整机断电恢复优先于节点级恢复，避免两个状态机互相覆盖对外阶段。 */
    if (s_power_recovery_stage != SYSTEM_RECOVERY_NONE)
    {
        s_health.recovery_stage = s_power_recovery_stage;
        s_health.recovery_attempt = 0U;
    }
    else
    {
        s_health.recovery_stage = s_imu_recovery_stage;
        s_health.recovery_attempt = s_imu_recovery_attempt;
    }
}

static uint16_t SystemHealth_ComputeState(void)
{
    /* 电源锁定和恢复流程优先于普通告警，确保对外状态只有一个明确主结论。 */
    const uint32_t degraded_flags =
        SYSTEM_HEALTH_FLAG_IMU_PARTIAL |
        SYSTEM_HEALTH_FLAG_IMU_ALL_INVALID |
        SYSTEM_HEALTH_FLAG_TOUCH_INVALID |
        SYSTEM_HEALTH_FLAG_FRAME_STALE |
        SYSTEM_HEALTH_FLAG_JOINT_INVALID |
        SYSTEM_HEALTH_FLAG_CAN1_ERROR_PASSIVE |
        SYSTEM_HEALTH_FLAG_CAN1_BUS_OFF |
        SYSTEM_HEALTH_FLAG_CAN2_ERROR_PASSIVE |
        SYSTEM_HEALTH_FLAG_CAN2_BUS_OFF |
        SYSTEM_HEALTH_FLAG_QUEUE_PRESSURE |
        SYSTEM_HEALTH_FLAG_POOL_EXHAUSTED;
    const uint32_t warning_flags =
        SYSTEM_HEALTH_FLAG_LOW_BATTERY |
        SYSTEM_HEALTH_FLAG_CRITICAL_BATTERY |
        SYSTEM_HEALTH_FLAG_BQ_COMM |
        SYSTEM_HEALTH_FLAG_GAUGE_COMM |
        SYSTEM_HEALTH_FLAG_VOLTAGE_MISMATCH |
        SYSTEM_HEALTH_FLAG_TEMP_LIMIT |
        SYSTEM_HEALTH_FLAG_CHARGE_FAULT |
        SYSTEM_HEALTH_FLAG_WATCHDOG_WARNING |
        SYSTEM_HEALTH_FLAG_TIME_UNSYNCED |
        SYSTEM_HEALTH_FLAG_CALIBRATION_ERROR |
        SYSTEM_HEALTH_FLAG_RS485_RX_OVERWRITE |
        SYSTEM_HEALTH_FLAG_RS485_UART_ERROR |
        SYSTEM_HEALTH_FLAG_RS485_TX_FAILED |
        SYSTEM_HEALTH_FLAG_SD_ERROR;

    if (s_power_state == SYSTEM_HEALTH_POWER_LOW_BAT_LOCKOUT)
    {
        return SYSTEM_HEALTH_LOCKOUT;
    }
    if ((s_power_state == SYSTEM_HEALTH_POWER_RECOVERY_FAULT) ||
        (s_health.recovery_stage == SYSTEM_RECOVERY_FAILED))
    {
        return SYSTEM_HEALTH_FAULT;
    }
    if ((s_power_state == SYSTEM_HEALTH_POWER_STOPPING) ||
        (s_power_state == SYSTEM_HEALTH_POWER_RECOVERING) ||
        (s_health.recovery_stage != SYSTEM_RECOVERY_NONE))
    {
        return SYSTEM_HEALTH_RECOVERING;
    }
    if (s_power_state == SYSTEM_HEALTH_POWER_USER_OFF)
    {
        return SYSTEM_HEALTH_OFF;
    }
    if (s_power_state == SYSTEM_HEALTH_POWER_INIT)
    {
        return SYSTEM_HEALTH_INIT;
    }
    if ((s_health.current_flags & degraded_flags) != 0UL)
    {
        return SYSTEM_HEALTH_DEGRADED;
    }
    if ((s_health.current_flags & warning_flags) != 0UL)
    {
        return SYSTEM_HEALTH_WARNING;
    }
    return SYSTEM_HEALTH_OK;
}

void SystemHealth_Init(void)
{
    taskENTER_CRITICAL();
    (void)memset(&s_health, 0, sizeof(s_health));
    (void)memset(s_fault_info, 0, sizeof(s_fault_info));
    s_health.version = SYSTEM_HEALTH_VERSION;
    s_health.state = SYSTEM_HEALTH_INIT;
    s_health.snapshot_age_ms = 0xFFFFU;
    s_power_state = SYSTEM_HEALTH_POWER_INIT;
    s_full_frame_seen = 0U;
    s_last_full_frame_ms = 0U;
    s_imu_recovery_stage = SYSTEM_RECOVERY_NONE;
    s_imu_recovery_target = 0U;
    s_imu_recovery_attempt = 0U;
    s_power_recovery_stage = SYSTEM_RECOVERY_NONE;
    taskEXIT_CRITICAL();
}

void SystemHealth_SetPowerState(uint8_t power_state)
{
    taskENTER_CRITICAL();
    s_power_state = power_state;
    if ((power_state == SYSTEM_HEALTH_POWER_ON_NORMAL) ||
        (power_state == SYSTEM_HEALTH_POWER_ON_LOW))
    {
        s_health.sensor_ready_flags |= SYSTEM_SENSOR_READY_POWER;
    }
    else
    {
        s_health.sensor_ready_flags &= (uint16_t)~(SYSTEM_SENSOR_READY_POWER |
                                                   SYSTEM_SENSOR_READY_FULL_FRAME |
                                                   SYSTEM_SENSOR_READY_JOINT);
        if ((power_state == SYSTEM_HEALTH_POWER_LOW_BAT_LOCKOUT) ||
            ((power_state == SYSTEM_HEALTH_POWER_USER_OFF) &&
             (s_health.recovery_stage == SYSTEM_RECOVERY_NONE)))
        {
            /* 外设确实处于关闭状态时，当前传感器故障清空，历史错误仍保留。 */
            SystemHealth_ClearFaultMask(SYSTEM_HEALTH_FLAG_IMU_PARTIAL |
                                        SYSTEM_HEALTH_FLAG_IMU_ALL_INVALID |
                                        SYSTEM_HEALTH_FLAG_TOUCH_INVALID |
                                        SYSTEM_HEALTH_FLAG_FRAME_STALE |
                                        SYSTEM_HEALTH_FLAG_JOINT_INVALID |
                                        SYSTEM_HEALTH_FLAG_CAN1_ERROR_PASSIVE |
                                        SYSTEM_HEALTH_FLAG_CAN1_BUS_OFF |
                                        SYSTEM_HEALTH_FLAG_CAN2_ERROR_PASSIVE |
                                        SYSTEM_HEALTH_FLAG_CAN2_BUS_OFF |
                                        SYSTEM_HEALTH_FLAG_IMU_CONFIG_FAILED |
                                        SYSTEM_HEALTH_FLAG_CAN_REINIT_FAILED |
                                        SYSTEM_HEALTH_FLAG_POWER_RECOVERY_FAIL |
                                        SYSTEM_HEALTH_FLAG_QUEUE_PRESSURE |
                                        SYSTEM_HEALTH_FLAG_POOL_EXHAUSTED);
        }
    }
    s_health.state = SystemHealth_ComputeState();
    taskEXIT_CRITICAL();
}

void SystemHealth_SetFault(uint32_t flag,
                           uint16_t error,
                           SystemHealthSource_t source,
                           uint16_t target,
                           uint8_t active)
{
    uint8_t index;
    uint8_t rising = 0U;

    if (SystemHealth_FlagIndex(flag, &index) == 0U)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (active != 0U)
    {
        rising = ((s_health.current_flags & flag) == 0UL) ? 1U : 0U;
        if ((rising == 0U) &&
            ((s_fault_info[index].error != error) ||
             (s_fault_info[index].source != (uint16_t)source) ||
             (s_fault_info[index].target != target)))
        {
            /* 同一故障位切换到新节点或新根因时，也保留为一次独立事件。 */
            rising = 1U;
        }
        s_health.current_flags |= flag;
        s_fault_info[index].error = error;
        s_fault_info[index].source = (uint16_t)source;
        s_fault_info[index].target = target;
        if (rising != 0U)
        {
            s_health.last_error = error;
            s_health.last_source = (uint16_t)source;
            s_health.last_target = target;
            s_health.error_seq++;
            s_health.error_count++;
            s_health.last_error_uptime_ms = HAL_GetTick();
        }
    }
    else
    {
        s_health.current_flags &= ~flag;
        (void)memset(&s_fault_info[index], 0, sizeof(s_fault_info[index]));
    }
    SystemHealth_SelectCurrentError();
    s_health.state = SystemHealth_ComputeState();
    taskEXIT_CRITICAL();
}

void SystemHealth_RecordEvent(uint16_t error,
                              SystemHealthSource_t source,
                              uint16_t target)
{
    taskENTER_CRITICAL();
    s_health.last_error = error;
    s_health.last_source = (uint16_t)source;
    s_health.last_target = target;
    s_health.error_seq++;
    s_health.error_count++;
    s_health.last_error_uptime_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

void SystemHealth_SetRecovery(SystemRecoveryStage_t stage,
                              uint16_t target,
                              uint8_t attempt,
                              uint8_t attempt_limit)
{
    taskENTER_CRITICAL();
    s_imu_recovery_stage = (uint16_t)stage;
    s_imu_recovery_target = (stage != SYSTEM_RECOVERY_NONE) ? target : 0U;
    s_imu_recovery_attempt = (uint16_t)(((uint16_t)attempt_limit << 8) | attempt);
    SystemHealth_UpdateRecovery();
    s_health.state = SystemHealth_ComputeState();
    taskEXIT_CRITICAL();
}

void SystemHealth_SetPowerRecovery(SystemRecoveryStage_t stage)
{
    taskENTER_CRITICAL();
    s_power_recovery_stage = (uint16_t)stage;
    SystemHealth_UpdateRecovery();
    s_health.state = SystemHealth_ComputeState();
    taskEXIT_CRITICAL();
}

void SystemHealth_SetLiveImuMask(uint16_t mask)
{
    taskENTER_CRITICAL();
    s_health.live_imu_mask = mask;
    if (mask == 0xFFFFU)
    {
        s_health.sensor_ready_flags |= SYSTEM_SENSOR_READY_IMU_ALL;
    }
    else
    {
        s_health.sensor_ready_flags &= (uint16_t)~SYSTEM_SENSOR_READY_IMU_ALL;
    }
    taskEXIT_CRITICAL();
}

void SystemHealth_SetSensorReady(uint16_t ready_flag, uint8_t ready)
{
    taskENTER_CRITICAL();
    if (ready != 0U)
    {
        s_health.sensor_ready_flags |= ready_flag;
    }
    else
    {
        s_health.sensor_ready_flags &= (uint16_t)~ready_flag;
    }
    taskEXIT_CRITICAL();
}

void SystemHealth_SetRs485UartDetail(uint16_t detail)
{
    taskENTER_CRITICAL();
    /* 保留最近一次HAL UART错误位，避免后续其他RS485事件覆盖诊断信息。 */
    s_health.rs485_uart_detail = detail;
    taskEXIT_CRITICAL();
}

void SystemHealth_ClearHistory(void)
{
    taskENTER_CRITICAL();
    /* 只清除历史记录，当前故障、恢复阶段和传感器就绪状态保持不变。 */
    s_health.last_error = SYSTEM_ERROR_NONE;
    s_health.last_source = SYSTEM_HEALTH_SOURCE_NONE;
    s_health.last_target = 0U;
    s_health.error_seq = 0UL;
    s_health.error_count = 0UL;
    s_health.last_error_uptime_ms = 0UL;
    s_health.rs485_uart_detail = 0U;
    taskEXIT_CRITICAL();
}

void SystemHealth_MarkFullFrame(uint8_t joint_valid)
{
    taskENTER_CRITICAL();
    s_last_full_frame_ms = HAL_GetTick();
    s_full_frame_seen = 1U;
    s_health.sensor_ready_flags |= SYSTEM_SENSOR_READY_FULL_FRAME;
    if (joint_valid != 0U)
    {
        s_health.sensor_ready_flags |= SYSTEM_SENSOR_READY_JOINT;
    }
    else
    {
        s_health.sensor_ready_flags &= (uint16_t)~SYSTEM_SENSOR_READY_JOINT;
    }
    taskEXIT_CRITICAL();

    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_JOINT_INVALID,
                          SYSTEM_ERROR_ALGORITHM_INVALID,
                          SYSTEM_HEALTH_SOURCE_PIPELINE,
                          0U,
                          (joint_valid == 0U) ? 1U : 0U);
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_FRAME_STALE,
                          SYSTEM_ERROR_FRAME_STALE,
                          SYSTEM_HEALTH_SOURCE_PIPELINE,
                          0U,
                          0U);
}

void SystemHealth_Service(void)
{
    uint8_t power_ready;
    uint8_t frame_seen;
    uint32_t last_frame_ms;
    uint32_t age_ms;

    taskENTER_CRITICAL();
    power_ready = ((s_power_state == SYSTEM_HEALTH_POWER_ON_NORMAL) ||
                   (s_power_state == SYSTEM_HEALTH_POWER_ON_LOW)) ? 1U : 0U;
    frame_seen = s_full_frame_seen;
    last_frame_ms = s_last_full_frame_ms;
    taskEXIT_CRITICAL();

    /* 只在外设正常供电时判定FullFrame超时，用户主动关机不记作传感器故障。 */
    age_ms = (frame_seen != 0U) ? (uint32_t)(HAL_GetTick() - last_frame_ms) : 0xFFFFFFFFUL;
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_FRAME_STALE,
                          SYSTEM_ERROR_FRAME_STALE,
                          SYSTEM_HEALTH_SOURCE_PIPELINE,
                          0U,
                          ((power_ready != 0U) &&
                           ((frame_seen == 0U) ||
                            (age_ms > SYSTEM_HEALTH_FULL_FRAME_TIMEOUT_MS))) ? 1U : 0U);

    taskENTER_CRITICAL();
    if ((power_ready != 0U) && (frame_seen != 0U) &&
        (age_ms <= SYSTEM_HEALTH_FULL_FRAME_TIMEOUT_MS))
    {
        s_health.sensor_ready_flags |= SYSTEM_SENSOR_READY_FULL_FRAME;
    }
    else
    {
        s_health.sensor_ready_flags &= (uint16_t)~SYSTEM_SENSOR_READY_FULL_FRAME;
    }
    s_health.state = SystemHealth_ComputeState();
    taskEXIT_CRITICAL();
}

void SystemHealth_GetSnapshot(SystemHealthSnapshot_t *snapshot)
{
    uint32_t age_ms;

    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_health;
    if ((s_power_recovery_stage == SYSTEM_RECOVERY_NONE) &&
        (s_imu_recovery_stage != SYSTEM_RECOVERY_NONE) &&
        (s_imu_recovery_target != 0U))
    {
        snapshot->current_target = s_imu_recovery_target;
    }
    if (s_full_frame_seen != 0U)
    {
        age_ms = (uint32_t)(HAL_GetTick() - s_last_full_frame_ms);
        snapshot->snapshot_age_ms = (age_ms > 0xFFFFUL) ? 0xFFFFU : (uint16_t)age_ms;
    }
    else
    {
        snapshot->snapshot_age_ms = 0xFFFFU;
    }
    taskEXIT_CRITICAL();
}
