#ifndef SYSTEM_HEALTH_H
#define SYSTEM_HEALTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SYSTEM_HEALTH_VERSION                    (0x0102U)

typedef enum
{
    SYSTEM_HEALTH_INIT = 0,
    SYSTEM_HEALTH_OK = 1,
    SYSTEM_HEALTH_WARNING = 2,
    SYSTEM_HEALTH_DEGRADED = 3,
    SYSTEM_HEALTH_RECOVERING = 4,
    SYSTEM_HEALTH_FAULT = 5,
    SYSTEM_HEALTH_OFF = 6,
    SYSTEM_HEALTH_LOCKOUT = 7
} SystemHealthState_t;

typedef enum
{
    SYSTEM_HEALTH_SOURCE_NONE = 0,
    SYSTEM_HEALTH_SOURCE_IMU = 1,
    SYSTEM_HEALTH_SOURCE_CAN1 = 2,
    SYSTEM_HEALTH_SOURCE_CAN2 = 3,
    SYSTEM_HEALTH_SOURCE_TOUCH = 4,
    SYSTEM_HEALTH_SOURCE_PIPELINE = 5,
    SYSTEM_HEALTH_SOURCE_POWER = 6,
    SYSTEM_HEALTH_SOURCE_BATTERY = 7,
    SYSTEM_HEALTH_SOURCE_CHARGER = 8,
    SYSTEM_HEALTH_SOURCE_WATCHDOG = 9,
    SYSTEM_HEALTH_SOURCE_RS485 = 10,
    SYSTEM_HEALTH_SOURCE_TIME_SYNC = 11,
    SYSTEM_HEALTH_SOURCE_CALIBRATION = 12,
    SYSTEM_HEALTH_SOURCE_STORAGE = 13
} SystemHealthSource_t;

typedef enum
{
    SYSTEM_RECOVERY_NONE = 0,
    SYSTEM_RECOVERY_LOSS_CONFIRM = 1,
    SYSTEM_RECOVERY_NODE_CONFIG = 2,
    SYSTEM_RECOVERY_NODE_VERIFY = 3,
    SYSTEM_RECOVERY_BUS_REINIT = 4,
    SYSTEM_RECOVERY_BUS_CONFIG = 5,
    SYSTEM_RECOVERY_BUS_VERIFY = 6,
    SYSTEM_RECOVERY_SAFE_STOP = 7,
    SYSTEM_RECOVERY_POWER_OFF_HOLD = 8,
    SYSTEM_RECOVERY_POWER_START = 9,
    SYSTEM_RECOVERY_SENSOR_WAIT = 10,
    SYSTEM_RECOVERY_FRAME_VERIFY = 11,
    SYSTEM_RECOVERY_FAILED = 12
} SystemRecoveryStage_t;

#define SYSTEM_HEALTH_FLAG_IMU_PARTIAL          (1UL << 0)
#define SYSTEM_HEALTH_FLAG_IMU_ALL_INVALID      (1UL << 1)
#define SYSTEM_HEALTH_FLAG_TOUCH_INVALID        (1UL << 2)
#define SYSTEM_HEALTH_FLAG_FRAME_STALE          (1UL << 3)
#define SYSTEM_HEALTH_FLAG_JOINT_INVALID        (1UL << 4)
#define SYSTEM_HEALTH_FLAG_CAN1_ERROR_PASSIVE   (1UL << 5)
#define SYSTEM_HEALTH_FLAG_CAN1_BUS_OFF         (1UL << 6)
#define SYSTEM_HEALTH_FLAG_CAN2_ERROR_PASSIVE   (1UL << 7)
#define SYSTEM_HEALTH_FLAG_CAN2_BUS_OFF         (1UL << 8)
#define SYSTEM_HEALTH_FLAG_IMU_CONFIG_FAILED    (1UL << 9)
#define SYSTEM_HEALTH_FLAG_CAN_REINIT_FAILED    (1UL << 10)
#define SYSTEM_HEALTH_FLAG_POWER_RECOVERY_FAIL  (1UL << 11)
#define SYSTEM_HEALTH_FLAG_LOW_BATTERY          (1UL << 12)
#define SYSTEM_HEALTH_FLAG_CRITICAL_BATTERY     (1UL << 13)
#define SYSTEM_HEALTH_FLAG_BQ_COMM              (1UL << 14)
#define SYSTEM_HEALTH_FLAG_GAUGE_COMM           (1UL << 15)
#define SYSTEM_HEALTH_FLAG_VOLTAGE_MISMATCH     (1UL << 16)
#define SYSTEM_HEALTH_FLAG_TEMP_LIMIT           (1UL << 17)
#define SYSTEM_HEALTH_FLAG_CHARGE_FAULT         (1UL << 18)
#define SYSTEM_HEALTH_FLAG_WATCHDOG_WARNING     (1UL << 19)
#define SYSTEM_HEALTH_FLAG_TIME_UNSYNCED         (1UL << 20)
#define SYSTEM_HEALTH_FLAG_CALIBRATION_ERROR     (1UL << 21)
#define SYSTEM_HEALTH_FLAG_RS485_RX_OVERWRITE    (1UL << 22)
#define SYSTEM_HEALTH_FLAG_RS485_UART_ERROR      (1UL << 23)
#define SYSTEM_HEALTH_FLAG_RS485_TX_FAILED       (1UL << 24)
#define SYSTEM_HEALTH_FLAG_QUEUE_PRESSURE        (1UL << 25)
#define SYSTEM_HEALTH_FLAG_POOL_EXHAUSTED        (1UL << 26)
#define SYSTEM_HEALTH_FLAG_SD_ERROR              (1UL << 27)

#define SYSTEM_SENSOR_READY_IMU_ALL              (1U << 0)
#define SYSTEM_SENSOR_READY_TOUCH                (1U << 1)
#define SYSTEM_SENSOR_READY_FULL_FRAME           (1U << 2)
#define SYSTEM_SENSOR_READY_JOINT                 (1U << 3)
#define SYSTEM_SENSOR_READY_POWER                (1U << 4)
#define SYSTEM_SENSOR_READY_TIME_SYNC            (1U << 5)
#define SYSTEM_SENSOR_READY_RS485                (1U << 6)

#define SYSTEM_ERROR_NONE                        (0x0000U)
#define SYSTEM_ERROR_IMU_NODE_STALE              (0x1001U)
#define SYSTEM_ERROR_IMU_CONFIG_FAILED           (0x1002U)
#define SYSTEM_ERROR_CAN_ERROR_PASSIVE           (0x2001U)
#define SYSTEM_ERROR_CAN_BUS_OFF                 (0x2002U)
#define SYSTEM_ERROR_CAN_REINIT_FAILED           (0x2003U)
#define SYSTEM_ERROR_CAN_VERIFY_FAILED           (0x2004U)
#define SYSTEM_ERROR_TOUCH_SYNC_TIMEOUT          (0x3001U)
#define SYSTEM_ERROR_TOUCH_DMA_TIMEOUT           (0x3002U)
#define SYSTEM_ERROR_TOUCH_DMA_ERROR             (0x3003U)
#define SYSTEM_ERROR_FRAME_STALE                 (0x4001U)
#define SYSTEM_ERROR_FRAME_TIME_MISMATCH         (0x4002U)
#define SYSTEM_ERROR_QUEUE_FULL                  (0x4003U)
#define SYSTEM_ERROR_POOL_EXHAUSTED              (0x4004U)
#define SYSTEM_ERROR_ALGORITHM_INVALID           (0x4005U)
#define SYSTEM_ERROR_ACQ_PAUSE_TIMEOUT           (0x5001U)
#define SYSTEM_ERROR_SYNC_START_FAILED           (0x5002U)
#define SYSTEM_ERROR_POWER_RECOVERY_TIMEOUT      (0x5003U)
#define SYSTEM_ERROR_BATTERY_LOW                 (0x6001U)
#define SYSTEM_ERROR_BATTERY_CRITICAL            (0x6002U)
#define SYSTEM_ERROR_BQ_COMM                     (0x6003U)
#define SYSTEM_ERROR_GAUGE_COMM                  (0x6004U)
#define SYSTEM_ERROR_VOLTAGE_MISMATCH            (0x6005U)
#define SYSTEM_ERROR_TEMPERATURE_LIMIT           (0x6006U)
#define SYSTEM_ERROR_CHARGE_FAULT                (0x6007U)
#define SYSTEM_ERROR_WATCHDOG_CONFIG             (0x7001U)
#define SYSTEM_ERROR_RS485_RX_OVERWRITE          (0x8001U)
#define SYSTEM_ERROR_RS485_UART                  (0x8002U)
#define SYSTEM_ERROR_RS485_TX                    (0x8003U)
#define SYSTEM_ERROR_TIME_UNSYNCED                (0x8004U)
#define SYSTEM_ERROR_CALIBRATION                 (0x9001U)
#define SYSTEM_ERROR_SD                          (0xA001U)

typedef struct
{
    /* current_*随故障恢复而清除，last_*保留最近一次事件供上位机追溯。 */
    uint16_t version;
    uint16_t state;
    uint32_t current_flags;
    uint16_t current_error;
    uint16_t current_source;
    uint16_t current_target;
    uint16_t recovery_stage;
    uint16_t recovery_attempt;
    uint16_t last_error;
    uint16_t last_source;
    uint16_t last_target;
    uint32_t error_seq;
    uint32_t error_count;
    uint32_t last_error_uptime_ms;
    uint16_t live_imu_mask;
    uint16_t sensor_ready_flags;
    uint16_t snapshot_age_ms;
    uint16_t rs485_uart_detail;
} SystemHealthSnapshot_t;

void SystemHealth_Init(void);
void SystemHealth_Service(void);
void SystemHealth_SetPowerState(uint8_t power_state);
void SystemHealth_SetFault(uint32_t flag,
                           uint16_t error,
                           SystemHealthSource_t source,
                           uint16_t target,
                           uint8_t active);
void SystemHealth_RecordEvent(uint16_t error,
                              SystemHealthSource_t source,
                              uint16_t target);
void SystemHealth_SetRecovery(SystemRecoveryStage_t stage,
                              uint16_t target,
                              uint8_t attempt,
                              uint8_t attempt_limit);
void SystemHealth_SetPowerRecovery(SystemRecoveryStage_t stage);
void SystemHealth_SetLiveImuMask(uint16_t mask);
void SystemHealth_SetSensorReady(uint16_t ready_flag, uint8_t ready);
void SystemHealth_SetRs485UartDetail(uint16_t detail);
void SystemHealth_ClearHistory(void);
void SystemHealth_MarkFullFrame(uint8_t joint_valid);
void SystemHealth_GetSnapshot(SystemHealthSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_HEALTH_H */
