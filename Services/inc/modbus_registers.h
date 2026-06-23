#ifndef __MODBUS_REGISTERS_H__
#define __MODBUS_REGISTERS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

#define MODBUS_SLAVE_ADDR_DEFAULT      0x01U

/* Register width helpers. */
#define MODBUS_REGS_U16                1U
#define MODBUS_REGS_U32                2U
#define MODBUS_REGS_FLOAT32            2U
#define MODBUS_REGS_U64                4U

/* Basic communication and time registers: 0x0000 ~ 0x000D. */
#define REG_SLAVE_ADDR                 0x0000U
#define REG_BAUDRATE_CODE              0x0001U
#define REG_UTC_TIMESTAMP_US           0x0002U
#define REG_LOCAL_UPTIME_US            0x0006U
#define REG_TIME_SYNC_UTC_US           0x000AU

#define REG_BASIC_STATUS_START         REG_SLAVE_ADDR
#define REG_BASIC_STATUS_END           0x000DU
#define REG_BASIC_STATUS_COUNT         14U

/* Command registers: 0x0020 ~ 0x003E. */
#define REG_CMD                        0x0020U
#define REG_CMD_PARAM                  0x0021U
#define REG_CMD_SEQ                    0x0022U
#define REG_CMD_ACK                    0x0023U
#define REG_CMD_ACK_SEQ                0x0024U
#define REG_CMD_ERROR                  0x0025U
#define REG_CMD_RESERVED_START         0x0026U
#define REG_CMD_RESERVED_END           0x003EU

#define REG_CMD_AREA_START             REG_CMD
#define REG_CMD_AREA_END               REG_CMD_RESERVED_END
#define REG_CMD_ACK_START              REG_CMD_ACK
#define REG_CMD_ACK_COUNT              3U

/* System status registers: 0x0040 ~ 0x0049. */
#define REG_SYSTEM_STATE               0x0040U
#define REG_WORK_MODE                  0x0041U
#define REG_LOG_STATE                  0x0042U
#define REG_SD_STATE                   0x0043U
#define REG_SENSOR_STATE               0x0044U
#define REG_COMM_STATE                 0x0045U
#define REG_SYSTEM_RESERVED_START      0x0046U
#define REG_SYSTEM_RESERVED_END        0x0047U
#define REG_TEMPERATURE_BOARD          0x0048U

#define REG_SYSTEM_STATUS_START        REG_SYSTEM_STATE
#define REG_SYSTEM_STATUS_END          0x0049U
#define REG_SYSTEM_STATUS_COUNT        10U

/* System status values. */
#define SYSTEM_STATE_READY             0x0001U
#define WORK_MODE_NORMAL               0x0000U
#define LOG_STATE_IDLE                 0x0000U
#define SD_STATE_NOT_READY             0x0000U
#define SD_STATE_READY                 0x0001U
#define SENSOR_STATE_ALL_OK            0xFFFFU
#define COMM_STATE_OK                  0x0001U

/* Power status registers: 0x0060 ~ 0x0063. */
#define REG_BAT_VOLTAGE                0x0060U
#define REG_BAT_CURRENT                0x0062U

#define REG_POWER_STATUS_START         REG_BAT_VOLTAGE
#define REG_POWER_STATUS_END           0x0063U
#define REG_POWER_STATUS_COUNT         4U

/* SD card and log status registers: 0x0081 ~ 0x00BF. */
#define REG_SD_FS_STATUS               0x0081U
#define REG_SD_LOG_STATUS              0x0082U
#define REG_SD_ERROR_CODE              0x0083U
#define REG_SD_TOTAL_SIZE_MB           0x0084U
#define REG_SD_FREE_SIZE_MB            0x0086U
#define REG_SD_USED_SIZE_MB            0x0088U
#define REG_SD_CURRENT_FILE_ID         0x008AU
#define REG_SD_CURRENT_FILE_SIZE       0x008CU
#define REG_SD_CURRENT_WRITE_CNT       0x0090U
#define REG_SD_LOG_CREATE_FILE         0x0092U
#define REG_SD_RESERVED0_START         0x0093U
#define REG_SD_RESERVED0_END           0x0099U
#define REG_SD_LOG_LENGTH              0x009AU
#define REG_SD_RESERVED1_START         0x009EU
#define REG_SD_RESERVED1_END           0x009FU
#define REG_SD_CURRENT_FILENAME        0x00A0U
#define REG_SD_LAST_FILENAME           0x00B0U

#define REG_SD_STATUS_START            REG_SD_FS_STATUS
#define REG_SD_STATUS_END              0x00BFU
#define REG_SD_STATUS_COUNT            63U
#define REG_SD_FILENAME_REG_COUNT      16U

/* SD and log status values. */
#define SD_FS_STATUS_NOT_MOUNTED       0x0000U
#define SD_FS_STATUS_MOUNTED           0x0001U
#define SD_LOG_STATUS_IDLE             0x0000U
#define SD_LOG_STATUS_RECORDING        0x0001U
#define SD_ERROR_NONE                  0x0000U

/* Work status register. */
#define REG_WORK_STATE                 0x0500U

/* IMU data, timestamp, status and calibration registers. */
#define MODBUS_IMU_COUNT               16U
#define MODBUS_IMU_FLOATS_PER_UNIT     10U
#define MODBUS_IMU_REGS_PER_UNIT       20U
#define MODBUS_IMU_DATA_FLOAT_COUNT    160U
#define MODBUS_IMU_DATA_REG_COUNT      320U
#define MODBUS_IMU_CALIB_FLOATS_PER_UNIT 4U
#define MODBUS_IMU_CALIB_REGS_PER_UNIT   (MODBUS_IMU_CALIB_FLOATS_PER_UNIT * MODBUS_REGS_FLOAT32)
#define MODBUS_IMU_CALIB_TABLE_REG_COUNT (MODBUS_IMU_COUNT * MODBUS_IMU_CALIB_REGS_PER_UNIT)

#define REG_IMU_DATA_START             0x1000U
#define REG_IMU_DATA_END               0x113FU
#define REG_IMU_TIMESTAMP_US           0x1140U
#define REG_IMU_STATUS_BITS            0x1144U
#define REG_IMU_OFFSET_START           0x1154U
#define REG_IMU_OFFSET_END             0x1293U

#define REG_IMU_CALIB_START            REG_IMU_OFFSET_START
#define REG_IMU_CALIB_END              REG_IMU_OFFSET_END
#define REG_IMU_CALIB_C_START          REG_IMU_CALIB_START
#define REG_IMU_CALIB_C_END            (REG_IMU_CALIB_C_START + MODBUS_IMU_CALIB_TABLE_REG_COUNT - 1U)
#define REG_IMU_CALIB_M_START          (REG_IMU_CALIB_C_END + 1U)
#define REG_IMU_CALIB_M_END            (REG_IMU_CALIB_M_START + MODBUS_IMU_CALIB_TABLE_REG_COUNT - 1U)
#define REG_IMU_CALIB_CTRL_START       (REG_IMU_CALIB_M_END + 1U)
#define REG_IMU_CALIB_CTRL_END         REG_IMU_CALIB_END

#define REG_IMU_CALIB_MAGIC            REG_IMU_CALIB_CTRL_START
#define REG_IMU_CALIB_COMMAND          (REG_IMU_CALIB_CTRL_START + 1U)
#define REG_IMU_CALIB_SEQ              (REG_IMU_CALIB_CTRL_START + 2U)
#define REG_IMU_CALIB_STATUS           (REG_IMU_CALIB_CTRL_START + 3U)
#define REG_IMU_CALIB_ERROR_INDEX      (REG_IMU_CALIB_CTRL_START + 4U)
#define REG_IMU_CALIB_LAST_APPLIED_SEQ (REG_IMU_CALIB_CTRL_START + 5U)

#define IMU_CALIB_MAGIC_VALUE          0xCA1BU
#define IMU_CALIB_CMD_NONE             0x0000U
#define IMU_CALIB_CMD_APPLY            0x0001U
#define IMU_CALIB_CMD_RESET_IDENTITY   0x0002U

#define IMU_CALIB_STATUS_IDLE          0x0000U
#define IMU_CALIB_STATUS_APPLIED       0x0001U
#define IMU_CALIB_STATUS_RESET_DONE    0x0002U
#define IMU_CALIB_STATUS_BAD_MAGIC     0x8001U
#define IMU_CALIB_STATUS_BAD_CMD       0x8002U
#define IMU_CALIB_STATUS_BAD_QUAT      0x8003U
#define IMU_CALIB_ERROR_NONE           0xFFFFU

#define REG_IMU_STATUS_START           REG_IMU_STATUS_BITS
#define REG_IMU_STATUS_COUNT           1U

#define REG_IMU_DATA_ADDR(index)       (REG_IMU_DATA_START + ((uint16_t)(index) * MODBUS_IMU_REGS_PER_UNIT))
#define REG_IMU_OFFSET_ADDR(index)     (REG_IMU_OFFSET_START + ((uint16_t)(index) * MODBUS_IMU_REGS_PER_UNIT))
#define REG_IMU_CALIB_C_ADDR(index)    (REG_IMU_CALIB_C_START + ((uint16_t)(index) * MODBUS_IMU_CALIB_REGS_PER_UNIT))
#define REG_IMU_CALIB_M_ADDR(index)    (REG_IMU_CALIB_M_START + ((uint16_t)(index) * MODBUS_IMU_CALIB_REGS_PER_UNIT))

#if REG_IMU_CALIB_M_END >= REG_IMU_CALIB_CTRL_START
#error "IMU calibration table overlaps calibration control registers"
#endif

#if REG_IMU_CALIB_CTRL_START > REG_IMU_CALIB_CTRL_END
#error "IMU calibration control registers exceed reserved IMU calibration range"
#endif

/* Joint angle data, timestamp and status registers. */
#define MODBUS_JOINT_COUNT             27U
#define MODBUS_JOINT_REGS_PER_VALUE    MODBUS_REGS_FLOAT32
#define MODBUS_JOINT_DATA_REG_COUNT    (MODBUS_JOINT_COUNT * MODBUS_JOINT_REGS_PER_VALUE)

#define REG_JOINT_DATA_START           0x1300U
#define REG_JOINT_DATA_END             (REG_JOINT_DATA_START + MODBUS_JOINT_DATA_REG_COUNT - 1U)
#define REG_JOINT_TIMESTAMP_US         0x1340U
#define REG_JOINT_STATUS_FLAGS         0x1344U
#define REG_JOINT_VALID_BITS_LOW       0x1345U
#define REG_JOINT_VALID_BITS_HIGH      0x1346U

#define REG_JOINT_STATUS_START         REG_JOINT_STATUS_FLAGS
#define REG_JOINT_STATUS_END           REG_JOINT_VALID_BITS_HIGH
#define REG_JOINT_STATUS_COUNT         3U
#define REG_JOINT_DATA_ADDR(index)     (REG_JOINT_DATA_START + ((uint16_t)(index) * MODBUS_JOINT_REGS_PER_VALUE))

#define JOINT_STATUS_SNAPSHOT_VALID    0x0001U
#define JOINT_STATUS_ALGORITHM_VALID   0x0002U

/* Touch/resistance matrix data, timestamp and status registers. */
#define MODBUS_R_POINT_COUNT           GLOVE_TOUCH_COUNT
#define MODBUS_R_STATUS_REG_COUNT      4U
#define MODBUS_R_DATA_REG_COUNT        128U

#define REG_R_DATA_START               0x2000U
#define REG_R_DATA_END                 0x207FU
#define REG_R_DATA_VALID_END           (REG_R_DATA_START + MODBUS_R_POINT_COUNT - 1U)
#define REG_R_TIMESTAMP_US             0x2080U
#define REG_R_STATUS_START             0x2084U
#define REG_R_STATUS_END               0x2087U
#define REG_R_STATUS_RESERVED_START    0x2088U
#define REG_R_STATUS_RESERVED_END      0x20C3U

#define REG_R_STATUS_FLAGS             REG_R_STATUS_START
#define REG_R_POINT_COUNT_REG          (REG_R_STATUS_START + 1U)
#define REG_R_DATA_REG_COUNT_REG       (REG_R_STATUS_START + 2U)
#define REG_R_STATUS_RESERVED          (REG_R_STATUS_START + 3U)

#define R_STATUS_SNAPSHOT_VALID        0x0001U
#define R_STATUS_TOUCH_VALID           0x0002U

#if MODBUS_R_POINT_COUNT > MODBUS_R_DATA_REG_COUNT
#error "MODBUS_R_DATA_REG_COUNT must cover all touch points"
#endif

/* Command values written to REG_CMD. */
#define CMD_NONE                       0x0000U
#define CMD_LOG_START                  0x0094U
#define CMD_LOG_STOP                   0x0096U
#define CMD_ACQ_START                  0x0501U
#define CMD_ACQ_STOP                   0x0502U

/* Command ACK values returned by REG_CMD_ACK. */
#define CMD_ACK_IDLE                   0x0000U
#define CMD_ACK_OK                     0x0001U
#define CMD_ACK_BUSY                   0x0002U
#define CMD_ACK_UNKNOWN_CMD            0x8001U
#define CMD_ACK_INVALID_PARAM          0x8002U
#define CMD_ACK_STATE_DENIED           0x8003U
#define CMD_ACK_FAILED                 0x8004U

/* Command error detail values returned by REG_CMD_ERROR. */
#define CMD_ERROR_NONE                 0x0000U
#define CMD_ERROR_INVALID_PARAM        0x0001U
#define CMD_ERROR_DUP_SEQ              0x0002U
#define CMD_ERROR_STATE_DENIED         0x0003U
#define CMD_ERROR_RESOURCE_NOT_READY   0x0004U
#define CMD_ERROR_START_FAILED         0x0005U
#define CMD_ERROR_STOP_FAILED          0x0006U
#define CMD_ERROR_TIMEOUT              0x0007U

/* Work state values returned by REG_WORK_STATE. */
#define WORK_STATE_IDLE                0x0000U
#define WORK_STATE_ACQUIRING           0x0001U
#define WORK_STATE_STOPPING            0x0002U
#define WORK_STATE_ERROR               0x8000U

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_REGISTERS_H__ */
