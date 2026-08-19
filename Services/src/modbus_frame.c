#include "modbus_frame.h"

#include <math.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "dataProcessTask.h"
#include "imuCanTask.h"
#include "modbus_registers.h"
#include "modbus_time_sync.h"
#include "systemManagerTask.h"
#include "system_watchdog.h"

#if MODBUS_JOINT_COUNT != GLOVE_JOINT_DOF_COUNT
#error "MODBUS_JOINT_COUNT must match GLOVE_JOINT_DOF_COUNT"
#endif

#define MODBUS_TIME_US_PER_SEC          1000000ULL
#define MODBUS_TIME_NS_PER_US           1000ULL
#define MODBUS_TIME_NS_PER_SEC          1000000000UL
#define MODBUS_SENSOR_SNAPSHOT_METADATA_REG_COUNT 10U
#define MODBUS_SENSOR_SNAPSHOT_REG_COUNT \
  (MODBUS_SENSOR_SNAPSHOT_METADATA_REG_COUNT + MODBUS_IMU_DATA_REG_COUNT + \
   MODBUS_JOINT_DATA_REG_COUNT + MODBUS_R_POINT_COUNT)
#define MODBUS_SENSOR_SNAPSHOT_DATA_SIZE (MODBUS_SENSOR_SNAPSHOT_REG_COUNT * 2U)
#define MODBUS_SENSOR_SNAPSHOT_TIMEOUT_MS 100U

typedef struct
{
  uint8_t valid;
  uint32_t frame_id;
  GloveTimestampUs_t timestamp_us;
  uint32_t valid_flags;
  GloveImuSample_t imu[GLOVE_IMU_COUNT];
  GloveQuaternion_t quat[GLOVE_IMU_COUNT];
} ModbusImuSnapshot_t;

typedef struct
{
  uint8_t valid;
  GloveTimestampUs_t timestamp_us;
  uint32_t valid_flags;
  float joint_angle_deg[GLOVE_JOINT_DOF_COUNT];
} ModbusJointSnapshot_t;

typedef struct
{
  uint8_t valid;
  GloveTimestampUs_t timestamp_us;
  uint32_t valid_flags;
  GloveTouchSample_t touch[GLOVE_TOUCH_COUNT];
} ModbusTouchSnapshot_t;

typedef struct
{
  ModbusImuSnapshot_t imu;
  ModbusJointSnapshot_t joint;
  ModbusTouchSnapshot_t touch;
  GlovePowerStatus_t power;
  SystemWatchdogStatus_t watchdog;
  GloveTimestampUs_t utc_timestamp_us;
  GloveTimestampUs_t local_uptime_us;
  GloveTimestampUs_t last_sync_utc_us;
  uint16_t imu_fresh_mask;
} ModbusReadSnapshot_t;

static uint8_t modbus_slave_address = MODBUS_SLAVE_ADDR_DEFAULT;
static ModbusImuSnapshot_t modbus_imu_snapshot;
static ModbusJointSnapshot_t modbus_joint_snapshot;
static ModbusTouchSnapshot_t modbus_touch_snapshot;
static uint32_t modbus_sensor_snapshot_tick;
/* 每个读请求只抓取一次数据，保证多字寄存器来自同一采样帧。 */
static ModbusReadSnapshot_t modbus_read_snapshot;
static uint8_t modbus_calib_initialized;
static GloveQuaternion_t modbus_calib_c_staging[GLOVE_IMU_COUNT];
static GloveQuaternion_t modbus_calib_m_staging[GLOVE_IMU_COUNT];
static GloveQuaternion_t modbus_calib_c_apply[GLOVE_IMU_COUNT];
static GloveQuaternion_t modbus_calib_m_apply[GLOVE_IMU_COUNT];
static uint16_t modbus_calib_magic;
static uint16_t modbus_calib_command;
static uint16_t modbus_calib_seq;
static uint16_t modbus_calib_status = IMU_CALIB_STATUS_IDLE;
static uint16_t modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
static uint16_t modbus_calib_last_applied_seq;

static uint8_t Modbus_IsSensorOutputReady(uint8_t power_state)
{
  return ((power_state == GLOVE_POWER_STATE_ON_NORMAL) ||
          (power_state == GLOVE_POWER_STATE_ON_LOW)) ? 1U : 0U;
}

static uint32_t Modbus_MsToTicks(uint32_t timeout_ms)
{
  uint64_t ticks = ((uint64_t)timeout_ms * (uint64_t)osKernelGetTickFreq() + 999ULL) / 1000ULL;

  if ((timeout_ms > 0U) && (ticks == 0ULL))
  {
    ticks = 1ULL;
  }
  return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

static void Modbus_CaptureReadSnapshot(void)
{
  uint32_t now_tick;
  uint32_t snapshot_timeout_ticks;

  /* 这些接口各自负责并发保护，避免在临界区内嵌套调用。 */
  SystemManagerTask_GetPowerStatus(&modbus_read_snapshot.power);
  SystemWatchdog_GetStatus(&modbus_read_snapshot.watchdog);
  modbus_read_snapshot.utc_timestamp_us = ModbusTimeSync_GetUtcTimestampUs();
  modbus_read_snapshot.local_uptime_us = ModbusTimeSync_GetLocalUptimeUs();
  modbus_read_snapshot.last_sync_utc_us = ModbusTimeSync_GetLastSyncUtcUs();
  now_tick = osKernelGetTickCount();
  snapshot_timeout_ticks = Modbus_MsToTicks(MODBUS_SENSOR_SNAPSHOT_TIMEOUT_MS);

  taskENTER_CRITICAL();
  modbus_read_snapshot.imu_fresh_mask = ImuCanTask_GetFreshMask();
  modbus_read_snapshot.imu = modbus_imu_snapshot;
  modbus_read_snapshot.joint = modbus_joint_snapshot;
  modbus_read_snapshot.touch = modbus_touch_snapshot;
  if ((Modbus_IsSensorOutputReady(modbus_read_snapshot.power.system_state) == 0U) ||
      (modbus_read_snapshot.imu.valid == 0U) ||
      (modbus_read_snapshot.joint.valid == 0U) ||
      (modbus_read_snapshot.touch.valid == 0U) ||
      ((uint32_t)(now_tick - modbus_sensor_snapshot_tick) > snapshot_timeout_ticks))
  {
    /* 电源未就绪或整组快照超时后统一返回无效零值，避免新旧传感器数据混用。 */
    modbus_read_snapshot.imu_fresh_mask = 0U;
    (void)memset(&modbus_read_snapshot.imu, 0, sizeof(modbus_read_snapshot.imu));
    (void)memset(&modbus_read_snapshot.joint, 0, sizeof(modbus_read_snapshot.joint));
    (void)memset(&modbus_read_snapshot.touch, 0, sizeof(modbus_read_snapshot.touch));
  }
  taskEXIT_CRITICAL();
}

static uint16_t Modbus_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t Modbus_ReadU32LeWords(const uint8_t *data)
{
  return ((uint32_t)Modbus_ReadU16(data)) |
         (((uint32_t)Modbus_ReadU16(&data[2])) << 16U);
}

static uint8_t Modbus_ReadRosTimeUsFromRegs(const uint8_t *data,
                                            uint64_t *timestamp_us)
{
  uint32_t sec;
  uint32_t nsec;

  if ((data == 0) || (timestamp_us == 0))
  {
    return 0U;
  }

  sec = Modbus_ReadU32LeWords(data);
  nsec = Modbus_ReadU32LeWords(&data[4]);
  if (nsec >= MODBUS_TIME_NS_PER_SEC)
  {
    return 0U;
  }

  *timestamp_us = ((uint64_t)sec * MODBUS_TIME_US_PER_SEC) +
                  ((uint64_t)nsec / MODBUS_TIME_NS_PER_US);
  return 1U;
}

static void Modbus_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)(value & 0xFFU);
}

static void Modbus_AppendCrc(uint8_t *frame, uint16_t len_without_crc)
{
  uint16_t crc = Modbus_Crc16(frame, len_without_crc);

  frame[len_without_crc] = (uint8_t)(crc & 0xFFU);
  frame[len_without_crc + 1U] = (uint8_t)(crc >> 8);
}

static uint8_t Modbus_IsReadableRegister(uint16_t reg_addr)
{
  if ((reg_addr >= REG_BASIC_STATUS_START) && (reg_addr <= REG_BASIC_STATUS_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_CMD_AREA_START) && (reg_addr <= REG_CMD_AREA_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_SYSTEM_STATUS_START) && (reg_addr <= REG_SYSTEM_STATUS_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_POWER_STATUS_START) && (reg_addr <= REG_POWER_STATUS_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_SD_STATUS_START) && (reg_addr <= REG_SD_STATUS_END))
  {
    return 1U;
  }

  if (reg_addr == REG_WORK_STATE)
  {
    return 1U;
  }

  if ((reg_addr >= REG_IMU_DATA_START) && (reg_addr <= REG_IMU_DATA_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_IMU_TIMESTAMP_US) && (reg_addr < (REG_IMU_TIMESTAMP_US + MODBUS_REGS_ROS_TIME)))
  {
    return 1U;
  }

  if (reg_addr == REG_IMU_STATUS_BITS)
  {
    return 1U;
  }

  if ((reg_addr >= REG_IMU_CALIB_START) && (reg_addr <= REG_IMU_CALIB_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_JOINT_DATA_START) && (reg_addr <= REG_JOINT_DATA_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_JOINT_TIMESTAMP_US) && (reg_addr < (REG_JOINT_TIMESTAMP_US + MODBUS_REGS_ROS_TIME)))
  {
    return 1U;
  }

  if ((reg_addr >= REG_JOINT_STATUS_START) && (reg_addr <= REG_JOINT_STATUS_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_R_DATA_START) && (reg_addr <= REG_R_DATA_END))
  {
    return 1U;
  }

  if ((reg_addr >= REG_R_TIMESTAMP_US) && (reg_addr < (REG_R_TIMESTAMP_US + MODBUS_REGS_ROS_TIME)))
  {
    return 1U;
  }

  if ((reg_addr >= REG_R_STATUS_START) && (reg_addr <= REG_R_STATUS_END))
  {
    return 1U;
  }

  return 0U;
}

static uint16_t Modbus_ReadFloatReg(float value, uint16_t word_offset)
{
  union
  {
    float f32;
    uint32_t u32;
  } raw;

  raw.f32 = value;
  if (word_offset == 0U)
  {
    return (uint16_t)(raw.u32 & 0xFFFFU);
  }

  return (uint16_t)(raw.u32 >> 16);
}

static uint16_t Modbus_ReadRosTimeRegFromUs(uint64_t timestamp_us,
                                            uint16_t word_offset)
{
  uint64_t sec64 = timestamp_us / MODBUS_TIME_US_PER_SEC;
  uint32_t sec;
  uint32_t nsec;

  if (sec64 > 0xFFFFFFFFULL)
  {
    sec = 0xFFFFFFFFUL;
    nsec = MODBUS_TIME_NS_PER_SEC - 1UL;
  }
  else
  {
    sec = (uint32_t)sec64;
    nsec = (uint32_t)((timestamp_us % MODBUS_TIME_US_PER_SEC) *
                      MODBUS_TIME_NS_PER_US);
  }

  switch (word_offset)
  {
    case 0U:
      return (uint16_t)(sec & 0xFFFFU);
    case 1U:
      return (uint16_t)(sec >> 16U);
    case 2U:
      return (uint16_t)(nsec & 0xFFFFU);
    case 3U:
      return (uint16_t)(nsec >> 16U);
    default:
      break;
  }

  return 0U;
}

static void Modbus_SetCalibrationIdentity(GloveQuaternion_t table[GLOVE_IMU_COUNT])
{
  if (table == 0)
  {
    return;
  }

  for (uint32_t i = 0U; i < GLOVE_IMU_COUNT; i++)
  {
    table[i].w = 1.0f;
    table[i].x = 0.0f;
    table[i].y = 0.0f;
    table[i].z = 0.0f;
  }
}

static uint8_t Modbus_IsWritableCalibrationReg(uint16_t reg_addr)
{
  if (((reg_addr >= REG_IMU_CALIB_C_START) && (reg_addr <= REG_IMU_CALIB_C_END)) ||
      ((reg_addr >= REG_IMU_CALIB_M_START) && (reg_addr <= REG_IMU_CALIB_M_END)))
  {
    return 1U;
  }

  switch (reg_addr)
  {
    case REG_IMU_CALIB_MAGIC:
    case REG_IMU_CALIB_COMMAND:
    case REG_IMU_CALIB_SEQ:
      return 1U;
    default:
      break;
  }

  return 0U;
}

static void Modbus_CalibrationEnsureInitialized(void)
{
  if (modbus_calib_initialized != 0U)
  {
    return;
  }

  Modbus_SetCalibrationIdentity(modbus_calib_c_staging);
  Modbus_SetCalibrationIdentity(modbus_calib_m_staging);
  modbus_calib_magic = 0U;
  modbus_calib_command = IMU_CALIB_CMD_NONE;
  modbus_calib_seq = 0U;
  modbus_calib_status = IMU_CALIB_STATUS_IDLE;
  modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
  modbus_calib_last_applied_seq = 0U;
  modbus_calib_initialized = 1U;
}

static float Modbus_GetQuaternionComponent(const GloveQuaternion_t *quat,
                                           uint16_t component)
{
  if (quat == 0)
  {
    return 0.0f;
  }

  switch (component)
  {
    case 0U:
      return quat->w;
    case 1U:
      return quat->x;
    case 2U:
      return quat->y;
    case 3U:
      return quat->z;
    default:
      return 0.0f;
  }
}

static void Modbus_SetQuaternionComponent(GloveQuaternion_t *quat,
                                          uint16_t component,
                                          float value)
{
  if (quat == 0)
  {
    return;
  }

  switch (component)
  {
    case 0U:
      quat->w = value;
      break;
    case 1U:
      quat->x = value;
      break;
    case 2U:
      quat->y = value;
      break;
    case 3U:
      quat->z = value;
      break;
    default:
      break;
  }
}

static GloveQuaternion_t *Modbus_GetCalibrationQuatForReg(uint16_t reg_addr,
                                                          uint16_t *word_in_quat)
{
  uint16_t reg_offset;

  if (word_in_quat == 0)
  {
    return 0;
  }

  if ((reg_addr >= REG_IMU_CALIB_C_START) && (reg_addr <= REG_IMU_CALIB_C_END))
  {
    reg_offset = (uint16_t)(reg_addr - REG_IMU_CALIB_C_START);
    *word_in_quat = (uint16_t)(reg_offset % MODBUS_IMU_CALIB_REGS_PER_UNIT);
    return &modbus_calib_c_staging[reg_offset / MODBUS_IMU_CALIB_REGS_PER_UNIT];
  }

  if ((reg_addr >= REG_IMU_CALIB_M_START) && (reg_addr <= REG_IMU_CALIB_M_END))
  {
    reg_offset = (uint16_t)(reg_addr - REG_IMU_CALIB_M_START);
    *word_in_quat = (uint16_t)(reg_offset % MODBUS_IMU_CALIB_REGS_PER_UNIT);
    return &modbus_calib_m_staging[reg_offset / MODBUS_IMU_CALIB_REGS_PER_UNIT];
  }

  return 0;
}

static uint16_t Modbus_ReadCalibrationReg(uint16_t reg_addr)
{
  uint16_t word_in_quat;
  GloveQuaternion_t *quat;
  uint16_t component;
  uint16_t word_offset;

  Modbus_CalibrationEnsureInitialized();

  quat = Modbus_GetCalibrationQuatForReg(reg_addr, &word_in_quat);
  if (quat != 0)
  {
    component = (uint16_t)(word_in_quat / MODBUS_REGS_FLOAT32);
    word_offset = (uint16_t)(word_in_quat % MODBUS_REGS_FLOAT32);
    return Modbus_ReadFloatReg(Modbus_GetQuaternionComponent(quat, component),
                               word_offset);
  }

  switch (reg_addr)
  {
    case REG_IMU_CALIB_MAGIC:
      return modbus_calib_magic;
    case REG_IMU_CALIB_COMMAND:
      return modbus_calib_command;
    case REG_IMU_CALIB_SEQ:
      return modbus_calib_seq;
    case REG_IMU_CALIB_STATUS:
      return modbus_calib_status;
    case REG_IMU_CALIB_ERROR_INDEX:
      return modbus_calib_error_index;
    case REG_IMU_CALIB_LAST_APPLIED_SEQ:
      return modbus_calib_last_applied_seq;
    default:
      break;
  }

  return 0U;
}

static void Modbus_WriteCalibrationFloatWord(GloveQuaternion_t *quat,
                                             uint16_t word_in_quat,
                                             uint16_t value)
{
  union
  {
    float f32;
    uint32_t u32;
  } raw;
  uint16_t component = (uint16_t)(word_in_quat / MODBUS_REGS_FLOAT32);
  uint16_t word_offset = (uint16_t)(word_in_quat % MODBUS_REGS_FLOAT32);

  if (quat == 0)
  {
    return;
  }

  raw.f32 = Modbus_GetQuaternionComponent(quat, component);
  if (word_offset == 0U)
  {
    raw.u32 = (raw.u32 & 0xFFFF0000UL) | value;
  }
  else
  {
    raw.u32 = (raw.u32 & 0x0000FFFFUL) | ((uint32_t)value << 16);
  }
  Modbus_SetQuaternionComponent(quat, component, raw.f32);
}

static uint8_t Modbus_IsFiniteCalibrationQuat(const GloveQuaternion_t *input)
{
  if (input == 0)
  {
    return 0U;
  }

  return (isfinite(input->w) &&
          isfinite(input->x) &&
          isfinite(input->y) &&
          isfinite(input->z)) ? 1U : 0U;
}

static uint8_t Modbus_NormalizeCalibrationQuat(const GloveQuaternion_t *input,
                                               GloveQuaternion_t *output)
{
  float norm_sq;
  float inv_norm;

  if ((input == 0) || (output == 0))
  {
    return 0U;
  }

  if (Modbus_IsFiniteCalibrationQuat(input) == 0U)
  {
    return 0U;
  }

  norm_sq = (input->w * input->w) +
            (input->x * input->x) +
            (input->y * input->y) +
            (input->z * input->z);
  if (!((norm_sq >= 0.25f) && (norm_sq <= 2.25f)))
  {
    return 0U;
  }

  inv_norm = 1.0f / sqrtf(norm_sq);
  output->w = input->w * inv_norm;
  output->x = input->x * inv_norm;
  output->y = input->y * inv_norm;
  output->z = input->z * inv_norm;

  return 1U;
}

static void Modbus_ClearCalibrationCommandLatch(void)
{
  modbus_calib_magic = 0U;
  modbus_calib_command = IMU_CALIB_CMD_NONE;
}

static uint8_t Modbus_BuildNormalizedCalibrationTables(
  GloveQuaternion_t c_table[GLOVE_IMU_COUNT],
  GloveQuaternion_t m_table[GLOVE_IMU_COUNT],
  uint16_t *error_index)
{
  if ((c_table == 0) || (m_table == 0) || (error_index == 0))
  {
    return 0U;
  }

  for (uint32_t i = 0U; i < GLOVE_IMU_COUNT; i++)
  {
    if (Modbus_NormalizeCalibrationQuat(&modbus_calib_c_staging[i],
                                        &c_table[i]) == 0U)
    {
      *error_index = (uint16_t)i;
      return 0U;
    }
  }

  for (uint32_t i = 0U; i < GLOVE_IMU_COUNT; i++)
  {
    if (Modbus_NormalizeCalibrationQuat(&modbus_calib_m_staging[i],
                                        &m_table[i]) == 0U)
    {
      *error_index = (uint16_t)(GLOVE_IMU_COUNT + i);
      return 0U;
    }
  }

  *error_index = IMU_CALIB_ERROR_NONE;
  return 1U;
}

static void Modbus_ProcessCalibrationCommand(void)
{
  uint16_t error_index = IMU_CALIB_ERROR_NONE;

  Modbus_CalibrationEnsureInitialized();

  if (modbus_calib_command == IMU_CALIB_CMD_NONE)
  {
    modbus_calib_status = IMU_CALIB_STATUS_IDLE;
    modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
    return;
  }

  if (modbus_calib_magic != IMU_CALIB_MAGIC_VALUE)
  {
    modbus_calib_status = IMU_CALIB_STATUS_BAD_MAGIC;
    modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
    Modbus_ClearCalibrationCommandLatch();
    return;
  }

  if (modbus_calib_command == IMU_CALIB_CMD_APPLY)
  {
    if (Modbus_BuildNormalizedCalibrationTables(modbus_calib_c_apply,
                                                modbus_calib_m_apply,
                                                &error_index) == 0U)
    {
      modbus_calib_status = IMU_CALIB_STATUS_BAD_QUAT;
      modbus_calib_error_index = error_index;
      Modbus_ClearCalibrationCommandLatch();
      return;
    }

    if (DataProcessTask_SetCalibrationTableWithSeq(modbus_calib_c_apply,
                                                   modbus_calib_m_apply,
                                                   modbus_calib_seq) != GLOVE_STATUS_OK)
    {
      modbus_calib_status = IMU_CALIB_STATUS_BAD_QUAT;
      modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
      Modbus_ClearCalibrationCommandLatch();
      return;
    }

    (void)memcpy(modbus_calib_c_staging,
                 modbus_calib_c_apply,
                 sizeof(modbus_calib_c_staging));
    (void)memcpy(modbus_calib_m_staging,
                 modbus_calib_m_apply,
                 sizeof(modbus_calib_m_staging));
    modbus_calib_status = IMU_CALIB_STATUS_APPLIED;
    modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
    modbus_calib_last_applied_seq = modbus_calib_seq;
    Modbus_ClearCalibrationCommandLatch();
    return;
  }

  if (modbus_calib_command == IMU_CALIB_CMD_RESET_IDENTITY)
  {
    Modbus_SetCalibrationIdentity(modbus_calib_c_staging);
    Modbus_SetCalibrationIdentity(modbus_calib_m_staging);
    (void)DataProcessTask_ResetCalibration();
    modbus_calib_status = IMU_CALIB_STATUS_RESET_DONE;
    modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
    modbus_calib_last_applied_seq = modbus_calib_seq;
    Modbus_ClearCalibrationCommandLatch();
    return;
  }

  modbus_calib_status = IMU_CALIB_STATUS_BAD_CMD;
  modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
  Modbus_ClearCalibrationCommandLatch();
}

static uint8_t Modbus_WriteCalibrationReg(uint16_t reg_addr, uint16_t value)
{
  uint16_t word_in_quat;
  GloveQuaternion_t *quat;

  Modbus_CalibrationEnsureInitialized();

  quat = Modbus_GetCalibrationQuatForReg(reg_addr, &word_in_quat);
  if (quat != 0)
  {
    Modbus_WriteCalibrationFloatWord(quat, word_in_quat, value);
    return 0U;
  }

  switch (reg_addr)
  {
    case REG_IMU_CALIB_MAGIC:
      modbus_calib_magic = value;
      break;
    case REG_IMU_CALIB_COMMAND:
      modbus_calib_command = value;
      return 1U;
    case REG_IMU_CALIB_SEQ:
      modbus_calib_seq = value;
      break;
    default:
      break;
  }

  return 0U;
}

static uint16_t Modbus_ReadImuStatusBits(void)
{
  uint32_t valid_flags;
  uint16_t snapshot_mask;

  valid_flags = (modbus_read_snapshot.imu.valid != 0U) ?
                modbus_read_snapshot.imu.valid_flags : 0U;

  snapshot_mask = (uint16_t)((valid_flags & GLOVE_FRAME_VALID_IMU_ALL_MASK) >>
                             GLOVE_FRAME_VALID_IMU_BIT_SHIFT);
  return (uint16_t)(snapshot_mask & modbus_read_snapshot.imu_fresh_mask);
}

static GloveTimestampUs_t Modbus_GetImuTimestampUs(void)
{
  GloveTimestampUs_t timestamp_us;

  timestamp_us = (modbus_read_snapshot.imu.valid != 0U) ?
                 modbus_read_snapshot.imu.timestamp_us : 0ULL;

  return timestamp_us;
}

static GloveTimestampUs_t Modbus_GetJointTimestampUs(void)
{
  GloveTimestampUs_t timestamp_us;

  timestamp_us = (modbus_read_snapshot.joint.valid != 0U) ?
                 modbus_read_snapshot.joint.timestamp_us : 0ULL;

  return timestamp_us;
}

static uint16_t Modbus_ReadJointStatusFlags(void)
{
  uint16_t status = 0U;

  if (modbus_read_snapshot.joint.valid != 0U)
  {
    status |= JOINT_STATUS_SNAPSHOT_VALID;
    if ((modbus_read_snapshot.joint.valid_flags & GLOVE_FRAME_FLAG_ALGORITHM_VALID) != 0U)
    {
      status |= JOINT_STATUS_ALGORITHM_VALID;
    }
    if ((modbus_read_snapshot.joint.valid_flags & GLOVE_FRAME_FLAG_IMU_CALIB_APPLIED) != 0U)
    {
      status |= JOINT_STATUS_IMU_CALIB_APPLIED;
    }
  }
  return status;
}

static uint16_t Modbus_ReadTouchStatusFlags(void)
{
  uint16_t status = 0U;

  if (modbus_read_snapshot.touch.valid != 0U)
  {
    status |= R_STATUS_SNAPSHOT_VALID;
    if ((modbus_read_snapshot.touch.valid_flags & GLOVE_FRAME_FLAG_TOUCH_VALID) != 0U)
    {
      status |= R_STATUS_TOUCH_VALID;
    }
  }
  return status;
}

static uint32_t Modbus_GetJointValidBits(void)
{
  uint32_t valid_bits = 0UL;

  if (modbus_read_snapshot.joint.valid != 0U)
  {
    for (uint32_t index = 0U; index < GLOVE_JOINT_DOF_COUNT; index++)
    {
      float value = modbus_read_snapshot.joint.joint_angle_deg[index];

      if ((value < 999999936.0f) && (value > -999999936.0f))
      {
        valid_bits |= (1UL << index);
      }
    }
  }
  return valid_bits;
}

static float Modbus_GetImuFloat(uint16_t imu_index, uint16_t float_index)
{
  float value = 0.0f;

  if ((imu_index >= GLOVE_IMU_COUNT) || (float_index >= MODBUS_IMU_FLOATS_PER_UNIT))
  {
    return 0.0f;
  }

  /* IMU超时或失联后直接返回0，禁止485继续输出历史缓存。 */
  if ((modbus_read_snapshot.imu_fresh_mask & (uint16_t)(1U << imu_index)) == 0U)
  {
    return 0.0f;
  }

  if (modbus_read_snapshot.imu.valid != 0U)
  {
    switch (float_index)
    {
      case 0U:
        value = modbus_read_snapshot.imu.imu[imu_index].accel_mps2.x;
        break;
      case 1U:
        value = modbus_read_snapshot.imu.imu[imu_index].accel_mps2.y;
        break;
      case 2U:
        value = modbus_read_snapshot.imu.imu[imu_index].accel_mps2.z;
        break;
      case 3U:
        value = modbus_read_snapshot.imu.imu[imu_index].gyro_radps.x;
        break;
      case 4U:
        value = modbus_read_snapshot.imu.imu[imu_index].gyro_radps.y;
        break;
      case 5U:
        value = modbus_read_snapshot.imu.imu[imu_index].gyro_radps.z;
        break;
      case 6U:
        value = modbus_read_snapshot.imu.quat[imu_index].w;
        break;
      case 7U:
        value = modbus_read_snapshot.imu.quat[imu_index].x;
        break;
      case 8U:
        value = modbus_read_snapshot.imu.quat[imu_index].y;
        break;
      case 9U:
        value = modbus_read_snapshot.imu.quat[imu_index].z;
        break;
      default:
        value = 0.0f;
        break;
    }
  }
  return value;
}

static float Modbus_GetJointFloat(uint16_t joint_index)
{
  float value = 0.0f;

  if (joint_index >= GLOVE_JOINT_DOF_COUNT)
  {
    return 0.0f;
  }

  if (modbus_read_snapshot.joint.valid != 0U)
  {
    value = modbus_read_snapshot.joint.joint_angle_deg[joint_index];
  }

  return value;
}

static uint16_t Modbus_GetTouchValue(uint16_t touch_index)
{
  uint16_t value = 0U;

  if (touch_index >= GLOVE_TOUCH_COUNT)
  {
    return 0U;
  }

  if (modbus_read_snapshot.touch.valid != 0U)
  {
    value = modbus_read_snapshot.touch.touch[touch_index].value;
  }

  return value;
}

static uint16_t Modbus_ReadImuDataReg(uint16_t reg_addr)
{
  uint16_t reg_offset = (uint16_t)(reg_addr - REG_IMU_DATA_START);
  uint16_t imu_index = (uint16_t)(reg_offset / MODBUS_IMU_REGS_PER_UNIT);
  uint16_t word_in_imu = (uint16_t)(reg_offset % MODBUS_IMU_REGS_PER_UNIT);
  uint16_t float_index = (uint16_t)(word_in_imu / MODBUS_REGS_FLOAT32);
  uint16_t word_offset = (uint16_t)(word_in_imu % MODBUS_REGS_FLOAT32);

  return Modbus_ReadFloatReg(Modbus_GetImuFloat(imu_index, float_index), word_offset);
}

static uint16_t Modbus_ReadJointDataReg(uint16_t reg_addr)
{
  uint16_t reg_offset = (uint16_t)(reg_addr - REG_JOINT_DATA_START);
  uint16_t joint_index = (uint16_t)(reg_offset / MODBUS_JOINT_REGS_PER_VALUE);
  uint16_t word_offset = (uint16_t)(reg_offset % MODBUS_JOINT_REGS_PER_VALUE);

  return Modbus_ReadFloatReg(Modbus_GetJointFloat(joint_index), word_offset);
}

static uint16_t Modbus_ReadTouchDataReg(uint16_t reg_addr)
{
  uint16_t touch_index = (uint16_t)(reg_addr - REG_R_DATA_START);

  if (touch_index >= MODBUS_R_POINT_COUNT)
  {
    return 0U;
  }

  return Modbus_GetTouchValue(touch_index);
}

static GloveTimestampUs_t Modbus_GetTouchTimestampUs(void)
{
  GloveTimestampUs_t timestamp_us;

  timestamp_us = (modbus_read_snapshot.touch.valid != 0U) ?
                 modbus_read_snapshot.touch.timestamp_us : 0ULL;

  return timestamp_us;
}

static uint16_t Modbus_ReadHoldingRegister(uint16_t reg_addr)
{
  if (reg_addr == REG_SLAVE_ADDR)
  {
    return (uint16_t)modbus_slave_address;
  }

  if (reg_addr == REG_BAUDRATE_CODE)
  {
    return 0U;
  }

  if ((reg_addr >= REG_UTC_TIMESTAMP_US) && (reg_addr < (REG_UTC_TIMESTAMP_US + MODBUS_REGS_ROS_TIME)))
  {
    return Modbus_ReadRosTimeRegFromUs(modbus_read_snapshot.utc_timestamp_us,
                                       (uint16_t)(reg_addr - REG_UTC_TIMESTAMP_US));
  }

  if ((reg_addr >= REG_LOCAL_UPTIME_US) && (reg_addr < (REG_LOCAL_UPTIME_US + MODBUS_REGS_ROS_TIME)))
  {
    return Modbus_ReadRosTimeRegFromUs(modbus_read_snapshot.local_uptime_us,
                                       (uint16_t)(reg_addr - REG_LOCAL_UPTIME_US));
  }

  if ((reg_addr >= REG_TIME_SYNC_UTC_US) && (reg_addr < (REG_TIME_SYNC_UTC_US + MODBUS_REGS_ROS_TIME)))
  {
    return Modbus_ReadRosTimeRegFromUs(modbus_read_snapshot.last_sync_utc_us,
                                       (uint16_t)(reg_addr - REG_TIME_SYNC_UTC_US));
  }

  switch (reg_addr)
  {
    case REG_CMD:
    case REG_CMD_PARAM:
    case REG_CMD_SEQ:
    case REG_CMD_ACK_SEQ:
    case REG_CMD_ERROR:
      return 0U;

    case REG_CMD_ACK:
      return CMD_ACK_IDLE;

    case REG_SYSTEM_STATE:
      return SYSTEM_STATE_READY;

    case REG_WORK_MODE:
      return WORK_MODE_NORMAL;

    case REG_LOG_STATE:
      return LOG_STATE_IDLE;

    case REG_SD_STATE:
      return SD_STATE_NOT_READY;

    case REG_SENSOR_STATE:
      return Modbus_ReadImuStatusBits();

    case REG_COMM_STATE:
      return COMM_STATE_OK;

    case REG_RESET_CAUSE:
      return modbus_read_snapshot.watchdog.reset_cause;

    case REG_WATCHDOG_STATUS:
      return modbus_read_snapshot.watchdog.status_flags;

    case REG_WORK_STATE:
      return WORK_STATE_IDLE;

    case REG_SD_FS_STATUS:
      return SD_FS_STATUS_NOT_MOUNTED;

    case REG_SD_LOG_STATUS:
      return SD_LOG_STATUS_IDLE;

    case REG_SD_ERROR_CODE:
      return SD_ERROR_NONE;

    case REG_SD_CURRENT_FILE_ID:
      return 0U;

    case REG_IMU_STATUS_BITS:
      return Modbus_ReadImuStatusBits();

    case REG_JOINT_STATUS_FLAGS:
      return Modbus_ReadJointStatusFlags();

    case REG_JOINT_VALID_BITS_LOW:
      return (uint16_t)(Modbus_GetJointValidBits() & 0xFFFFU);

    case REG_JOINT_VALID_BITS_HIGH:
      return (uint16_t)((Modbus_GetJointValidBits() >> 16) & 0xFFFFU);

    case REG_R_STATUS_FLAGS:
      return Modbus_ReadTouchStatusFlags();

    case REG_R_POINT_COUNT_REG:
      return (uint16_t)MODBUS_R_POINT_COUNT;

    case REG_R_DATA_REG_COUNT_REG:
      return (uint16_t)MODBUS_R_DATA_REG_COUNT;

    case REG_R_STATUS_RESERVED:
      return 0U;

    default:
      break;
  }

  if ((reg_addr >= REG_SYSTEM_RESERVED_START) && (reg_addr <= REG_SYSTEM_RESERVED_END))
  {
    return 0U;
  }

  if ((reg_addr >= REG_TEMPERATURE_BOARD) && (reg_addr < (REG_TEMPERATURE_BOARD + MODBUS_REGS_FLOAT32)))
  {
    return Modbus_ReadFloatReg(25.0f, (uint16_t)(reg_addr - REG_TEMPERATURE_BOARD));
  }

  if ((reg_addr >= REG_BAT_VOLTAGE) && (reg_addr < (REG_BAT_VOLTAGE + MODBUS_REGS_FLOAT32)))
  {
    return Modbus_ReadFloatReg((float)modbus_read_snapshot.power.battery_voltage_mv / 1000.0f,
                               (uint16_t)(reg_addr - REG_BAT_VOLTAGE));
  }

  if ((reg_addr >= REG_BAT_CURRENT) && (reg_addr < (REG_BAT_CURRENT + MODBUS_REGS_FLOAT32)))
  {
    return Modbus_ReadFloatReg((float)modbus_read_snapshot.power.battery_current_ma / 1000.0f,
                               (uint16_t)(reg_addr - REG_BAT_CURRENT));
  }

  if ((reg_addr >= REG_BAT_SOC) && (reg_addr < (REG_BAT_SOC + MODBUS_REGS_FLOAT32)))
  {
    return Modbus_ReadFloatReg((float)modbus_read_snapshot.power.soc_centi_percent / 100.0f,
                               (uint16_t)(reg_addr - REG_BAT_SOC));
  }

  switch (reg_addr)
  {
    case REG_POWER_STATE: return modbus_read_snapshot.power.system_state;
    case REG_CHARGE_STATE: return modbus_read_snapshot.power.charge_state;
    case REG_POWER_FLAGS: return modbus_read_snapshot.power.flags;
    case REG_POWER_FAULT: return modbus_read_snapshot.power.fault_code;
    case REG_BQ_DIAGNOSTIC:
      return (uint16_t)(((uint16_t)modbus_read_snapshot.power.bq_diagnostic_stage << 8) |
                        modbus_read_snapshot.power.bq_last_status);
    case REG_BQ_CHARGER_EVENTS:
      return modbus_read_snapshot.power.bq_charger_events;
    case REG_BQ_FAULT_EVENTS:
      return modbus_read_snapshot.power.bq_fault_events;
    case REG_BQ_INTERRUPT_COUNT:
      return modbus_read_snapshot.power.bq_interrupt_count;
    default: break;
  }

  if ((reg_addr >= REG_VBUS_VOLTAGE) && (reg_addr < (REG_VBUS_VOLTAGE + MODBUS_REGS_FLOAT32)))
  {
    return Modbus_ReadFloatReg((float)modbus_read_snapshot.power.vbus_voltage_mv / 1000.0f,
                               (uint16_t)(reg_addr - REG_VBUS_VOLTAGE));
  }

  if ((reg_addr >= REG_INPUT_CURRENT) && (reg_addr < (REG_INPUT_CURRENT + MODBUS_REGS_FLOAT32)))
  {
    return Modbus_ReadFloatReg((float)modbus_read_snapshot.power.input_current_ma / 1000.0f,
                               (uint16_t)(reg_addr - REG_INPUT_CURRENT));
  }

  if ((reg_addr >= REG_SD_TOTAL_SIZE_MB) && (reg_addr <= REG_SD_STATUS_END))
  {
    return 0U;
  }

  if ((reg_addr >= REG_IMU_DATA_START) && (reg_addr <= REG_IMU_DATA_END))
  {
    return Modbus_ReadImuDataReg(reg_addr);
  }

  if ((reg_addr >= REG_IMU_TIMESTAMP_US) && (reg_addr < (REG_IMU_TIMESTAMP_US + MODBUS_REGS_ROS_TIME)))
  {
    return Modbus_ReadRosTimeRegFromUs(Modbus_GetImuTimestampUs(),
                                       (uint16_t)(reg_addr - REG_IMU_TIMESTAMP_US));
  }

  if ((reg_addr >= REG_IMU_CALIB_START) && (reg_addr <= REG_IMU_CALIB_END))
  {
    return Modbus_ReadCalibrationReg(reg_addr);
  }

  if ((reg_addr >= REG_JOINT_DATA_START) && (reg_addr <= REG_JOINT_DATA_END))
  {
    return Modbus_ReadJointDataReg(reg_addr);
  }

  if ((reg_addr >= REG_JOINT_TIMESTAMP_US) && (reg_addr < (REG_JOINT_TIMESTAMP_US + MODBUS_REGS_ROS_TIME)))
  {
    return Modbus_ReadRosTimeRegFromUs(Modbus_GetJointTimestampUs(),
                                       (uint16_t)(reg_addr - REG_JOINT_TIMESTAMP_US));
  }

  if ((reg_addr >= REG_R_DATA_START) && (reg_addr <= REG_R_DATA_END))
  {
    return Modbus_ReadTouchDataReg(reg_addr);
  }

  if ((reg_addr >= REG_R_TIMESTAMP_US) && (reg_addr < (REG_R_TIMESTAMP_US + MODBUS_REGS_ROS_TIME)))
  {
    return Modbus_ReadRosTimeRegFromUs(Modbus_GetTouchTimestampUs(),
                                       (uint16_t)(reg_addr - REG_R_TIMESTAMP_US));
  }

  return 0U;
}

static ModbusResult_t Modbus_BuildException(uint8_t slave_addr,
                                            uint8_t func,
                                            uint8_t exception_code,
                                            uint8_t *tx_buf,
                                            uint16_t tx_buf_size,
                                            uint16_t *tx_len)
{
  if ((tx_buf == 0) || (tx_len == 0) || (tx_buf_size < 5U))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  tx_buf[0] = slave_addr;
  tx_buf[1] = (uint8_t)(func | 0x80U);
  tx_buf[2] = exception_code;
  Modbus_AppendCrc(tx_buf, 3U);
  *tx_len = 5U;

  return MODBUS_RESULT_RESPONSE_READY;
}

static ModbusResult_t Modbus_HandleReadHoldingRegs(uint8_t response_addr,
                                                   uint16_t start_reg,
                                                   uint16_t reg_count,
                                                   uint8_t *tx_buf,
                                                   uint16_t tx_buf_size,
                                                   uint16_t *tx_len)
{
  uint16_t index;
  uint16_t reg_addr;
  uint16_t value;
  uint16_t len_without_crc;

  if ((tx_buf == 0) || (tx_len == 0))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  if ((reg_count == 0U) || (reg_count > MODBUS_MAX_READ_REG_COUNT))
  {
    return Modbus_BuildException(response_addr,
                                 MB_FC_READ_HOLDING_REGS,
                                 MB_EX_ILLEGAL_DATA_VALUE,
                                 tx_buf,
                                 tx_buf_size,
                                 tx_len);
  }

  if (tx_buf_size < (uint16_t)(5U + (reg_count * 2U)))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  for (index = 0U; index < reg_count; index++)
  {
    reg_addr = (uint16_t)(start_reg + index);
    if (Modbus_IsReadableRegister(reg_addr) == 0U)
    {
      return Modbus_BuildException(response_addr,
                                   MB_FC_READ_HOLDING_REGS,
                                   MB_EX_ILLEGAL_DATA_ADDRESS,
                                   tx_buf,
                                   tx_buf_size,
                                   tx_len);
    }
  }

  if ((reg_count * 2U) > 0xFFU)
  {
    return Modbus_BuildException(response_addr,
                                 MB_FC_READ_HOLDING_REGS,
                                 MB_EX_ILLEGAL_DATA_VALUE,
                                 tx_buf,
                                 tx_buf_size,
                                 tx_len);
  }

  Modbus_CaptureReadSnapshot();

  tx_buf[0] = response_addr;
  tx_buf[1] = MB_FC_READ_HOLDING_REGS;
  tx_buf[2] = (uint8_t)(reg_count * 2U);

  for (index = 0U; index < reg_count; index++)
  {
    value = Modbus_ReadHoldingRegister((uint16_t)(start_reg + index));
    Modbus_WriteU16(&tx_buf[3U + (index * 2U)], value);
  }

  len_without_crc = (uint16_t)(3U + (reg_count * 2U));
  Modbus_AppendCrc(tx_buf, len_without_crc);
  *tx_len = (uint16_t)(len_without_crc + 2U);

  return MODBUS_RESULT_RESPONSE_READY;
}

static uint16_t Modbus_WriteSnapshotRegisterRange(uint8_t *tx_buf,
                                                  uint16_t write_offset,
                                                  uint16_t start_reg,
                                                  uint16_t reg_count)
{
  uint16_t index;

  for (index = 0U; index < reg_count; index++)
  {
    Modbus_WriteU16(&tx_buf[write_offset],
                    Modbus_ReadHoldingRegister((uint16_t)(start_reg + index)));
    write_offset = (uint16_t)(write_offset + 2U);
  }

  return write_offset;
}

static ModbusResult_t Modbus_HandleReadSensorSnapshot(uint8_t response_addr,
                                                      uint8_t *tx_buf,
                                                      uint16_t tx_buf_size,
                                                      uint16_t *tx_len)
{
  uint16_t index;
  uint16_t write_offset;
  uint16_t len_without_crc;

  if ((tx_buf == 0) || (tx_len == 0))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  len_without_crc = (uint16_t)(4U + MODBUS_SENSOR_SNAPSHOT_DATA_SIZE);
  if (tx_buf_size < (uint16_t)(len_without_crc + 2U))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  /* 整个大帧只抓取一次快照，保证三类数据来自同一个FullFrame。 */
  Modbus_CaptureReadSnapshot();
  tx_buf[0] = response_addr;
  tx_buf[1] = MB_FC_READ_SENSOR_SNAPSHOT;
  tx_buf[2] = (uint8_t)(MODBUS_SENSOR_SNAPSHOT_DATA_SIZE >> 8);
  tx_buf[3] = (uint8_t)(MODBUS_SENSOR_SNAPSHOT_DATA_SIZE & 0xFFU);

  write_offset = 4U;
  /* 元数据依次为帧号、时间戳、电源状态、IMU掩码、关节状态和触摸状态。 */
  Modbus_WriteU16(&tx_buf[write_offset],
                  (uint16_t)(modbus_read_snapshot.imu.frame_id & 0xFFFFU));
  write_offset = (uint16_t)(write_offset + 2U);
  Modbus_WriteU16(&tx_buf[write_offset],
                  (uint16_t)(modbus_read_snapshot.imu.frame_id >> 16U));
  write_offset = (uint16_t)(write_offset + 2U);
  for (index = 0U; index < MODBUS_REGS_ROS_TIME; index++)
  {
    Modbus_WriteU16(&tx_buf[write_offset],
                    Modbus_ReadRosTimeRegFromUs(modbus_read_snapshot.imu.timestamp_us,
                                                index));
    write_offset = (uint16_t)(write_offset + 2U);
  }
  Modbus_WriteU16(&tx_buf[write_offset], modbus_read_snapshot.power.system_state);
  write_offset = (uint16_t)(write_offset + 2U);
  Modbus_WriteU16(&tx_buf[write_offset], Modbus_ReadImuStatusBits());
  write_offset = (uint16_t)(write_offset + 2U);
  Modbus_WriteU16(&tx_buf[write_offset], Modbus_ReadJointStatusFlags());
  write_offset = (uint16_t)(write_offset + 2U);
  Modbus_WriteU16(&tx_buf[write_offset], Modbus_ReadTouchStatusFlags());
  write_offset = (uint16_t)(write_offset + 2U);

  write_offset = Modbus_WriteSnapshotRegisterRange(tx_buf,
                                                    write_offset,
                                                    REG_IMU_DATA_START,
                                                    MODBUS_IMU_DATA_REG_COUNT);
  write_offset = Modbus_WriteSnapshotRegisterRange(tx_buf,
                                                    write_offset,
                                                    REG_JOINT_DATA_START,
                                                    MODBUS_JOINT_DATA_REG_COUNT);
  write_offset = Modbus_WriteSnapshotRegisterRange(tx_buf,
                                                    write_offset,
                                                    REG_R_DATA_START,
                                                    MODBUS_R_POINT_COUNT);
  if (write_offset != len_without_crc)
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  Modbus_AppendCrc(tx_buf, len_without_crc);
  *tx_len = (uint16_t)(len_without_crc + 2U);
  return MODBUS_RESULT_RESPONSE_READY;
}

static uint8_t Modbus_IsRegRangeWithin(uint16_t start_reg,
                                       uint16_t reg_count,
                                       uint16_t range_start,
                                       uint16_t range_end)
{
  uint32_t end_reg;

  if (reg_count == 0U)
  {
    return 0U;
  }

  end_reg = (uint32_t)start_reg + (uint32_t)reg_count - 1UL;
  return (((uint32_t)start_reg >= (uint32_t)range_start) &&
          (end_reg <= (uint32_t)range_end)) ? 1U : 0U;
}

static ModbusResult_t Modbus_BuildWriteMultipleAck(uint8_t response_addr,
                                                   uint16_t start_reg,
                                                   uint16_t reg_count,
                                                   uint8_t *tx_buf,
                                                   uint16_t tx_buf_size,
                                                   uint16_t *tx_len)
{
  if ((tx_buf == 0) || (tx_len == 0) || (tx_buf_size < 8U))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  tx_buf[0] = response_addr;
  tx_buf[1] = MB_FC_WRITE_MULTIPLE_REGS;
  Modbus_WriteU16(&tx_buf[2], start_reg);
  Modbus_WriteU16(&tx_buf[4], reg_count);
  Modbus_AppendCrc(tx_buf, 6U);
  *tx_len = 8U;

  return MODBUS_RESULT_RESPONSE_READY;
}

static ModbusResult_t Modbus_BuildWriteSingleAck(uint8_t response_addr,
                                                 uint16_t reg_addr,
                                                 uint16_t value,
                                                 uint8_t *tx_buf,
                                                 uint16_t tx_buf_size,
                                                 uint16_t *tx_len)
{
  if ((tx_buf == 0) || (tx_len == 0) || (tx_buf_size < 8U))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  tx_buf[0] = response_addr;
  tx_buf[1] = MB_FC_WRITE_SINGLE_REG;
  Modbus_WriteU16(&tx_buf[2], reg_addr);
  Modbus_WriteU16(&tx_buf[4], value);
  Modbus_AppendCrc(tx_buf, 6U);
  *tx_len = 8U;

  return MODBUS_RESULT_RESPONSE_READY;
}

static ModbusResult_t Modbus_HandleCalibrationWrite(uint8_t response_addr,
                                                    uint16_t start_reg,
                                                    uint16_t reg_count,
                                                    const uint8_t *data_buf,
                                                    uint8_t *tx_buf,
                                                    uint16_t tx_buf_size,
                                                    uint16_t *tx_len)
{
  uint8_t command_written = 0U;

  if (data_buf == 0)
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  for (uint16_t index = 0U; index < reg_count; index++)
  {
    uint16_t reg_addr = (uint16_t)(start_reg + index);

    if (Modbus_IsWritableCalibrationReg(reg_addr) == 0U)
    {
      return Modbus_BuildException(response_addr,
                                   MB_FC_WRITE_MULTIPLE_REGS,
                                   MB_EX_ILLEGAL_DATA_ADDRESS,
                                   tx_buf,
                                   tx_buf_size,
                                   tx_len);
    }
  }

  for (uint16_t index = 0U; index < reg_count; index++)
  {
    uint16_t reg_addr = (uint16_t)(start_reg + index);
    uint16_t value = Modbus_ReadU16(&data_buf[index * 2U]);

    if (Modbus_WriteCalibrationReg(reg_addr, value) != 0U)
    {
      command_written = 1U;
    }
  }

  if (command_written != 0U)
  {
    Modbus_ProcessCalibrationCommand();
  }

  return Modbus_BuildWriteMultipleAck(response_addr,
                                      start_reg,
                                      reg_count,
                                      tx_buf,
                                      tx_buf_size,
                                      tx_len);
}

static ModbusResult_t Modbus_HandleWriteSingleReg(uint8_t response_addr,
                                                  uint16_t reg_addr,
                                                  uint16_t value,
                                                  uint8_t *tx_buf,
                                                  uint16_t tx_buf_size,
                                                  uint16_t *tx_len)
{
  if ((tx_buf == 0) || (tx_len == 0) || (tx_buf_size < 8U))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  if ((reg_addr < REG_IMU_CALIB_START) ||
      (reg_addr > REG_IMU_CALIB_END) ||
      (Modbus_IsWritableCalibrationReg(reg_addr) == 0U))
  {
    return Modbus_BuildException(response_addr,
                                 MB_FC_WRITE_SINGLE_REG,
                                 MB_EX_ILLEGAL_DATA_ADDRESS,
                                 tx_buf,
                                 tx_buf_size,
                                 tx_len);
  }

  if (Modbus_WriteCalibrationReg(reg_addr, value) != 0U)
  {
    Modbus_ProcessCalibrationCommand();
  }

  return Modbus_BuildWriteSingleAck(response_addr,
                                    reg_addr,
                                    value,
                                    tx_buf,
                                    tx_buf_size,
                                    tx_len);
}

static ModbusResult_t Modbus_HandleWriteMultipleRegs(uint8_t response_addr,
                                                     uint16_t start_reg,
                                                     uint16_t reg_count,
                                                     const uint8_t *data_buf,
                                                     uint8_t byte_count,
                                                     uint8_t *tx_buf,
                                                     uint16_t tx_buf_size,
                                                     uint16_t *tx_len)
{
  if ((data_buf == 0) || (tx_buf == 0) || (tx_len == 0) || (tx_buf_size < 8U))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  if ((reg_count == 0U) ||
      (reg_count > MODBUS_MAX_WRITE_REG_COUNT) ||
      ((uint16_t)byte_count != (reg_count * 2U)))
  {
    return Modbus_BuildException(response_addr,
                                 MB_FC_WRITE_MULTIPLE_REGS,
                                 MB_EX_ILLEGAL_DATA_VALUE,
                                 tx_buf,
                                 tx_buf_size,
                                 tx_len);
  }

  if ((start_reg == REG_TIME_SYNC_UTC_US) && (reg_count == MODBUS_REGS_ROS_TIME))
  {
    uint64_t utc_us;

    if (Modbus_ReadRosTimeUsFromRegs(data_buf, &utc_us) == 0U)
    {
      return Modbus_BuildException(response_addr,
                                   MB_FC_WRITE_MULTIPLE_REGS,
                                   MB_EX_ILLEGAL_DATA_VALUE,
                                   tx_buf,
                                   tx_buf_size,
                                   tx_len);
    }

    ModbusTimeSync_SetUtcFromMaster(utc_us);

    return Modbus_BuildWriteMultipleAck(response_addr,
                                        start_reg,
                                        reg_count,
                                        tx_buf,
                                        tx_buf_size,
                                        tx_len);
  }

  if (Modbus_IsRegRangeWithin(start_reg,
                              reg_count,
                              REG_IMU_CALIB_START,
                              REG_IMU_CALIB_END) != 0U)
  {
    return Modbus_HandleCalibrationWrite(response_addr,
                                         start_reg,
                                         reg_count,
                                         data_buf,
                                         tx_buf,
                                         tx_buf_size,
                                         tx_len);
  }

  return Modbus_BuildException(response_addr,
                               MB_FC_WRITE_MULTIPLE_REGS,
                               MB_EX_ILLEGAL_DATA_ADDRESS,
                               tx_buf,
                               tx_buf_size,
                               tx_len);
}

void Modbus_SetSlaveAddress(uint8_t address)
{
  if ((address > MODBUS_BROADCAST_ADDR) && (address <= 0xF7U))
  {
    modbus_slave_address = address;
  }
}

uint8_t Modbus_GetSlaveAddress(void)
{
  return modbus_slave_address;
}

uint16_t Modbus_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t pos;
  uint8_t bit;

  if (data == 0)
  {
    return 0U;
  }

  for (pos = 0U; pos < len; pos++)
  {
    crc ^= data[pos];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void Modbus_InvalidateSensorSnapshots(void)
{
  taskENTER_CRITICAL();
  (void)memset(&modbus_imu_snapshot, 0, sizeof(modbus_imu_snapshot));
  (void)memset(&modbus_joint_snapshot, 0, sizeof(modbus_joint_snapshot));
  (void)memset(&modbus_touch_snapshot, 0, sizeof(modbus_touch_snapshot));
  modbus_sensor_snapshot_tick = 0U;
  taskEXIT_CRITICAL();
}

void Modbus_UpdateFullFrameSnapshot(const GloveFullFrame_t *frame)
{
  GloveTimestampUs_t frame_timestamp_us;
  GlovePowerStatus_t power;

  if (frame == 0)
  {
    return;
  }

  SystemManagerTask_GetPowerStatus(&power);
  if (Modbus_IsSensorOutputReady(power.system_state) == 0U)
  {
    /* 恢复完成前拒绝旧队列或中间帧重新激活485快照。 */
    return;
  }

  frame_timestamp_us = frame->raw.timestamp_us;

  taskENTER_CRITICAL();
  modbus_imu_snapshot.frame_id = frame->frame_id;
  modbus_imu_snapshot.timestamp_us = frame_timestamp_us;
  modbus_imu_snapshot.valid_flags = frame->raw.valid_flags;
  (void)memcpy(modbus_imu_snapshot.imu,
               frame->raw.imu,
               sizeof(modbus_imu_snapshot.imu));
  (void)memcpy(modbus_imu_snapshot.quat,
               frame->raw.quat,
               sizeof(modbus_imu_snapshot.quat));
  modbus_imu_snapshot.valid = 1U;

  modbus_joint_snapshot.timestamp_us = frame_timestamp_us;
  modbus_joint_snapshot.valid_flags = frame->processed.valid_flags;
  (void)memcpy(modbus_joint_snapshot.joint_angle_deg,
               frame->processed.joint_angle_deg,
               sizeof(modbus_joint_snapshot.joint_angle_deg));
  modbus_joint_snapshot.valid = 1U;

  modbus_touch_snapshot.timestamp_us = frame_timestamp_us;
  modbus_touch_snapshot.valid_flags = frame->raw.valid_flags;
  (void)memcpy(modbus_touch_snapshot.touch,
               frame->raw.touch,
               sizeof(modbus_touch_snapshot.touch));
  modbus_touch_snapshot.valid = 1U;
  modbus_sensor_snapshot_tick = osKernelGetTickCount();
  taskEXIT_CRITICAL();
}

ModbusResult_t Modbus_ProcessRequest(const uint8_t *rx_buf,
                                     uint16_t rx_len,
                                     uint8_t *tx_buf,
                                     uint16_t tx_buf_size,
                                     uint16_t *tx_len)
{
  uint8_t request_addr;
  uint8_t response_addr;
  uint8_t func;
  uint8_t byte_count;
  uint16_t start_reg;
  uint16_t reg_count;
  uint16_t reg_value;
  uint16_t expected_len;
  uint16_t frame_crc;
  uint16_t calc_crc;

  if (tx_len != 0)
  {
    *tx_len = 0U;
  }

  if ((rx_buf == 0) || (tx_buf == 0) || (tx_len == 0) || (rx_len < MODBUS_MIN_RTU_FRAME_LEN))
  {
    return MODBUS_RESULT_FRAME_ERROR;
  }

  frame_crc = (uint16_t)(((uint16_t)rx_buf[rx_len - 1U] << 8) | rx_buf[rx_len - 2U]);
  calc_crc = Modbus_Crc16(rx_buf, (uint16_t)(rx_len - 2U));
  if (frame_crc != calc_crc)
  {
    return MODBUS_RESULT_NO_RESPONSE;
  }

  request_addr = rx_buf[0];
  func = rx_buf[1];

  if ((request_addr != modbus_slave_address) && (request_addr != MODBUS_BROADCAST_ADDR))
  {
    return MODBUS_RESULT_NO_RESPONSE;
  }

  if ((func != MB_FC_READ_HOLDING_REGS) &&
      (func != MB_FC_WRITE_SINGLE_REG) &&
      (func != MB_FC_WRITE_MULTIPLE_REGS) &&
      (func != MB_FC_READ_SENSOR_SNAPSHOT))
  {
    if (request_addr == MODBUS_BROADCAST_ADDR)
    {
      return MODBUS_RESULT_NO_RESPONSE;
    }

    return Modbus_BuildException(modbus_slave_address,
                                 func,
                                 MB_EX_ILLEGAL_FUNCTION,
                                 tx_buf,
                                 tx_buf_size,
                                 tx_len);
  }

  if (func == MB_FC_READ_SENSOR_SNAPSHOT)
  {
    if (request_addr == MODBUS_BROADCAST_ADDR)
    {
      return MODBUS_RESULT_NO_RESPONSE;
    }
    if (rx_len != MODBUS_SENSOR_SNAPSHOT_REQ_LEN)
    {
      return Modbus_BuildException(modbus_slave_address,
                                   func,
                                   MB_EX_ILLEGAL_DATA_VALUE,
                                   tx_buf,
                                   tx_buf_size,
                                   tx_len);
    }

    return Modbus_HandleReadSensorSnapshot(request_addr,
                                           tx_buf,
                                           tx_buf_size,
                                           tx_len);
  }

  if (func == MB_FC_READ_HOLDING_REGS)
  {
    if (rx_len != MODBUS_READ_REQ_LEN)
    {
      if (request_addr == MODBUS_BROADCAST_ADDR)
      {
        return MODBUS_RESULT_NO_RESPONSE;
      }

      return Modbus_BuildException(modbus_slave_address,
                                   func,
                                   MB_EX_ILLEGAL_DATA_VALUE,
                                   tx_buf,
                                   tx_buf_size,
                                   tx_len);
    }

    start_reg = Modbus_ReadU16(&rx_buf[2]);
    reg_count = Modbus_ReadU16(&rx_buf[4]);

    response_addr = (request_addr == MODBUS_BROADCAST_ADDR) ? modbus_slave_address : request_addr;
    if ((request_addr == MODBUS_BROADCAST_ADDR) &&
        ((start_reg != REG_SLAVE_ADDR) || (reg_count != 1U)))
    {
      return MODBUS_RESULT_NO_RESPONSE;
    }

    return Modbus_HandleReadHoldingRegs(response_addr,
                                        start_reg,
                                        reg_count,
                                        tx_buf,
                                        tx_buf_size,
                                        tx_len);
  }

  if (request_addr == MODBUS_BROADCAST_ADDR)
  {
    return MODBUS_RESULT_NO_RESPONSE;
  }

  if (func == MB_FC_WRITE_SINGLE_REG)
  {
    if (rx_len != MODBUS_WRITE_SINGLE_REQ_LEN)
    {
      return Modbus_BuildException(modbus_slave_address,
                                   func,
                                   MB_EX_ILLEGAL_DATA_VALUE,
                                   tx_buf,
                                   tx_buf_size,
                                   tx_len);
    }

    start_reg = Modbus_ReadU16(&rx_buf[2]);
    reg_value = Modbus_ReadU16(&rx_buf[4]);

    return Modbus_HandleWriteSingleReg(request_addr,
                                       start_reg,
                                       reg_value,
                                       tx_buf,
                                       tx_buf_size,
                                       tx_len);
  }

  if (rx_len < 9U)
  {
    return Modbus_BuildException(modbus_slave_address,
                                 func,
                                 MB_EX_ILLEGAL_DATA_VALUE,
                                 tx_buf,
                                 tx_buf_size,
                                 tx_len);
  }

  start_reg = Modbus_ReadU16(&rx_buf[2]);
  reg_count = Modbus_ReadU16(&rx_buf[4]);
  byte_count = rx_buf[6];
  expected_len = (uint16_t)(9U + byte_count);

  if (rx_len != expected_len)
  {
    return Modbus_BuildException(modbus_slave_address,
                                 func,
                                 MB_EX_ILLEGAL_DATA_VALUE,
                                 tx_buf,
                                 tx_buf_size,
                                 tx_len);
  }

  return Modbus_HandleWriteMultipleRegs(request_addr,
                                        start_reg,
                                        reg_count,
                                        &rx_buf[7],
                                        byte_count,
                                        tx_buf,
                                        tx_buf_size,
                                        tx_len);
}
