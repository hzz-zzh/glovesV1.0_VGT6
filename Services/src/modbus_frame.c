#include "modbus_frame.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "dataProcessTask.h"
#include "modbus_registers.h"
#include "modbus_time_sync.h"

#if MODBUS_JOINT_COUNT != GLOVE_JOINT_DOF_COUNT
#error "MODBUS_JOINT_COUNT must match GLOVE_JOINT_DOF_COUNT"
#endif

typedef struct
{
  uint8_t valid;
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

static uint8_t modbus_slave_address = MODBUS_SLAVE_ADDR_DEFAULT;
static ModbusImuSnapshot_t modbus_imu_snapshot;
static ModbusJointSnapshot_t modbus_joint_snapshot;
static ModbusTouchSnapshot_t modbus_touch_snapshot;
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

static uint16_t Modbus_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint64_t Modbus_ReadU64FromRegs(const uint8_t *data)
{
  uint64_t value = 0U;
  uint8_t index;

  for (index = 0U; index < 4U; index++)
  {
    value |= ((uint64_t)Modbus_ReadU16(&data[index * 2U])) << (index * 16U);
  }

  return value;
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

  if ((reg_addr >= REG_IMU_TIMESTAMP_US) && (reg_addr < (REG_IMU_TIMESTAMP_US + MODBUS_REGS_U64)))
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

  if ((reg_addr >= REG_JOINT_TIMESTAMP_US) && (reg_addr < (REG_JOINT_TIMESTAMP_US + MODBUS_REGS_U64)))
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

  if ((reg_addr >= REG_R_TIMESTAMP_US) && (reg_addr < (REG_R_TIMESTAMP_US + MODBUS_REGS_U64)))
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

static uint16_t Modbus_ReadU64Reg(uint64_t value, uint16_t word_offset)
{
  return (uint16_t)((value >> (word_offset * 16U)) & 0xFFFFU);
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

static uint8_t Modbus_NormalizeCalibrationQuat(const GloveQuaternion_t *input,
                                               GloveQuaternion_t *output)
{
  float norm_sq;
  float inv_norm;

  if ((input == 0) || (output == 0))
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
      return;
    }

    if (DataProcessTask_SetCalibrationTable(modbus_calib_c_apply,
                                            modbus_calib_m_apply) != GLOVE_STATUS_OK)
    {
      modbus_calib_status = IMU_CALIB_STATUS_BAD_QUAT;
      modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
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
    return;
  }

  modbus_calib_status = IMU_CALIB_STATUS_BAD_CMD;
  modbus_calib_error_index = IMU_CALIB_ERROR_NONE;
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

  taskENTER_CRITICAL();
  valid_flags = (modbus_imu_snapshot.valid != 0U) ? modbus_imu_snapshot.valid_flags : 0U;
  taskEXIT_CRITICAL();

  return (uint16_t)((valid_flags & GLOVE_FRAME_VALID_IMU_ALL_MASK) >>
                    GLOVE_FRAME_VALID_IMU_BIT_SHIFT);
}

static GloveTimestampUs_t Modbus_GetImuTimestampUs(void)
{
  GloveTimestampUs_t timestamp_us;

  taskENTER_CRITICAL();
  timestamp_us = (modbus_imu_snapshot.valid != 0U) ? modbus_imu_snapshot.timestamp_us : 0ULL;
  taskEXIT_CRITICAL();

  return timestamp_us;
}

static GloveTimestampUs_t Modbus_GetJointTimestampUs(void)
{
  GloveTimestampUs_t timestamp_us;

  taskENTER_CRITICAL();
  timestamp_us = (modbus_joint_snapshot.valid != 0U) ? modbus_joint_snapshot.timestamp_us : 0ULL;
  taskEXIT_CRITICAL();

  return timestamp_us;
}

static uint16_t Modbus_ReadJointStatusFlags(void)
{
  uint16_t status = 0U;

  taskENTER_CRITICAL();
  if (modbus_joint_snapshot.valid != 0U)
  {
    status |= JOINT_STATUS_SNAPSHOT_VALID;
    if ((modbus_joint_snapshot.valid_flags & GLOVE_FRAME_FLAG_ALGORITHM_VALID) != 0U)
    {
      status |= JOINT_STATUS_ALGORITHM_VALID;
    }
  }
  taskEXIT_CRITICAL();

  return status;
}

static uint16_t Modbus_ReadTouchStatusFlags(void)
{
  uint16_t status = 0U;

  taskENTER_CRITICAL();
  if (modbus_touch_snapshot.valid != 0U)
  {
    status |= R_STATUS_SNAPSHOT_VALID;
    if ((modbus_touch_snapshot.valid_flags & GLOVE_FRAME_FLAG_TOUCH_VALID) != 0U)
    {
      status |= R_STATUS_TOUCH_VALID;
    }
  }
  taskEXIT_CRITICAL();

  return status;
}

static uint32_t Modbus_GetJointValidBits(void)
{
  uint32_t valid_bits = 0UL;

  taskENTER_CRITICAL();
  if (modbus_joint_snapshot.valid != 0U)
  {
    for (uint32_t index = 0U; index < GLOVE_JOINT_DOF_COUNT; index++)
    {
      float value = modbus_joint_snapshot.joint_angle_deg[index];

      if ((value < 999999936.0f) && (value > -999999936.0f))
      {
        valid_bits |= (1UL << index);
      }
    }
  }
  taskEXIT_CRITICAL();

  return valid_bits;
}

static float Modbus_GetImuFloat(uint16_t imu_index, uint16_t float_index)
{
  float value = 0.0f;

  if ((imu_index >= GLOVE_IMU_COUNT) || (float_index >= MODBUS_IMU_FLOATS_PER_UNIT))
  {
    return 0.0f;
  }

  taskENTER_CRITICAL();
  if (modbus_imu_snapshot.valid != 0U)
  {
    switch (float_index)
    {
      case 0U:
        value = modbus_imu_snapshot.imu[imu_index].accel_mps2.x;
        break;
      case 1U:
        value = modbus_imu_snapshot.imu[imu_index].accel_mps2.y;
        break;
      case 2U:
        value = modbus_imu_snapshot.imu[imu_index].accel_mps2.z;
        break;
      case 3U:
        value = modbus_imu_snapshot.imu[imu_index].gyro_radps.x;
        break;
      case 4U:
        value = modbus_imu_snapshot.imu[imu_index].gyro_radps.y;
        break;
      case 5U:
        value = modbus_imu_snapshot.imu[imu_index].gyro_radps.z;
        break;
      case 6U:
        value = modbus_imu_snapshot.quat[imu_index].w;
        break;
      case 7U:
        value = modbus_imu_snapshot.quat[imu_index].x;
        break;
      case 8U:
        value = modbus_imu_snapshot.quat[imu_index].y;
        break;
      case 9U:
        value = modbus_imu_snapshot.quat[imu_index].z;
        break;
      default:
        value = 0.0f;
        break;
    }
  }
  taskEXIT_CRITICAL();

  return value;
}

static float Modbus_GetJointFloat(uint16_t joint_index)
{
  float value = 0.0f;

  if (joint_index >= GLOVE_JOINT_DOF_COUNT)
  {
    return 0.0f;
  }

  taskENTER_CRITICAL();
  if (modbus_joint_snapshot.valid != 0U)
  {
    value = modbus_joint_snapshot.joint_angle_deg[joint_index];
  }
  taskEXIT_CRITICAL();

  return value;
}

static uint16_t Modbus_GetTouchValue(uint16_t touch_index)
{
  uint16_t value = 0U;

  if (touch_index >= GLOVE_TOUCH_COUNT)
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  if (modbus_touch_snapshot.valid != 0U)
  {
    value = modbus_touch_snapshot.touch[touch_index].value;
  }
  taskEXIT_CRITICAL();

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

  taskENTER_CRITICAL();
  timestamp_us = (modbus_touch_snapshot.valid != 0U) ? modbus_touch_snapshot.timestamp_us : 0ULL;
  taskEXIT_CRITICAL();

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

  if ((reg_addr >= REG_UTC_TIMESTAMP_US) && (reg_addr < (REG_UTC_TIMESTAMP_US + MODBUS_REGS_U64)))
  {
    return Modbus_ReadU64Reg(ModbusTimeSync_GetUtcTimestampUs(), (uint16_t)(reg_addr - REG_UTC_TIMESTAMP_US));
  }

  if ((reg_addr >= REG_LOCAL_UPTIME_US) && (reg_addr < (REG_LOCAL_UPTIME_US + MODBUS_REGS_U64)))
  {
    return Modbus_ReadU64Reg(ModbusTimeSync_GetLocalUptimeUs(), (uint16_t)(reg_addr - REG_LOCAL_UPTIME_US));
  }

  if ((reg_addr >= REG_TIME_SYNC_UTC_US) && (reg_addr < (REG_TIME_SYNC_UTC_US + MODBUS_REGS_U64)))
  {
    return Modbus_ReadU64Reg(ModbusTimeSync_GetLastSyncUtcUs(), (uint16_t)(reg_addr - REG_TIME_SYNC_UTC_US));
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
    return Modbus_ReadFloatReg(0.0f, (uint16_t)(reg_addr - REG_BAT_VOLTAGE));
  }

  if ((reg_addr >= REG_BAT_CURRENT) && (reg_addr < (REG_BAT_CURRENT + MODBUS_REGS_FLOAT32)))
  {
    return Modbus_ReadFloatReg(0.0f, (uint16_t)(reg_addr - REG_BAT_CURRENT));
  }

  if ((reg_addr >= REG_SD_TOTAL_SIZE_MB) && (reg_addr <= REG_SD_STATUS_END))
  {
    return 0U;
  }

  if ((reg_addr >= REG_IMU_DATA_START) && (reg_addr <= REG_IMU_DATA_END))
  {
    return Modbus_ReadImuDataReg(reg_addr);
  }

  if ((reg_addr >= REG_IMU_TIMESTAMP_US) && (reg_addr < (REG_IMU_TIMESTAMP_US + MODBUS_REGS_U64)))
  {
    return Modbus_ReadU64Reg(Modbus_GetImuTimestampUs(), (uint16_t)(reg_addr - REG_IMU_TIMESTAMP_US));
  }

  if ((reg_addr >= REG_IMU_CALIB_START) && (reg_addr <= REG_IMU_CALIB_END))
  {
    return Modbus_ReadCalibrationReg(reg_addr);
  }

  if ((reg_addr >= REG_JOINT_DATA_START) && (reg_addr <= REG_JOINT_DATA_END))
  {
    return Modbus_ReadJointDataReg(reg_addr);
  }

  if ((reg_addr >= REG_JOINT_TIMESTAMP_US) && (reg_addr < (REG_JOINT_TIMESTAMP_US + MODBUS_REGS_U64)))
  {
    return Modbus_ReadU64Reg(Modbus_GetJointTimestampUs(), (uint16_t)(reg_addr - REG_JOINT_TIMESTAMP_US));
  }

  if ((reg_addr >= REG_R_DATA_START) && (reg_addr <= REG_R_DATA_END))
  {
    return Modbus_ReadTouchDataReg(reg_addr);
  }

  if ((reg_addr >= REG_R_TIMESTAMP_US) && (reg_addr < (REG_R_TIMESTAMP_US + MODBUS_REGS_U64)))
  {
    return Modbus_ReadU64Reg(Modbus_GetTouchTimestampUs(), (uint16_t)(reg_addr - REG_R_TIMESTAMP_US));
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

  if ((start_reg == REG_TIME_SYNC_UTC_US) && (reg_count == MODBUS_REGS_U64))
  {
    uint64_t utc_us = Modbus_ReadU64FromRegs(data_buf);
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

void Modbus_UpdateFullFrameSnapshot(const GloveFullFrame_t *frame)
{
  if (frame == 0)
  {
    return;
  }

  taskENTER_CRITICAL();
  modbus_imu_snapshot.timestamp_us = frame->raw.timestamp_us;
  modbus_imu_snapshot.valid_flags = frame->raw.valid_flags;
  (void)memcpy(modbus_imu_snapshot.imu,
               frame->raw.imu,
               sizeof(modbus_imu_snapshot.imu));
  (void)memcpy(modbus_imu_snapshot.quat,
               frame->raw.quat,
               sizeof(modbus_imu_snapshot.quat));
  modbus_imu_snapshot.valid = 1U;

  modbus_joint_snapshot.timestamp_us = frame->processed.timestamp_us;
  modbus_joint_snapshot.valid_flags = frame->processed.valid_flags;
  (void)memcpy(modbus_joint_snapshot.joint_angle_deg,
               frame->processed.joint_angle_deg,
               sizeof(modbus_joint_snapshot.joint_angle_deg));
  modbus_joint_snapshot.valid = 1U;

  modbus_touch_snapshot.timestamp_us = frame->raw.timestamp_us;
  modbus_touch_snapshot.valid_flags = frame->raw.valid_flags;
  (void)memcpy(modbus_touch_snapshot.touch,
               frame->raw.touch,
               sizeof(modbus_touch_snapshot.touch));
  modbus_touch_snapshot.valid = 1U;
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

  if ((func != MB_FC_READ_HOLDING_REGS) && (func != MB_FC_WRITE_MULTIPLE_REGS))
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
