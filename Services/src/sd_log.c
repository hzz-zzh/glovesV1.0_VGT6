#include "sd_log.h"

#include "app_config.h"
#include "app_version.h"
#include "cmsis_os2.h"
#include "ff.h"
#include "glove_hand_config.h"
#include "main.h"
#include "modbus_time_sync.h"
#include "sd_diskio.h"
#include "system_health.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define SD_LOG_CONTENT_SIZE             (SD_LOG_BLOCK_SIZE - 3U)
#define SD_LOG_CRC_OFFSET               (SD_LOG_CONTENT_SIZE)
#define SD_LOG_SEPARATOR_OFFSET         (SD_LOG_BLOCK_SIZE - 1U)
#define SD_LOG_MOUNT_RETRY_COUNT        (1U)
#define SD_LOG_MOUNT_RETRY_DELAY_MS     (100U)
#define SD_LOG_SYNC_INTERVAL_BYTES      (64U * 1024U)
#define SD_LOG_MAX_FILE_ID              (9999U)
#define SD_LOG_BUFFERED_RECORDS         (4U)
#define SD_LOG_WRITE_BUFFER_SIZE        (SD_LOG_BLOCK_SIZE * SD_LOG_BUFFERED_RECORDS)
#define SD_LOG_RECORD_BUDGET_MS         (5U)
#define SD_LOG_FLOAT32_BYTES             (4U)
#define SD_LOG_U16_BYTES                 (2U)
#define SD_LOG_IMU_FLOAT_COUNT          (GLOVE_IMU_COUNT * 10U)
#define SD_LOG_PAYLOAD_OFFSET           (SD_LOG_RECORD_HEADER_SIZE)
/* 文件格式宽度必须是编译期常量，不能在预处理#if表达式中使用sizeof。 */
#define SD_LOG_IMU_PAYLOAD_BYTES        (SD_LOG_IMU_FLOAT_COUNT * SD_LOG_FLOAT32_BYTES)
#define SD_LOG_JOINT_PAYLOAD_BYTES      (GLOVE_JOINT_DOF_COUNT * SD_LOG_FLOAT32_BYTES)
#define SD_LOG_TOUCH_PAYLOAD_BYTES      (GLOVE_TOUCH_COUNT * SD_LOG_U16_BYTES)
#define SD_LOG_BASELINE_PAYLOAD_BYTES   (GLOVE_TOUCH_COUNT * SD_LOG_U16_BYTES)
#define SD_LOG_PAYLOAD_END              (SD_LOG_PAYLOAD_OFFSET + \
                                         SD_LOG_IMU_PAYLOAD_BYTES + \
                                         SD_LOG_JOINT_PAYLOAD_BYTES + \
                                         SD_LOG_TOUCH_PAYLOAD_BYTES + \
                                         SD_LOG_BASELINE_PAYLOAD_BYTES)

#if SD_LOG_PAYLOAD_END > SD_LOG_CONTENT_SIZE
#error "SD_LOG_BLOCK_SIZE is too small for the configured payload sizes"
#endif

static FATFS sd_log_fs;
static FIL sd_log_file;
static DIR sd_log_dir;
static FILINFO sd_log_info;

static SdLogStatusSnapshot_t sd_log_status = {
  .fs_status = SD_LOG_FS_NOT_MOUNTED,
  .log_status = SD_LOG_RECORD_IDLE,
  .error_code = SD_LOG_ERROR_NONE,
  .format_version = SD_LOG_FORMAT_VERSION,
  .block_size = SD_LOG_BLOCK_SIZE
};

/* 4帧合并为一次FatFs写入，降低200Hz记录时的文件系统和DMA调用次数。 */
static __ALIGNED(4) uint8_t sd_log_write_buffer[SD_LOG_WRITE_BUFFER_SIZE];
static uint8_t sd_log_buffered_records;
static uint32_t sd_log_unsynced_bytes;

static void SdLog_ReleaseMediaSession(void)
{
  /* 先注销FatFs对象，再复位SDMMC会话，下一次Start必须重新识别介质。 */
  (void)f_mount(NULL, "0:", 0U);
  memset(&sd_log_fs, 0, sizeof(sd_log_fs));
  memset(&sd_log_file, 0, sizeof(sd_log_file));
  memset(&sd_log_dir, 0, sizeof(sd_log_dir));
  memset(&sd_log_info, 0, sizeof(sd_log_info));
  sd_log_buffered_records = 0U;
  sd_log_unsynced_bytes = 0U;
  sd_log_status.fs_status = SD_LOG_FS_NOT_MOUNTED;
  SdDisk_Deinitialize();
}

static uint16_t SdLog_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t index = 0U; index < len; index++)
  {
    crc ^= data[index];
    for (uint8_t bit = 0U; bit < 8U; bit++)
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

static void SdLog_SetError(uint16_t error_code)
{
  sd_log_status.error_code = error_code;
  sd_log_status.log_status = SD_LOG_RECORD_ERROR;
}

static void SdLog_WriteU16Le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8);
}

static void SdLog_WriteU32Le(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
  data[2] = (uint8_t)((value >> 16) & 0xFFU);
  data[3] = (uint8_t)(value >> 24);
}

static void SdLog_WriteU64Le(uint8_t *data, uint64_t value)
{
  for (uint8_t index = 0U; index < 8U; index++)
  {
    data[index] = (uint8_t)(value >> (index * 8U));
  }
}

static void SdLog_WriteFloatLe(uint8_t *data, float value)
{
  union
  {
    float f32;
    uint32_t u32;
  } converter;

  converter.f32 = value;
  SdLog_WriteU32Le(data, converter.u32);
}

static void SdLog_BuildFrameBlock(uint8_t block[SD_LOG_BLOCK_SIZE],
                                  const GloveFullFrame_t *frame)
{
  SystemHealthSnapshot_t health;
  uint16_t crc;
  uint16_t offset = SD_LOG_PAYLOAD_OFFSET;

  memset(block, 0, SD_LOG_BLOCK_SIZE);
  SystemHealth_GetSnapshot(&health);

  memcpy(&block[0], "GLV2", 4U);
  SdLog_WriteU16Le(&block[4], SD_LOG_FORMAT_VERSION);
  SdLog_WriteU16Le(&block[6], SD_LOG_RECORD_HEADER_SIZE);
  SdLog_WriteU16Le(&block[8], SD_LOG_BLOCK_SIZE);
  SdLog_WriteU16Le(&block[10], GLOVE_IMU_COUNT);
  SdLog_WriteU16Le(&block[12], GLOVE_JOINT_DOF_COUNT);
  SdLog_WriteU16Le(&block[14], GLOVE_TOUCH_COUNT);
  block[16] = (uint8_t)GloveHandConfig_GetHandSide();
  block[17] = ModbusTimeSync_IsSynced();
  SdLog_WriteU16Le(&block[18], GLOVE_FW_VERSION_MAJOR);
  SdLog_WriteU16Le(&block[20], GLOVE_FW_VERSION_MINOR);
  SdLog_WriteU16Le(&block[22], GLOVE_FW_VERSION_PATCH);
  SdLog_WriteU32Le(&block[24], frame->frame_id);
  SdLog_WriteU64Le(&block[28], frame->timestamp_us);
  SdLog_WriteU32Le(&block[36], frame->valid_flags);
  SdLog_WriteU32Le(&block[40], frame->raw.valid_flags);
  SdLog_WriteU32Le(&block[44], frame->processed.valid_flags);
  SdLog_WriteU16Le(&block[48],
                   (uint16_t)((frame->raw.valid_flags &
                               GLOVE_FRAME_VALID_IMU_ALL_MASK) >>
                              GLOVE_FRAME_VALID_IMU_BIT_SHIFT));
  SdLog_WriteU16Le(&block[50], (uint16_t)frame->processed.process_status);
  SdLog_WriteU16Le(&block[52], frame->processed.calibration_seq);
  block[54] = frame->processed.calibration_applied;
  SdLog_WriteU32Le(&block[56], frame->raw.imu_sensor_seq);
  SdLog_WriteU32Le(&block[60], frame->raw.touch_sensor_seq);
  SdLog_WriteU64Le(&block[64], frame->raw.imu_timestamp_us);
  SdLog_WriteU64Le(&block[72], frame->raw.touch_timestamp_us);
  SdLog_WriteU32Le(&block[80], health.current_flags);
  SdLog_WriteU16Le(&block[84], health.state);
  SdLog_WriteU16Le(&block[86], health.current_error);
  SdLog_WriteU16Le(&block[88], health.current_source);
  SdLog_WriteU16Le(&block[90], health.current_target);
  SdLog_WriteU16Le(&block[92], health.live_imu_mask);
  SdLog_WriteU16Le(&block[94], health.sensor_ready_flags);

  for (uint16_t index = 0U; index < GLOVE_IMU_COUNT; index++)
  {
    const GloveImuSample_t *imu = &frame->raw.imu[index];
    const GloveQuaternion_t *quat = &frame->raw.quat[index];
    const float values[10] = {
      imu->accel_mps2.x, imu->accel_mps2.y, imu->accel_mps2.z,
      imu->gyro_radps.x, imu->gyro_radps.y, imu->gyro_radps.z,
      quat->w, quat->x, quat->y, quat->z
    };

    for (uint8_t value_index = 0U; value_index < 10U; value_index++)
    {
      SdLog_WriteFloatLe(&block[offset], values[value_index]);
      offset += sizeof(float);
    }
  }

  for (uint16_t index = 0U; index < GLOVE_JOINT_DOF_COUNT; index++)
  {
    SdLog_WriteFloatLe(&block[offset], frame->processed.joint_angle_deg[index]);
    offset += sizeof(float);
  }

  for (uint16_t index = 0U; index < GLOVE_TOUCH_COUNT; index++)
  {
    SdLog_WriteU16Le(&block[offset], frame->raw.touch[index].value);
    offset += sizeof(uint16_t);
  }

  for (uint16_t index = 0U; index < GLOVE_TOUCH_COUNT; index++)
  {
    SdLog_WriteU16Le(&block[offset], frame->raw.touch[index].baseline);
    offset += sizeof(uint16_t);
  }

  crc = SdLog_Crc16(block, SD_LOG_CONTENT_SIZE);
  SdLog_WriteU16Le(&block[SD_LOG_CRC_OFFSET], crc);
  block[SD_LOG_SEPARATOR_OFFSET] = 0U;
}

static GloveStatus_t SdLog_ResultToStatus(FRESULT result)
{
  if (result == FR_OK)
  {
    return GLOVE_STATUS_OK;
  }
  if ((result == FR_NOT_READY) || (result == FR_NOT_ENABLED))
  {
    return GLOVE_STATUS_NOT_READY;
  }
  return GLOVE_STATUS_ERROR;
}

static void SdLog_UpdateCapacity(void)
{
  FATFS *fs;
  DWORD free_clusters;

  if (f_getfree("0:", &free_clusters, &fs) == FR_OK)
  {
    const uint64_t total_sectors = (uint64_t)(fs->n_fatent - 2U) * fs->csize;
    const uint64_t free_sectors = (uint64_t)free_clusters * fs->csize;
    const uint64_t total_mb = total_sectors / 2048U;
    const uint64_t free_mb = free_sectors / 2048U;

    sd_log_status.total_size_mb = (uint32_t)total_mb;
    sd_log_status.free_size_mb = (uint32_t)free_mb;
    sd_log_status.used_size_mb = (uint32_t)(total_mb - free_mb);
  }
}

static uint16_t SdLog_ParseLogFileId(const char *filename)
{
  uint16_t file_id = 0U;

  if ((filename == NULL) ||
      (strncmp(filename, "LOG", 3U) != 0) ||
      (strlen(filename) != 11U) ||
      (strcmp(&filename[7], ".BIN") != 0))
  {
    return 0U;
  }

  for (uint8_t index = 3U; index < 7U; index++)
  {
    if ((filename[index] < '0') || (filename[index] > '9'))
    {
      return 0U;
    }
    file_id = (uint16_t)((file_id * 10U) + (uint16_t)(filename[index] - '0'));
  }
  return file_id;
}

static void SdLog_UpdateNextFileId(void)
{
  FRESULT result;
  uint16_t file_id;
  uint16_t max_file_id = 0U;

  /* 文件号属于当前介质，重新挂载后不能沿用上一张卡的扫描结果。 */
  sd_log_status.current_file_id = 0U;
  result = f_opendir(&sd_log_dir, "0:/");
  if (result != FR_OK)
  {
    return;
  }

  for (;;)
  {
    result = f_readdir(&sd_log_dir, &sd_log_info);
    if ((result != FR_OK) || (sd_log_info.fname[0] == '\0'))
    {
      break;
    }
    file_id = SdLog_ParseLogFileId(sd_log_info.fname);
    if (file_id > max_file_id)
    {
      max_file_id = file_id;
    }
  }

  (void)f_closedir(&sd_log_dir);
  sd_log_status.current_file_id = max_file_id;
}

static FRESULT SdLog_MountOnce(void)
{
  FRESULT result;

  result = f_mount(&sd_log_fs, "0:", 1U);

  if (result != FR_OK)
  {
    SdLog_ReleaseMediaSession();
    SdLog_SetError((uint16_t)result);
    return result;
  }
  if (sd_log_fs.fs_type != FS_EXFAT)
  {
    SdLog_ReleaseMediaSession();
    SdLog_SetError(SD_LOG_ERROR_NOT_EXFAT);
    return FR_NO_FILESYSTEM;
  }

  sd_log_status.fs_status = SD_LOG_FS_MOUNTED;
  sd_log_status.error_code = SD_LOG_ERROR_NONE;
  if ((sd_log_status.log_status != SD_LOG_RECORD_RECORDING) &&
      (sd_log_status.log_status != SD_LOG_RECORD_PREPARING))
  {
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
  }
  SdLog_UpdateCapacity();
  SdLog_UpdateNextFileId();
  return FR_OK;
}

static FRESULT SdLog_Mount(void)
{
  FRESULT result = FR_NOT_READY;

  for (uint8_t retry = 0U; retry < SD_LOG_MOUNT_RETRY_COUNT; retry++)
  {
    result = SdLog_MountOnce();
    if (result == FR_OK)
    {
      break;
    }
    osDelay(SD_LOG_MOUNT_RETRY_DELAY_MS);
  }
  return result;
}

static FRESULT SdLog_CreateFile(void)
{
  FRESULT result;

  if (sd_log_status.fs_status != SD_LOG_FS_MOUNTED)
  {
    result = SdLog_Mount();
    if (result != FR_OK)
    {
      return result;
    }
  }
  if (sd_log_file.obj.fs != NULL)
  {
    (void)f_close(&sd_log_file);
    memset(&sd_log_file, 0, sizeof(sd_log_file));
  }
  if (sd_log_status.current_file_id >= SD_LOG_MAX_FILE_ID)
  {
    SdLog_SetError(SD_LOG_ERROR_FILE_ID_FULL);
    return FR_DENIED;
  }

  sd_log_status.current_file_id++;
  (void)snprintf(sd_log_status.current_filename,
                 sizeof(sd_log_status.current_filename),
                 "0:/LOG%04u.BIN",
                 sd_log_status.current_file_id);
  result = f_open(&sd_log_file,
                  sd_log_status.current_filename,
                  FA_CREATE_NEW | FA_WRITE);
  if (result != FR_OK)
  {
    SdLog_SetError((uint16_t)result);
    return result;
  }

  sd_log_status.current_file_size = 0U;
  sd_log_status.current_write_count = 0U;
  sd_log_status.last_write_time_ms = 0U;
  sd_log_status.max_write_time_ms = 0U;
  sd_log_status.slow_write_count = 0U;
  sd_log_status.error_code = SD_LOG_ERROR_NONE;
  sd_log_buffered_records = 0U;
  sd_log_unsynced_bytes = 0U;
  return FR_OK;
}

static FRESULT SdLog_FlushWriteBuffer(void)
{
  UINT written = 0U;
  FRESULT result;
  uint32_t started_ms;
  uint32_t elapsed_ms;
  uint32_t bytes;
  uint8_t record_count;

  if (sd_log_buffered_records == 0U)
  {
    return FR_OK;
  }

  record_count = sd_log_buffered_records;
  bytes = (uint32_t)record_count * SD_LOG_BLOCK_SIZE;
  started_ms = HAL_GetTick();
  result = f_write(&sd_log_file, sd_log_write_buffer, bytes, &written);

  /* 写失败后丢弃本批缓存，避免短写情况下再次写入造成文件内容重复。 */
  sd_log_buffered_records = 0U;
  if ((result != FR_OK) || (written != bytes))
  {
    sd_log_status.current_file_size -= bytes;
    sd_log_status.current_file_size += written;
    sd_log_status.current_write_count -= record_count;
    SdLog_SetError((result == FR_OK) ? SD_LOG_ERROR_WRITE_SHORT : (uint16_t)result);
    result = (result == FR_OK) ? FR_DISK_ERR : result;
  }
  else
  {
    sd_log_unsynced_bytes += written;
    if (sd_log_unsynced_bytes >= SD_LOG_SYNC_INTERVAL_BYTES)
    {
      result = f_sync(&sd_log_file);
      if (result != FR_OK)
      {
        SdLog_SetError((uint16_t)result);
      }
      else
      {
        sd_log_unsynced_bytes = 0U;
      }
    }
  }

  /* 统计包含周期性f_sync，才能真实反映最坏落盘阻塞时间。 */
  elapsed_ms = HAL_GetTick() - started_ms;
  sd_log_status.last_write_time_ms = elapsed_ms;
  if (elapsed_ms > sd_log_status.max_write_time_ms)
  {
    sd_log_status.max_write_time_ms = elapsed_ms;
  }
  if (elapsed_ms > ((uint32_t)record_count * SD_LOG_RECORD_BUDGET_MS))
  {
    sd_log_status.slow_write_count++;
  }
  return result;
}

GloveStatus_t SdLog_Init(void)
{
  FRESULT result;

  if (sd_log_status.fs_status == SD_LOG_FS_MOUNTED)
  {
    return GLOVE_STATUS_OK;
  }
  result = SdLog_Mount();
  return SdLog_ResultToStatus(result);
}

GloveStatus_t SdLog_Start(void)
{
  FRESULT result;

  if (sd_log_status.log_status == SD_LOG_RECORD_RECORDING)
  {
    return GLOVE_STATUS_OK;
  }
  /* 文件时间必须可信；上位机开始录制前会自动下发一次UTC时间。 */
  if (ModbusTimeSync_IsSynced() == 0U)
  {
    SdLog_SetError(SD_LOG_ERROR_TIME_UNSYNCED);
    return GLOVE_STATUS_NOT_READY;
  }
  if (sd_log_status.log_status == SD_LOG_RECORD_ERROR)
  {
    if (sd_log_file.obj.fs != NULL)
    {
      (void)f_close(&sd_log_file);
      memset(&sd_log_file, 0, sizeof(sd_log_file));
    }
  }

  sd_log_status.log_status = SD_LOG_RECORD_PREPARING;
  sd_log_status.error_code = SD_LOG_ERROR_NONE;
  /* 即使上一次处于idle，也重新建立介质会话以支持停止后的拔插。 */
  SdLog_ReleaseMediaSession();
  result = SdLog_CreateFile();
  if (result != FR_OK)
  {
    return SdLog_ResultToStatus(result);
  }
  sd_log_status.log_status = SD_LOG_RECORD_RECORDING;
  sd_log_status.error_code = SD_LOG_ERROR_NONE;
  return GLOVE_STATUS_OK;
}

GloveStatus_t SdLog_Stop(void)
{
  FRESULT result = FR_OK;
  FRESULT close_result;
  const uint8_t had_record_error =
      (sd_log_status.log_status == SD_LOG_RECORD_ERROR) ? 1U : 0U;

  if ((sd_log_status.log_status != SD_LOG_RECORD_RECORDING) &&
      (sd_log_file.obj.fs == NULL))
  {
    /* 已关闭文件的历史错误可通过停止命令确认并恢复到idle，详细错误仍保留在Health历史中。 */
    SdLog_ReleaseMediaSession();
    if (sd_log_status.log_status == SD_LOG_RECORD_ERROR)
    {
      sd_log_status.log_status = SD_LOG_RECORD_IDLE;
      sd_log_status.error_code = SD_LOG_ERROR_NONE;
    }
    return GLOVE_STATUS_OK;
  }
  if (had_record_error == 0U)
  {
    sd_log_status.log_status = SD_LOG_RECORD_STOPPING;
  }

  if (sd_log_file.obj.fs != NULL)
  {
    result = SdLog_FlushWriteBuffer();
    if ((result == FR_OK) && (sd_log_unsynced_bytes != 0U))
    {
      result = f_sync(&sd_log_file);
      if (result == FR_OK)
      {
        sd_log_unsynced_bytes = 0U;
      }
      else
      {
        SdLog_SetError((uint16_t)result);
      }
    }

    close_result = f_close(&sd_log_file);
    if (close_result != FR_OK)
    {
      SdLog_SetError((uint16_t)close_result);
      result = close_result;
    }
    else
    {
      memset(&sd_log_file, 0, sizeof(sd_log_file));
    }
  }

  strncpy(sd_log_status.last_filename,
          sd_log_status.current_filename,
          sizeof(sd_log_status.last_filename));
  sd_log_status.last_filename[sizeof(sd_log_status.last_filename) - 1U] = '\0';

  if ((had_record_error == 0U) &&
      (sd_log_status.log_status != SD_LOG_RECORD_ERROR))
  {
    SdLog_UpdateCapacity();
    SdLog_ReleaseMediaSession();
    /* IDLE只在文件关闭、文件系统卸载和SDMMC反初始化全部完成后发布。 */
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
    sd_log_status.error_code = SD_LOG_ERROR_NONE;
  }
  else
  {
    SdLog_ReleaseMediaSession();
    sd_log_status.log_status = SD_LOG_RECORD_ERROR;
  }

  return (sd_log_status.log_status == SD_LOG_RECORD_ERROR) ?
         GLOVE_STATUS_ERROR : SdLog_ResultToStatus(result);
}

GloveStatus_t SdLog_WriteFullFrame(const GloveFullFrame_t *frame)
{
  uint8_t *block;
  FRESULT result;

  if (frame == NULL)
  {
    return GLOVE_STATUS_INVALID_PARAM;
  }
  if (sd_log_status.log_status != SD_LOG_RECORD_RECORDING)
  {
    return GLOVE_STATUS_NOT_READY;
  }

  block = &sd_log_write_buffer[(uint32_t)sd_log_buffered_records * SD_LOG_BLOCK_SIZE];
  SdLog_BuildFrameBlock(block, frame);
  sd_log_buffered_records++;
  sd_log_status.current_file_size += SD_LOG_BLOCK_SIZE;
  sd_log_status.current_write_count++;

  if (sd_log_buffered_records >= SD_LOG_BUFFERED_RECORDS)
  {
    result = SdLog_FlushWriteBuffer();
    if (result != FR_OK)
    {
      return GLOVE_STATUS_ERROR;
    }
  }
  return GLOVE_STATUS_OK;
}

uint8_t SdLog_IsRecording(void)
{
  return (sd_log_status.log_status == SD_LOG_RECORD_RECORDING) ? 1U : 0U;
}

void SdLog_GetStatus(SdLogStatusSnapshot_t *status)
{
  if (status != NULL)
  {
    taskENTER_CRITICAL();
    *status = sd_log_status;
    taskEXIT_CRITICAL();
  }
}
