#include "sd_log.h"

#include "cmsis_os2.h"
#include "ff.h"
#include "glove_hand_config.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define SD_LOG_CONTENT_SIZE           (SD_LOG_BLOCK_SIZE - 3U)
#define SD_LOG_FRAME_HEAD             (0xA5U)
#define SD_LOG_FRAME_TAIL             (0x5AU)
#define SD_LOG_SEPARATOR              (0x00U)
#define SD_LOG_HAND_RIGHT             (0x00U)
#define SD_LOG_HAND_LEFT              (0x80U)
#define SD_LOG_DATA_TYPE_MASK         (0x7FU)
#define SD_LOG_HAND_FLAG_MASK         (0x80U)
#define SD_LOG_DATA_TYPE_IMU          (0x01U)
#define SD_LOG_DATA_TYPE_JOINT        (0x02U)
#define SD_LOG_DATA_TYPE_TACTILE      (0x03U)
#define SD_LOG_MOUNT_RETRY_COUNT      (1U)
#define SD_LOG_MOUNT_RETRY_DELAY_MS   (100U)
#define SD_LOG_SYNC_INTERVAL_BYTES    (64U * 1024U)
#define SD_LOG_MAX_FILE_ID            (9999U)
#define SD_LOG_SECTION_OVERHEAD_BYTES (2U + 8U + 1U)
#define SD_LOG_FLOAT_BYTES            (4U)
#define SD_LOG_U16_BYTES              (2U)
#define SD_LOG_IMU_PAYLOAD_BYTES      (GLOVE_IMU_COUNT * 10U * SD_LOG_FLOAT_BYTES)
#define SD_LOG_JOINT_PAYLOAD_BYTES    (GLOVE_JOINT_DOF_COUNT * SD_LOG_FLOAT_BYTES)
#define SD_LOG_TOUCH_PAYLOAD_BYTES    (GLOVE_TOUCH_COUNT * SD_LOG_U16_BYTES)
#define SD_LOG_MIN_CONTENT_SIZE       (SD_LOG_IMU_PAYLOAD_BYTES + \
                                       SD_LOG_JOINT_PAYLOAD_BYTES + \
                                       SD_LOG_TOUCH_PAYLOAD_BYTES + \
                                       (3U * SD_LOG_SECTION_OVERHEAD_BYTES))

#if SD_LOG_MIN_CONTENT_SIZE > SD_LOG_CONTENT_SIZE
#error "SD_LOG_BLOCK_SIZE is too small for the configured payload sizes"
#endif

static FATFS sd_log_fs;
static FIL sd_log_file;
static DIR sd_log_dir;
static FILINFO sd_log_info;

static SdLogStatusSnapshot_t sd_log_status = {
  .fs_status = SD_LOG_FS_NOT_MOUNTED,
  .log_status = SD_LOG_RECORD_IDLE,
  .error_code = SD_LOG_ERROR_NONE
};

static __ALIGNED(4) uint8_t sd_log_block[SD_LOG_BLOCK_SIZE];
static float sd_log_imu_payload[GLOVE_IMU_COUNT * 10U];
static float sd_log_joint_payload[GLOVE_JOINT_DOF_COUNT];
static uint16_t sd_log_touch_payload[GLOVE_TOUCH_COUNT];
static uint32_t sd_log_unsynced_bytes;

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

static uint8_t SdLog_MakeDataId(uint8_t hand_flag, uint8_t data_type)
{
  return (uint8_t)((hand_flag & SD_LOG_HAND_FLAG_MASK) |
                   (data_type & SD_LOG_DATA_TYPE_MASK));
}

static uint8_t SdLog_GetHandFlag(void)
{
  return (GloveHandConfig_GetHandSide() == GLOVE_HAND_LEFT) ?
         SD_LOG_HAND_LEFT :
         SD_LOG_HAND_RIGHT;
}

static void SdLog_WriteU64Le(uint8_t *data, uint64_t value)
{
  for (uint8_t index = 0U; index < 8U; index++)
  {
    data[index] = (uint8_t)(value >> (index * 8U));
  }
}

static void SdLog_UpdatePayloads(const GloveFullFrame_t *frame)
{
  uint16_t payload_index = 0U;

  for (uint16_t imu_index = 0U; imu_index < GLOVE_IMU_COUNT; imu_index++)
  {
    sd_log_imu_payload[payload_index++] = frame->raw.imu[imu_index].accel_mps2.x;
    sd_log_imu_payload[payload_index++] = frame->raw.imu[imu_index].accel_mps2.y;
    sd_log_imu_payload[payload_index++] = frame->raw.imu[imu_index].accel_mps2.z;
    sd_log_imu_payload[payload_index++] = frame->raw.imu[imu_index].gyro_radps.x;
    sd_log_imu_payload[payload_index++] = frame->raw.imu[imu_index].gyro_radps.y;
    sd_log_imu_payload[payload_index++] = frame->raw.imu[imu_index].gyro_radps.z;
    sd_log_imu_payload[payload_index++] = frame->raw.quat[imu_index].w;
    sd_log_imu_payload[payload_index++] = frame->raw.quat[imu_index].x;
    sd_log_imu_payload[payload_index++] = frame->raw.quat[imu_index].y;
    sd_log_imu_payload[payload_index++] = frame->raw.quat[imu_index].z;
  }

  memcpy(sd_log_joint_payload,
         frame->processed.joint_angle_deg,
         sizeof(sd_log_joint_payload));

  for (uint16_t touch_index = 0U; touch_index < GLOVE_TOUCH_COUNT; touch_index++)
  {
    sd_log_touch_payload[touch_index] = frame->raw.touch[touch_index].value;
  }
}

static void SdLog_BuildFrameBlock(uint8_t block[SD_LOG_BLOCK_SIZE],
                                  const GloveFullFrame_t *frame)
{
  uint16_t offset = 0U;
  uint16_t crc;
  const uint8_t hand_flag = SdLog_GetHandFlag();
  const uint64_t timestamp_us = frame->timestamp_us;

  SdLog_UpdatePayloads(frame);
  memset(block, 0, SD_LOG_BLOCK_SIZE);

  block[offset++] = SD_LOG_FRAME_HEAD;
  block[offset++] = SdLog_MakeDataId(hand_flag, SD_LOG_DATA_TYPE_IMU);
  memcpy(&block[offset], sd_log_imu_payload, sizeof(sd_log_imu_payload));
  offset = (uint16_t)(offset + sizeof(sd_log_imu_payload));
  SdLog_WriteU64Le(&block[offset], timestamp_us);
  offset = (uint16_t)(offset + 8U);
  block[offset++] = SD_LOG_FRAME_TAIL;

  block[offset++] = SD_LOG_FRAME_HEAD;
  block[offset++] = SdLog_MakeDataId(hand_flag, SD_LOG_DATA_TYPE_JOINT);
  memcpy(&block[offset], sd_log_joint_payload, sizeof(sd_log_joint_payload));
  offset = (uint16_t)(offset + sizeof(sd_log_joint_payload));
  SdLog_WriteU64Le(&block[offset], timestamp_us);
  offset = (uint16_t)(offset + 8U);
  block[offset++] = SD_LOG_FRAME_TAIL;

  block[offset++] = SD_LOG_FRAME_HEAD;
  block[offset++] = SdLog_MakeDataId(hand_flag, SD_LOG_DATA_TYPE_TACTILE);
  memcpy(&block[offset], sd_log_touch_payload, sizeof(sd_log_touch_payload));
  offset = (uint16_t)(offset + sizeof(sd_log_touch_payload));
  SdLog_WriteU64Le(&block[offset], timestamp_us);
  offset = (uint16_t)(offset + 8U);
  block[offset++] = SD_LOG_FRAME_TAIL;

  offset = SD_LOG_CONTENT_SIZE;
  crc = SdLog_Crc16(block, SD_LOG_CONTENT_SIZE);
  block[offset++] = (uint8_t)(crc & 0xFFU);
  block[offset++] = (uint8_t)(crc >> 8);
  block[offset++] = SD_LOG_SEPARATOR;
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
  FRESULT result = f_mount(&sd_log_fs, "0:", 1U);

  if (result != FR_OK)
  {
    sd_log_status.fs_status = SD_LOG_FS_NOT_MOUNTED;
    SdLog_SetError((uint16_t)result);
    return result;
  }

  if (sd_log_fs.fs_type != FS_EXFAT)
  {
    (void)f_mount(NULL, "0:", 0U);
    memset(&sd_log_fs, 0, sizeof(sd_log_fs));
    sd_log_status.fs_status = SD_LOG_FS_NOT_MOUNTED;
    SdLog_SetError(SD_LOG_ERROR_NOT_EXFAT);
    return FR_NO_FILESYSTEM;
  }

  sd_log_status.fs_status = SD_LOG_FS_MOUNTED;
  sd_log_status.error_code = SD_LOG_ERROR_NONE;
  if (sd_log_status.log_status != SD_LOG_RECORD_RECORDING)
  {
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
  }
  SdLog_UpdateCapacity();

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

  if (sd_log_status.current_file_id == 0U)
  {
    SdLog_UpdateNextFileId();
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
                  FA_CREATE_ALWAYS | FA_WRITE);
  if (result != FR_OK)
  {
    SdLog_SetError((uint16_t)result);
    return result;
  }

  sd_log_status.current_file_size = 0U;
  sd_log_status.current_write_count = 0U;
  sd_log_status.error_code = SD_LOG_ERROR_NONE;
  sd_log_status.log_status = SD_LOG_RECORD_IDLE;
  sd_log_unsynced_bytes = 0U;

  return FR_OK;
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

  if (sd_log_status.log_status == SD_LOG_RECORD_ERROR)
  {
    if (sd_log_file.obj.fs != NULL)
    {
      (void)f_close(&sd_log_file);
      memset(&sd_log_file, 0, sizeof(sd_log_file));
    }
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
  }

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

  if ((sd_log_status.log_status != SD_LOG_RECORD_RECORDING) &&
      (sd_log_file.obj.fs == NULL))
  {
    return GLOVE_STATUS_OK;
  }

  sd_log_status.log_status = SD_LOG_RECORD_STOPPING;

  if ((sd_log_file.obj.fs != NULL) && (sd_log_unsynced_bytes != 0U))
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

  if (sd_log_file.obj.fs != NULL)
  {
    result = f_close(&sd_log_file);
    if (result != FR_OK)
    {
      SdLog_SetError((uint16_t)result);
    }
    else
    {
      memset(&sd_log_file, 0, sizeof(sd_log_file));
    }
  }

  if (sd_log_status.log_status != SD_LOG_RECORD_ERROR)
  {
    strncpy(sd_log_status.last_filename,
            sd_log_status.current_filename,
            sizeof(sd_log_status.last_filename));
    sd_log_status.last_filename[sizeof(sd_log_status.last_filename) - 1U] = '\0';
    sd_log_status.log_status = SD_LOG_RECORD_IDLE;
    sd_log_status.error_code = SD_LOG_ERROR_NONE;
    SdLog_UpdateCapacity();
  }

  if (sd_log_status.log_status == SD_LOG_RECORD_ERROR)
  {
    return GLOVE_STATUS_ERROR;
  }

  return SdLog_ResultToStatus(result);
}

GloveStatus_t SdLog_WriteFullFrame(const GloveFullFrame_t *frame)
{
  UINT written = 0U;
  FRESULT result;

  if (frame == NULL)
  {
    return GLOVE_STATUS_INVALID_PARAM;
  }

  if (sd_log_status.log_status != SD_LOG_RECORD_RECORDING)
  {
    return GLOVE_STATUS_NOT_READY;
  }

  SdLog_BuildFrameBlock(sd_log_block, frame);

  result = f_write(&sd_log_file, sd_log_block, SD_LOG_BLOCK_SIZE, &written);
  if ((result != FR_OK) || (written != SD_LOG_BLOCK_SIZE))
  {
    SdLog_SetError((result == FR_OK) ? SD_LOG_ERROR_WRITE_SHORT : (uint16_t)result);
    return GLOVE_STATUS_ERROR;
  }

  sd_log_status.current_file_size += written;
  sd_log_status.current_write_count++;
  sd_log_unsynced_bytes += written;

  if (sd_log_unsynced_bytes >= SD_LOG_SYNC_INTERVAL_BYTES)
  {
    result = f_sync(&sd_log_file);
    if (result != FR_OK)
    {
      SdLog_SetError((uint16_t)result);
      return GLOVE_STATUS_ERROR;
    }
    sd_log_unsynced_bytes = 0U;
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
    __disable_irq();
    *status = sd_log_status;
    __enable_irq();
  }
}
