#ifndef SD_LOG_H
#define SD_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

#define SD_LOG_BLOCK_SIZE             (1024U)
#define SD_LOG_FILENAME_BYTES         (32U)

#define SD_LOG_ERROR_NONE             (0U)
#define SD_LOG_ERROR_NOT_EXFAT        (0x8001U)
#define SD_LOG_ERROR_WRITE_SHORT      (0x8002U)
#define SD_LOG_ERROR_FILE_ID_FULL     (0x8003U)

typedef enum
{
  SD_LOG_FS_NOT_MOUNTED = 0U,
  SD_LOG_FS_MOUNTED = 1U
} SdLogFsStatus_t;

typedef enum
{
  SD_LOG_RECORD_IDLE = 0U,
  SD_LOG_RECORD_RECORDING = 1U,
  SD_LOG_RECORD_STOPPING = 2U,
  SD_LOG_RECORD_ERROR = 0x8000U
} SdLogRecordStatus_t;

typedef struct
{
  uint16_t fs_status;
  uint16_t log_status;
  uint16_t error_code;
  uint16_t current_file_id;
  uint64_t current_file_size;
  uint32_t current_write_count;
  uint32_t total_size_mb;
  uint32_t free_size_mb;
  uint32_t used_size_mb;
  char current_filename[SD_LOG_FILENAME_BYTES];
  char last_filename[SD_LOG_FILENAME_BYTES];
} SdLogStatusSnapshot_t;

GloveStatus_t SdLog_Init(void);
GloveStatus_t SdLog_Start(void);
GloveStatus_t SdLog_Stop(void);
GloveStatus_t SdLog_WriteFullFrame(const GloveFullFrame_t *frame);
uint8_t SdLog_IsRecording(void);
void SdLog_GetStatus(SdLogStatusSnapshot_t *status);

#ifdef __cplusplus
}
#endif

#endif /* SD_LOG_H */
