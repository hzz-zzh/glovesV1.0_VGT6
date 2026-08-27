#include "storageTask.h"

#include "app_config.h"
#include "cmsis_os2.h"
#include "data_manager.h"
#include "FreeRTOS.h"
#include "main.h"
#include "sd_log.h"
#include "system_health.h"
#include "task.h"
#include "uart_redirect.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define STORAGE_FULL_FRAME_GET_TIMEOUT_MS  (20U)
#define STORAGE_KEY_DEBOUNCE_MS            (50U)
#define STORAGE_USER_KEY_ACTIVE_LEVEL      GPIO_PIN_SET
#define STORAGE_SIM_FRAME_PERIOD_MS        (20U)
#define STORAGE_KEY_DEBUG_PRINT            (0U)
#define STORAGE_KEY_LEVEL_PRINT_MS         (1000U)

volatile uint32_t storage_task_stack_min_words = UINT32_MAX;
volatile uint32_t storage_sim_frame_count = 0U;
volatile uint32_t storage_sim_write_count = 0U;
volatile uint32_t storage_key_press_count = 0U;

extern volatile uint32_t sd_disk_last_hal_status;
extern volatile uint32_t sd_disk_last_hal_error;
extern volatile uint32_t sd_disk_last_result;
extern volatile uint32_t sd_disk_init_count;
extern volatile uint32_t sd_disk_last_state_before;
extern volatile uint32_t sd_disk_last_state_after;
extern volatile uint32_t sd_disk_last_card_state;

static void StorageTask_UpdateStackWatermark(void)
{
  const UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);

  if ((uint32_t)free_words < storage_task_stack_min_words)
  {
    storage_task_stack_min_words = (uint32_t)free_words;
  }
}

static void StorageTask_ReportSdResult(GloveStatus_t result)
{
  SdLogStatusSnapshot_t status;

  SdLog_GetStatus(&status);
  SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_SD_ERROR,
                        SYSTEM_ERROR_SD,
                        SYSTEM_HEALTH_SOURCE_STORAGE,
                        status.error_code,
                        (result == GLOVE_STATUS_OK) ? 0U : 1U);
}

static void StorageTask_ToggleRecording(void)
{
  GloveStatus_t result;
  SdLogStatusSnapshot_t status;
  const char *filename;

  if (SdLog_IsRecording() != 0U)
  {
    result = SdLog_Stop();
    StorageTask_ReportSdResult(result);
    SdLog_GetStatus(&status);
    filename = (status.last_filename[0] != '\0') ?
               status.last_filename :
               status.current_filename;

    if (result == GLOVE_STATUS_OK)
    {
      printf("[SD] record stopped file=%s blocks=%lu err=0x%04X\r\n",
             filename,
             (unsigned long)status.current_write_count,
             status.error_code);
    }
    else
    {
      printf("[SD] record stop failed status=%u err=0x%04X file=%s\r\n",
             (unsigned int)result,
             status.error_code,
             filename);
    }
  }
  else
  {
    result = SdLog_Start();
    StorageTask_ReportSdResult(result);
    SdLog_GetStatus(&status);

    if (result == GLOVE_STATUS_OK)
    {
      printf("[SD] record started file=%s\r\n", status.current_filename);
    }
    else
    {
      printf("[SD] record start failed status=%u err=0x%04X\r\n",
             (unsigned int)result,
             status.error_code);
      printf("[SD] disk init=%lu result=%lu hal=%lu err=0x%08lX state_before=%lu state_after=%lu card=%lu\r\n",
             (unsigned long)sd_disk_init_count,
             (unsigned long)sd_disk_last_result,
             (unsigned long)sd_disk_last_hal_status,
             (unsigned long)sd_disk_last_hal_error,
             (unsigned long)sd_disk_last_state_before,
             (unsigned long)sd_disk_last_state_after,
             (unsigned long)sd_disk_last_card_state);
    }
  }
}

static void StorageTask_PollUserKey(void)
{
  static uint8_t initialized = 0U;
  static uint8_t last_raw_pressed = 0U;
  static uint8_t stable_pressed = 0U;
  static uint32_t last_change_tick = 0U;
  static uint32_t last_level_print_tick = 0U;

  const GPIO_PinState key_level = HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin);
  const uint8_t raw_pressed = (key_level == STORAGE_USER_KEY_ACTIVE_LEVEL) ? 1U : 0U;
  const uint32_t now = osKernelGetTickCount();

  if (initialized == 0U)
  {
    initialized = 1U;
    last_raw_pressed = raw_pressed;
    stable_pressed = raw_pressed;
    last_change_tick = now;
    last_level_print_tick = now;
#if STORAGE_KEY_DEBUG_PRINT
    printf("[SD] key init pressed=%u level=%u active=%u\r\n",
           (unsigned int)raw_pressed,
           (unsigned int)key_level,
           (unsigned int)STORAGE_USER_KEY_ACTIVE_LEVEL);
#endif
    return;
  }

#if STORAGE_KEY_DEBUG_PRINT
  if ((now - last_level_print_tick) >= STORAGE_KEY_LEVEL_PRINT_MS)
  {
    last_level_print_tick = now;
    printf("[SD] key level=%u pressed=%u tick=%lu\r\n",
           (unsigned int)key_level,
           (unsigned int)raw_pressed,
           (unsigned long)now);
  }
#endif

  if (raw_pressed != last_raw_pressed)
  {
    last_raw_pressed = raw_pressed;
    last_change_tick = now;
#if STORAGE_KEY_DEBUG_PRINT
    printf("[SD] key raw changed pressed=%u tick=%lu\r\n",
           (unsigned int)raw_pressed,
           (unsigned long)now);
#endif
  }

  if (((now - last_change_tick) >= STORAGE_KEY_DEBOUNCE_MS) &&
      (raw_pressed != stable_pressed))
  {
    stable_pressed = raw_pressed;
#if STORAGE_KEY_DEBUG_PRINT
    printf("[SD] key stable pressed=%u tick=%lu\r\n",
           (unsigned int)stable_pressed,
           (unsigned long)now);
#endif
    if (stable_pressed != 0U)
    {
      storage_key_press_count++;
#if STORAGE_KEY_DEBUG_PRINT
      printf("[SD] key press count=%lu\r\n",
             (unsigned long)storage_key_press_count);
#endif
      StorageTask_ToggleRecording();
    }
  }
}

#if (APP_ENABLE_STORAGE_SIM_DATA != 0U)
static void StorageTask_BuildSimFrame(GloveFullFrame_t *frame)
{
  const uint32_t frame_id = storage_sim_frame_count + 1U;
  const GloveTimestampUs_t timestamp_us =
    (GloveTimestampUs_t)osKernelGetTickCount() * 1000ULL;

  memset(frame, 0, sizeof(*frame));

  frame->frame_id = frame_id;
  frame->timestamp_us = timestamp_us;
  frame->valid_flags = GLOVE_FRAME_FLAG_IMU_VALID |
                       GLOVE_FRAME_FLAG_QUAT_VALID |
                       GLOVE_FRAME_FLAG_TOUCH_VALID |
                       GLOVE_FRAME_FLAG_ALGORITHM_VALID |
                       GLOVE_FRAME_VALID_IMU_ALL_MASK;

  frame->raw.frame_id = frame_id;
  frame->raw.timestamp_us = timestamp_us;
  frame->raw.valid_flags = frame->valid_flags;

  frame->processed.frame_id = frame_id;
  frame->processed.timestamp_us = timestamp_us;
  frame->processed.valid_flags = GLOVE_FRAME_FLAG_ALGORITHM_VALID;

  for (uint16_t imu_index = 0U; imu_index < GLOVE_IMU_COUNT; imu_index++)
  {
    const float base = ((float)frame_id * 0.01f) + (float)imu_index;

    frame->raw.imu[imu_index].accel_mps2.x = base;
    frame->raw.imu[imu_index].accel_mps2.y = base + 0.10f;
    frame->raw.imu[imu_index].accel_mps2.z = base + 0.20f;
    frame->raw.imu[imu_index].gyro_radps.x = base * 0.10f;
    frame->raw.imu[imu_index].gyro_radps.y = base * 0.10f + 0.01f;
    frame->raw.imu[imu_index].gyro_radps.z = base * 0.10f + 0.02f;

    frame->raw.quat[imu_index].w = 1.0f;
    frame->raw.quat[imu_index].x = base * 0.001f;
    frame->raw.quat[imu_index].y = base * 0.002f;
    frame->raw.quat[imu_index].z = base * 0.003f;
    frame->processed.imu_attitude[imu_index] = frame->raw.quat[imu_index];
  }

  for (uint16_t joint_index = 0U; joint_index < GLOVE_JOINT_DOF_COUNT; joint_index++)
  {
    frame->processed.joint_angle_deg[joint_index] =
      (float)(frame_id % 360U) + ((float)joint_index * 1.5f);
  }

  for (uint16_t touch_index = 0U; touch_index < GLOVE_TOUCH_COUNT; touch_index++)
  {
    frame->raw.touch[touch_index].value =
      (uint16_t)((frame_id + ((uint32_t)touch_index * 3U)) & 0x0FFFU);
    frame->raw.touch[touch_index].baseline = 1000U;
  }

  storage_sim_frame_count = frame_id;
}

static void StorageTask_WriteSimFrameIfDue(void)
{
  static GloveFullFrame_t sim_frame;
  static uint32_t last_sim_tick = 0U;
  GloveStatus_t result;

  const uint32_t now = osKernelGetTickCount();

  if (SdLog_IsRecording() == 0U)
  {
    return;
  }

  if ((now - last_sim_tick) < STORAGE_SIM_FRAME_PERIOD_MS)
  {
    return;
  }

  last_sim_tick = now;
  StorageTask_BuildSimFrame(&sim_frame);
  result = SdLog_WriteFullFrame(&sim_frame);
  StorageTask_ReportSdResult(result);
  if (result == GLOVE_STATUS_OK)
  {
    storage_sim_write_count++;
  }
}
#endif

void StorageTask(void *argument)
{
  (void)argument;

#if STORAGE_KEY_DEBUG_PRINT
  printf("[SD] storage task started\r\n");
#endif

  for (;;)
  {
    GloveFullFrameBlock_t *full = NULL;

    StorageTask_PollUserKey();

    if (DataManager_GetFullFrame(DATA_CONSUMER_STORAGE,
                                 &full,
                                 STORAGE_FULL_FRAME_GET_TIMEOUT_MS) == GLOVE_STATUS_OK)
    {
      if (SdLog_IsRecording() != 0U)
      {
        StorageTask_ReportSdResult(SdLog_WriteFullFrame(&full->frame));
      }
      (void)DataManager_ReleaseFullFrame(full);
    }
#if (APP_ENABLE_STORAGE_SIM_DATA != 0U)
    else
    {
      StorageTask_WriteSimFrameIfDue();
    }
#endif

    StorageTask_UpdateStackWatermark();
  }
}
