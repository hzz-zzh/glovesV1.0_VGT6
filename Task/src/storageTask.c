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
#define STORAGE_CONTROL_QUEUE_DEPTH        (2U)
#define STORAGE_CONTROL_DONE_FLAG          (1UL << 0)
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

typedef enum
{
  STORAGE_CONTROL_START = 1U,
  STORAGE_CONTROL_STOP = 2U
} StorageControlCommand_t;

typedef struct
{
  StorageControlCommand_t command;
  uint32_t request_id;
} StorageControlRequest_t;

static osMessageQueueId_t s_storage_control_queue;
static osEventFlagsId_t s_storage_control_events;
static osMutexId_t s_storage_request_mutex;
static volatile GloveStatus_t s_storage_control_result = GLOVE_STATUS_NOT_READY;
static volatile uint32_t s_storage_completed_request_id;
static uint32_t s_storage_next_request_id;

static void StorageTask_DiscardPendingFrames(void);

static uint32_t StorageTask_MsToTicks(uint32_t timeout_ms)
{
  if (timeout_ms == osWaitForever)
  {
    return osWaitForever;
  }
  uint64_t ticks = ((uint64_t)timeout_ms * (uint64_t)osKernelGetTickFreq() + 999ULL) / 1000ULL;

  if ((timeout_ms != 0U) && (ticks == 0ULL))
  {
    ticks = 1ULL;
  }
  return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

GloveStatus_t StorageTask_ControlInit(void)
{
  const osMessageQueueAttr_t queue_attr = {.name = "storageControl"};
  const osEventFlagsAttr_t event_attr = {.name = "storageControlDone"};
  const osMutexAttr_t mutex_attr = {.name = "storageRequest"};

  if ((s_storage_control_queue != NULL) &&
      (s_storage_control_events != NULL) &&
      (s_storage_request_mutex != NULL))
  {
    return GLOVE_STATUS_OK;
  }

  s_storage_control_queue = osMessageQueueNew(STORAGE_CONTROL_QUEUE_DEPTH,
                                               sizeof(StorageControlRequest_t),
                                               &queue_attr);
  s_storage_control_events = osEventFlagsNew(&event_attr);
  s_storage_request_mutex = osMutexNew(&mutex_attr);

  return ((s_storage_control_queue != NULL) &&
          (s_storage_control_events != NULL) &&
          (s_storage_request_mutex != NULL)) ?
         GLOVE_STATUS_OK : GLOVE_STATUS_NO_MEMORY;
}

static GloveStatus_t StorageTask_RequestControl(StorageControlCommand_t command,
                                                uint32_t timeout_ms)
{
  uint32_t timeout_ticks = StorageTask_MsToTicks(timeout_ms);
  uint32_t remaining_ticks = timeout_ticks;
  uint32_t started_tick;
  uint32_t flags;
  GloveStatus_t result;
  StorageControlRequest_t request;

  if ((s_storage_control_queue == NULL) ||
      (s_storage_control_events == NULL) ||
      (s_storage_request_mutex == NULL))
  {
    return GLOVE_STATUS_NOT_READY;
  }

  if (osMutexAcquire(s_storage_request_mutex, timeout_ticks) != osOK)
  {
    return GLOVE_STATUS_TIMEOUT;
  }

  s_storage_next_request_id++;
  if (s_storage_next_request_id == 0U)
  {
    s_storage_next_request_id = 1U;
  }
  request.command = command;
  request.request_id = s_storage_next_request_id;
  (void)osEventFlagsClear(s_storage_control_events, STORAGE_CONTROL_DONE_FLAG);
  if (osMessageQueuePut(s_storage_control_queue, &request, 0U, 0U) != osOK)
  {
    (void)osMutexRelease(s_storage_request_mutex);
    return GLOVE_STATUS_QUEUE_FULL;
  }

  started_tick = osKernelGetTickCount();
  for (;;)
  {
    flags = osEventFlagsWait(s_storage_control_events,
                             STORAGE_CONTROL_DONE_FLAG,
                             osFlagsWaitAny,
                             remaining_ticks);
    if ((flags & osFlagsError) != 0U)
    {
      result = GLOVE_STATUS_TIMEOUT;
      break;
    }
    if (s_storage_completed_request_id == request.request_id)
    {
      result = s_storage_control_result;
      break;
    }

    /* 前一笔超时请求可能迟到，使用request_id避免误认其完成事件。 */
    if (timeout_ticks != osWaitForever)
    {
      const uint32_t elapsed_ticks = osKernelGetTickCount() - started_tick;
      if (elapsed_ticks >= timeout_ticks)
      {
        result = GLOVE_STATUS_TIMEOUT;
        break;
      }
      remaining_ticks = timeout_ticks - elapsed_ticks;
    }
  }
  (void)osMutexRelease(s_storage_request_mutex);
  return result;
}

static GloveStatus_t StorageTask_RequestControlAsync(StorageControlCommand_t command)
{
  StorageControlRequest_t request;

  if (s_storage_control_queue == NULL)
  {
    return GLOVE_STATUS_NOT_READY;
  }

  request.command = command;
  request.request_id = 0U;
  return (osMessageQueuePut(s_storage_control_queue, &request, 0U, 0U) == osOK) ?
         GLOVE_STATUS_OK : GLOVE_STATUS_QUEUE_FULL;
}

GloveStatus_t StorageTask_RequestStart(uint32_t timeout_ms)
{
  return StorageTask_RequestControl(STORAGE_CONTROL_START, timeout_ms);
}

GloveStatus_t StorageTask_RequestStop(uint32_t timeout_ms)
{
  return StorageTask_RequestControl(STORAGE_CONTROL_STOP, timeout_ms);
}

GloveStatus_t StorageTask_RequestStartAsync(void)
{
  return StorageTask_RequestControlAsync(STORAGE_CONTROL_START);
}

GloveStatus_t StorageTask_RequestStopAsync(void)
{
  return StorageTask_RequestControlAsync(STORAGE_CONTROL_STOP);
}

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

static GloveStatus_t StorageTask_StartRecordingInternal(void)
{
  GloveStatus_t result;
  SdLogStatusSnapshot_t status;

  if (SdLog_IsRecording() != 0U)
  {
    return GLOVE_STATUS_OK;
  }

  /* 文件准备期间不向Storage队列投递数据，开始边界从文件就绪后的首帧算起。 */
  DataManager_SetFullFrameStorageEnabled(0U);
  StorageTask_DiscardPendingFrames();
  result = SdLog_Start();
  if (result == GLOVE_STATUS_OK)
  {
    StorageTask_DiscardPendingFrames();
    DataManager_SetFullFrameStorageEnabled(1U);
  }
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
  return result;
}

static void StorageTask_DiscardPendingFrames(void)
{
  GloveFullFrameBlock_t *full = NULL;

  while (DataManager_GetFullFrame(DATA_CONSUMER_STORAGE, &full, 0U) == GLOVE_STATUS_OK)
  {
    (void)DataManager_ReleaseFullFrame(full);
    full = NULL;
  }
}

static void StorageTask_DrainPendingFrames(void)
{
  GloveFullFrameBlock_t *full = NULL;

  while (DataManager_GetFullFrame(DATA_CONSUMER_STORAGE, &full, 0U) == GLOVE_STATUS_OK)
  {
    if (SdLog_IsRecording() != 0U)
    {
      StorageTask_ReportSdResult(SdLog_WriteFullFrame(&full->frame));
    }
    (void)DataManager_ReleaseFullFrame(full);
    full = NULL;
  }
}

static GloveStatus_t StorageTask_StopRecordingInternal(void)
{
  GloveStatus_t result;
  SdLogStatusSnapshot_t status;
  const char *filename;

  /* 先关闭新帧投递，再把停止命令之前已经排队的数据全部写完。 */
  DataManager_SetFullFrameStorageEnabled(0U);
  StorageTask_DrainPendingFrames();
  result = SdLog_Stop();
  StorageTask_ReportSdResult(result);
  SdLog_GetStatus(&status);
  filename = (status.last_filename[0] != '\0') ?
             status.last_filename : status.current_filename;

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
  return result;
}

static void StorageTask_ToggleRecording(void)
{
  if (SdLog_IsRecording() != 0U)
  {
    (void)StorageTask_StopRecordingInternal();
  }
  else
  {
    (void)StorageTask_StartRecordingInternal();
  }
}

static void StorageTask_ServiceControl(void)
{
  StorageControlRequest_t request;

  if ((s_storage_control_queue == NULL) || (s_storage_control_events == NULL))
  {
    return;
  }

  while (osMessageQueueGet(s_storage_control_queue, &request, NULL, 0U) == osOK)
  {
    if (request.command == STORAGE_CONTROL_START)
    {
      s_storage_control_result = StorageTask_StartRecordingInternal();
    }
    else if (request.command == STORAGE_CONTROL_STOP)
    {
      s_storage_control_result = StorageTask_StopRecordingInternal();
    }
    else
    {
      s_storage_control_result = GLOVE_STATUS_INVALID_PARAM;
    }
    if (request.request_id != 0U)
    {
      s_storage_completed_request_id = request.request_id;
      (void)osEventFlagsSet(s_storage_control_events, STORAGE_CONTROL_DONE_FLAG);
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

    StorageTask_ServiceControl();
    StorageTask_PollUserKey();

    if (DataManager_GetFullFrame(DATA_CONSUMER_STORAGE,
                                 &full,
                                 STORAGE_FULL_FRAME_GET_TIMEOUT_MS) == GLOVE_STATUS_OK)
    {
      if (SdLog_IsRecording() != 0U)
      {
        GloveStatus_t write_status = SdLog_WriteFullFrame(&full->frame);
        StorageTask_ReportSdResult(write_status);
        if (write_status != GLOVE_STATUS_OK)
        {
          /* 写入异常后立即同步并关闭文件，避免错误状态下继续持有文件句柄。 */
          DataManager_SetFullFrameStorageEnabled(0U);
          (void)SdLog_Stop();
        }
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
