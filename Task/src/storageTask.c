#include "storageTask.h"

#include "cmsis_os2.h"
#include "data_manager.h"
#include "FreeRTOS.h"
#include "main.h"
#include "sd_log.h"
#include "task.h"
#include <stdint.h>

#define STORAGE_FULL_FRAME_GET_TIMEOUT_MS  (20U)
#define STORAGE_KEY_DEBOUNCE_MS            (50U)
#define STORAGE_USER_KEY_ACTIVE_LEVEL      GPIO_PIN_RESET

volatile uint32_t storage_task_stack_min_words = UINT32_MAX;

static void StorageTask_UpdateStackWatermark(void)
{
  const UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);

  if ((uint32_t)free_words < storage_task_stack_min_words)
  {
    storage_task_stack_min_words = (uint32_t)free_words;
  }
}

static void StorageTask_ToggleRecording(void)
{
  if (SdLog_IsRecording() != 0U)
  {
    (void)SdLog_Stop();
  }
  else
  {
    (void)SdLog_Start();
  }
}

static void StorageTask_PollUserKey(void)
{
  static uint8_t last_raw_pressed = 0U;
  static uint8_t stable_pressed = 0U;
  static uint32_t last_change_tick = 0U;

  const uint8_t raw_pressed =
    (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == STORAGE_USER_KEY_ACTIVE_LEVEL) ? 1U : 0U;
  const uint32_t now = osKernelGetTickCount();

  if (raw_pressed != last_raw_pressed)
  {
    last_raw_pressed = raw_pressed;
    last_change_tick = now;
  }

  if (((now - last_change_tick) >= STORAGE_KEY_DEBOUNCE_MS) &&
      (raw_pressed != stable_pressed))
  {
    stable_pressed = raw_pressed;
    if (stable_pressed != 0U)
    {
      StorageTask_ToggleRecording();
    }
  }
}

void StorageTask(void *argument)
{
  (void)argument;

  osDelay(500U);
  (void)SdLog_Init();

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
        (void)SdLog_WriteFullFrame(&full->frame);
      }
      (void)DataManager_ReleaseFullFrame(full);
    }

    StorageTask_UpdateStackWatermark();
  }
}
