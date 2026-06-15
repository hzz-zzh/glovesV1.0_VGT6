#include "touchAdcTask.h"

#include "cmsis_os2.h"
#include "data_manager.h"
#include "main.h"

#define TOUCH_ADC_PLACEHOLDER_PERIOD_MS (10U)
#define TOUCH_ADC_PLACEHOLDER_QUEUE_TIMEOUT_MS (0U)

static GloveTimestampUs_t TouchAdcTask_GetTimeUs(void)
{
  return (GloveTimestampUs_t)HAL_GetTick() * 1000ULL;
}

static void TouchAdcTask_FillPlaceholder(GloveTouchSensorBlock_t *block, uint32_t seq)
{
  uint32_t index;

  if (block == NULL)
  {
    return;
  }

  block->data.sensor_seq = seq;
  block->data.timestamp_us = TouchAdcTask_GetTimeUs();
  block->data.valid_flags = GLOVE_FRAME_FLAG_TOUCH_VALID;

  for (index = 0U; index < GLOVE_TOUCH_COUNT; index++)
  {
    block->data.touch[index].value = 0U;
    block->data.touch[index].baseline = 0U;
  }
}

void TouchAdcTask(void *argument)
{
  uint32_t seq = 0U;

  (void)argument;

  for (;;)
  {
    GloveTouchSensorBlock_t *touch = DataManager_AllocTouchSensor();
    if (touch != NULL)
    {
      TouchAdcTask_FillPlaceholder(touch, seq);
      (void)DataManager_PublishTouchSensor(touch, TOUCH_ADC_PLACEHOLDER_QUEUE_TIMEOUT_MS);
      seq++;
    }

    osDelay(TOUCH_ADC_PLACEHOLDER_PERIOD_MS);
  }
}
