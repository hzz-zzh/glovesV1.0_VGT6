#include "touchAdcTask.h"

#include "cmsis_os2.h"
#include "data_manager.h"
#include "main.h"

#define TOUCH_ADC_PERIOD_MS             (10U)
#define TOUCH_ADC_QUEUE_TIMEOUT_MS      (0U)
#define TOUCH_ADC_DMA_TIMEOUT_MS        (5U)
#define TOUCH_ADC_DMA_DONE_FLAG         (1UL << 0)
#define TOUCH_ADC_DMA_ERROR_FLAG        (1UL << 1)
#define TOUCH_ADC_CHANNEL_COUNT         (8U)
#define TOUCH_ADC_COLUMN_COUNT          (16U)
#define TOUCH_ADC_FINGER_COLUMN_COUNT   (4U)
#define TOUCH_ADC_PALM_FIRST_COLUMN     (4U)
#define TOUCH_ADC_PALM_LAST_COLUMN      (9U)
#define TOUCH_ADC_FINGER_BASE_COUNT     (60U)
#define TOUCH_ADC_PALM_ROWS             (12U)
#define TOUCH_ADC_ROWS_PER_FINGER       (3U)
#define TOUCH_ADC_POINTS_PER_FINGER     (12U)
#define TOUCH_ADC_INVALID_INDEX         (0xFFFFU)
#define TOUCH_ADC_MUX_SETTLE_NOP_COUNT  (64U)

static const uint8_t s_touch_rows_sel_high[TOUCH_ADC_CHANNEL_COUNT] =
{
  0U, 1U, 4U, 5U, 8U, 9U, 14U, 12U
};

static const uint8_t s_touch_rows_sel_low[TOUCH_ADC_CHANNEL_COUNT] =
{
  3U, 2U, 7U, 6U, 11U, 10U, 13U, 15U
};

static uint32_t s_touch_adc_dma_storage[TOUCH_ADC_CHANNEL_COUNT / 2U];
static osThreadId_t s_touch_adc_task_id = NULL;

static GloveTimestampUs_t TouchAdcTask_GetTimeUs(void)
{
  return (GloveTimestampUs_t)HAL_GetTick() * 1000ULL;
}

static uint32_t TouchAdcTask_MsToTicks(uint32_t timeout_ms)
{
  uint64_t ticks;

  if (timeout_ms == osWaitForever)
  {
    return osWaitForever;
  }

  ticks = ((uint64_t)timeout_ms * (uint64_t)osKernelGetTickFreq() + 999ULL) / 1000ULL;
  if ((timeout_ms > 0U) && (ticks == 0ULL))
  {
    ticks = 1ULL;
  }

  return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

static void TouchAdcTask_Notify(uint32_t flags)
{
  if (s_touch_adc_task_id != NULL)
  {
    (void)osThreadFlagsSet(s_touch_adc_task_id, flags);
  }
}

static void TouchAdcTask_MuxSettle(void)
{
  volatile uint32_t delay;

  for (delay = 0U; delay < TOUCH_ADC_MUX_SETTLE_NOP_COUNT; delay++)
  {
    __NOP();
  }
}

static void TouchAdcTask_DisableColumns(void)
{
  HAL_GPIO_WritePin(TOUCH_COL_SEL3_GPIO_Port, TOUCH_COL_SEL3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOUCH_COL_SEL4_GPIO_Port, TOUCH_COL_SEL4_Pin, GPIO_PIN_RESET);
}

static void TouchAdcTask_SelectColumn(uint8_t col)
{
  uint8_t decoded_addr = (uint8_t)(7U - (col & 0x07U));

  TouchAdcTask_DisableColumns();

  HAL_GPIO_WritePin(TOUCH_COL_SEL0_GPIO_Port,
                    TOUCH_COL_SEL0_Pin,
                    ((decoded_addr & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOUCH_COL_SEL1_GPIO_Port,
                    TOUCH_COL_SEL1_Pin,
                    ((decoded_addr & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOUCH_COL_SEL2_GPIO_Port,
                    TOUCH_COL_SEL2_Pin,
                    ((decoded_addr & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if (col < 8U)
  {
    HAL_GPIO_WritePin(TOUCH_COL_SEL4_GPIO_Port, TOUCH_COL_SEL4_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(TOUCH_COL_SEL3_GPIO_Port, TOUCH_COL_SEL3_Pin, GPIO_PIN_SET);
  }

  TouchAdcTask_MuxSettle();
}

static void TouchAdcTask_SelectRowGroup(GPIO_PinState row_sel)
{
  HAL_GPIO_WritePin(TOUCH_ROW_SEL0_GPIO_Port, TOUCH_ROW_SEL0_Pin, row_sel);
  TouchAdcTask_MuxSettle();
}

static uint16_t TouchAdcTask_MapTouchIndex(uint8_t row, uint8_t col)
{
  uint8_t row_from_top;
  uint8_t finger_index;
  uint8_t point_in_finger;
  uint16_t palm_index;

  if (col < TOUCH_ADC_FINGER_COLUMN_COUNT)
  {
    if (row > 14U)
    {
      return TOUCH_ADC_INVALID_INDEX;
    }

    row_from_top = (uint8_t)(14U - row);
    finger_index = (uint8_t)(row_from_top / TOUCH_ADC_ROWS_PER_FINGER);
    point_in_finger = (uint8_t)((col * TOUCH_ADC_ROWS_PER_FINGER) +
                                (row_from_top % TOUCH_ADC_ROWS_PER_FINGER));
    return (uint16_t)((finger_index * TOUCH_ADC_POINTS_PER_FINGER) + point_in_finger);
  }

  if ((col >= TOUCH_ADC_PALM_FIRST_COLUMN) &&
      (col <= TOUCH_ADC_PALM_LAST_COLUMN) &&
      (row < TOUCH_ADC_PALM_ROWS))
  {
    palm_index = (uint16_t)(((uint16_t)(col - TOUCH_ADC_PALM_FIRST_COLUMN) *
                             TOUCH_ADC_PALM_ROWS) +
                            (uint16_t)(11U - row));
    return (uint16_t)(TOUCH_ADC_FINGER_BASE_COUNT + palm_index);
  }

  return TOUCH_ADC_INVALID_INDEX;
}

static void TouchAdcTask_StoreSample(GloveTouchSensorBlock_t *block,
                                     uint8_t row,
                                     uint8_t col,
                                     uint16_t adc_value)
{
  uint16_t touch_index;

  if (block == NULL)
  {
    return;
  }

  touch_index = TouchAdcTask_MapTouchIndex(row, col);
  if ((touch_index != TOUCH_ADC_INVALID_INDEX) && (touch_index < GLOVE_TOUCH_COUNT))
  {
    block->data.touch[touch_index].value = adc_value;
    block->data.touch[touch_index].baseline = 0U;
  }
}

static GloveStatus_t TouchAdcTask_ReadAdcDma(const uint16_t **samples)
{
  uint32_t flags;

  if (samples == NULL)
  {
    return GLOVE_STATUS_INVALID_PARAM;
  }

  *samples = NULL;
  (void)osThreadFlagsClear(TOUCH_ADC_DMA_DONE_FLAG | TOUCH_ADC_DMA_ERROR_FLAG);

  if (HAL_ADC_Start_DMA(&hadc1,
                        s_touch_adc_dma_storage,
                        TOUCH_ADC_CHANNEL_COUNT) != HAL_OK)
  {
    return GLOVE_STATUS_ERROR;
  }

  flags = osThreadFlagsWait(TOUCH_ADC_DMA_DONE_FLAG | TOUCH_ADC_DMA_ERROR_FLAG,
                            osFlagsWaitAny,
                            TouchAdcTask_MsToTicks(TOUCH_ADC_DMA_TIMEOUT_MS));

  (void)HAL_ADC_Stop_DMA(&hadc1);

  if ((flags & osFlagsError) != 0U)
  {
    return GLOVE_STATUS_TIMEOUT;
  }

  if ((flags & TOUCH_ADC_DMA_ERROR_FLAG) != 0U)
  {
    return GLOVE_STATUS_ERROR;
  }

  *samples = (const uint16_t *)s_touch_adc_dma_storage;
  return GLOVE_STATUS_OK;
}

static GloveStatus_t TouchAdcTask_CaptureRowGroup(GloveTouchSensorBlock_t *block,
                                                  uint8_t col,
                                                  GPIO_PinState row_sel,
                                                  const uint8_t *rows)
{
  const uint16_t *samples;
  GloveStatus_t status;
  uint32_t lane;

  if ((block == NULL) || (rows == NULL))
  {
    return GLOVE_STATUS_INVALID_PARAM;
  }

  TouchAdcTask_SelectRowGroup(row_sel);

  status = TouchAdcTask_ReadAdcDma(&samples);
  if (status != GLOVE_STATUS_OK)
  {
    return status;
  }

  for (lane = 0U; lane < TOUCH_ADC_CHANNEL_COUNT; lane++)
  {
    TouchAdcTask_StoreSample(block, rows[lane], col, samples[lane]);
  }

  return GLOVE_STATUS_OK;
}

static GloveStatus_t TouchAdcTask_CaptureFrame(GloveTouchSensorBlock_t *block, uint32_t seq)
{
  uint32_t index;
  uint8_t col;
  GloveStatus_t status;

  if (block == NULL)
  {
    return GLOVE_STATUS_INVALID_PARAM;
  }

  block->data.sensor_seq = seq;
  block->data.timestamp_us = TouchAdcTask_GetTimeUs();
  block->data.valid_flags = GLOVE_FRAME_FLAG_NONE;

  for (index = 0U; index < GLOVE_TOUCH_COUNT; index++)
  {
    block->data.touch[index].value = 0U;
    block->data.touch[index].baseline = 0U;
  }

  for (col = 0U; col < TOUCH_ADC_COLUMN_COUNT; col++)
  {
    TouchAdcTask_SelectColumn(col);

    status = TouchAdcTask_CaptureRowGroup(block,
                                          col,
                                          GPIO_PIN_SET,
                                          s_touch_rows_sel_high);
    if (status != GLOVE_STATUS_OK)
    {
      TouchAdcTask_DisableColumns();
      return status;
    }

    status = TouchAdcTask_CaptureRowGroup(block,
                                          col,
                                          GPIO_PIN_RESET,
                                          s_touch_rows_sel_low);
    if (status != GLOVE_STATUS_OK)
    {
      TouchAdcTask_DisableColumns();
      return status;
    }
  }

  TouchAdcTask_DisableColumns();
  block->data.valid_flags = GLOVE_FRAME_FLAG_TOUCH_VALID;
  return GLOVE_STATUS_OK;
}

void TouchAdcTask(void *argument)
{
  uint32_t seq = 0U;
  uint32_t period_ticks;
  uint32_t next_wake_tick;

  (void)argument;

  s_touch_adc_task_id = osThreadGetId();
  period_ticks = TouchAdcTask_MsToTicks(TOUCH_ADC_PERIOD_MS);
  next_wake_tick = osKernelGetTickCount();

  for (;;)
  {
    GloveTouchSensorBlock_t *touch = DataManager_AllocTouchSensor();
    if (touch != NULL)
    {
      if (TouchAdcTask_CaptureFrame(touch, seq) == GLOVE_STATUS_OK)
      {
        (void)DataManager_PublishTouchSensor(touch, TOUCH_ADC_QUEUE_TIMEOUT_MS);
        seq++;
      }
      else
      {
        (void)DataManager_ReleaseTouchSensor(touch);
      }
    }

    next_wake_tick += period_ticks;
    if (osDelayUntil(next_wake_tick) != osOK)
    {
      next_wake_tick = osKernelGetTickCount();
      osDelay(period_ticks);
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    TouchAdcTask_Notify(TOUCH_ADC_DMA_DONE_FLAG);
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    TouchAdcTask_Notify(TOUCH_ADC_DMA_ERROR_FLAG);
  }
}
