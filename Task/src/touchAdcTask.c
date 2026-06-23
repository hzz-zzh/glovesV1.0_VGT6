#include "touchAdcTask.h"

#include <stdint.h>
#include <stdio.h>

#include "acq_sync.h"
#include "cmsis_os2.h"
#include "data_manager.h"
#include "main.h"

#define TOUCH_ADC_DEBUG_ENABLE         (0U)
#define TOUCH_ADC_DEBUG_PRINT_PERIOD   (1U)
#define TOUCH_ADC_DEBUG_PRINT_DETAILS  (1U)
#define TOUCH_ADC_DEBUG_PRINT_STATUS   (1U)
#define TOUCH_ADC_DEBUG_ERROR_PERIOD   (50U)
#define TOUCH_ADC_DEBUG_ALLOC_PERIOD   (100U)
#define TOUCH_ADC_DEBUG_PUBLISH_PERIOD (100U)
#define TOUCH_ADC_DEBUG_TRACE_FIRST_FRAME (0U)
#define TOUCH_ADC_MUX_HOLD_TEST_ENABLE (0U)
#define TOUCH_ADC_MUX_HOLD_COL         (14U)
#define TOUCH_ADC_SYNC_WAIT_TIMEOUT_MS  (osWaitForever)
#define TOUCH_ADC_QUEUE_TIMEOUT_MS      (0U)
#define TOUCH_ADC_DMA_TIMEOUT_MS        (5U)
#define TOUCH_ADC_DMA_DONE_FLAG         (1UL << 0)
#define TOUCH_ADC_DMA_ERROR_FLAG        (1UL << 1)
#define TOUCH_ADC_CHANNEL_COUNT         (8U)
#define TOUCH_ADC_COLUMN_COUNT          (16U)
#define TOUCH_ADC_SCAN_FIRST_COLUMN     (7U)
#define TOUCH_ADC_SCAN_LAST_COLUMN      (14U)
#define TOUCH_ADC_FINGER_FIRST_COLUMN   (7U)
#define TOUCH_ADC_FINGER_LAST_COLUMN    (8U)
#define TOUCH_ADC_FINGER_FIRST_ROW      (5U)
#define TOUCH_ADC_FINGER_LAST_ROW       (14U)
#define TOUCH_ADC_PALM_FIRST_COLUMN     (9U)
#define TOUCH_ADC_PALM_LAST_COLUMN      (14U)
#define TOUCH_ADC_PALM_FIRST_ROW        (5U)
#define TOUCH_ADC_PALM_LAST_ROW         (12U)
#define TOUCH_ADC_PALM_ROWS             (8U)
#define TOUCH_ADC_ROWS_PER_FINGER       (2U)
#define TOUCH_ADC_POINTS_PER_FINGER     (4U)
#define TOUCH_ADC_FINGER_COUNT          (5U)
#define TOUCH_ADC_FINGER_BASE_COUNT     (TOUCH_ADC_FINGER_COUNT * TOUCH_ADC_POINTS_PER_FINGER)
#define TOUCH_ADC_INVALID_INDEX         (0xFFFFU)
#define TOUCH_ADC_MUX_SETTLE_NOP_COUNT  (64U)

#if GLOVE_TOUCH_COUNT != (TOUCH_ADC_FINGER_BASE_COUNT + \
    ((TOUCH_ADC_PALM_LAST_COLUMN - TOUCH_ADC_PALM_FIRST_COLUMN + 1U) * TOUCH_ADC_PALM_ROWS))
#error "GLOVE_TOUCH_COUNT must match the 68-point touch layout"
#endif

/* ROW_SEL0 high selects the first row in each analog switch pair. */
static const uint8_t s_touch_rows_sel_high[TOUCH_ADC_CHANNEL_COUNT] =
{
  0U, 1U, 4U, 5U, 8U, 9U, 14U, 12U
};

/* ROW_SEL0 low selects the second row in each analog switch pair. */
static const uint8_t s_touch_rows_sel_low[TOUCH_ADC_CHANNEL_COUNT] =
{
  3U, 2U, 7U, 6U, 11U, 10U, 13U, 15U
};

static uint32_t s_touch_adc_dma_storage[TOUCH_ADC_CHANNEL_COUNT / 2U];
static osThreadId_t s_touch_adc_task_id = NULL;
static uint8_t s_touch_adc_trace_frame = 0U;
static uint8_t s_touch_adc_trace_dma_once = 0U;
static uint8_t s_touch_adc_debug_print_once = 1U;

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

#if (TOUCH_ADC_DEBUG_ENABLE != 0U)
static const char *const s_touch_debug_finger_tags[TOUCH_ADC_FINGER_COUNT] =
{
  "A", "B", "C", "D", "E"
};

static void TouchAdcTask_PrintValues(const GloveTouchSensorBlock_t *block,
                                     uint32_t base,
                                     uint32_t count)
{
  uint32_t index;

  for (index = 0U; index < count; index++)
  {
    printf("%u", (unsigned int)block->data.touch[base + index].value);
    if ((index + 1U) < count)
    {
      printf(",");
    }
  }
  printf("\r\n");
}

static void TouchAdcTask_PrintFingerValues(uint32_t seq,
                                           const GloveTouchSensorBlock_t *block,
                                           uint32_t finger)
{
  uint32_t base;

  if ((block == NULL) || (finger >= TOUCH_ADC_FINGER_COUNT))
  {
    return;
  }

  base = finger * TOUCH_ADC_POINTS_PER_FINGER;
  printf("[TOUCH_PART] seq=%lu %s=",
         (unsigned long)seq,
         s_touch_debug_finger_tags[finger]);
  TouchAdcTask_PrintValues(block, base, TOUCH_ADC_POINTS_PER_FINGER);
}

static void TouchAdcTask_PrintPalmColumnValues(uint32_t seq,
                                               const GloveTouchSensorBlock_t *block,
                                               uint32_t palm_column)
{
  uint32_t palm_point;
  uint32_t base;

  if (block == NULL)
  {
    return;
  }

  palm_point = palm_column * TOUCH_ADC_PALM_ROWS;
  base = TOUCH_ADC_FINGER_BASE_COUNT + palm_point;
  printf("[TOUCH_PART] seq=%lu F%lu_F%lu=",
         (unsigned long)seq,
         (unsigned long)palm_point,
         (unsigned long)(palm_point + TOUCH_ADC_PALM_ROWS - 1U));
  TouchAdcTask_PrintValues(block, base, TOUCH_ADC_PALM_ROWS);
}
#endif

static void TouchAdcTask_Notify(uint32_t flags)
{
  if (s_touch_adc_task_id != NULL)
  {
    (void)osThreadFlagsSet(s_touch_adc_task_id, flags);
  }
}

static void TouchAdcTask_Trace(const char *stage, uint32_t a, uint32_t b)
{
#if ((TOUCH_ADC_DEBUG_ENABLE != 0U) && (TOUCH_ADC_DEBUG_TRACE_FIRST_FRAME != 0U))
  if ((s_touch_adc_trace_frame != 0U) && (stage != NULL))
  {
    printf("[TOUCH_TRACE] %s a=%lu b=%lu\r\n",
           stage,
           (unsigned long)a,
           (unsigned long)b);
  }
#else
  (void)stage;
  (void)a;
  (void)b;
#endif
}

static void TouchAdcTask_PrintSamples(uint32_t seq, const GloveTouchSensorBlock_t *block)
{
#if (TOUCH_ADC_DEBUG_ENABLE != 0U)
  if ((block != NULL) &&
      ((s_touch_adc_debug_print_once != 0U) ||
       ((seq % TOUCH_ADC_DEBUG_PRINT_PERIOD) == 0U)))
  {
    uint32_t finger;
    uint32_t index;
    uint32_t finger_sum[TOUCH_ADC_FINGER_COUNT] = {0U};
    uint32_t palm_sum = 0U;
    uint32_t palm_count = GLOVE_TOUCH_COUNT - TOUCH_ADC_FINGER_BASE_COUNT;
    uint32_t palm_column_count = TOUCH_ADC_PALM_LAST_COLUMN - TOUCH_ADC_PALM_FIRST_COLUMN + 1U;

    s_touch_adc_debug_print_once = 0U;

    for (finger = 0U; finger < TOUCH_ADC_FINGER_COUNT; finger++)
    {
      uint32_t base = finger * TOUCH_ADC_POINTS_PER_FINGER;
      for (index = 0U; index < TOUCH_ADC_POINTS_PER_FINGER; index++)
      {
        finger_sum[finger] += block->data.touch[base + index].value;
      }
    }

    for (index = TOUCH_ADC_FINGER_BASE_COUNT; index < GLOVE_TOUCH_COUNT; index++)
    {
      palm_sum += block->data.touch[index].value;
    }

    printf("[TOUCH] seq=%lu count=%lu A=%lu B=%lu C=%lu D=%lu E=%lu F=%lu\r\n",
           (unsigned long)seq,
           (unsigned long)GLOVE_TOUCH_COUNT,
           (unsigned long)(finger_sum[0] / TOUCH_ADC_POINTS_PER_FINGER),
           (unsigned long)(finger_sum[1] / TOUCH_ADC_POINTS_PER_FINGER),
           (unsigned long)(finger_sum[2] / TOUCH_ADC_POINTS_PER_FINGER),
           (unsigned long)(finger_sum[3] / TOUCH_ADC_POINTS_PER_FINGER),
           (unsigned long)(finger_sum[4] / TOUCH_ADC_POINTS_PER_FINGER),
           (unsigned long)(palm_sum / palm_count));

#if (TOUCH_ADC_DEBUG_PRINT_DETAILS != 0U)
    for (finger = 0U; finger < TOUCH_ADC_FINGER_COUNT; finger++)
    {
      TouchAdcTask_PrintFingerValues(seq, block, finger);
    }

    for (index = 0U; index < palm_column_count; index++)
    {
      TouchAdcTask_PrintPalmColumnValues(seq, block, index);
    }
#endif
  }
#else
  (void)seq;
  (void)block;
#endif
}

static void TouchAdcTask_PrintStartup(void)
{
#if ((TOUCH_ADC_DEBUG_ENABLE != 0U) && (TOUCH_ADC_DEBUG_PRINT_STATUS != 0U))
  printf("[TOUCH] task_start id=0x%08lX trigger=tim2_sync channels=%lu count=%lu\r\n",
         (unsigned long)(uintptr_t)s_touch_adc_task_id,
         (unsigned long)TOUCH_ADC_CHANNEL_COUNT,
         (unsigned long)GLOVE_TOUCH_COUNT);
#endif
}

static void TouchAdcTask_PrintError(uint32_t seq,
                                    GloveStatus_t status,
                                    uint32_t error_count)
{
#if ((TOUCH_ADC_DEBUG_ENABLE != 0U) && (TOUCH_ADC_DEBUG_PRINT_STATUS != 0U))
  if ((error_count == 1U) ||
      ((error_count % TOUCH_ADC_DEBUG_ERROR_PERIOD) == 0U))
  {
    printf("[TOUCH] capture_error seq=%lu status=%u count=%lu\r\n",
           (unsigned long)seq,
           (unsigned int)status,
           (unsigned long)error_count);
  }
#else
  (void)seq;
  (void)status;
  (void)error_count;
#endif
}

static void TouchAdcTask_PrintAllocError(uint32_t alloc_fail_count)
{
#if ((TOUCH_ADC_DEBUG_ENABLE != 0U) && (TOUCH_ADC_DEBUG_PRINT_STATUS != 0U))
  if ((alloc_fail_count == 1U) ||
      ((alloc_fail_count % TOUCH_ADC_DEBUG_ALLOC_PERIOD) == 0U))
  {
    printf("[TOUCH] alloc_touch_failed count=%lu\r\n",
           (unsigned long)alloc_fail_count);
  }
#else
  (void)alloc_fail_count;
#endif
}

static void TouchAdcTask_PrintPublishError(uint32_t seq,
                                           GloveStatus_t status,
                                           uint32_t publish_fail_count)
{
#if ((TOUCH_ADC_DEBUG_ENABLE != 0U) && (TOUCH_ADC_DEBUG_PRINT_STATUS != 0U))
  if (publish_fail_count == 1U)
  {
    printf("[TOUCH] publish_failed seq=%lu status=%u count=%lu\r\n",
           (unsigned long)seq,
           (unsigned int)status,
           (unsigned long)publish_fail_count);
  }
#else
  (void)seq;
  (void)status;
  (void)publish_fail_count;
#endif
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
  /* The 74AC238 outputs are wired Y7..Y0 to SENSOR_COL base..base+7. */
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

#if (TOUCH_ADC_MUX_HOLD_TEST_ENABLE != 0U)
static void TouchAdcTask_ForceMuxGpioOutput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = TOUCH_COL_SEL2_Pin;
  HAL_GPIO_Init(TOUCH_COL_SEL2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TOUCH_COL_SEL0_Pin | TOUCH_COL_SEL1_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TOUCH_ROW_SEL0_Pin | TOUCH_COL_SEL3_Pin | TOUCH_COL_SEL4_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void TouchAdcTask_PrintMuxHoldReadback(uint8_t col, uint8_t decoded_addr)
{
  printf("[TOUCH_MUX_TEST] hold_col=%u addr=%u exp=%u%u%u%u%u read=%u%u%u%u%u row=%u\r\n",
         (unsigned int)col,
         (unsigned int)decoded_addr,
         (unsigned int)((decoded_addr & 0x01U) != 0U),
         (unsigned int)((decoded_addr & 0x02U) != 0U),
         (unsigned int)((decoded_addr & 0x04U) != 0U),
         (unsigned int)(col >= 8U),
         (unsigned int)(col < 8U),
         (unsigned int)(HAL_GPIO_ReadPin(TOUCH_COL_SEL0_GPIO_Port, TOUCH_COL_SEL0_Pin) == GPIO_PIN_SET),
         (unsigned int)(HAL_GPIO_ReadPin(TOUCH_COL_SEL1_GPIO_Port, TOUCH_COL_SEL1_Pin) == GPIO_PIN_SET),
         (unsigned int)(HAL_GPIO_ReadPin(TOUCH_COL_SEL2_GPIO_Port, TOUCH_COL_SEL2_Pin) == GPIO_PIN_SET),
         (unsigned int)(HAL_GPIO_ReadPin(TOUCH_COL_SEL3_GPIO_Port, TOUCH_COL_SEL3_Pin) == GPIO_PIN_SET),
         (unsigned int)(HAL_GPIO_ReadPin(TOUCH_COL_SEL4_GPIO_Port, TOUCH_COL_SEL4_Pin) == GPIO_PIN_SET),
         (unsigned int)(HAL_GPIO_ReadPin(TOUCH_ROW_SEL0_GPIO_Port, TOUCH_ROW_SEL0_Pin) == GPIO_PIN_SET));
}

static void TouchAdcTask_RunMuxHoldTest(void)
{
  uint8_t col = (uint8_t)(TOUCH_ADC_MUX_HOLD_COL % TOUCH_ADC_COLUMN_COUNT);
  uint8_t decoded_addr = (uint8_t)(7U - (col & 0x07U));

  TouchAdcTask_ForceMuxGpioOutput();
  TouchAdcTask_SelectColumn(col);
  TouchAdcTask_SelectRowGroup(GPIO_PIN_SET);

  for (;;)
  {
    TouchAdcTask_SelectColumn(col);
    TouchAdcTask_SelectRowGroup(GPIO_PIN_SET);
    TouchAdcTask_PrintMuxHoldReadback(col, decoded_addr);
    osDelay(1000U);
  }
}
#endif

/* touch[] follows layout labels: A0..E3, then F0..F47. */
static uint16_t TouchAdcTask_MapTouchIndex(uint8_t row, uint8_t col)
{
  uint8_t row_from_top;
  uint8_t finger_index;
  uint8_t point_in_finger;
  uint16_t palm_index;

  if ((col >= TOUCH_ADC_FINGER_FIRST_COLUMN) &&
      (col <= TOUCH_ADC_FINGER_LAST_COLUMN) &&
      (row >= TOUCH_ADC_FINGER_FIRST_ROW) &&
      (row <= TOUCH_ADC_FINGER_LAST_ROW))
  {
    row_from_top = (uint8_t)(TOUCH_ADC_FINGER_LAST_ROW - row);
    finger_index = (uint8_t)(row_from_top / TOUCH_ADC_ROWS_PER_FINGER);
    point_in_finger = (uint8_t)(((col - TOUCH_ADC_FINGER_FIRST_COLUMN) * TOUCH_ADC_ROWS_PER_FINGER) +
                                (row_from_top % TOUCH_ADC_ROWS_PER_FINGER));
    return (uint16_t)((finger_index * TOUCH_ADC_POINTS_PER_FINGER) + point_in_finger);
  }

  if ((col >= TOUCH_ADC_PALM_FIRST_COLUMN) &&
      (col <= TOUCH_ADC_PALM_LAST_COLUMN) &&
      (row >= TOUCH_ADC_PALM_FIRST_ROW) &&
      (row <= TOUCH_ADC_PALM_LAST_ROW))
  {
    palm_index = (uint16_t)(((uint16_t)(col - TOUCH_ADC_PALM_FIRST_COLUMN) *
                             TOUCH_ADC_PALM_ROWS) +
                            (uint16_t)(TOUCH_ADC_PALM_LAST_ROW - row));
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
  HAL_StatusTypeDef hal_status;
  uint8_t trace_dma = s_touch_adc_trace_dma_once;

  if (samples == NULL)
  {
    return GLOVE_STATUS_INVALID_PARAM;
  }

  *samples = NULL;
  if (trace_dma != 0U)
  {
    TouchAdcTask_Trace("dma_clear_flags", 0U, 0U);
  }
  (void)osThreadFlagsClear(TOUCH_ADC_DMA_DONE_FLAG | TOUCH_ADC_DMA_ERROR_FLAG);

  if (trace_dma != 0U)
  {
    TouchAdcTask_Trace("adc_start_dma_begin", 0U, HAL_ADC_GetState(&hadc1));
  }

  hal_status = HAL_ADC_Start_DMA(&hadc1,
                                 s_touch_adc_dma_storage,
                                 TOUCH_ADC_CHANNEL_COUNT);
  if (trace_dma != 0U)
  {
    TouchAdcTask_Trace("adc_start_dma_done",
                       (uint32_t)hal_status,
                       HAL_ADC_GetState(&hadc1));
  }

  if (hal_status != HAL_OK)
  {
    s_touch_adc_trace_dma_once = 0U;
    return GLOVE_STATUS_ERROR;
  }

  if (trace_dma != 0U)
  {
    TouchAdcTask_Trace("dma_wait_begin", 0U, TOUCH_ADC_DMA_TIMEOUT_MS);
  }

  flags = osThreadFlagsWait(TOUCH_ADC_DMA_DONE_FLAG | TOUCH_ADC_DMA_ERROR_FLAG,
                            osFlagsWaitAny,
                            TouchAdcTask_MsToTicks(TOUCH_ADC_DMA_TIMEOUT_MS));

  if (trace_dma != 0U)
  {
    TouchAdcTask_Trace("dma_wait_done", flags, HAL_ADC_GetState(&hadc1));
  }

  (void)HAL_ADC_Stop_DMA(&hadc1);

  if (trace_dma != 0U)
  {
    TouchAdcTask_Trace("adc_stop_dma_done", 0U, HAL_ADC_GetState(&hadc1));
    s_touch_adc_trace_dma_once = 0U;
  }

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
  TouchAdcTask_Trace("row_selected", col, (uint32_t)row_sel);

  status = TouchAdcTask_ReadAdcDma(&samples);
  TouchAdcTask_Trace("row_adc_done", col, (uint32_t)status);
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

static GloveStatus_t TouchAdcTask_CaptureFrame(GloveTouchSensorBlock_t *block,
                                               const AcqSyncSnapshot_t *sync)
{
  uint32_t index;
  uint32_t seq;
  uint8_t col;
  GloveStatus_t status;

  if ((block == NULL) || (sync == NULL) || (sync->valid == 0U))
  {
    return GLOVE_STATUS_INVALID_PARAM;
  }

  seq = sync->seq;

#if (TOUCH_ADC_DEBUG_TRACE_FIRST_FRAME != 0U)
  s_touch_adc_trace_frame = (seq == 0U) ? 1U : 0U;
  s_touch_adc_trace_dma_once = (seq == 0U) ? 1U : 0U;
#endif
  TouchAdcTask_Trace("frame_begin", seq, GLOVE_TOUCH_COUNT);

  block->data.sensor_seq = sync->seq;
  block->data.timestamp_us = sync->timestamp_us;
  block->data.valid_flags = GLOVE_FRAME_FLAG_NONE;

  for (index = 0U; index < GLOVE_TOUCH_COUNT; index++)
  {
    block->data.touch[index].value = 0U;
    block->data.touch[index].baseline = 0U;
  }
  TouchAdcTask_Trace("frame_clear_done", seq, 0U);

  for (col = TOUCH_ADC_SCAN_FIRST_COLUMN; col <= TOUCH_ADC_SCAN_LAST_COLUMN; col++)
  {
    TouchAdcTask_Trace("col_begin", col, 0U);
    TouchAdcTask_SelectColumn(col);
    TouchAdcTask_Trace("col_selected", col, 0U);

    TouchAdcTask_Trace("row_high_begin", col, 0U);
    status = TouchAdcTask_CaptureRowGroup(block,
                                          col,
                                          GPIO_PIN_SET,
                                          s_touch_rows_sel_high);
    TouchAdcTask_Trace("row_high_done", col, (uint32_t)status);
    if (status != GLOVE_STATUS_OK)
    {
      TouchAdcTask_DisableColumns();
      return status;
    }

    TouchAdcTask_Trace("row_low_begin", col, 0U);
    status = TouchAdcTask_CaptureRowGroup(block,
                                          col,
                                          GPIO_PIN_RESET,
                                          s_touch_rows_sel_low);
    TouchAdcTask_Trace("row_low_done", col, (uint32_t)status);
    if (status != GLOVE_STATUS_OK)
    {
      TouchAdcTask_DisableColumns();
      return status;
    }
  }

  TouchAdcTask_DisableColumns();
  block->data.valid_flags = GLOVE_FRAME_FLAG_TOUCH_VALID;
  TouchAdcTask_Trace("frame_done", seq, 0U);
  s_touch_adc_trace_frame = 0U;
  return GLOVE_STATUS_OK;
}

void TouchAdcTask(void *argument)
{
  uint32_t seq = 0U;
  uint32_t error_count = 0U;
  uint32_t alloc_fail_count = 0U;
  uint32_t publish_fail_count = 0U;
  AcqSyncSnapshot_t sync;
  GloveStatus_t status;
  GloveStatus_t publish_status;

  (void)argument;

  s_touch_adc_task_id = osThreadGetId();
  AcqSync_RegisterTouchTask(s_touch_adc_task_id);
  TouchAdcTask_PrintStartup();
#if (TOUCH_ADC_MUX_HOLD_TEST_ENABLE != 0U)
  TouchAdcTask_RunMuxHoldTest();
#endif

  for (;;)
  {
    GloveTouchSensorBlock_t *touch;

    if (AcqSync_WaitForTouchSync(&sync, TOUCH_ADC_SYNC_WAIT_TIMEOUT_MS) != osOK)
    {
      error_count++;
      TouchAdcTask_PrintError(seq, GLOVE_STATUS_TIMEOUT, error_count);
      continue;
    }

    seq = sync.seq;
    touch = DataManager_AllocTouchSensor();

    if (touch != NULL)
    {
      alloc_fail_count = 0U;
      status = TouchAdcTask_CaptureFrame(touch, &sync);
      if (status == GLOVE_STATUS_OK)
      {
        error_count = 0U;
        TouchAdcTask_PrintSamples(seq, touch);
        publish_status = DataManager_PublishTouchSensor(touch, TOUCH_ADC_QUEUE_TIMEOUT_MS);
        if (publish_status != GLOVE_STATUS_OK)
        {
          publish_fail_count++;
          TouchAdcTask_PrintPublishError(seq, publish_status, publish_fail_count);
        }
        else
        {
          publish_fail_count = 0U;
        }
      }
      else
      {
        error_count++;
        TouchAdcTask_PrintError(seq, status, error_count);
        (void)DataManager_ReleaseTouchSensor(touch);
      }
    }
    else
    {
      alloc_fail_count++;
      TouchAdcTask_PrintAllocError(alloc_fail_count);
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
