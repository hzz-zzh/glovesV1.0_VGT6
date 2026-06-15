#include "RS485_uasrt.h"
#include "modbus_frame.h"
#include "rs485Task.h"
#include "main.h"
#include <string.h>

static uint8_t rs485_rx_dma_buffer[RS485_RX_BUFFER_SIZE];

static uint8_t rs485_rx_work_buffer[RS485_RX_BUFFER_SIZE];
static uint8_t rs485_tx_buffer[RS485_TX_BUFFER_SIZE];
static uint8_t rs485_response_buffer[RS485_TX_BUFFER_SIZE];
static volatile uint16_t rs485_rx_frame_size = 0U;
static volatile uint8_t rs485_rx_frame_ready = 0U;
static volatile uint8_t rs485_tx_busy = 0U;
static volatile uint32_t rs485_init_calls = 0U;
static volatile uint32_t rs485_rx_start_ok = 0U;
static volatile uint32_t rs485_tx_requests = 0U;
static volatile uint32_t rs485_tx_from_init = 0U;
static volatile uint32_t rs485_tx_from_echo_task = 0U;
static volatile uint32_t rs485_tx_dma_irq = 0U;
static volatile uint32_t rs485_tx_cplt_callback = 0U;
static volatile uint32_t rs485_rx_events = 0U;
static volatile uint32_t rs485_rx_bytes = 0U;
static volatile uint32_t rs485_rx_overwrite = 0U;
static volatile uint32_t rs485_rx_taken = 0U;
static volatile uint32_t rs485_modbus_response_ready = 0U;
static volatile uint32_t rs485_modbus_no_response = 0U;
static volatile uint32_t rs485_modbus_frame_error = 0U;
static volatile uint32_t rs485_tx_send_fail = 0U;
static volatile uint32_t rs485_tx_done = 0U;
static volatile uint32_t rs485_errors = 0U;

static void RS485_SetReceiveMode(void)
{
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, RS485_RX_GPIO_LEVEL);
}

static void RS485_SetTransmitMode(void)
{
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, RS485_TX_GPIO_LEVEL);
}

static void RS485_DirectionSwitchDelay(void)
{
  for (volatile uint32_t delay = 0U; delay < 200U; delay++)
  {
    __NOP();
  }
}

HAL_StatusTypeDef RS485_StartReceive(void)
{
  HAL_StatusTypeDef status;

  RS485_SetReceiveMode();

  /* IDLE or full-buffer events enter HAL_UARTEx_RxEventCallback(). */
  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rs485_rx_dma_buffer, RS485_RX_BUFFER_SIZE);
  if (status == HAL_OK)
  {
    rs485_rx_start_ok++;
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }

  return status;
}

HAL_StatusTypeDef RS485_Init(void)
{
  HAL_StatusTypeDef status;

  rs485_init_calls++;

  status = RS485_StartReceive();

  return status;
}

HAL_StatusTypeDef RS485_SendDMA(const uint8_t *data, uint16_t size)
{
  HAL_StatusTypeDef status;                         

  if ((data == NULL) || (size == 0U) || (size > RS485_TX_BUFFER_SIZE))
  {
    return HAL_ERROR;
  }

  rs485_tx_requests++;

  __disable_irq();
  if (rs485_tx_busy != 0U)
  {
    __enable_irq();
    return HAL_BUSY;
  }
  rs485_tx_busy = 1U;
  __enable_irq();

  memcpy(rs485_tx_buffer, data, size);

  /* Half-duplex RS485: stop RX, enable DE, then launch TX DMA. */
  (void)HAL_UART_AbortReceive(&huart1);
  __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_TCF);
  RS485_SetTransmitMode();
  RS485_DirectionSwitchDelay();

  status = HAL_UART_Transmit_DMA(&huart1, rs485_tx_buffer, size);
  if (status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmatx, DMA_IT_HT);
  }

  if (status != HAL_OK)
  {
    rs485_errors++;
    RS485_SetReceiveMode();
    rs485_tx_busy = 0U;
    (void)RS485_StartReceive();
  }

  return status;
}

HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t size)
{
  return RS485_SendDMA(data, size);
}

uint8_t RS485_TakeRxFrame(uint8_t *data, uint16_t *size, uint16_t max_size)
{
  uint16_t frame_size;
  uint8_t has_frame = 0U;

  if ((data == NULL) || (size == NULL))
  {
    return 0U;
  }

  __disable_irq();
  if (rs485_rx_frame_ready == 0U)
  {
    __enable_irq();
    return 0U;
  }

  frame_size = rs485_rx_frame_size;
  if (frame_size > max_size)
  {
    frame_size = max_size;
  }

  /* The RS485 task owns frame data copying; the RX callback only marks ready. */
  memcpy(data, rs485_rx_dma_buffer, frame_size);
  rs485_rx_frame_ready = 0U;
  rs485_rx_frame_size = 0U;
  has_frame = 1U;
  __enable_irq();

  *size = frame_size;
  if (has_frame != 0U)
  {
    rs485_rx_taken++;
    (void)RS485_StartReceive();
  }

  return has_frame;
}

uint8_t RS485_IsTxBusy(void)
{
  return rs485_tx_busy;
}

void RS485_OnTxDmaIrq(void)
{
  /* DMA complete is earlier than UART TC; do not switch DE here. */
  rs485_tx_dma_irq++;
}

void RS485_ProcessTxEvent(void)
{
  /* TX completion processing is driven by UART TC, then RX is armed again. */
  if ((rs485_tx_busy != 0U) && (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != RESET))
  {
    rs485_tx_busy = 0U;
    rs485_tx_done++;
    RS485_SetReceiveMode();
    RS485_DirectionSwitchDelay();
    (void)RS485_StartReceive();
  }
}

void RS485_ProcessRxFrame(void)
{
  uint16_t rx_size = 0U;
  uint16_t tx_size = 0U;
  ModbusResult_t modbus_result;

  if (rs485_tx_busy != 0U)
  {
    return;
  }

  if (RS485_TakeRxFrame(rs485_rx_work_buffer, &rx_size, sizeof(rs485_rx_work_buffer)) != 0U)
  {
    modbus_result = Modbus_ProcessRequest(rs485_rx_work_buffer,
                                          rx_size,
                                          rs485_response_buffer,
                                          sizeof(rs485_response_buffer),
                                          &tx_size);
    if ((modbus_result == MODBUS_RESULT_RESPONSE_READY) && (tx_size > 0U))
    {
      rs485_tx_from_echo_task++;
      rs485_modbus_response_ready++;
      if (RS485_Send(rs485_response_buffer, tx_size) != HAL_OK)
      {
        rs485_tx_send_fail++;
      }
    }
    else if (modbus_result == MODBUS_RESULT_NO_RESPONSE)
    {
      rs485_modbus_no_response++;
    }
    else
    {
      rs485_modbus_frame_error++;
    }
  }
}

void RS485_PollEcho(void)
{
  RS485_ProcessTxEvent();
  RS485_ProcessRxFrame();
}

void RS485_GetStatus(RS485_StatusTypeDef *status)
{
  if (status == NULL)
  {
    return;
  }

  __disable_irq();
  status->init_calls = rs485_init_calls;
  status->rx_start_ok = rs485_rx_start_ok;
  status->tx_requests = rs485_tx_requests;
  status->tx_from_init = rs485_tx_from_init;
  status->tx_from_echo_task = rs485_tx_from_echo_task;
  status->tx_dma_irq = rs485_tx_dma_irq;
  status->tx_cplt_callback = rs485_tx_cplt_callback;
  status->rx_events = rs485_rx_events;
  status->rx_bytes = rs485_rx_bytes;
  status->rx_overwrite = rs485_rx_overwrite;
  status->rx_taken = rs485_rx_taken;
  status->modbus_response_ready = rs485_modbus_response_ready;
  status->modbus_no_response = rs485_modbus_no_response;
  status->modbus_frame_error = rs485_modbus_frame_error;
  status->tx_send_fail = rs485_tx_send_fail;
  status->tx_done = rs485_tx_done;
  status->errors = rs485_errors;
  status->tx_busy = rs485_tx_busy;
  __enable_irq();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t frame_size;

  if (huart->Instance == USART1)
  {
    frame_size = Size;
    if (frame_size > RS485_RX_BUFFER_SIZE)
    {
      frame_size = RS485_RX_BUFFER_SIZE;
    }

    if (frame_size > 0U)
    {
      if (rs485_rx_frame_ready != 0U)
      {
        rs485_rx_overwrite++;
      }

      /* Callback only records frame metadata; the RS485 task copies bytes. */
      rs485_rx_frame_size = frame_size;
      rs485_rx_frame_ready = 1U;
      rs485_rx_events++;
      rs485_rx_bytes += frame_size;

      /* Wake RS485 task; copying and protocol parsing stay outside the callback. */
      RS485_TaskNotifyRxFrame();
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* In DMA normal mode HAL calls this after USART TC, not just DMA TC. */
    rs485_tx_cplt_callback++;

    /* The task owns post-TC processing: clear busy, switch DE, restart RX. */
    RS485_TaskNotifyTxComplete();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    rs485_errors++;
    rs485_tx_busy = 0U;
    RS485_SetReceiveMode();
    (void)RS485_StartReceive();
    RS485_TaskNotifyTxError();
  }
}
