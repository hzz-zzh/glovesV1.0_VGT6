#ifndef __RS485_UASRT_H__
#define __RS485_UASRT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define RS485_RX_BUFFER_SIZE 1024U
#define RS485_TX_BUFFER_SIZE 1024U
#define RS485_RX_GPIO_LEVEL GPIO_PIN_RESET
#define RS485_TX_GPIO_LEVEL GPIO_PIN_SET

typedef struct
{
  uint32_t init_calls;
  uint32_t rx_start_ok;
  uint32_t tx_requests;
  uint32_t tx_from_init;
  uint32_t tx_from_echo_task;
  uint32_t tx_dma_irq;
  uint32_t tx_cplt_callback;
  uint32_t rx_events;
  uint32_t rx_bytes;
  uint32_t rx_overwrite;
  uint32_t rx_taken;
  uint32_t modbus_response_ready;
  uint32_t modbus_no_response;
  uint32_t modbus_frame_error;
  uint32_t tx_send_fail;
  uint32_t tx_done;
  uint32_t errors;
  uint8_t tx_busy;
} RS485_StatusTypeDef;

/* Initialize RS485 helper and start the first receive path if init TX fails. */
HAL_StatusTypeDef RS485_Init(void);

/* Put RS485 transceiver in receive mode and arm USART1 ReceiveToIdle DMA. */
HAL_StatusTypeDef RS485_StartReceive(void);

/* Start one half-duplex DMA transmit. The caller must handle HAL_BUSY. */
HAL_StatusTypeDef RS485_SendDMA(const uint8_t *data, uint16_t size);

/* Public send facade. It currently maps to DMA transmit. */
HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t size);

/* Copy one completed RX frame out of the halted DMA buffer in task context. */
uint8_t RS485_TakeRxFrame(uint8_t *data, uint16_t *size, uint16_t max_size);
uint8_t RS485_IsTxBusy(void);

/* Compatibility helper for old polling tests: process TX then RX once. */
void RS485_PollEcho(void);

/* Event-driven task entry points for RX frame processing and TX completion. */
void RS485_ProcessRxFrame(void);
void RS485_ProcessTxEvent(void);

void RS485_GetStatus(RS485_StatusTypeDef *status);

/* Called from the GPDMA TX IRQ; only counts DMA IRQ, not UART TC completion. */
void RS485_OnTxDmaIrq(void);

#ifdef __cplusplus
}
#endif

#endif
