#include "uart_redirect.h"
#include "stdio.h"
#include "cmsis_os2.h"
#include "main.h"

static volatile uint8_t s_uart_printf_enabled = 1U;
/* TOUCH 与 SOLVED 可能由不同任务发送，必须串行占用 USART2。 */
static osMutexId_t s_uart_tx_mutex = NULL;
static const osMutexAttr_t s_uart_tx_mutex_attributes = {
  .name = "uart2DataTx"
};

void UartRedirect_Init(void)
{
  if (s_uart_tx_mutex == NULL)
  {
    s_uart_tx_mutex = osMutexNew(&s_uart_tx_mutex_attributes);
  }
}

/* Redirect standard output to USART2 while generic output is enabled. */
int fputc(int ch, FILE *f){
  if (s_uart_printf_enabled != 0U)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
  }
  return ch; 
}

/* Redirect standard input to USART2. */
int fgetc(FILE *f){
  int ch;
  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch; 
}

void UartRedirect_SetPrintfEnabled(uint8_t enabled)
{
  s_uart_printf_enabled = (enabled != 0U) ? 1U : 0U;
}

void UartRedirect_WriteData(const char *data, uint16_t length)
{
  uint8_t mutex_locked = 0U;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  if ((s_uart_tx_mutex != NULL) && (osKernelGetState() == osKernelRunning))
  {
    if (osMutexAcquire(s_uart_tx_mutex, osWaitForever) != osOK)
    {
      return;
    }
    mutex_locked = 1U;
  }

  (void)HAL_UART_Transmit(&huart2, (const uint8_t *)data, length, 0xffffU);

  if (mutex_locked != 0U)
  {
    (void)osMutexRelease(s_uart_tx_mutex);
  }
}

