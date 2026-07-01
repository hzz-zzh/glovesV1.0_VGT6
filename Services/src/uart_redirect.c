#include "uart_redirect.h"
#include "stdio.h"
#include "main.h"

static volatile uint8_t s_uart_printf_enabled = 1U;

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

void UartRedirect_WriteData(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart2, data, length, 0xffffU);
}

