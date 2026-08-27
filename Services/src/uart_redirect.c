#include "uart_redirect.h"

#include "app_config.h"
#include "main.h"

#if (APP_ENABLE_DEBUG_UART_OUTPUT != 0U)
#define UART_REDIRECT_BUFFER_SIZE       (4096U)
#define UART_REDIRECT_TX_CHUNK_SIZE     (256U)
#define UART_REDIRECT_TX_TIMEOUT_MS     (5U)

static uint8_t s_uart_redirect_buffer[UART_REDIRECT_BUFFER_SIZE];
static volatile uint16_t s_uart_redirect_head;
static volatile uint16_t s_uart_redirect_tail;
static volatile uint32_t s_uart_redirect_dropped;

static uint32_t UartRedirect_Lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void UartRedirect_Unlock(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}
#endif

int fputc(int ch, FILE *f)
{
#if (APP_ENABLE_DEBUG_UART_OUTPUT != 0U)
  uint16_t next_head;
  uint32_t primask;

  (void)f;
  primask = UartRedirect_Lock();
  next_head = (uint16_t)((s_uart_redirect_head + 1U) % UART_REDIRECT_BUFFER_SIZE);
  if (next_head != s_uart_redirect_tail)
  {
    s_uart_redirect_buffer[s_uart_redirect_head] = (uint8_t)ch;
    s_uart_redirect_head = next_head;
  }
  else
  {
    s_uart_redirect_dropped++;
  }
  UartRedirect_Unlock(primask);
#else
  /* 量产固件不缓存、不发送调试日志，避免占用串口和中断时间。 */
  (void)f;
#endif

  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0U;

  (void)f;
  (void)HAL_UART_Receive(&huart2, &ch, 1U, 10U);
  return (int)ch;
}

void UartRedirect_Flush(uint32_t max_bytes)
{
#if (APP_ENABLE_DEBUG_UART_OUTPUT != 0U)
  uint8_t tx_chunk[UART_REDIRECT_TX_CHUNK_SIZE];
  uint32_t total = 0U;

  while (total < max_bytes)
  {
    uint16_t count = 0U;
    uint32_t primask = UartRedirect_Lock();

    while ((s_uart_redirect_tail != s_uart_redirect_head) &&
           (count < UART_REDIRECT_TX_CHUNK_SIZE) &&
           ((total + count) < max_bytes))
    {
      tx_chunk[count++] = s_uart_redirect_buffer[s_uart_redirect_tail];
      s_uart_redirect_tail =
          (uint16_t)((s_uart_redirect_tail + 1U) % UART_REDIRECT_BUFFER_SIZE);
    }
    UartRedirect_Unlock(primask);

    if (count == 0U)
    {
      break;
    }

    /* 串口异常只影响低优先级调试任务，已经出队的日志允许丢弃。 */
    (void)HAL_UART_Transmit(&huart2, tx_chunk, count, UART_REDIRECT_TX_TIMEOUT_MS);
    total += count;
  }
#else
  (void)max_bytes;
#endif
}

uint32_t UartRedirect_GetDroppedCount(void)
{
#if (APP_ENABLE_DEBUG_UART_OUTPUT != 0U)
  return s_uart_redirect_dropped;
#else
  return 0U;
#endif
}
