#ifndef UART_DEBUG_TASK_H
#define UART_DEBUG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 串口调试线程入口
 *
 * 该线程通过 printf 输出调试信息，printf 的底层输出由 uart_redirect.c 重定向到 UART
 */
void UartDebugTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* UART_DEBUG_TASK_H */
