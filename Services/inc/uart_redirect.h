#ifndef UART_REDIRECT_H
#define UART_REDIRECT_H
#include <stdint.h>
#include "stdio.h"

int fputc(int ch, FILE *f);
int fgetc(FILE *f);

/** 创建串口发送互斥锁，防止多个 RTOS 任务输出的数据帧相互穿插。 */
void UartRedirect_Init(void);

/** Enable or suppress the generic printf UART output. */
void UartRedirect_SetPrintfEnabled(uint8_t enabled);

/** Send an application data stream even when generic printf is suppressed. */
void UartRedirect_WriteData(const char *data, uint16_t length);

#endif


