#ifndef UART_REDIRECT_H
#define UART_REDIRECT_H
#include <stdint.h>
#include "stdio.h"




int fputc(int ch, FILE *f);
int fgetc(FILE *f);
void UartRedirect_Flush(uint32_t max_bytes);
uint32_t UartRedirect_GetDroppedCount(void);
#endif


