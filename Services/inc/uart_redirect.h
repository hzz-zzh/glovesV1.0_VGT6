#ifndef UART_REDIRECT_H
#define UART_REDIRECT_H
#include <stdint.h>
#include "stdio.h"

int fputc(int ch, FILE *f);
int fgetc(FILE *f);

/** Enable or suppress the generic printf UART output. */
void UartRedirect_SetPrintfEnabled(uint8_t enabled);

/** Send an application data stream even when generic printf is suppressed. */
void UartRedirect_WriteData(const uint8_t *data, uint16_t length);

#endif


