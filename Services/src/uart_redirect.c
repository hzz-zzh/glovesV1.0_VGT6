#include "uart_redirect.h"
#include "stdio.h"
#include "main.h"

/*fputc():向文件(或设备)写入一个字符*/
int fputc(int ch, FILE *f){
  /*重定向print ，huart是一个usart的结构体，如果需要换串口，改这个就行*/
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch; 
}

/*fgetc():向文件(或设备)读取一个字符*/
int fgetc(FILE *f){
  int ch;
  /*重定向print,huart是一个usart的结构体，如果需要换串口，改这个就行*/
  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 0xffff);
  return ch; 
}




