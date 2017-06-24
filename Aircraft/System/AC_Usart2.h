#ifndef __CY_USART2_H
#define __CY_USART2_H

#include "stm32f10x.h"
#include <string.h>

extern uint8_t Usart2_SendData[256];
extern uint8_t Usart2_ReceData[256];

#define USE_USART2_IRQ//开中断
#define USE_USART2_DMA//开串口DMA

//#define USE_USART2_IT_TXE //开发送
#define USE_USART2_DMA_TX//开DMA发送

#define USE_USART2_DMA_RX//开DMA接收
#define USE_USART2_IT_IDLE//开空闲中断
//#define USE_USART2_IT_RXNE//开接收中断

/*DMA配置*/
#ifdef  USE_USART2_DMA
#define DMA1_CH7_MemoryBaseAddr ((u32)&Usart2_SendData)
#define DMA1_CH6_MemoryBaseAddr ((u32)&Usart2_ReceData)
#define DMA1_CH6_BufferSize 256
#endif
void Usart2_Init(u32 bound);
void Usart2_Put_char(unsigned char c);
void Usart2_Put_str(uint8_t * str,uint8_t len);
void Usart2_Send(uint8_t *data,uint16_t len);

#endif

