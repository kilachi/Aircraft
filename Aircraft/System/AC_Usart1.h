#ifndef __CY_USART1_H
#define __CY_USART1_H

#include "stm32f10x.h"
#include <string.h>
extern uint8_t Usart1_SendData[256];
extern uint8_t Usart1_ReceData[256];

#define USE_USART1_IRQ//开中断
#define USE_USART1_DMA//开串口DMA

//#define USE_USART1_IT_TXE //开发送
#define USE_USART1_DMA_TX//开DMA发送

#define USE_USART1_DMA_RX//开DMA接收
#define USE_USART1_IT_IDLE//开空闲中断
//#define USE_USART1_IT_RXNE//开接收中断

/*DMA配置*/
#ifdef  USE_USART1_DMA
#define DMA1_CH4_MemoryBaseAddr ((u32)&Usart1_SendData)
#define DMA1_CH5_MemoryBaseAddr ((u32)&Usart1_ReceData)
#define DMA1_CH5_BufferSize 50
#endif
void Usart1_Init(u32 bound);
void Usart1_Put_char(unsigned char c);
void Usart1_Put_str(uint8_t * str,uint8_t len);
void Usart1_Send(uint8_t *data,uint16_t len);

#endif

