#ifndef __CY_USART2_H
#define __CY_USART2_H

#include "stm32f10x.h"
#include <string.h>

extern uint8_t Usart2_SendData[256];
extern uint8_t Usart2_ReceData[256];

#define USE_USART2_IRQ//���ж�
#define USE_USART2_DMA//������DMA

//#define USE_USART2_IT_TXE //������
#define USE_USART2_DMA_TX//��DMA����

#define USE_USART2_DMA_RX//��DMA����
#define USE_USART2_IT_IDLE//�������ж�
//#define USE_USART2_IT_RXNE//�������ж�

/*DMA����*/
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

