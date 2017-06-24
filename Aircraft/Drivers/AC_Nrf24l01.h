#ifndef __AC_NRF24L01_H
#define __AC_NRF24L01_H

#include "stm32f10x.h"


#define Mode_RX			  	1			//普通接收
#define Mode_TX			  	2			//普通发送
#define Mode_RX2				3			//接收模式2,用于双向传输
#define Mode_TX2				4			//发送模式2,用于双向传输

#define TX_PLOAD_WIDTH  255  	
#define RX_PLOAD_WIDTH  255 	
#define TX_ADR_WIDTH    5   	
#define RX_ADR_WIDTH    5   	

#define SPI_GPIO_CE	 GPIOB
#define SPI_Pin_CE	 GPIO_Pin_11
#define SPI_GPIO_CSN GPIOB
#define SPI_Pin_CSN  GPIO_Pin_12

#define CE_H()  SPI_GPIO_CE->BSRR=SPI_Pin_CE
#define CE_L()  SPI_GPIO_CE->BRR =SPI_Pin_CE

#define CSN_H() SPI_GPIO_CSN->BSRR=SPI_Pin_CSN
#define CSN_L() SPI_GPIO_CSN->BRR=SPI_Pin_CSN

//发送数据包,用于model 2/4
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len);
//发送数据包,用于model 3
void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len);
//NRF数据监测事件
void NRF_CheckEvent(void);

void 		NRF_Init(u8 mode, u8 ch);
uint8_t SPI_RW(u8 dat);
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value);
uint8_t NRF_Read_Reg(uint8_t reg);
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
uint8_t NRF_Check(void);

#endif 

