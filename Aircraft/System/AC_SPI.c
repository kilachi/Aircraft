/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_Flash.c
 * 描述    ：Flash
**********************************************************************************/
#include "AC_SPI.h"

void spi1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );
		/*Config SPI1 Pin SCK,MISO,MOSI*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI1, &SPI_InitStructure); 
	/* Enable SPI1 */ 
	SPI_Cmd(SPI1, ENABLE);
}
uint8_t spi1_rw(uint8_t dat) 
{ 
	uint8_t retry=0;				 	
	/* 当 SPI发送缓冲器非空时等待 */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){
//		if((retry++)>200)return 0;
	}
	/* 通过 SPI2发送一字节数据 */ 
	SPI_I2S_SendData(SPI1, dat); 
	/* 当SPI接收缓冲器为空时等待 */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){
//		if((retry++)>200)return 0;
	}
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI1); 
}
void spi2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );
		/*Config SPI1 Pin SCK,MISO,MOSI*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI2, &SPI_InitStructure); 
	/* Enable SPI2 */ 
	SPI_Cmd(SPI2, ENABLE);
}
uint8_t spi2_rw(uint8_t dat) 
{ 
	uint8_t retry=0;				 	
	/* 当 SPI发送缓冲器非空时等待 */ 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){
//		if((retry++)>200)return 0;
	}
	/* 通过 SPI2发送一字节数据 */ 
	SPI_I2S_SendData(SPI2, dat); 
	/* 当SPI接收缓冲器为空时等待 */ 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){
//		if((retry++)>200)return 0;
	}
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI2); 
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
