/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_Usart2.c
 * 描述    ：Usart2
**********************************************************************************/
#include "CY_Usart2.h"

uint8_t Usart2_SendData[256];
uint8_t Usart2_ReceData[256];

void USART2_IRQHandler(void)//串口中断函数
{
	u8 len;
/****************************空闲中断********************************************************/	
#ifdef USE_USART2_IT_IDLE	
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		len=USART2->SR;len=USART2->DR;
		DMA_Cmd(DMA1_Channel6, DISABLE);
		len=DMA1_CH6_BufferSize-DMA_GetCurrDataCounter(DMA1_Channel6); 
		if(len>200)
		{

		}
//		DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5);
		DMA1_Channel6->CNDTR = DMA1_CH6_BufferSize;
		DMA_Cmd(DMA1_Channel6, ENABLE);	
	}
#endif
/****************************发送中断********************************************************/				
#ifdef USE_USART2_IT_TXE	
	if((USART2->SR & (1<<7))&&(USART2->CR1 & USART_CR1_TXEIE))	
	{
		USART2->DR = Usart2_SendData[Usart2_sendcnt++];
		
		if(Usart2_sendcnt==Usart2_bufcnt)
			USART2->CR1 &= ~USART_CR1_TXEIE;//关闭TXE中断
	}
#endif
/****************************接收中断	********************************************************/	
#ifdef USE_USART2_IT_RXNE	
	if(USART2->SR & (1<<5))
	{
		len=USART2->DR;
	}
/****************************ORE中断********************************************************/			
	if(USART2->SR & USART_SR_ORE)
	{
		len=USART2->DR;
	}	
#endif
/*****************************异常*******************************************************/		
	if(USART_GetITStatus(USART2, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET)
	{  
		USART_ClearITPendingBit(USART2, USART_IT_PE | USART_IT_FE | USART_IT_NE); 
	} 	
	USART_ClearITPendingBit(USART2, USART_IT_IDLE);			
}
void Usart2_Init(u32 bound)
{
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
	
#ifdef USE_USART2_IRQ
	//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);
#endif	
	//USART2_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART2_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	//USART SET
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;												 //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	               //收发模式
	//配置USART2时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出	
	USART_Init(USART2, &USART_InitStructure); 	
	USART_ClockInit(USART2, &USART_ClockInitStruct);
#ifdef USE_USART2_IRQ
	#ifdef USE_USART2_IT_IDLE
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	#endif
	#ifdef USE_USART2_IT_TXE
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	#endif
	#ifdef USE_USART2_IT_RXNE
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	#endif
#endif
	USART_Cmd(USART2, ENABLE);           
	USART_ClearFlag(USART2, USART_FLAG_TC); 

#ifdef USE_USART2_DMA
	#ifdef USE_USART2_DMA_TX
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = DMA1_CH7_MemoryBaseAddr; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; //内存->外设
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 		//单次
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 										
	DMA_InitStructure.DMA_BufferSize = 0; 															
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 								
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 														
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE); 
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TE, ENABLE); 		
  DMA_Cmd(DMA1_Channel7, DISABLE);   	
	#endif
	#ifdef USE_USART2_DMA_RX
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = DMA1_CH6_MemoryBaseAddr; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //外设->内存
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 	//循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 										
	DMA_InitStructure.DMA_BufferSize = DMA1_CH6_BufferSize; 															
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 							
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 														
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	
	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE); 
	DMA_ITConfig(DMA1_Channel6, DMA_IT_TE, ENABLE); 
  DMA_Cmd(DMA1_Channel6, ENABLE); 			
	#endif	
#endif	
}

void DMA1_Channel7_IRQHandler(void) 
{  
	DMA_ClearITPendingBit(DMA1_IT_TC7); 
	DMA_ClearITPendingBit(DMA1_IT_TE7);  
	DMA_Cmd(DMA1_Channel7, DISABLE);
} 
void DMA1_Channel6_IRQHandler(void) 
{  
#ifdef USE_USART2_DMA_TX
	DMA_ClearITPendingBit(DMA1_IT_TC6); 
	DMA_ClearITPendingBit(DMA1_IT_TE6);  
#endif
} 
void Usart2_Put_char(unsigned char c)
{
#ifdef USE_USART2_IT_TXE
	Usart2_SendData[Usart2_bufcnt++]= c;
  if(!(USART2->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);       
#endif
}
void Usart2_Put_str(uint8_t * str,uint8_t len)
{
#ifdef USE_USART2_IT_TXE	
	uint16_t  i;
	for(i=0;i<len;i++)
	{
	 Usart2_Put_char(str[i]);
	}
#endif
}
void Usart2_Send(uint8_t *data,uint16_t len)
{
#ifdef USE_USART2_DMA_TX
	uint16_t i;
	for(i=0;i<len;i++)
		Usart2_SendData[i]=data[i];
	
	DMA_Cmd(DMA1_Channel7, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel7,len);
	DMA_Cmd(DMA1_Channel7, ENABLE);	
#endif
#ifdef USE_USART2_IT_TXE	
	uint16_t  i;
	for(i=0;i<len;i++)
	{
	 Usart2_Put_char(str[i]);
	}
#endif	
}
