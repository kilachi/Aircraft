/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * �ļ���  ��CY_Usart1.c
 * ����    ��Usart1
**********************************************************************************/
#include "CY_Usart1.h"

uint8_t Usart1_SendData[256];
uint8_t Usart1_ReceData[256];
//static uint8_t Usart1_sendcnt;
//static uint8_t Usart1_bufcnt;

void USART1_IRQHandler(void)//�����жϺ���
{
	u8 len;
/****************************�����ж�********************************************************/	
#ifdef USE_USART1_IT_IDLE	
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		len=USART1->SR;len=USART1->DR;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		len=DMA1_CH5_BufferSize-DMA_GetCurrDataCounter(DMA1_Channel5); 
		if(len>0)
		{
// 			Data_Receive_Anl(Usart1_ReceData,len);
		}
		DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5);
		DMA1_Channel5->CNDTR = DMA1_CH5_BufferSize;
		DMA_Cmd(DMA1_Channel5, ENABLE);	
	}
#endif
/****************************�����ж�********************************************************/				
#ifdef USE_USART1_IT_TXE	
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))	
	{
		USART1->DR = Usart1_SendData[Usart1_sendcnt++];
		
		if(Usart1_sendcnt==Usart1_bufcnt)
			USART1->CR1 &= ~USART_CR1_TXEIE;//�ر�TXE�ж�
	}
#endif
/****************************�����ж�	********************************************************/	
#ifdef USE_USART1_IT_RXNE	
	if(USART1->SR & (1<<5))
	{
		len=USART1->DR;
	}
/****************************ORE�ж�********************************************************/			
	if(USART1->SR & USART_SR_ORE)
	{
		len=USART1->DR;
	}	
#endif
/*****************************�쳣*******************************************************/		
	if(USART_GetITStatus(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET)
	{  
		USART_ClearITPendingBit(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE); 
	} 	
	USART_ClearITPendingBit(USART1, USART_IT_IDLE);			
}
void Usart1_Init(u32 bound)
{
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
	
#ifdef USE_USART1_IRQ
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);
#endif	
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	//USART SET
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;												 //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	               //�շ�ģʽ
	//����USART1ʱ��
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���	
	USART_Init(USART1, &USART_InitStructure); 	
	USART_ClockInit(USART1, &USART_ClockInitStruct);
#ifdef USE_USART1_IRQ
	#ifdef USE_USART1_IT_IDLE
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	#endif
	#ifdef USE_USART1_IT_TXE
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	#endif
	#ifdef USE_USART1_IT_RXNE
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	#endif
#endif
	USART_Cmd(USART1, ENABLE);           
	USART_ClearFlag(USART1, USART_FLAG_TC); 

#ifdef USE_USART1_DMA
	#ifdef USE_USART1_DMA_TX
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = DMA1_CH4_MemoryBaseAddr; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; //�ڴ�->����
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 		//����
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 										
	DMA_InitStructure.DMA_BufferSize = 0; 															
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 								
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 														
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE); 
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TE, ENABLE); 		
  DMA_Cmd(DMA1_Channel4, DISABLE);   	
	#endif
	#ifdef USE_USART1_DMA_RX
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = DMA1_CH5_MemoryBaseAddr; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //����->�ڴ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 	//ѭ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 										
	DMA_InitStructure.DMA_BufferSize = DMA1_CH5_BufferSize; 															
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 							
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 														
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE); 
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TE, ENABLE); 
  DMA_Cmd(DMA1_Channel5, ENABLE); 			
	#endif	
#endif	
}

void DMA1_Channel4_IRQHandler(void) 
{  
	DMA_ClearITPendingBit(DMA1_IT_TC4); 
	DMA_ClearITPendingBit(DMA1_IT_TE4);  
	DMA_Cmd(DMA1_Channel4, DISABLE);
} 
void DMA1_Channel5_IRQHandler(void) 
{  
#ifdef USE_USART1_DMA_TX
	DMA_ClearITPendingBit(DMA1_IT_TC5); 
	DMA_ClearITPendingBit(DMA1_IT_TE5);  
#endif
} 
void Usart1_Put_char(unsigned char c)
{
#ifdef USE_USART1_IT_TXE
	Usart1_SendData[Usart1_bufcnt++]= c;
  if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);       
#endif
}
void Usart1_Put_str(uint8_t * str,uint8_t len)
{
#ifdef USE_USART1_IT_TXE	
	uint16_t  i;
	for(i=0;i<len;i++)
	{
	 Usart1_Put_char(str[i]);
	}
#endif
}
void Usart1_Send(uint8_t *data,uint16_t len)
{
#ifdef USE_USART1_DMA_TX
	uint16_t i;
	for(i=0;i<len;i++)
		Usart1_SendData[i]=data[i];
	
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel4,len);
	DMA_Cmd(DMA1_Channel4, ENABLE);	
#endif
#ifdef USE_USART1_IT_TXE	
	uint16_t  i;
	for(i=0;i<len;i++)
	{
	 Usart1_Put_char(str[i]);
	}
#endif	
}
