/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_LED.c
 * 描述    ：配置文件
**********************************************************************************/
#include "AC_LED.h" 

#define RCC_APB_LED 	RCC_APB2Periph_GPIOB
#define GPIO_LED 			GPIOB
#define	GPIO_Pin_LED	GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8

#define LED_ON			GPIO_LED->BRR=GPIO_Pin_LED
#define LED_OFF			GPIO_LED->BSRR =GPIO_Pin_LED
#define LED_FRONT_ON			GPIO_LED->BRR=GPIO_Pin_5|GPIO_Pin_8
#define LED_FRONT_OFF			GPIO_LED->BSRR =GPIO_Pin_5|GPIO_Pin_8
#define LED_BACK_ON			GPIO_LED->BRR=GPIO_Pin_6|GPIO_Pin_7
#define LED_BACK_OFF			GPIO_LED->BSRR =GPIO_Pin_6|GPIO_Pin_7


#define LED_Times 	20

void led_run(uint8_t flag)
{
	static u16 cnt;
	if(flag)
	{
		cnt++;
		if(cnt>=LED_Times)
		{
			LED_BACK_OFF;
			if(cnt >= (LED_Times+2))
			{
				LED_BACK_ON;
				cnt=0;
			}
		}		
	}else 
	{
		cnt=0;
		LED_ON;
	}
}
void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB_LED,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_LED;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_LED, &GPIO_InitStructure);
	
  GPIO_SetBits(GPIO_LED,GPIO_Pin_LED);
}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
