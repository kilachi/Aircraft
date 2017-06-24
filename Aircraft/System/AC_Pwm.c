/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_Pwm.c
 * 描述    ：Tim2_Pwm
**********************************************************************************/
#include "AC_Pwm.h" 

//设置PWM
void set_motor_out(int16_t pwm[4])
{
	TIM->CCR1 = pwm[0]; 
	TIM->CCR2 = pwm[1];	
//	TIM->CCR3 = pwm[2]; 
//	TIM->CCR4 = pwm[3];	
}
void set_motor_out_all(int16_t pwm){
	controler.pwm_output[0]=controler.pwm_output[1]=controler.pwm_output[2]=controler.pwm_output[3] = pwm;
	TIM->CCR1 = pwm; 
	TIM->CCR2 = pwm;	
	TIM->CCR3 = pwm; 
	TIM->CCR4 = pwm;	
}

void pwm_out_init(uint16_t hz)//HZ
{
	float Prescaler =(((float)(72000/hz)) - 1);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//定义定时器结构体
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_TIM, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Tim;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIO_Tim, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 999; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //采样分频TIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //向上计数
	
	TIM_TimeBaseInit(TIM, &TIM_TimeBaseStructure);	//初始化TIM
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //配置为PWM模式1	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //使能通道1
	TIM_OCInitStructure.TIM_Pulse = CCR_VAL;				//超过这个值电平跳变
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //当定时器小于CCR1时为高电平
	TIM_OC1Init(TIM,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //使能通道2
	TIM_OCInitStructure.TIM_Pulse = CCR_VAL;
	TIM_OC2Init(TIM,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //使能通道3
	TIM_OCInitStructure.TIM_Pulse = CCR_VAL;
	TIM_OC3Init(TIM,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //使能通道4
	TIM_OCInitStructure.TIM_Pulse = CCR_VAL;
	TIM_OC4Init(TIM,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM, ENABLE);
	TIM_Cmd(TIM, ENABLE); //开启时钟
}






/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
