/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * �ļ���  ��CY_Pwm.c
 * ����    ��Tim2_Pwm
**********************************************************************************/
#include "AC_Pwm.h" 

//����PWM
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
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//���嶨ʱ���ṹ��
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_TIM, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Tim;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIO_Tim, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 999; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //������ƵTIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //���ϼ���
	
	TIM_TimeBaseInit(TIM, &TIM_TimeBaseStructure);	//��ʼ��TIM
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //����ΪPWMģʽ1	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //ʹ��ͨ��1
	TIM_OCInitStructure.TIM_Pulse = CCR_VAL;				//�������ֵ��ƽ����
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //����ʱ��С��CCR1ʱΪ�ߵ�ƽ
	TIM_OC1Init(TIM,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //ʹ��ͨ��2
	TIM_OCInitStructure.TIM_Pulse = CCR_VAL;
	TIM_OC2Init(TIM,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //ʹ��ͨ��3
	TIM_OCInitStructure.TIM_Pulse = CCR_VAL;
	TIM_OC3Init(TIM,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM,TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //ʹ��ͨ��4
	TIM_OCInitStructure.TIM_Pulse = CCR_VAL;
	TIM_OC4Init(TIM,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM, ENABLE);
	TIM_Cmd(TIM, ENABLE); //����ʱ��
}






/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
