#ifndef __AC_PWM_H
#define __AC_PWM_H

#include "Aircraft_Config.h"

//PWM初始值
#define CCR_VAL 0
//使用定时器
#define RCC_TIM RCC_APB1Periph_TIM2
#define TIM	TIM2
//IO引脚
#define RCC_GPIO RCC_APB2Periph_GPIOA
#define GPIO_Pin_Tim (GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3)
#define GPIO_Tim	GPIOA



void pwm_out_init(uint16_t hz);
void set_motor_out(int16_t pwm[4]);
void set_motor_out_all(int16_t pwm);

#endif

