/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_Scheduler.c
 * 描述    ：任务调度
**********************************************************************************/
#include "AC_Timer.h"

_sysTick sysTick;


void systick_irq(void)
{
	sysTick.loop_1ms++;
	sysTick.loop_2ms++;
	sysTick.loop_5ms++;
	sysTick.loop_10ms++;
	sysTick.loop_50ms++;
	sysTick.sysTickMs++;
	if((sysTick.sysTickMs%1000) ==0){
		if((++sysTick.time_s)>=60){//秒
			sysTick.time_s=0;
			if((++sysTick.time_m)>=60){//分
				sysTick.time_m=0;
				if((++sysTick.time_h)>=60){//时
					sysTick.time_h=0;					
				}							
			}			
		}
	}
}
void delay_us(uint16_t ms)
{
	
}
void delay_ms(uint16_t ms)
{
	uint32_t time=sysTick.sysTickMs;
	while(sysTick.sysTickMs<(time+ms));
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
