#ifndef __AC_TIMER_H
#define __AC_TIMER_H
#include "Aircraft_Config.h"

/*结构体定义*/
typedef struct {
	volatile uint32_t sysTickMs;
	uint8_t	time_h;
	uint8_t	time_m;
	uint8_t	time_s;
	
	uint8_t loop_1ms;
	uint8_t loop_2ms;
	uint8_t loop_5ms;
	uint8_t loop_10ms;
	uint8_t loop_50ms;
	
}_sysTick;

extern _sysTick sysTick;


void systick_irq(void);
void delay_ms(uint16_t ms);

#endif

