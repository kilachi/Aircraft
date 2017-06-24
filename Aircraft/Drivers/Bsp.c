/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_Bsp_Init.c
 * 描述    ：硬件初始化
**********************************************************************************/
#include "Aircraft_Config.h"

void system_initialize(void)
{
	/*Interrupt priority setting*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	

	/*USB init*/
	USB_HID_Init();
	
	/*Configuration ticking clock*/
	SysTick_Config(SystemCoreClock / 1000);	
	
	/*Flash init*/	
	Flash_init();
	
	/*Param init*/	
	Param_Init();
	
	/*Config LED PIN*/
	led_init();
	
	/*Config SPI*/
	spi1_init();
	spi2_init();
	
	/*Config MPU6000*/
	mpu6000_init();
	
	/*Config NRF24L01*/
	NRF_Init(Mode_TX2,80);
	
	/*Config MS5611*/
	MS5611_Init();
	
	/*PWM init HZ*/
	pwm_out_init(24000);
}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/

