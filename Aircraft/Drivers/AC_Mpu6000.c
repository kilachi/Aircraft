/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：Mpu6000
**********************************************************************************/
#include "AC_Mpu6000.h"

void mpu6000_Reset(uint8_t data)
{
	CS_L();
	spi1_rw(MPU6000_RA_PWR_MGMT_1);
	spi1_rw(data);
	CS_H();
}
void mpu6000_setClockSource(uint8_t source)
{
	CS_L();
	spi1_rw(MPU6000_RA_PWR_MGMT_1);
	spi1_rw(source);
	CS_H();
}
void mpu6000_SPIEnabled(uint8_t enabled)  //使能SPI总线
{
	CS_L();
	spi1_rw(MPU6000_RA_USER_CTRL);	//写命令+寄存器地址
	spi1_rw(enabled);
	CS_H();

}
void mpu6000_setSampleRateDIV(uint8_t rate)  //采样率分频，典型值：0x07(125Hz)
{
	CS_L();
	spi1_rw(MPU6000_RA_SMPRT_DIV);	//写命令+寄存器地址
	spi1_rw(rate);
	CS_H();
}
void mpu6000_setDLPH(uint8_t rate)	  //低通滤波频率，典型值：0x06(5Hz)
{
	CS_L();
	spi1_rw(MPU6000_RA_CONFIG);	//写命令+寄存器地址
	spi1_rw(rate);
	CS_H();
}
void mpu6000_setFullScaleGyroRange(uint8_t range) 
{
	CS_L();
	spi1_rw(MPU6000_RA_GYRO_CONFIG);
	spi1_rw(range);
	CS_H();
}
void mpu6000_setFullScaleAccelRange(uint8_t range)
{
	CS_L();
	spi1_rw(MPU6000_RA_ACCEL_CONFIG);
	spi1_rw(range);
	CS_H();
} 
void mpu6000_ITEnabled(uint8_t enabled)  //开数据中断，即传感器数据准备好了以后就触发中断
{
	CS_L();
	spi1_rw(MPU6000_RA_INT_ENABLE);	//写命令+寄存器地址
	spi1_rw(enabled);
	CS_H();
}
void mpu6000_setITMode(uint8_t mode)  //配置中断模式，高电平触发，读寄存器就清除中断
{
	CS_L();
	spi1_rw(MPU6000_RA_INT_PIN_CFG);	//写命令+寄存器地址
	spi1_rw(mode);
	CS_H();
}

void mpu6000_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	

	/*Config MPU Pin CS*/ 
	GPIO_InitStructure.GPIO_Pin = MPU_CS_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(MPU_CS,GPIO_Pin_0);	
	
	/*Config MPU Pin IRQ*/ 
	GPIO_InitStructure.GPIO_Pin = MPU_IRQ_PIN;		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   
	GPIO_Init(MPU_IRQ, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);
	EXTI_InitStructure.EXTI_Line=EXTI_Line4;	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
  NVIC_Init(&NVIC_InitStructure);  	  
}
void mpu6000_init(void)
{
	mpu6000_config();
	
	mpu6000_Reset(0x80);
	delay_ms(250);
	mpu6000_setClockSource(0x03);    //设置时钟源为陀螺仪的z轴
	delay_ms(10);
	mpu6000_SPIEnabled(0x10);  //使能SPI总线，屏蔽I2C总线
	delay_ms(10);
	mpu6000_setSampleRateDIV(0x07);  //采样率分频，典型值：0x07(125Hz)
	delay_ms(10);
	mpu6000_setDLPH(0x06);	  //低通滤波频率，典型值：0x06(5Hz)
	delay_ms(10);
	mpu6000_setFullScaleGyroRange(0x18); //陀螺仪最大量程 +-2000度每秒,不自检
	delay_ms(10);
	mpu6000_setFullScaleAccelRange(0x10);	//加速度度最大量程 +-8G，不自检
	delay_ms(10);
	mpu6000_ITEnabled(0x01);   //开数据中断，即传感器数据准备好了以后就触发中断
	delay_ms(10);
	mpu6000_setITMode(0x30);  //配置中断模式，高电平触发，读寄存器就清除中断
	delay_ms(10);
}
void mpu6000_readdata(unsigned char *Buffer,uint8_t ReadAddr,uint8_t NumByteToRead)   
{ 
	uint8_t i;										    
	CS_L();                         
	spi1_rw(ReadAddr|=0x80);         //发送读取命令+读取数据的首寄存器地址   
	for(i=0;i<NumByteToRead;i++)
	{ 
		Buffer[i]=spi1_rw(0xFF);   //循环读数 
	}
	CS_H();
}  
int16_t* mpu6000_read(void)
{
	static int16_t mpu6000_data[7];
	uint8_t mpu6000_buffer[14];

	mpu6000_readdata(mpu6000_buffer,MPU6000_RA_ACCEL_XOUT_H,14);
	
	/*机头为Y+*/
	mpu6000_data[0] = ((((int16_t)mpu6000_buffer[0]) << 8) | mpu6000_buffer[1]);//x
	mpu6000_data[1] = ((((int16_t)mpu6000_buffer[2]) << 8) | mpu6000_buffer[3]);//y
	mpu6000_data[2] = ((((int16_t)mpu6000_buffer[4]) << 8) | mpu6000_buffer[5]);//z
	
	/*陀螺仪*/
	mpu6000_data[3] = ((((int16_t)mpu6000_buffer[8]) << 8) | mpu6000_buffer[9]);//x
	mpu6000_data[4] = ((((int16_t)mpu6000_buffer[10]) << 8) | mpu6000_buffer[11]);//y
	mpu6000_data[5] = ((((int16_t)mpu6000_buffer[12]) << 8) | mpu6000_buffer[13]);//z

	mpu6000_data[6] = ((((int16_t)mpu6000_buffer[6]) << 8) | mpu6000_buffer[7]);

	return mpu6000_data;
}
void EXTI4_IRQHandler(void)	//中断服务程序，MPU6000数据准备好了以后即会触发中断
{
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET){	  
//		mpu_update();
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
