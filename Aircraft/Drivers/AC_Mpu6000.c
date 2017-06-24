/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * ����    ��Mpu6000
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
void mpu6000_SPIEnabled(uint8_t enabled)  //ʹ��SPI����
{
	CS_L();
	spi1_rw(MPU6000_RA_USER_CTRL);	//д����+�Ĵ�����ַ
	spi1_rw(enabled);
	CS_H();

}
void mpu6000_setSampleRateDIV(uint8_t rate)  //�����ʷ�Ƶ������ֵ��0x07(125Hz)
{
	CS_L();
	spi1_rw(MPU6000_RA_SMPRT_DIV);	//д����+�Ĵ�����ַ
	spi1_rw(rate);
	CS_H();
}
void mpu6000_setDLPH(uint8_t rate)	  //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
{
	CS_L();
	spi1_rw(MPU6000_RA_CONFIG);	//д����+�Ĵ�����ַ
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
void mpu6000_ITEnabled(uint8_t enabled)  //�������жϣ�������������׼�������Ժ�ʹ����ж�
{
	CS_L();
	spi1_rw(MPU6000_RA_INT_ENABLE);	//д����+�Ĵ�����ַ
	spi1_rw(enabled);
	CS_H();
}
void mpu6000_setITMode(uint8_t mode)  //�����ж�ģʽ���ߵ�ƽ���������Ĵ���������ж�
{
	CS_L();
	spi1_rw(MPU6000_RA_INT_PIN_CFG);	//д����+�Ĵ�����ַ
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
	mpu6000_setClockSource(0x03);    //����ʱ��ԴΪ�����ǵ�z��
	delay_ms(10);
	mpu6000_SPIEnabled(0x10);  //ʹ��SPI���ߣ�����I2C����
	delay_ms(10);
	mpu6000_setSampleRateDIV(0x07);  //�����ʷ�Ƶ������ֵ��0x07(125Hz)
	delay_ms(10);
	mpu6000_setDLPH(0x06);	  //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
	delay_ms(10);
	mpu6000_setFullScaleGyroRange(0x18); //������������� +-2000��ÿ��,���Լ�
	delay_ms(10);
	mpu6000_setFullScaleAccelRange(0x10);	//���ٶȶ�������� +-8G�����Լ�
	delay_ms(10);
	mpu6000_ITEnabled(0x01);   //�������жϣ�������������׼�������Ժ�ʹ����ж�
	delay_ms(10);
	mpu6000_setITMode(0x30);  //�����ж�ģʽ���ߵ�ƽ���������Ĵ���������ж�
	delay_ms(10);
}
void mpu6000_readdata(unsigned char *Buffer,uint8_t ReadAddr,uint8_t NumByteToRead)   
{ 
	uint8_t i;										    
	CS_L();                         
	spi1_rw(ReadAddr|=0x80);         //���Ͷ�ȡ����+��ȡ���ݵ��׼Ĵ�����ַ   
	for(i=0;i<NumByteToRead;i++)
	{ 
		Buffer[i]=spi1_rw(0xFF);   //ѭ������ 
	}
	CS_H();
}  
int16_t* mpu6000_read(void)
{
	static int16_t mpu6000_data[7];
	uint8_t mpu6000_buffer[14];

	mpu6000_readdata(mpu6000_buffer,MPU6000_RA_ACCEL_XOUT_H,14);
	
	/*��ͷΪY+*/
	mpu6000_data[0] = ((((int16_t)mpu6000_buffer[0]) << 8) | mpu6000_buffer[1]);//x
	mpu6000_data[1] = ((((int16_t)mpu6000_buffer[2]) << 8) | mpu6000_buffer[3]);//y
	mpu6000_data[2] = ((((int16_t)mpu6000_buffer[4]) << 8) | mpu6000_buffer[5]);//z
	
	/*������*/
	mpu6000_data[3] = ((((int16_t)mpu6000_buffer[8]) << 8) | mpu6000_buffer[9]);//x
	mpu6000_data[4] = ((((int16_t)mpu6000_buffer[10]) << 8) | mpu6000_buffer[11]);//y
	mpu6000_data[5] = ((((int16_t)mpu6000_buffer[12]) << 8) | mpu6000_buffer[13]);//z

	mpu6000_data[6] = ((((int16_t)mpu6000_buffer[6]) << 8) | mpu6000_buffer[7]);

	return mpu6000_data;
}
void EXTI4_IRQHandler(void)	//�жϷ������MPU6000����׼�������Ժ󼴻ᴥ���ж�
{
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET){	  
//		mpu_update();
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
