/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * �ļ���  ��CY_I2C_Soft.c
 * ����    ��ģ��I2C
**********************************************************************************/
#include "CY_I2C_Soft.h"

void I2C_Soft_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
 	RCC_APB2PeriphClockCmd(RCC_GPIO_I2C, ENABLE);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SCL;				//SCL
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SDA;				//SDA
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);		
}
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
****************************************************************************** */
void I2C_delay(void)
{
		return ;
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Return         : Wheather	 Start
****************************************************************************** */
unsigned char I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return 0;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L;
	I2C_delay();
	if(SDA_read) return 0;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	SDA_L;
	I2C_delay();
	return 1;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
****************************************************************************** */
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
unsigned char I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
	  I2C_delay();
      return 0;
	}
	SCL_L;
	I2C_delay();
	return 1;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(unsigned char SendByte) //���ݴӸ�λ����λ//
{
    unsigned char i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //���ݴӸ�λ����λ//
{ 
    unsigned char i=8;
    unsigned char ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
			SCL_H;
      I2C_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 
//���ֽ�д��*******************************************
unsigned char I2C_Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//���ø���ʼ��ַ+������ַ 
    if(!I2C_WaitAck())
		{
			I2C_Stop();
			return 0;
		}
    I2C_SendByte(REG_Address );   //���õ���ʼ��ַ      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    return 1;
}
//���ֽڶ�ȡ*****************************************
unsigned char I2C_Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   
		unsigned char REG_data;     	
		if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ 
    if(!I2C_WaitAck())
		{
			I2C_Stop();
			return 0;
		}
    I2C_SendByte((unsigned char) REG_Address);   //���õ���ʼ��ַ      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

		REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return 1;
	return REG_data;

}						      
//��ȡ����ֽڵ�����
void I2C_Mult_Read(unsigned char SlaveAddress,unsigned char star_addr,unsigned char num,unsigned char* recv_buf)
{
	unsigned char i;
	
	I2C_Start();

	I2C_SendByte(SlaveAddress);//�����豸��ַ
	I2C_WaitAck();

	I2C_SendByte(star_addr);//������ʼ��ַ
	I2C_WaitAck();
	
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);//�����豸��ַ+���ź�
	I2C_WaitAck();
	
	for(i=0;i<num;i++)
	{
		recv_buf[i] = I2C_RadeByte();
		if(i == (num-1)) I2C_NoAck();//���һ��������Ҫ��NOACK
		else I2C_Ack();
	}
	I2C_Stop();//ֹͣ�ź�
	I2C_delay();
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/

