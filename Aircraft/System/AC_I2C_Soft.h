#ifndef __CY_I2C_SOFT_H
#define __CY_I2C_SOFT_H

#include "stm32f10x.h"

//模拟IIC接口定义:************************************
#define GPIO_Pin_SCL			GPIO_Pin_0
#define GPIO_Pin_SDA			GPIO_Pin_1
//#define GPIO_Pin_SCL			GPIO_Pin_10
//#define GPIO_Pin_SDA			GPIO_Pin_11

#define RCC_GPIO_I2C			RCC_APB2Periph_GPIOA
#define GPIO_I2C			    GPIOA
//****************************
/*模拟IIC端口输出输入定义*/
#define SCL_H         GPIO_I2C->BSRR = GPIO_Pin_SCL
#define SCL_L         GPIO_I2C->BRR  = GPIO_Pin_SCL
   
#define SDA_H         GPIO_I2C->BSRR = GPIO_Pin_SDA
#define SDA_L         GPIO_I2C->BRR  = GPIO_Pin_SDA

#define SCL_read      GPIO_I2C->IDR  & GPIO_Pin_SCL
#define SDA_read      GPIO_I2C->IDR  & GPIO_Pin_SDA
//****************************
void 					I2C_Soft_Init(void);
void 				  I2C_delay(void);
unsigned char I2C_Start(void);
void					I2C_Stop(void);
void			    I2C_Ack(void);
void 					I2C_NoAck(void);
unsigned char I2C_WaitAck(void); 	 
void				  I2C_SendByte(unsigned char SendByte); 
unsigned char I2C_RadeByte(void); 
unsigned char I2C_Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char I2C_Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
void 				  I2C_Mult_Read(unsigned char SlaveAddress,unsigned char star_addr,unsigned char num,unsigned char* recv_buf);


#endif





