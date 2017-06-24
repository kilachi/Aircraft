/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：Nrf24l01
**********************************************************************************/
#include "AC_Nrf24l01.h"
#include "AC_SPI.h"
#include "AC_DT.h" 

//***************************************NRF24L01寄存器指令*******************************************************
#define NRF_READ_REG    0x00  	// 读寄存器指令
#define NRF_WRITE_REG   0x20 	  // 写寄存器指令
#define R_RX_PL_WID   	0x60
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	  // 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留
//*************************************SPI(nRF24L01)寄存器地址****************************************************
#define NRF_CONFIG      0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define NRFRegSTATUS    0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道1接收数据长度
#define RX_PW_P2        0x13  // 接收频道2接收数据长度
#define RX_PW_P3        0x14  // 接收频道3接收数据长度
#define RX_PW_P4        0x15  // 接收频道4接收数据长度
#define RX_PW_P5        0x16  // 接收频道5接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
//**************************************************************************************
//*********************************************NRF24L01*************************************
#define RX_DR				6		//中断标志
#define TX_DS				5
#define MAX_RT			4

//本地地址
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0xAA,0xAA,0x00,0x28,0x05};	
//接收地址
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0xAA,0xAA,0x00,0x28,0x05};	
//监测事件
void NRF_CheckEvent(void)
{
	static uint8_t NRF_RX[RX_PLOAD_WIDTH];		  //需要接收的数据
	uint8_t sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
	
	if(sta & (1<<RX_DR))//接收到数据
	{ 
		u8 rx_len = NRF_Read_Reg(R_RX_PL_WID);
		if(rx_len<33)
		{
			NRF_Read_Buf(RD_RX_PLOAD,NRF_RX,rx_len);//读取数据
			Data_Receive_Anl(NRF_RX,rx_len);			
		}else
		{
			NRF_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 	
		}
	}
	if(sta & (1<<TX_DS))//发送完成并接收到应答信号中断
	{

	}
	if(sta & MAX_RT)//达到最大重发次数中断
	{
		NRF_Write_Reg(FLUSH_TX,0xff);//清除RX FIFO寄存器 
	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);//清除nrf的中断标志位
}

/**************NRF初始化*********************************************************/
void NRF_Init(u8 mode, u8 ch)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/*配置SPI的CE引脚,和SPI的 CSN 引脚:*/
	GPIO_InitStructure.GPIO_Pin = SPI_Pin_CE; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(SPI_GPIO_CE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_Pin_CSN; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(SPI_GPIO_CSN, &GPIO_InitStructure);	

	CE_L();
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); //写TX节点地址  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); //使能通道0的自动应答 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,ch); //设置RF通道为CHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	//NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);//设置TX发射参数,0db增益,1Mbps,低噪声增益开启
	//NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x27);//设置TX发射参数,0db增益,250Kbps,低噪声增益开启
		
	if(mode==1){				//RX
		NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);   		 
		NRF_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
	}else if(mode==2){		//TX
		NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);   	
		NRF_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
	}else if(mode==3){		//RX2
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		NRF_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
		
		spi2_rw(0x50);
		spi2_rw(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}else{								//TX2
		NRF_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		
		spi2_rw(0x50);
		spi2_rw(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
	CE_H();
	NRF_Check();
}

/*
*****************************************************************
* 写寄存器
*****************************************************************
*/
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	CSN_L();					  
	status = spi2_rw(reg);  
	spi2_rw(value);		  /* 写数据 */
	CSN_H();					  /* 禁止该器件 */
  return 	status;
}
/*
*****************************************************************
* 读寄存器
*****************************************************************
*/
uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	CSN_L();					 
	spi2_rw(reg);			  
	reg_val = spi2_rw(0);	/* 读取该寄存器返回数据 */
	CSN_H();					    /* 禁止该器件 */
  return 	reg_val;
}
/*
*****************************************************************
*
* 写缓冲区
*
*****************************************************************
*/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	CSN_L();				        /* 选通器件 */
	status = spi2_rw(reg); 	/* 写寄存器地址 */
	for(i=0; i<uchars; i++)
	{
		spi2_rw(pBuf[i]);	 	/* 写数据 */
	}
	CSN_H();						 
  return 	status;	
}
/*
*****************************************************************
* 读缓冲区
*****************************************************************
*/
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	CSN_L();						/* 选通器件 */
	status = spi2_rw(reg);	/* 写寄存器地址 */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = spi2_rw(0); /* 读取返回数据 */ 	
	}
	CSN_H();						/* 禁止该器件 */
  return 	status;
}

/******************************************************************
* 				发送数据包
******************************************************************/
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
	CE_L();		 //StandBy I模式		
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // 装载数据	
	CE_H();		 //置高CE，激发数据发送
}
void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
	CE_L();		 //StandBy I模式	
	NRF_Write_Buf(0xa8, tx_buf, len); 			 // 装载数据
	CE_H();		 //置高CE
}
/******************************************************************
* 				检测NRF
******************************************************************/
uint8_t NRF_Check(void)
{
	u8 buf2[5];
	u8 i;
	/*写入5个字节的地址. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5);//写入5个字节的地址.
	/*读出写入的地址 */ 
	NRF_Read_Buf(TX_ADDR,buf2,5); //读出写入的地址  
	/*比较*/ 
	for(i=0;i<5;i++)
	{
		if(buf2[i]!=TX_ADDRESS[i])
			break;   
	}
	if(i!=5){
		return 0;//MCU与NRF不成功连接 
	}
	return 1; //MCU与NRF成功连接 
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/

