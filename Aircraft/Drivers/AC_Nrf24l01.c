/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * ����    ��Nrf24l01
**********************************************************************************/
#include "AC_Nrf24l01.h"
#include "AC_SPI.h"
#include "AC_DT.h" 

//***************************************NRF24L01�Ĵ���ָ��*******************************************************
#define NRF_READ_REG    0x00  	// ���Ĵ���ָ��
#define NRF_WRITE_REG   0x20 	  // д�Ĵ���ָ��
#define R_RX_PL_WID   	0x60
#define RD_RX_PLOAD     0x61  	// ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0  	// д��������ָ��
#define FLUSH_TX        0xE1 	  // ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  	// ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP             0xFF  	// ����
//*************************************SPI(nRF24L01)�Ĵ�����ַ****************************************************
#define NRF_CONFIG      0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define NRFRegSTATUS    0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��1�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��2�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��3�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��4�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��5�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������
//**************************************************************************************
//*********************************************NRF24L01*************************************
#define RX_DR				6		//�жϱ�־
#define TX_DS				5
#define MAX_RT			4

//���ص�ַ
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0xAA,0xAA,0x00,0x28,0x05};	
//���յ�ַ
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0xAA,0xAA,0x00,0x28,0x05};	
//����¼�
void NRF_CheckEvent(void)
{
	static uint8_t NRF_RX[RX_PLOAD_WIDTH];		  //��Ҫ���յ�����
	uint8_t sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
	
	if(sta & (1<<RX_DR))//���յ�����
	{ 
		u8 rx_len = NRF_Read_Reg(R_RX_PL_WID);
		if(rx_len<33)
		{
			NRF_Read_Buf(RD_RX_PLOAD,NRF_RX,rx_len);//��ȡ����
			Data_Receive_Anl(NRF_RX,rx_len);			
		}else
		{
			NRF_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 	
		}
	}
	if(sta & (1<<TX_DS))//������ɲ����յ�Ӧ���ź��ж�
	{

	}
	if(sta & MAX_RT)//�ﵽ����ط������ж�
	{
		NRF_Write_Reg(FLUSH_TX,0xff);//���RX FIFO�Ĵ��� 
	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);//���nrf���жϱ�־λ
}

/**************NRF��ʼ��*********************************************************/
void NRF_Init(u8 mode, u8 ch)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/*����SPI��CE����,��SPI�� CSN ����:*/
	GPIO_InitStructure.GPIO_Pin = SPI_Pin_CE; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(SPI_GPIO_CE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_Pin_CSN; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(SPI_GPIO_CSN, &GPIO_InitStructure);	

	CE_L();
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); //дTX�ڵ��ַ  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); //ʹ��ͨ��0���Զ�Ӧ�� 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10�� 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,ch); //����RFͨ��ΪCHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��
	//NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);//����TX�������,0db����,1Mbps,���������濪��
	//NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x27);//����TX�������,0db����,250Kbps,���������濪��
		
	if(mode==1){				//RX
		NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);   		 
		NRF_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
	}else if(mode==2){		//TX
		NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);   	
		NRF_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
	}else if(mode==3){		//RX2
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		NRF_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
		
		spi2_rw(0x50);
		spi2_rw(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}else{								//TX2
		NRF_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
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
* д�Ĵ���
*****************************************************************
*/
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	CSN_L();					  
	status = spi2_rw(reg);  
	spi2_rw(value);		  /* д���� */
	CSN_H();					  /* ��ֹ������ */
  return 	status;
}
/*
*****************************************************************
* ���Ĵ���
*****************************************************************
*/
uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	CSN_L();					 
	spi2_rw(reg);			  
	reg_val = spi2_rw(0);	/* ��ȡ�üĴ����������� */
	CSN_H();					    /* ��ֹ������ */
  return 	reg_val;
}
/*
*****************************************************************
*
* д������
*
*****************************************************************
*/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	CSN_L();				        /* ѡͨ���� */
	status = spi2_rw(reg); 	/* д�Ĵ�����ַ */
	for(i=0; i<uchars; i++)
	{
		spi2_rw(pBuf[i]);	 	/* д���� */
	}
	CSN_H();						 
  return 	status;	
}
/*
*****************************************************************
* ��������
*****************************************************************
*/
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	CSN_L();						/* ѡͨ���� */
	status = spi2_rw(reg);	/* д�Ĵ�����ַ */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = spi2_rw(0); /* ��ȡ�������� */ 	
	}
	CSN_H();						/* ��ֹ������ */
  return 	status;
}

/******************************************************************
* 				�������ݰ�
******************************************************************/
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
	CE_L();		 //StandBy Iģʽ		
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // װ�ؽ��ն˵�ַ
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // װ������	
	CE_H();		 //�ø�CE���������ݷ���
}
void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
	CE_L();		 //StandBy Iģʽ	
	NRF_Write_Buf(0xa8, tx_buf, len); 			 // װ������
	CE_H();		 //�ø�CE
}
/******************************************************************
* 				���NRF
******************************************************************/
uint8_t NRF_Check(void)
{
	u8 buf2[5];
	u8 i;
	/*д��5���ֽڵĵ�ַ. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5);//д��5���ֽڵĵ�ַ.
	/*����д��ĵ�ַ */ 
	NRF_Read_Buf(TX_ADDR,buf2,5); //����д��ĵ�ַ  
	/*�Ƚ�*/ 
	for(i=0;i<5;i++)
	{
		if(buf2[i]!=TX_ADDRESS[i])
			break;   
	}
	if(i!=5){
		return 0;//MCU��NRF���ɹ����� 
	}
	return 1; //MCU��NRF�ɹ����� 
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/

