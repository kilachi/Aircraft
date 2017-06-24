/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * �ļ���  ��CY_Flash.c
 * ����    ��Flash
**********************************************************************************/
#include "AC_Flash.h"

#define Flash_Parm param
#define _Flash_Parm _param
#define Flash_Parm_ID param.CPU_ID

//Flash��ʼ��
void Flash_init(void)
{
	/* ���� HSI */
	RCC_HSICmd(ENABLE);
	/* ���� FLASH ���ƿ�*/
	FLASH_Unlock();
	/* ���һЩ��־λ */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	/* ���� FLASH ���ƿ�*/
	FLASH_Lock();
}
//Flashд
void Flash_write(void)
{
	u8 dataLenght,q;
	u16 qq;
	/* ���� FLASH ���ƿ�*/
	FLASH_Unlock();
	dataLenght=sizeof(Flash_Parm);
	/* ������ʼ��ַΪ Flash_Addr �� FLASH ҳ */
	FLASH_ErasePage(Flash_Addr);
	for(q=0;q<dataLenght;q++)
	{
		qq=*(((uint16_t *)&Flash_Parm)+q);
		FLASH_ProgramHalfWord(Flash_Addr+q*2,qq);
	}
	/* ���� FLASH ���ƿ�*/
	FLASH_Lock();
}
//Flash��
uint16_t Flash_read(void)
{
	/* ���� FLASH ���ƿ�*/
	FLASH_Unlock();
	
	*(_Flash_Parm*)((&Flash_Parm))=*(_Flash_Parm*)(Flash_Addr);
	
	/* ���� FLASH ���ƿ�*/
	FLASH_Lock();
	
	return Flash_Parm_ID;
}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
