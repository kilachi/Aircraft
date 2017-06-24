/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_Flash.c
 * 描述    ：Flash
**********************************************************************************/
#include "AC_Flash.h"

#define Flash_Parm param
#define _Flash_Parm _param
#define Flash_Parm_ID param.CPU_ID

//Flash初始化
void Flash_init(void)
{
	/* 开启 HSI */
	RCC_HSICmd(ENABLE);
	/* 解锁 FLASH 控制块*/
	FLASH_Unlock();
	/* 清除一些标志位 */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	/* 锁定 FLASH 控制块*/
	FLASH_Lock();
}
//Flash写
void Flash_write(void)
{
	u8 dataLenght,q;
	u16 qq;
	/* 解锁 FLASH 控制块*/
	FLASH_Unlock();
	dataLenght=sizeof(Flash_Parm);
	/* 擦除起始地址为 Flash_Addr 的 FLASH 页 */
	FLASH_ErasePage(Flash_Addr);
	for(q=0;q<dataLenght;q++)
	{
		qq=*(((uint16_t *)&Flash_Parm)+q);
		FLASH_ProgramHalfWord(Flash_Addr+q*2,qq);
	}
	/* 锁定 FLASH 控制块*/
	FLASH_Lock();
}
//Flash读
uint16_t Flash_read(void)
{
	/* 解锁 FLASH 控制块*/
	FLASH_Unlock();
	
	*(_Flash_Parm*)((&Flash_Parm))=*(_Flash_Parm*)(Flash_Addr);
	
	/* 锁定 FLASH 控制块*/
	FLASH_Lock();
	
	return Flash_Parm_ID;
}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
