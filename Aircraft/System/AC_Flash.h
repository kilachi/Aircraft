#ifndef __AC_FLASH_H
#define __AC_FLASH_H

#include "Aircraft_Config.h"

void Flash_init(void);
uint16_t Flash_read(void);
void Flash_write(void);

#define Flash_Addr 0x800FC00 //63*1024

#endif

