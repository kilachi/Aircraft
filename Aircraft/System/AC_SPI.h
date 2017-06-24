#ifndef __AC_SPI_H
#define __AC_SPI_H

#include "stm32f10x.h"

void spi1_init(void);
void spi2_init(void);
uint8_t spi1_rw(uint8_t dat);
uint8_t spi2_rw(uint8_t dat);

#endif

