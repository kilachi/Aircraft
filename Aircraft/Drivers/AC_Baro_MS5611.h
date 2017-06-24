#ifndef __AC_BARO_MS5611_H
#define __AC_BARO_MS5611_H

#include "Aircraft_Config.h"

void MS5611_Init(void);
u8 MS5611_ReadData(void);
float get_ms5611_pressure(void);
float get_ms5611_temperature(void);

#define MS5611_ADDR             0xee //0x77 //

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR							0x08	//CMD_ADC_4096

#define BARO_CS	GPIOB
#define BARO_CS_PIN	GPIO_Pin_9
#define BARO_CS_H()  BARO_CS->BSRR=BARO_CS_PIN
#define BARO_CS_L()  BARO_CS->BRR =BARO_CS_PIN

#endif
