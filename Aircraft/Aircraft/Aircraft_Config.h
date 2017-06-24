#ifndef __AIRCRAFT_CONFIG_H
#define __AIRCRAFT_CONFIG_H

#include "stm32f10x.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Bsp.h"
#include "AC_SPI.h"
#include "AC_Mpu6000.h"
#include "AC_LED.h" 
#include "AC_Nrf24l01.h"
#include "AC_Baro_MS5611.h"
#include "AC_Pwm.h" 
#include "AC_Flash.h"
#include "AC_hid.h"

#include "Scheduler.h"
#include "AC_Timer.h"
#include "AC_FastMath.h" 
#include "AC_Filter.h" 
#include "AC_Mpu.h"
#include "AC_AHRS.h" 
#include "AC_Baro.h"
#include "AC_Baro_Buffer.h"
#include "AC_Baro_Glitch.h"
#include "AC_InertialNav.h" 
#include "AC_DT.h" 
#include "AC_RcData.h"
#include "AC_Controler.h" 
#include "AC_AltHold.h" 
#include "AC_Throttle.h"
#include "AC_Param.h"


typedef struct {
	
	uint8_t motor_armed : 1;
	uint8_t motor_flying : 1;
	
	uint8_t fly_mode : 2;
	uint8_t thr_low : 1;
	
	uint8_t	mpu_calibration : 1;
	uint8_t mpu_healthly : 1;
	
	uint8_t compass_calibration : 1;
	uint8_t compass_healthly : 1;
	
	uint8_t baro_calibration : 1;
	uint8_t baro_healthly : 1;
	uint8_t baro_glitching : 1;
	
	uint8_t param_savepid : 1;
	uint8_t param_saveoffset : 1;	
	
	uint8_t close_ctrl_pwm : 1;
}_flag;
extern _flag flag;
	
#define true	1
#define false 0
	
#endif

