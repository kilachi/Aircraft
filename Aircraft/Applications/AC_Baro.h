#ifndef __AC_BARO_H
#define __AC_BARO_H

#include "Aircraft_Config.h"
#include "AC_Baro_Buffer.h"
#include "AC_Baro_Glitch.h"

typedef struct {
	
	float alt_cm;
	float alt_cm_lpf;
	
	float speed;
	float speed_lpf;
	float temperature;
	float pressure;
	float pressure_offset;
	float temperature_offset;
		
	u8 glitching;
}_baro;
extern _baro baro;

void baro_init(void);
void baro_update(float T);
void baro_correct(float alt,float T);
float baro_get_altitude(float pressure, float _ground_temperature);
void baro_set_alt(float new_altitude);
void baro_calibration(void);
void baro_reset(void);
#endif
