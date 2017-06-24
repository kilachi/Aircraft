#ifndef __AC_MPU_H
#define __AC_MPU_H

#include "Aircraft_Config.h"

/*
#define MPU_OFFSET_TIME	1500	// *2=ms
#define MPU_FILTER_SIZE	10
#define MPU_FILTER_TYPE	1
*/

typedef struct {
	
	int16_t accel[3];
	int16_t gyro[3];
	
	int16_t accel_offset[3];
	int16_t gyro_offset[3];	
	
	float temperature;	
	
	float accel_lpf[3];
	float gyro_lpf[3];
	float gyro_deg[3];
}_mpu;
extern _mpu mpu;

void mpu_update(void);
void mpu_calibration(void);
void mpu_filter(void);

#endif
