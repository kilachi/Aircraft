#ifndef __AC_AHRS_H
#define __AC_AHRS_H

#include "Aircraft_Config.h"

#define AHRS_WITH_MAG		0
#define AHRS_ONLY_MPU		1

typedef struct {
	float q[4];
	float err[3];
	float err_i[3];
	float reference_vx[3];
	float reference_vy[3];
	float reference_vz[3];
	float angle[3];
	float heading;
	float accel_ef[3];
}_ahrs;
extern _ahrs ahrs;

void ahrs_update(float a[3], float g[3], float m[3], float deltaT);
void Quart_to_euler(float *euler);
void Quart_from_euler(float *euler);
void get_accel_ef(void);

#define halfT 0.5f	 

#endif

