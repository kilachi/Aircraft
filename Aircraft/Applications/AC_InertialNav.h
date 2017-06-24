#ifndef __CY_IMU_H
#define __CY_IMU_H

#include "Aircraft_Config.h"

typedef struct {
	float acc_correct_z;	
	
	float velocity_z;
	float velocity_increase;	
	
	float pos_z;
	float pos_base_z;
	float pos_err_z;
	float pos_correct_z;
}_inertial_nav;
extern _inertial_nav inertial;

void inertial_nav(float T);

#endif

