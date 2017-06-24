#ifndef __AC_PARAM_H
#define __AC_PARAM_H

#include "Aircraft_Config.h"

typedef struct
{
	uint16_t CPU_ID;
	
	float outer_pid_kp[3];
//	float outer_pid_ki[3];
	float outer_pid_kd[3];
	
	float inner_pid_kp[3];
	float inner_pid_ki[3];
	float inner_pid_kd[3];	
	
	int16_t	accel_offset[3];
	int16_t gyro_offset[3];
//	int16_t mag_offset[3];
	
	float alt_pos_kp;
	float alt_vel_kp;
	float alt_accel_kp;
	float alt_accel_ki;
	float alt_accel_kd;		
}_param;


extern _param param;

void Param_Init(void);
void Param_DeInit(void);
void Param_save(void);
void Param_read(void);
void Param_savecheck(void);
uint16_t Get_CPU_ID(void);


#endif

