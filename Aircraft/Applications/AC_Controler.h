#ifndef __AC_CONTROLER_H
#define __AC_CONTROLER_H

#include "Aircraft_Config.h"

/*CONTROLER*/
#define MAX_CTRL_ANGLE	45.0f		//ң���ܴﵽ�����Ƕ�
#define MAX_CTRL_YAW		120.0f	//ң���ܴﵽ�����Ƕ�
#define THROTTLE_IDLING	150			//��������
#define MIN_THROTTLE		1000
#define MAX_THROTTLE		2000

typedef struct {
	
	float throttle_base;
	float throttle_out;
	
	/*ң����ԭʼֵ*/
	float thr;
	float roll;
	float pitch;
	float yaw;
	
	/*Ŀ�����ֵ*/
	float target_roll;
	float target_pitch;
	float target_yaw;
	float target_alt;
	
	int16_t pwm_output[4];
	/*��̬*/
	float angle_error[3];
	float rate_error[3];
	float rate_last_error[3];
	float inertial[3];
	float derivative[3];
	float angle_out[3];
		
	float outer_pid_kp[3];
	float outer_pid_ki[3];
	float outer_pid_kd[3];	
	float inner_pid_kp[3];
	float inner_pid_ki[3];
	float inner_pid_kd[3];	

	/*����*/
	float alt_target;
	float alt_err;
	float alt_target_vel;
	float alt_vel_err;
	float alt_accel_feedforward;
	float alt_accel_err;
	float alt_inertial;
	float alt_accel_out;
	
	float alt_pos_kp;
	float alt_vel_kp;
	float alt_accel_kp;
	float alt_accel_ki;
	float alt_accel_kd;
}_ctrl;
extern _ctrl controler;

void attitude_ctrl_update(float T);
void pid_update(float T);
void moto_output(float thr,float ctrl_output[3]);

static float get_p(u8 i, float error);
static float get_i(u8 i, float error, float dt);
static float get_d(u8 i, float error, float dt);

#endif

