/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ×÷Õß		 £ºCyrus
 * ÃèÊö    £º×ËÌ¬
**********************************************************************************/
#include "AC_InertialNav.h" 
#include "AC_Baro.h"

_inertial_nav inertial;

#define inertial_time_constant_z	3.5f
#define k1_z	3.0f/inertial_time_constant_z
#define k2_z	3.0f/(inertial_time_constant_z*inertial_time_constant_z)
#define k3_z	1.0f/(inertial_time_constant_z*inertial_time_constant_z*inertial_time_constant_z)

void inertial_nav(float T)
{	
	if(flag.mpu_calibration||flag.baro_calibration||!flag.baro_healthly)
		return;
	
	get_accel_ef();
	
	inertial.acc_correct_z += inertial.pos_err_z *k3_z *T;

	inertial.velocity_z += inertial.pos_err_z *k2_z *T;
	
	inertial.pos_correct_z += inertial.pos_err_z *k1_z *T;

	inertial.velocity_increase = (ahrs.accel_ef[2] + inertial.acc_correct_z) *T;

	inertial.pos_base_z += (inertial.velocity_z + inertial.velocity_increase *0.5f) *T;

	inertial.velocity_z += inertial.velocity_increase;

	inertial.pos_z = inertial.pos_base_z + inertial.pos_correct_z;
	
	baro_buffer_push_back(inertial.pos_base_z);
	/*
	DT_user_data[0] = baro.alt_cm_lpf;
	DT_user_data[1] = inertial.pos_z;
	DT_user_data[2] = inertial.velocity_z;
	DT_user_data[3] = ahrs.accel_ef[2];	
	DT_user_data[4] = controler.throttle_out;	
	DT_user_data[5] = controler.throttle_base;	
	DT_user_data[6] = controler.target_alt;
	*/
}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
