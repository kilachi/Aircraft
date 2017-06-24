#ifndef __AC_ALTHOLD_H
#define __AC_ALTHOLD_H

#include "Aircraft_Config.h"

#define ALT_EXP_SPEED  ( 0.4f *get_deathzoom( (controler.thr-500), 50 ) )

void althold_run(float T);

void pos_update_z_controller(void);
void alt_to_rate_z(void);
void rate_to_accel_z(float target_vel_z);
void accel_to_throttle(float target_accel_z);

void calc_leash_length_z(void);
float calc_leash_length(float speed_cms, float accel_cms, float kP);

float althold_get_i(float error, float dt);


#endif

