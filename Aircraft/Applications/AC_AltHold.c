/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：高度控制
**********************************************************************************/
#include "AC_AltHold.h" 
#include "AC_Controler.h" 
#include "AC_InertialNav.h" 

#define MAX_ALT					1000	//CM
#define MAX_SPEED_UP 		250.0f	//CM

static float _accel_z_cms = 250;
static float _speed_up_cms = 250;
static float _speed_down_cms = -250;

static float _leash_up_z;
static float _leash_down_z;
static uint8_t recalc_leash_z = true;
static uint8_t reset_rate_to_accel_z = false;
static uint8_t freeze_ff_z = false;
static uint8_t reset_accel_to_throttle = false;

void althold_run(float T)
{		
	//怠速检测
	throttle_idling(controler.throttle_out);
	
	controler.target_alt += get_deathzoom(controler.thr - 500,50)/500.0f *MAX_SPEED_UP * 0.005f;
	controler.target_alt = LIMIT(controler.target_alt, -MAX_ALT, MAX_ALT);		
	
	//三环定高
	pos_update_z_controller();
	
	controler.throttle_out = LIMIT(controler.throttle_base + controler.alt_accel_out /*+补偿*/,0,800);
		
	if(inertial.velocity_z<60.0f){
		controler.throttle_base = controler.throttle_base*0.99f + controler.throttle_out*0.01f;
	}
}
void pos_update_z_controller(void)
{
	calc_leash_length_z();

	controler.alt_target = controler.target_alt;
	alt_to_rate_z();	
}
void alt_to_rate_z()
{
	float curr_alt = inertial.pos_z;//_inav.get_altitude();
	float linear_distance;  

	// do not let target alt get above limit
	if (MAX_ALT > 0 && controler.alt_target > MAX_ALT) {
			controler.alt_target = MAX_ALT;
	}

	// calculate altitude error
	controler.alt_err = controler.alt_target - curr_alt;

	// do not let target altitude get too far from current altitude
	if (controler.alt_err > _leash_up_z) {
			controler.alt_target = curr_alt + _leash_up_z;
			controler.alt_err = _leash_up_z;
	}
	if (controler.alt_err < -_leash_down_z) {
			controler.alt_target = curr_alt - _leash_down_z;
			controler.alt_err = -_leash_down_z;
	}

	// check kP to avoid division by zero
	if (controler.alt_pos_kp != 0.0f) {
			linear_distance = _accel_z_cms/(2.0f*controler.alt_pos_kp*controler.alt_pos_kp);
			if (controler.alt_err > 2*linear_distance ) {
					controler.alt_target_vel = safe_sqrt(2.0f*_accel_z_cms*(controler.alt_err-linear_distance));
			}else if (controler.alt_err < -2.0f*linear_distance) {
					controler.alt_target_vel = -safe_sqrt(2.0f*_accel_z_cms*(-controler.alt_err-linear_distance));
			}else{
					controler.alt_target_vel = controler.alt_err*controler.alt_pos_kp;
			}
	}else{
			controler.alt_target_vel = 0;
	}
	
	// call rate based throttle controller which will update accel based throttle controller targets
	rate_to_accel_z(controler.alt_target_vel);	
}
void rate_to_accel_z(float target_vel)
{
	float curr_vel = LIMIT(inertial.velocity_z,-500,500);//_inav.get_velocity();
	float p;                                // used to capture pid values for logging
	float desired_accel;                    // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors
	static float last_alt_target_vel;

	if (target_vel < _speed_down_cms) {
			target_vel = _speed_down_cms;
	}
	if (target_vel > _speed_up_cms) {
			target_vel = _speed_up_cms;
	}

	// reset last velocity target to current target
    if (reset_rate_to_accel_z) {
        last_alt_target_vel = target_vel;
        reset_rate_to_accel_z = false;
    }

	// feed forward desired acceleration calculation
		if (!freeze_ff_z) {
			controler.alt_accel_feedforward = (target_vel - last_alt_target_vel)/0.005f;
			} else {
			// stop the feed forward being calculated during a known discontinuity
			freeze_ff_z = false;
		}


	// store this iteration's velocities for the next iteration
	last_alt_target_vel = target_vel;

	// reset velocity error and filter if this controller has just been engaged
	if (reset_rate_to_accel_z) {
			// Reset Filter
			controler.alt_vel_err = 0;
//			_vel_error_filter.reset(0);
			desired_accel = 0;
			reset_rate_to_accel_z = false;
	} else {	
			// calculate rate error and filter with cut off frequency of 2 Hz
			controler.alt_vel_err = 0.82f*controler.alt_vel_err + 0.18f*(target_vel - curr_vel);
	}

	// calculate p
	p = controler.alt_vel_kp * controler.alt_vel_err;

	// consolidate and constrain target acceleration
	desired_accel = controler.alt_accel_feedforward + p;
	desired_accel = LIMIT(desired_accel, -1300, 1300);

	// set target for accel based throttle controller
	accel_to_throttle(desired_accel);	
}
void accel_to_throttle(float target_accel_z)
{
	float z_accel_meas;         // actual acceleration
	int32_t p,i;              // used to capture pid values for logging

	// Calculate Earth Frame Z acceleration
	z_accel_meas = ahrs.accel_ef[2];

	// reset target altitude if this controller has just been engaged
	if (reset_accel_to_throttle) {
			// Reset Filter
			controler.alt_vel_err = 0;
//			_accel_error_filter.reset(0);
			reset_accel_to_throttle = 0;
	} else {
			// calculate accel error and Filter with fc = 2 Hz
			controler.alt_accel_err = 0.94f*controler.alt_vel_err + 0.06f*(LIMIT(target_accel_z - z_accel_meas, -2000, 2000));
	}

	// separately calculate p, i, d values for logging
	p = controler.alt_accel_kp*controler.alt_accel_err;
	i = controler.alt_inertial;
	// get i term
	if((get_deathzoom(controler.thr - 500,50)==0)||(i>0&&controler.alt_vel_err<0) || (i<0&&controler.alt_vel_err>0)){
		i = althold_get_i(controler.alt_vel_err,0.005f);
	}
	controler.alt_accel_out = p + i;
}


//=======================================================================================
//计算束缚
void calc_leash_length_z(void)
{
	if (recalc_leash_z) {
			_leash_up_z = calc_leash_length(_speed_up_cms, _accel_z_cms, controler.alt_pos_kp);
			_leash_down_z = calc_leash_length(-_speed_down_cms, _accel_z_cms, controler.alt_pos_kp);
			recalc_leash_z = 0;
	}	
}
#define POSCONTROL_ACCELERATION_MIN             50.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
#define POSCONTROL_LEASH_LENGTH_MIN             100.0f  // minimum leash lengths in cm
float calc_leash_length(float speed_cms, float accel_cms, float kP)
{
    float leash_length;

    // sanity check acceleration and avoid divide by zero
    if (accel_cms <= 0.0f) {
        accel_cms = POSCONTROL_ACCELERATION_MIN;
    }

    // avoid divide by zero
    if (kP <= 0.0f) {
        return POSCONTROL_LEASH_LENGTH_MIN;
    }

    // calculate leash length
    if(speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f*kP*kP)) + (speed_cms*speed_cms / (2.0f*accel_cms));
    }

    // ensure leash is at least 1m long
    if( leash_length < POSCONTROL_LEASH_LENGTH_MIN ) {
        leash_length = POSCONTROL_LEASH_LENGTH_MIN;
    }

    return leash_length;
}

#define _imax	100
float althold_get_i(float error, float dt)
{
//	static float _integrator;
    if((controler.alt_accel_ki != 0) && (dt != 0)) {
        controler.alt_inertial += ((float)error * controler.alt_accel_ki) * dt;
        if (controler.alt_inertial < -_imax) {
            controler.alt_inertial = -_imax;
        } else if (controler.alt_inertial > _imax) {
            controler.alt_inertial = _imax;
        }
        return controler.alt_inertial;
    }
    return 0;
}


/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
