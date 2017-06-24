/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ×÷Õß		 £ºCyrus
 * ÃèÊö    £ºBmp085
**********************************************************************************/
#include "AC_Baro_Glitch.h"
#include "AC_Baro.h"
#include "AC_Baro_Buffer.h"

#define dt 0.01f

void baro_check_glitching(void)
{
	static u8 initialised;
	static float _last_good_alt,_last_good_vel;
	float accel_based_distance;     
	int32_t alt_projected;         
	int32_t distance_cm;            
	u8 all_ok;                   

	// exit immediately if baro is unhealthy
	if (!flag.baro_healthly) {
		flag.baro_glitching = true;
		return;
	}
		
	// if not initialised or disabled update last good alt and exit
	if (!initialised) {
		_last_good_alt = baro.alt_cm_lpf;
		_last_good_vel = _baro_get_climb_rate();
		initialised = true;
		flag.baro_glitching = false;
		return;
	}

	alt_projected = _last_good_alt + (_last_good_vel * dt);

	int32_t baro_alt = baro.alt_cm_lpf;

	if (!flag.baro_healthly) {
		flag.baro_glitching = true;
		return;
	}
		
	distance_cm = ABS(alt_projected - baro_alt);

	if (distance_cm <= 500) {
			all_ok = true;
	}else{
			accel_based_distance = 0.5f * 1500 * dt * dt;
			all_ok = (distance_cm <= accel_based_distance);
	}

	if (all_ok) {
			_last_good_alt = baro.alt_cm_lpf;
			_last_good_vel = _baro_get_climb_rate();
	}

	flag.baro_glitching = !all_ok;	
}


/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
