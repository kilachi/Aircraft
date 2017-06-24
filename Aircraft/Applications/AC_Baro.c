/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：Bmp085
**********************************************************************************/
#include "AC_Baro.h"
#include "AC_Baro_MS5611.h"
#include "AC_InertialNav.h" 

_baro baro;

void baro_init(void){
	MS5611_Init();
}

void baro_update(float T)
{
	if(MS5611_ReadData()){
		baro_get_altitude(baro.pressure, baro.temperature_offset);		
		baro_correct(baro.alt_cm_lpf,T);
	}
}
void baro_correct(float alt,float T)
{	
	//保存历史高度和时间
	_climb_rate_filter_update(alt, sysTick.sysTickMs);
	//更新毛刺标志
	baro_check_glitching();
	
	static uint8_t first_reads = 0;

	// discard first 10 reads but perform some initialisation
	if( first_reads <= 10 ) {
			baro_set_alt(alt);
			first_reads++;
	}
	//计算高度误差
	if (flag.baro_glitching) {
		//有毛刺，降低err
		inertial.pos_err_z *= 0.89715f;
	}else{
		//本次没毛刺，但上次有毛刺，重来
		if(baro.glitching) {
			baro_set_alt(alt);
			inertial.pos_err_z = 0.0f;
			//buffer_clear
		}else{//没毛刺
			float hist_position_base_z;
			//buffer满了
			if(_hist_position_estimate_z_is_full()){
				hist_position_base_z = baro_get_peek(0);
				//不用buffer
//				hist_position_base_z = inertial.pos_base_z;
			}else{
				hist_position_base_z = inertial.pos_base_z;
			}
				inertial.pos_err_z = alt - (hist_position_base_z + inertial.pos_correct_z);
			}	
	}
	baro.glitching = flag.baro_glitching;
}

float baro_get_altitude(float pressure, float _ground_temperature)
{
	static double x,p;

	baro.temperature = get_ms5611_temperature()*0.01f;
	baro.pressure = get_ms5611_pressure()*0.01f;
	
	if(flag.baro_calibration || (baro.pressure_offset==0.0000f&&baro.temperature_offset==0.0000f)){
		baro_calibration();
		flag.baro_healthly = 0;
		return 0;
	}else{
			flag.baro_healthly = 1;
	}
	// on faster CPUs use a more exact calculation
	float scaling = pressure / baro.pressure_offset;
	float temp    = _ground_temperature + 273.15f;
	// This is an exact calculation that is within +-2.5m of the standard atmosphere tables
	// in the troposphere (up to 11,000 m amsl).
	baro.alt_cm = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)))  *100.0f;		

	// sanity check altitude
	if (isnan(baro.alt_cm) || isinf(baro.alt_cm)) {
		flag.baro_healthly = false;
	} else {
		baro.alt_cm_lpf = get_kalman_filter(baro.alt_cm,&x,&p,0.1f,20.1f);	
		flag.baro_healthly = true;
	}
	
	return baro.alt_cm_lpf;
}
void baro_calibration(void)
{
	static u8 reset_cnt;
	if(reset_cnt<200){
		reset_cnt++;
	}else{
			reset_cnt = 0;
			baro.alt_cm_lpf = 0;
			flag.baro_calibration = 0;	
			baro_reset();	
	}
}
void baro_reset(void)
{
	baro_set_alt(baro.alt_cm_lpf);
	baro.pressure_offset = baro.pressure;			
	baro.temperature_offset = baro.temperature;			
}
void baro_set_alt(float new_altitude)
{
	inertial.pos_base_z = new_altitude;
	inertial.pos_z = new_altitude; // _position = _position_base + _position_correction
	inertial.pos_correct_z = 0;
//	inertial.acc_correct_z = 0;
//	inertial.velocity_z = 0;
//	inertial.velocity_increase = 0;
	baro_buffer_clear(); // reset z history to avoid fake z velocity at next baro calibration (next rearm)
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
