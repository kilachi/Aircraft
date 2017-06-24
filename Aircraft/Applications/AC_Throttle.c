/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：接收数据
**********************************************************************************/
#include "AC_Throttle.h"

void throttle_update(float T)
{
	switch(flag.fly_mode)
	{
		case	MODE_Stabilize:
			stabilize_run(T);
		break;
		case	MODE_Baro:		
			if(throttle_check()){
				althold_run(T);
			}
		break;
		case	MODE_Ultra:
			if(throttle_check()){
				althold_run(T);
			}
		break;
		case	MODE_Hold:
			if(throttle_check()){
				throttle_idling(controler.throttle_out);
				rate_to_accel_z(ALT_EXP_SPEED);
				controler.throttle_out = LIMIT(controler.throttle_base + controler.alt_accel_out /*+补偿*/,0,800);					
				if(inertial.velocity_z<60.0f){
					controler.throttle_base = controler.throttle_base*0.99f + controler.throttle_out*0.01f;
				}

			}
		break;	
	}	
}
u8 throttle_check(void)
{
	//未准备飞
	if(!flag.motor_armed || !flag.motor_flying){
		static u16 timeout;
		if(flag.motor_armed){
			//怠速
			controler.throttle_base = THROTTLE_IDLING;
			set_motor_out_all(THROTTLE_IDLING);
			//5S无操作上锁
			if(++timeout >= 1000){
				timeout = 0;
				flag.motor_armed = 0;
			}
			//起飞检测
			if(controler.thr > 600){
				if(flag.baro_healthly){
					flag.motor_flying = 1;
				}else
				{
					flag.motor_armed =0;
				}
			}
			if(!flag.motor_flying){
				baro_reset();
			}
		}else{
			timeout = 0;
		}
		return 0;
	}else{
		return 1;
	}
}
void stabilize_run(float T)
{
	if(!flag.motor_armed){
		return;
	}
	controler.throttle_out = LIMIT(controler.thr,0,800);
	
	if(controler.thr > 10){
		flag.motor_flying = 1;
	}else{
		flag.motor_flying = 0;
		set_motor_out_all(0);			
	}
}
void throttle_idling(float thr)//5ms
{
	static u16 cnt;	
	if(flag.motor_armed && flag.motor_flying && flag.thr_low && (thr<=200)){
			if(++cnt >= 600){//3S
				cnt = 0;
				flag.motor_armed = 0;
				flag.motor_flying= 0;
			}
//			set_motor_out_all(THROTTLE_IDLING);
			flag.close_ctrl_pwm = 1;
	}else{
		cnt = 0;
		flag.close_ctrl_pwm = 0;
	}
}	
void throttle_channel(void)
{
	controler.thr = CH[CH_THR];
	controler.roll = CH[CH_ROLL];
	controler.pitch = CH[CH_PITCH];
	controler.yaw = CH[CH_YAW];	
	
	if(controler.thr < 100)	flag.thr_low = 1;

#if	THROTTLE_AT9	
	if(CH[CH_MODE]<50)					throttle_setmode(MODE_Stabilize);
	else if(CH[CH_MODE]<250)		throttle_setmode(MODE_Baro);
	else if(CH[CH_MODE]<450)		throttle_setmode(MODE_Hold);
	else if(CH[CH_MODE]<650)		throttle_setmode(MODE_Hold);	
	else if(CH[CH_MODE]<850)		throttle_setmode(MODE_Hold);	
	else if(CH[CH_MODE]>950)		throttle_setmode(MODE_Hold);	
		
#endif
#if	THROTTLE_AT6	
	if(CH[CH_MODE]<250)					throttle_setmode(MODE_Stabilize);
	else if(CH[CH_MODE]<500)		throttle_setmode(MODE_Baro);
	else if(CH[CH_MODE]<750)		throttle_setmode(MODE_Ultra);
	else if(CH[CH_MODE]>750)		throttle_setmode(MODE_Hold);	
#endif
}

void throttle_setmode(u8 mode)
{
	static u8 last_fly_mode = 0xFF;
	
	flag.fly_mode =mode;
	if(last_fly_mode == flag.fly_mode)
		return;
	
	switch(flag.fly_mode)
	{
		case	MODE_Stabilize:
			if(!flag.motor_flying){
				flag.motor_armed = 0;
				set_motor_out_all(0);
			}
		break;
		case	MODE_Baro:			
			controler.target_alt = inertial.pos_z;			
			controler.throttle_base = controler.throttle_out;					
			if(!flag.motor_flying ){
				flag.motor_armed = 0;
				flag.motor_flying = 0;		
				set_motor_out_all(0);				
			}
		break;
		case	MODE_Ultra:
			controler.target_alt = inertial.pos_z;			
			controler.throttle_base = controler.throttle_out;					
			if(!flag.motor_flying ){
				flag.motor_armed = 0;
				flag.motor_flying = 0;					
				set_motor_out_all(0);				
			}			
		break;
		case	MODE_Hold:
			controler.target_alt = inertial.pos_z;			
			controler.throttle_base = controler.throttle_out;					
			if(!flag.motor_flying ){
				flag.motor_armed = 0;
				flag.motor_flying = 0;					
				set_motor_out_all(0);				
			}			
		break;	
	}			
	last_fly_mode = flag.fly_mode;
}


/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
