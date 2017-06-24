/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_Scheduler.c
 * 描述    ：任务调度
**********************************************************************************/
#include "Aircraft_Config.h"

void fast_loop(void)
{
	/*1ms~1000Hz*/
	if(sysTick.loop_1ms>=1){sysTick.loop_1ms=0;
		NRF_CheckEvent();
		Data_Exchange();
	}	
	
	/*2ms~500Hz*/
	if(sysTick.loop_2ms>=2){sysTick.loop_2ms=0;
		mpu_update();
		ahrs_update(mpu.accel_lpf, mpu.gyro_deg, 0, 0.002f);
		inertial_nav(0.002f);
		attitude_ctrl_update(0.002f);
	}

	/*5ms~200Hz*/
	if(sysTick.loop_5ms>=5){sysTick.loop_5ms=0;
		rc_update(rc_in,0.005f);
		throttle_update(0.005f);
	}
	
	/*10ms~100Hz*/
	if(sysTick.loop_10ms>=10){sysTick.loop_10ms=0;
		baro_update(0.01f);
	}	

	/*50ms~20Hz*/
	if(sysTick.loop_50ms>=50){sysTick.loop_50ms=0;
		led_run(1);
		Param_savecheck();
	}		

}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
