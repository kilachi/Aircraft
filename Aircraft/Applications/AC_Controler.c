/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * ����    ��PID
**********************************************************************************/
#include "AC_Controler.h" 
#include "AC_AltHold.h" 
#include "AC_RcData.h"
#include "AC_AHRS.h" 
#include "AC_Mpu.h"

_ctrl controler;

#define ff	1.0f
static float feedforward[3];
static int16_t pid_smooth[3];

void attitude_ctrl_update(float T)
{
	//δ׼����
	if(!flag.motor_armed || !flag.motor_flying){
		//��������
		controler.target_yaw=ahrs.angle[2];		
		controler.target_alt = inertial.pos_z;
		//������������
		controler.inertial[0]=controler.inertial[1]=controler.inertial[2]=0;
		feedforward[0]=feedforward[1]=feedforward[2]=0;
		controler.alt_inertial = 0;
		pid_smooth[0]=pid_smooth[1]=pid_smooth[2]=0;
		
		flag.close_ctrl_pwm = 0;
		if(!flag.motor_armed){
			controler.throttle_base = 0;
			controler.throttle_out = 0;
			set_motor_out_all(0);			
		}
		return;
	}
	
	controler.target_roll = MAX_CTRL_ANGLE *get_deathzoom(controler.roll - 500,10)/500.0f;
	controler.target_pitch = -MAX_CTRL_ANGLE *get_deathzoom(controler.pitch - 500,10)/500.0f;

	controler.target_yaw += get_deathzoom(CH[CH_YAW] - 500,50)/500.0f *MAX_CTRL_YAW *T;
	controler.target_yaw = to_180_degrees(controler.target_yaw);//ת[-180.0,+180.0]					

	controler.angle_error[0] = controler.target_roll - ahrs.angle[0];
	controler.angle_error[1] = controler.target_pitch - ahrs.angle[1];
	controler.angle_error[2] = -to_180_degrees(controler.target_yaw - ahrs.angle[2]);

	pid_update(T);
	
	//������̬���
	if(flag.close_ctrl_pwm){
		float anele_out_to_zero[3] = {0,0,0};
		moto_output(controler.throttle_out,anele_out_to_zero);	
	}else{
		moto_output(controler.throttle_out,controler.angle_out);
	}
}

void pid_update(float T)
{
	//�⻷
	controler.rate_error[0] = controler.angle_error[0]*controler.outer_pid_kp[0] - mpu.gyro_deg[1] *100.0f;
	controler.rate_error[1] = controler.angle_error[1]*controler.outer_pid_kp[1] - mpu.gyro_deg[0] *100.0f;
	controler.rate_error[2] = controler.angle_error[2]*controler.outer_pid_kp[2] - mpu.gyro_deg[2] *100.0f;

	//�ڻ�
	for(uint8_t i=0;i<3;i++){				
		feedforward[i] = feedforward[i]*0.86f + controler.angle_error[i]*ff*0.14f;
		controler.angle_out[i] = get_p(i,controler.rate_error[i])  + 
														 get_i(i,controler.rate_error[i],T)+ 
														 get_d(i,controler.rate_error[i],T)+
														 feedforward[i];		
	}
}

void moto_output(float thr,float ctrl_output[3])
{
	u8 i;
	int16_t motor[4];	
	
	//ƽ�����
	for(i=0;i<3;i++){
		pid_smooth[i]	+= LIMIT((int16_t)(ctrl_output[i]-pid_smooth[i]), -50, 50);
	}
//	thr_smooth += LIMIT((int16_t)(thr-thr_smooth), -60, 60);
	
	//��
	motor[0] = thr + pid_smooth[0] + pid_smooth[1] + pid_smooth[2]; //M1	ǰ��
	motor[1] = thr + pid_smooth[0] - pid_smooth[1] - pid_smooth[2]; //M2	����
	motor[2] = thr - pid_smooth[0] - pid_smooth[1] + pid_smooth[2]; //M3	����
	motor[3] = thr - pid_smooth[0] + pid_smooth[1] - pid_smooth[2]; //M4	ǰ��		
		
	for(i=0;i<4;i++){
		if(flag.fly_mode == MODE_Stabilize){
			controler.pwm_output[i] = LIMIT(motor[i], 0, 1000);
		}else{
			controler.pwm_output[i] = LIMIT(motor[i], THROTTLE_IDLING, 1000);			
		}
	}
	if(flag.motor_armed){
		set_motor_out(controler.pwm_output);
	}else{
		set_motor_out_all(0);
	}
}

static float get_p(u8 i, float error)
{
	return controler.inner_pid_kp[i]*error;
}
static float get_i(u8 i, float error, float dt)
{
//	if((controler.inertial[i]>=0&&error<0)||(controler.inertial[i]<=0&&error>0)){
		controler.inertial[i] += controler.inner_pid_ki[i]*error*dt;
		controler.inertial[i] = LIMIT(controler.inertial[i], -200.0f, 200.0f);	
//	}
	return controler.inertial[i];
}
static const float  _filter=7.9577e-3;// ΢����ĵ�ͨ�˲�����ֹƵ��
static float get_d(u8 i, float error, float dt)
{
	if((dt==0)||(controler.inner_pid_kd[i]==0)){
		return 0;
	}
	float derivative;
	if (isnan(controler.derivative[i])) {		
		derivative = 0;
		controler.derivative[i] = 0;
	} else {
		derivative = (error -controler.rate_last_error[i]) / dt;
	}
	// һ����ɢ��ͨ�˲��������͸�Ƶ�����Կ������ĸ���
  derivative = controler.derivative[i] + (dt / ( _filter + dt)) * (derivative - controler.derivative[i]);

	controler.rate_last_error[i]	= error;
	controler.derivative[i] = derivative;
	
	return  controler.inner_pid_kd[i] * derivative;
}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
