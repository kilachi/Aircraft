/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 文件名  ：CY_Param.c
 * 描述    ：Param
**********************************************************************************/
#include "AC_Param.h"

_param param;

void Param_savecheck(void)
{
	if(flag.param_savepid||flag.param_saveoffset){
		Param_save();
		flag.param_savepid=0;
		flag.param_saveoffset=0;
	}
}
//准备参数
void Param_read(void)
{
	memcpy(&mpu.accel_offset,&param.accel_offset,sizeof(int16_t)*3);
	memcpy(&mpu.gyro_offset,&param.gyro_offset,sizeof(int16_t)*3);

	memcpy(&controler.outer_pid_kp,&param.outer_pid_kp,sizeof(float)*3);
//	memcpy(&controler.outer_pid_ki,&param.outer_pid_ki,sizeof(float)*3);
	memcpy(&controler.outer_pid_kd,&param.outer_pid_kd,sizeof(float)*3);

	memcpy(&controler.inner_pid_kp,&param.inner_pid_kp,sizeof(float)*3);
	memcpy(&controler.inner_pid_ki,&param.inner_pid_ki,sizeof(float)*3);
	memcpy(&controler.inner_pid_kd,&param.inner_pid_kd,sizeof(float)*3);
	
	controler.alt_pos_kp   = param.alt_pos_kp;
	controler.alt_vel_kp   = param.alt_vel_kp;
	controler.alt_accel_kp = param.alt_accel_kp;
	controler.alt_accel_ki = param.alt_accel_ki;
	controler.alt_accel_kd = param.alt_accel_kd;	
}
void Param_save(void)
{
	memcpy(&param.accel_offset,&mpu.accel_offset,sizeof(float)*3);
	memcpy(&param.gyro_offset,&mpu.gyro_offset,sizeof(float)*3);

	
	memcpy(&param.outer_pid_kp,&controler.outer_pid_kp,sizeof(float)*3);
//	memcpy(&param.outer_pid_ki,&controler.outer_pid_ki,sizeof(float)*3);
	memcpy(&param.outer_pid_kd,&controler.outer_pid_kd,sizeof(float)*3);

	memcpy(&param.inner_pid_kp,&controler.inner_pid_kp,sizeof(float)*3);
	memcpy(&param.inner_pid_ki,&controler.inner_pid_ki,sizeof(float)*3);
	memcpy(&param.inner_pid_kd,&controler.inner_pid_kd,sizeof(float)*3);
	
	param.alt_pos_kp   = controler.alt_pos_kp;
	param.alt_vel_kp   = controler.alt_vel_kp;
	param.alt_accel_kp = controler.alt_accel_kp;
	param.alt_accel_ki = controler.alt_accel_ki;
	param.alt_accel_kd = controler.alt_accel_kd;
	
	Flash_write();
}
//参数初始化
void Param_Init(void)
{
	if(Flash_read() != Get_CPU_ID())//从未初始化
	{
		Param_DeInit();
	}
	Param_read();
}

//恢复默认
void Param_DeInit(void)
{
	param.CPU_ID = Get_CPU_ID();

	Flash_write();
}

uint16_t Get_CPU_ID(void)
{
	u32 CpuID[3];
	CpuID[0]=*(vu32*)(0x1ffff7e8);
	CpuID[1]=*(vu32*)(0x1ffff7ec);
	CpuID[2]=*(vu32*)(0x1ffff7f0);
	return ((CpuID[0]>>1)+(CpuID[1]>>2)+(CpuID[2]>>3));
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
