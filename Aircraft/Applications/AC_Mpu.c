/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：Mpu6050
**********************************************************************************/
#include "AC_Mpu.h"
#include "AC_Mpu6000.h"

_mpu mpu;

/*MPU*/
#define MPU_OFFSET_TIME		1500	//*2=ms
#define MPU_FILTER_TYPE		1			//0为滑动滤波 1为低通滤波
#define MPU_FILTER_SIZE		10

void mpu_update(void)
{
	int16_t* mpudata;
	mpudata = mpu6000_read();
	
	mpu.accel[0] = mpudata[0] - mpu.accel_offset[0];
	mpu.accel[1] = mpudata[1] - mpu.accel_offset[1];
	mpu.accel[2] = mpudata[2] - mpu.accel_offset[2];
	mpu.gyro[0]  = mpudata[3] - mpu.gyro_offset[0];
	mpu.gyro[1]  = mpudata[4] - mpu.gyro_offset[1];
	mpu.gyro[2]  = mpudata[5] - mpu.gyro_offset[2];		
	mpu.temperature = mpudata[6]/340.0f + 36.5f;;

	mpu_calibration();	
	
	mpu_filter();

	for(u8 i=0;i<3;i++){
		mpu.gyro_deg[i] = mpu.gyro_lpf[i] *GYRO_TO_RADIAN;		
	}
}

void mpu_filter(void)
{
#if MPU_FILTER_TYPE
	u8 i;
	static u8 first = 1;
	static u8 gyro_index[3];
	static float gyro_buff[3][MPU_FILTER_SIZE];
	static float accel_2nd_factor[3][3];
	static float accel_2nd_last_data[3][2];
	if(first){
		for(i=0;i<3;i++){
			get_2nd_factor(0.002f,10.0f,accel_2nd_factor[i]);
		}
		first = 0;
	}
	for(i=0;i<3;i++){
		mpu.accel_lpf[i]=get_2nd_filter(mpu.accel[i],accel_2nd_factor[i],accel_2nd_last_data[i]) *ACCEL_TO_9_8G;
	}
	for(i=0;i<3;i++){
		mpu.gyro_lpf[i]=get_average(mpu.gyro[i],gyro_buff[i],&gyro_index[i],MPU_FILTER_SIZE);
	}
/*	
	for(i=0;i<3;i++){
		static int16_t gyro_last_data[3];
		mpu.gyro_lpf[i]=get_1nd_filter(mpu.gyro[i],&gyro_last_data[i],0.386f);
	}	
*/	
#else
	static float accel_buff[3][MPU_FILTER_SIZE],gyro_buff[3][MPU_FILTER_SIZE];
	static uint8_t i,accel_index[3],gyro_index[3];
	for(i=0;i<3;i++){
		mpu.accel_lpf[i]=get_average(mpu.accel[i],accel_buff[i],&accel_index[i],MPU_FILTER_SIZE) *ACCEL_TO_9_8G;
	}
	for(i=0;i<3;i++){
		mpu.gyro_lpf[i]=get_average(mpu.gyro[i],gyro_buff[i],&gyro_index[i],MPU_FILTER_SIZE);
	}	
#endif
}
void mpu_calibration(void)
{
	if(!flag.mpu_calibration || flag.motor_armed)
		return;
	
	static uint16_t cnt;
	static int32_t acctemp[3],gyrotemp[3];
	if(cnt==0)
	{
		mpu.accel_offset[0]=mpu.accel_offset[1]=mpu.accel_offset[2]=0;
		acctemp[0]=acctemp[1]=acctemp[2]=0;
		mpu.gyro_offset[0]=mpu.gyro_offset[1]=mpu.gyro_offset[2]=0;
		gyrotemp[0]=gyrotemp[1]=gyrotemp[2]=0;
		cnt=1;
		return;
	}
	acctemp[0] +=mpu.accel[0];
	acctemp[1] +=mpu.accel[1];
	acctemp[2] +=mpu.accel[2];

	gyrotemp[0] +=mpu.gyro[0];
	gyrotemp[1] +=mpu.gyro[1];
	gyrotemp[2] +=mpu.gyro[2];
	
	if(cnt >= MPU_OFFSET_TIME)
	{
		mpu.accel_offset[0] = acctemp[0]/cnt;
		mpu.accel_offset[1] = acctemp[1]/cnt;
		mpu.accel_offset[2] = acctemp[2]/cnt- 4096;		

		mpu.gyro_offset[0] = gyrotemp[0]/cnt;
		mpu.gyro_offset[1] = gyrotemp[1]/cnt;
		mpu.gyro_offset[2] = gyrotemp[2]/cnt;		

		cnt = 0;
		flag.mpu_calibration = 0;
		flag.param_saveoffset = 1;
		return;
	}
	cnt++;	
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
