/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：通信协议
**********************************************************************************/
#include "AC_DT.h" 
#include "Bsp.h"
#include "Scheduler.h"

//发送标志
_fsend f;
//发送缓存
u8 data_to_send[50];

//发送集合
void  Data_Exchange(void)
{
	static u8 cnt;
	
	static u8 user_cnt 	  = 10;
	static u8 status_cnt 	= 15;
	static u8 senser2_cnt = 30;
	static u8 rcdata_cnt 	= 40;
	static u8 motopwm_cnt	= 45;
	static u8 senser_cnt 	= 50;
	static u8 speed_cnt   = 200;
	static u8 location_cnt   = 220;
	static u8 power_cnt		=	255;
	static u8 pid_delay;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.send_senser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))
		f.send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-2))
		f.send_power = 1;		
	
	if((cnt % speed_cnt) == (speed_cnt-3))
		f.send_speed = 1;		
	
	if((cnt % location_cnt) == (location_cnt-3))
	{
		f.send_location += 1;		
	}
	
	if(++cnt>200) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,100,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)
	{
		f.send_status = 0;
		ANO_DT_Send_Status(ahrs.angle[0],ahrs.angle[1],ahrs.angle[2], 0, flag.fly_mode, flag.motor_armed);	
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_speed)
	{
		f.send_speed = 0;
//		ANO_DT_Send_Speed(wx_acc_mms2,wy_acc_mms2,accel_mms2_z);
		ANO_DT_Send_Speed(0,0,ahrs.accel_ef[2]);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_user)
	{
		f.send_user = 0;
		ANO_DT_Send_User();
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)
	{
		f.send_senser = 0;
		ANO_DT_Send_Senser( mpu.accel_lpf[0], mpu.accel_lpf[1], mpu.accel_lpf[2], 
												mpu.gyro_lpf[0], mpu.gyro_lpf[1], mpu.gyro_lpf[2], 
												0,0,0);
//		ANO_DT_Send_Senser( mpu.accel[0], mpu.accel[1], mpu.accel[2], 
//												mpu.gyro_lpf[0], mpu.gyro_lpf[1], mpu.gyro_lpf[2], 
//												compass.mag_lpf[0], compass.mag_lpf[1], compass.mag_lpf[2]);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser2)
	{
		f.send_senser2 = 0;
		ANO_DT_Send_Senser2(baro.alt_cm_lpf *100,inertial.pos_z *100);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		ANO_DT_Send_RCData(CH[CH_THR]+1000,CH[CH_YAW]+1000,CH[CH_ROLL]+1000,CH[CH_PITCH]+1000,
											 CH[CH_MODE]+1000,CH[CH_HEAD]+1000,0,
											 0,0,0);		
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		ANO_DT_Send_MotoPWM(controler.pwm_output[0],controler.pwm_output[1],controler.pwm_output[2],controler.pwm_output[3],0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
		ANO_DT_Send_Power(123,456);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1,controler.outer_pid_kp[0],controler.outer_pid_ki[0],controler.outer_pid_kd[0],
											controler.outer_pid_kp[1],controler.outer_pid_ki[1],controler.outer_pid_kd[1],
											controler.outer_pid_kp[2],controler.outer_pid_ki[2],controler.outer_pid_kd[2]);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		if(++pid_delay <= 10)
			return;
		pid_delay = 0;

		f.send_pid2 = 0;
		ANO_DT_Send_PID(2,controler.inner_pid_kp[0],controler.inner_pid_ki[0],controler.inner_pid_kd[0],
											controler.inner_pid_kp[1],controler.inner_pid_ki[1],controler.inner_pid_kd[1],
											controler.inner_pid_kp[2],controler.inner_pid_ki[2],controler.inner_pid_kd[2]);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		if(++pid_delay <= 10)
			return;
		pid_delay = 0;

		f.send_pid3 = 0;
//		ANO_DT_Send_PID(3, controler.alt_pos_kp, controler.alt_vel_kp, 0,
//											 controler.alt_accel_kp, controler.alt_accel_ki, controler.alt_accel_kd,
//		0,0,0
//		);
	}
	else if(f.send_pid4)
	{
		if(++pid_delay <= 10)
			return;
		pid_delay = 0;

		f.send_pid4 = 0;
//		ANO_DT_Send_PID(4,gps_dis_PID.kp,gps_dis_PID.ki,gps_dis_PID.kd,
//											baro_PID.kp,baro_PID.ki,baro_PID.kd,
//											0						,0						,0						);
	}
	else if(f.send_location == 2)
	{
		
		f.send_location = 0;
//		ANO_DT_Send_Location(0,0,0 *10000000,0  *10000000,0);
//		ANO_DT_Send_Location(GpsInfo.Statue,GpsInfo.SatelliteNum,(int32_t)(GpsInfo.coord_df[LAT]*100000)+GpsInfo.coord_s[LAT],(int32_t)(GpsInfo.coord_df[LON]*100000)+GpsInfo.coord_s[LON],exp_height/10);
//		ANO_DT_Send_Location(GpsInfo.Statue,GpsInfo.SatelliteNum,GpsInfo.coord[LAT]*10000000,GpsInfo.coord[LON]*10000000,0);
	}

	Usb_Hid_Send();
}
//数据应答处理
void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	u8 i;	
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
/////////////////////////////////////////////////////////////////////////////////////
	switch(*(data_buf+2))
	{
		//校准
		case 0x01:
			if(*(data_buf+4)==0X01)		
				flag.mpu_calibration = 1;
			else if(*(data_buf+4)==0X02)		
				flag.mpu_calibration = 1;
			else if(*(data_buf+4)==0X03)
			{
				flag.mpu_calibration = 1;
				flag.baro_calibration = 1;
			}else if(*(data_buf+4)==0X04)
			{
				flag.compass_calibration = 1;
			}else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
			{
				//acc_3d_calibrate_f = 1;
			}
			else if(*(data_buf+4)==0X20)
			{
				//acc_3d_step = 0; //退出，6面校准步清0
			}
		break;
			
		case 0x02:
		if(*(data_buf+4)==0X01)//发送PID
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;			
		}
		else if(*(data_buf+4)==0X02)
		{
			
		}
		else if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		else if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
//			Param_DeInit();
		}
	break;
	//接收遥控数据
	case 0x03:
		rc_in[2] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);//thr
		rc_in[3] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);//yaw
		rc_in[0] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);//roll
		rc_in[1] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);//pitch
	
  	rc_in[4] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);//mode
		rc_in[5] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
		rc_in[6] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
		rc_in[7] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
		rc_dog(1);
	break;
	//接收PID1
	case 0x10:
		controler.outer_pid_kp[0] = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5)) / 1000;
		controler.outer_pid_ki[0] = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7)) / 1000;
		controler.outer_pid_kd[0] = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9)) / 1000;

		controler.outer_pid_kp[1] = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11)) / 1000;
		controler.outer_pid_ki[1] = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13)) / 1000;
		controler.outer_pid_kd[1] = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15)) / 1000;
	
		controler.outer_pid_kp[2] = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17)) / 1000;
		controler.outer_pid_ki[2] = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19)) / 1000;
		controler.outer_pid_kd[2] = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21)) / 1000;
		Send_Check(*(data_buf+2),sum);
	break;
	case 0x11://PID2
		controler.inner_pid_kp[0] = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5)) / 1000;
		controler.inner_pid_ki[0] = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7)) / 1000;
		controler.inner_pid_kd[0] = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9)) / 1000;

		controler.inner_pid_kp[1] = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11)) / 1000;
		controler.inner_pid_ki[1] = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13)) / 1000;
		controler.inner_pid_kd[1] = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15)) / 1000;
	
		controler.inner_pid_kp[2] = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17)) / 1000;
		controler.inner_pid_ki[2] = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19)) / 1000;
		controler.inner_pid_kd[2] = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21)) / 1000;
		Send_Check(*(data_buf+2),sum);
	break;
	case 0x12://PID3
		controler.alt_pos_kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5)) / 1000;
		controler.alt_vel_kp = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7)) / 1000;
//		height_PID.kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9)) / 1000;
	
		controler.alt_accel_kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11)) / 1000;
		controler.alt_accel_ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13)) / 1000;
		controler.alt_accel_kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15)) / 1000;
	
//		gps_sp_PID.kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17)) / 1000;
//		gps_sp_PID.ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19)) / 1000;
//		gps_sp_PID.kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21)) / 1000;
//		
		Send_Check(*(data_buf+2),sum);
	break;
	case 0x13://PID4
//		gps_dis_PID.kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5)) / 1000;
//		gps_dis_PID.ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7)) / 1000;
//		gps_dis_PID.kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9)) / 1000;
//	
//		baro_PID.kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11)) / 1000;
//		baro_PID.ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13)) / 1000;
//		baro_PID.kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15)) / 1000;
//	
		Send_Check(*(data_buf+2),sum);
	flag.param_savepid = 1;
	break;
	case 0x14://PID5
		Send_Check(*(data_buf+2),sum);
	break;
	case 0x15://PID6
		Send_Check(*(data_buf+2),sum);
	break;
/////////////////////////////////////////////////////////////////////////////////////////////////
	case 0x16:

	break;
	case 0x17:

	break;
	}
}

void Send_Check(u8 head, u16 check)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check;
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	Send_Data(data_to_send,8);
}


void Send_Data(u8 *dataToSend , u8 length)
{

#ifdef DT_USE_NRF_TX
	NRF_TxPacket(data_to_send,length);
#endif
#ifdef DT_USE_NRF_RX
	NRF_TxPacket_AP(data_to_send,length);
#endif	
#if DT_USE_USART1
	Usart1_Send(data_to_send, length);
#endif
#if DT_USE_USART2
	Usart2_Send(data_to_send, length);
#endif
#ifdef DT_USE_USB_HID
	Usb_Hid_Adddata(data_to_send,length);
#endif
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);

}

void ANO_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x04;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=state;
	data_to_send[_cnt++]=sat_num;
	
	_temp2 = lon;//经度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	_temp2 = lat;//纬度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	
	_temp = (s16)(100 *back_home_angle);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);

}


void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
}
float DT_user_data[10];
void ANO_DT_Send_User()
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; //用户数据
	data_to_send[_cnt++]=0;
		
	for(u8 i=0;i<10;i++){
		_temp = (s16)DT_user_data[i];					
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
	}
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
