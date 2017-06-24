#ifndef _AC_RCDATA_H
#define _AC_RCDATA_H

#include "Aircraft_Config.h"

//#define CH_NUM			6	//接收机通道数量
#define MODE_Stabilize		0
#define MODE_Hold					1
#define MODE_Baro					2
#define MODE_Ultra				3
#define CH_OFFSET 0

#define THROTTLE_AT9			1
#if	THROTTLE_AT9	
#define CH_NUM	9	//接收机通道数量
enum{
	CH_ROLL,
	CH_PITCH,
	CH_THR,
	CH_YAW,
	CH_MODE,
	CH_HEAD,
};
#endif


extern uint16_t rc_in[8];
extern s16 CH[CH_NUM];

void rc_update(u16 data[8],float T);
void rc_parse(float T);
void rc_setmode(float T);
void rc_dog(u8 ch_mode); //400ms内必须调用一次


#endif

