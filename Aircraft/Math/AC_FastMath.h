#ifndef __AC_FASTMATH_H
#define __AC_FASTMATH_H

#include "Aircraft_Config.h"

#define ACCEL_TO_9_8G		0.00239f
#define GYRO_TO_ANGLE 	0.06103f				//角速度变成度   此参数对应陀螺2000度每秒
#define GYRO_TO_RADIAN	0.0010653f				//角速度变成弧度	此参数对应陀螺2000度每秒
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度
#define RADIAN_TO_ANGLE 57.2890265f //*0.01745 = /57.3	弧度转角度

#define ABS(x)						  ( (x)>0?(x):-(x) )
#define MAX(x, y)						(((x) < (y)) ? (y) : (x))
#define MIN(a, b)						((a) < (b) ? (a) : (b))
#define LIMIT( x,min,max )  ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

float inv_sqrtf(float x);//快速开根
float fast_sqrtf(float number);
float safe_sqrt(float v);
float fast_pow(float a);//计算浮点数平方
float fast_atan2(float y, float x);//atan2
float to_180_degrees(float x);
void simple_3d_trans(float *ref, float *in, float *out);

#define M_PI 3.1415926535898f

#endif

