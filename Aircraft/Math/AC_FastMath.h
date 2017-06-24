#ifndef __AC_FASTMATH_H
#define __AC_FASTMATH_H

#include "Aircraft_Config.h"

#define ACCEL_TO_9_8G		0.00239f
#define GYRO_TO_ANGLE 	0.06103f				//���ٶȱ�ɶ�   �˲�����Ӧ����2000��ÿ��
#define GYRO_TO_RADIAN	0.0010653f				//���ٶȱ�ɻ���	�˲�����Ӧ����2000��ÿ��
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	�Ƕ�ת����
#define RADIAN_TO_ANGLE 57.2890265f //*0.01745 = /57.3	����ת�Ƕ�

#define ABS(x)						  ( (x)>0?(x):-(x) )
#define MAX(x, y)						(((x) < (y)) ? (y) : (x))
#define MIN(a, b)						((a) < (b) ? (a) : (b))
#define LIMIT( x,min,max )  ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

float inv_sqrtf(float x);//���ٿ���
float fast_sqrtf(float number);
float safe_sqrt(float v);
float fast_pow(float a);//���㸡����ƽ��
float fast_atan2(float y, float x);//atan2
float to_180_degrees(float x);
void simple_3d_trans(float *ref, float *in, float *out);

#define M_PI 3.1415926535898f

#endif

