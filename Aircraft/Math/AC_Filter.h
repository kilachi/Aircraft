#ifndef __AC_FILTER_H
#define __AC_FILTER_H

#include "Aircraft_Config.h"

float get_average(float data,float * buff,u8 * index, u8 size);
float get_deathzoom(float x,float zoom);
float get_median(u8 item,u8 width_num,float in);
void get_2nd_factor(float deltaT, float Fcut,float* lpf_param);
float get_1nd_filter(int16_t data, int16_t* last_data, float lpf_1nd_factor);
float get_2nd_filter(int16_t data, float* lpf_2nd_factor,float* last_2nd_data);
double get_kalman_filter(const double data,double* x_last,double* p_last,double Q,double R);

#endif

