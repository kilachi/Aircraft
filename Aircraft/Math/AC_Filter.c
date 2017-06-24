/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：滤波
**********************************************************************************/
#include "AC_Filter.h" 
#include "AC_FastMath.h" 

/*       
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好       
*/
double get_kalman_filter(const double data,double* x_last,double* p_last,double Q,double R)
{
	double x_mid = *x_last;
	double x_now;
	double p_mid ;
	double p_now;
	double kg;       

	x_mid=*x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=*p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(data-x_mid);//估计出的最优值			 
	p_now=(1.0f-kg)*p_mid;//最优值对应的covariance       
	*p_last = p_now; //更新covariance值
	*x_last = x_now; //更新系统状态值

	return x_now; 	
}
/*一阶低通滤波*/
float get_1nd_filter(int16_t data, int16_t* last_data, float lpf_1nd_factor)
{	
	float lpf_1nd_data = *last_data * (1 - lpf_1nd_factor) + data * lpf_1nd_factor;
	*last_data = data;
	return lpf_1nd_data;
}
/*二阶低通滤波*/
float get_2nd_filter(int16_t data, float* lpf_2nd_factor,float* last_2nd_data)
{	
	float lpf_2nd_data = data * lpf_2nd_factor[0] + last_2nd_data[0] * lpf_2nd_factor[1] - last_2nd_data[1] * lpf_2nd_factor[2];
	
	last_2nd_data[1] = last_2nd_data[0];
	last_2nd_data[0] = lpf_2nd_data;
	
	return lpf_2nd_data;
}
void get_2nd_factor(float deltaT, float Fcut,float* lpf_param)
{
	float a = 1 / (2 * M_PI * Fcut * deltaT);
	lpf_param[0] = 1 / (a*a + 3*a + 1);	//b0
	lpf_param[1] = (2*a*a + 3*a) / (a*a + 3*a + 1);	//a1
	lpf_param[2] = (a*a) / (a*a + 3*a + 1);	//a2	
}

float get_average(float data,float * buff,u8 * index, u8 size)
{
	float sum=0.0;
	u8 i;
	
	buff[*index]=data;
	
	if((++*index)==size) *index=0;
	
	for(i=0; i<size; i++) 
	{
		sum += buff[i];
	}
	return sum / size;	
}

float get_deathzoom(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}
  return (t);
}

#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  4
float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];
u8 med_fil_cnt[MED_FIL_ITEM];
float get_median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}

		
		return ( tmp[(u16)width_num/2] );
	}
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
