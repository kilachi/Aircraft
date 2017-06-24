/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * ����    ����������
**********************************************************************************/
#include "AC_RcData.h"

uint16_t rc_in[8];
int16_t CH[CH_NUM];

/////////////////////////
s16 MAX_CH[CH_NUM]  = {1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900   };	//ҡ�����
s16 MIN_CH[CH_NUM]  = {1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100   };	//ҡ����С
char CH_DIR[CH_NUM] = {0    ,0    ,0    ,1    ,0    ,0    ,0    ,0    ,0      };  //ҡ�˷���

float CH_filter[CH_NUM];
u8 NS,CH_Error[CH_NUM];
u16 NS_cnt,CLR_CH_Error[CH_NUM];

//ң������
void rc_update(u16 CH_In[8],float T)
{		
	u8 i;
	float filter_A = 3.14f *20 *T;
//======================================================================
	for( i = 0;i < CH_NUM ; i++ )
	{
		if( (u16)CH_In[i] > 2500 || (u16)CH_In[i] < 500 )
		{
			CH_Error[i]=1;
			CLR_CH_Error[i] = 0;
		}
		else
		{
			CLR_CH_Error[i]++;
			if( CLR_CH_Error[i] > 200 )
			{
				CLR_CH_Error[i] = 2000;
				CH_Error[i] = 0;
			}
		}

		if( NS )
		{
			if( CH_Error[i] ) //��ͨ�����ݴ���
			{
				
			}
			else
			{				
				if( MAX_CH[i] > MIN_CH[i] )//������
				{
					if( !CH_DIR[i] )
					{
						CH[i] =   LIMIT ( (s16)( ( CH_In[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), 0, 1000); //��һ�������+-500
					}
					else
					{
						CH[i] = LIMIT ( (s16)( ( MAX_CH[i] - CH_In[i])/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), 0, 1000); //��һ�������+-500
					}
				}	
				else
				{
					flag.motor_armed = 0;
				}
			}
		}
		else //δ�ӽ��ջ������źţ�ң�عرջ�ʧ�źţ�
		{
			flag.motor_armed = 0;
			for( i = 0;i < CH_NUM ; i++ )
			{
				CH[i]=0;
			}			
		}
		if( ABS(CH_In[i] - CH_filter[i]) <100 )
		{
			CH_filter[i] += filter_A *(CH[i] - CH_filter[i]) ;
		}
		else
		{
			CH_filter[i] += 0.5f *filter_A *( CH[i] - CH_filter[i]) ;
		}
	}	
	if(++NS_cnt>200)  // 400ms  δ���ź��ߡ�
	{
		NS_cnt = 0;
		NS = 0;
	}
//======================================================================
	rc_parse(T);	
	throttle_channel();
}
void rc_parse(float T)
{
	static u16 lockcnt;
	
	if((CH[CH_THR]<100)&&(CH[CH_YAW]>900)/*&&(CH[CH_ROLL]<100)&&(CH[CH_PITCH]<100)*/){//�ڰ�
		lockcnt += 1000*T;
		if(lockcnt>100){
			lockcnt=0;
			flag.motor_armed = 1;//����
		}
	}else if((CH[CH_THR]<100)&&(CH[CH_YAW]<100)/*&&(CH[CH_ROLL]>900)&&(CH[CH_PITCH]<100)*/){//���
		lockcnt += 1000*T;
		if(lockcnt>100){
			lockcnt=0;
			flag.motor_armed = 0;//����
			flag.motor_flying = 0;
		}
	}else{
		lockcnt=0;
	}		
}

void rc_dog(u8 ch_mode) //400ms�ڱ������һ��
{
	NS = ch_mode;
	NS_cnt = 0;
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
