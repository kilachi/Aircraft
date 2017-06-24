/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ����		 ��Cyrus
 * ����    ����Ԫ��
**********************************************************************************/
#include "AC_AHRS.h" 

_ahrs ahrs={1.0f};

#if AHRS_WITH_MAG
#define Kp 0.6f                
#define Ki 0.1f                
void ahrs_update(float a[3], float g[3], float m[3], float deltaT)
{		
	float acc[3];
	float gyro[3];
	float mag[3];
	
	acc[0]=a[0];acc[1]=a[1];acc[2]=a[2];
	gyro[0]=g[0]*RADIAN_TO_ANGLE;gyro[1]=g[1]*RADIAN_TO_ANGLE;gyro[2]=g[2]*RADIAN_TO_ANGLE;
	mag[0]=m[0];mag[1]=m[1];mag[2]=m[2];
	deltaT *=0.5f;
	/*=========================================================================================*/
	float mag_norm_tmp;
	static float mag_tmp[3];
	static float mag_sim_3d[3];
	static float yaw_correct;
	static float mag_norm ,mag_norm_xyz ;

	mag_norm_tmp = 20 *(6.28f *deltaT);		
	mag_norm_xyz = fast_sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	
	if( mag_norm_xyz != 0){
		mag_tmp[0] += mag_norm_tmp *( (float)mag[0] /( mag_norm_xyz ) - mag_tmp[0]);
		mag_tmp[1] += mag_norm_tmp *( (float)mag[1] /( mag_norm_xyz ) - mag_tmp[1]);	
		mag_tmp[2] += mag_norm_tmp *( (float)mag[2] /( mag_norm_xyz ) - mag_tmp[2]);	
	}

	simple_3d_trans(ahrs.reference_vz,mag_tmp,mag_sim_3d);
	
	mag_norm = fast_sqrtf(mag_sim_3d[0] * mag_sim_3d[0] + mag_sim_3d[1] *mag_sim_3d[1]);
	
	if( mag_sim_3d[0] != 0 && mag_sim_3d[1] != 0 && mag_sim_3d[2] != 0 && mag_norm != 0){
		ahrs.heading = fast_atan2( ( mag_sim_3d[1]/mag_norm ) , ( mag_sim_3d[0]/mag_norm) ) *57.3f;		
	}
	/*=========================================================================================*/	
	// �����Ч��������
	ahrs.reference_vx[0] = 1 - 2*(ahrs.q[2]*ahrs.q[2] + ahrs.q[3]*ahrs.q[3]);
	ahrs.reference_vx[1] = 2*(ahrs.q[1]*ahrs.q[2] - ahrs.q[0]*ahrs.q[3]);
	ahrs.reference_vx[2] = 2*(ahrs.q[1]*ahrs.q[3] + ahrs.q[0]*ahrs.q[2]);
	
	ahrs.reference_vy[0] = 2*(ahrs.q[1]*ahrs.q[2] + ahrs.q[0]*ahrs.q[3]);
	ahrs.reference_vy[1] = 1 - 2*(ahrs.q[1]*ahrs.q[1] + ahrs.q[3]*ahrs.q[3]);
	ahrs.reference_vy[2] = 2*(ahrs.q[2]*ahrs.q[3] - ahrs.q[0]*ahrs.q[1]);
	
	ahrs.reference_vz[0] = 2*(ahrs.q[1]*ahrs.q[3] - ahrs.q[0]*ahrs.q[2]);
	ahrs.reference_vz[1] = 2*(ahrs.q[0]*ahrs.q[1] + ahrs.q[2]*ahrs.q[3]);
	ahrs.reference_vz[2] = 1 - 2*(ahrs.q[1]*ahrs.q[1] + ahrs.q[2]*ahrs.q[2]);
	
	//���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	//�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
	//���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������       
	//=============================================================================
	float norm_acc;
	static float norm_acc_lpf;
	// ������ٶ�������ģ
	norm_acc = fast_sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);   
	norm_acc_lpf +=  10 *(6.28f *deltaT) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001

	if(abs(acc[0])<10.5f && abs(acc[1])<10.5f && abs(acc[2])<10.5f ){	
		//�ѼӼƵ���ά����ת�ɵ�λ������
		acc[0] = acc[0] / norm_acc;//4096.0f;
		acc[1] = acc[1] / norm_acc;//4096.0f;
		acc[2] = acc[2] / norm_acc;//4096.0f; 
		
		if( 9.0f < norm_acc && norm_acc < 10.5f ){
			float err_tmp[3];
			static float err_lpf[3];
			/* ��˵õ���� */
			err_tmp[0] = acc[1]*ahrs.reference_vz[2] - acc[2]*ahrs.reference_vz[1];
			err_tmp[1] = acc[2]*ahrs.reference_vz[0] - acc[0]*ahrs.reference_vz[2];
			
			/* ����ͨ */
			err_lpf[0] += 1 *(6.28f *deltaT) *( err_tmp[0]  - err_lpf[0] );
			err_lpf[1] += 1 *(6.28f *deltaT) *( err_tmp[1]  - err_lpf[1] );
			
			ahrs.err[0] = err_lpf[0];//
			ahrs.err[1] = err_lpf[1];//
		}
	}
	else
	{
		ahrs.err[0] = 0; 
		ahrs.err[1] = 0  ;
	}
	/* ������ */
	ahrs.err_i[0] += ahrs.err[0] *Ki *2 *deltaT ;
	ahrs.err_i[1] += ahrs.err[1] *Ki *2 *deltaT ;
	ahrs.err_i[2] += ahrs.err[2] *Ki *2 *deltaT ;
	
	/* �����޷� */
	ahrs.err_i[0] = LIMIT(ahrs.err_i[0], - 2*GYRO_TO_ANGLE ,2*GYRO_TO_ANGLE );
	ahrs.err_i[1] = LIMIT(ahrs.err_i[1], - 2*GYRO_TO_ANGLE ,2*GYRO_TO_ANGLE );
	ahrs.err_i[2] = LIMIT(ahrs.err_i[2], - 2*GYRO_TO_ANGLE ,2*GYRO_TO_ANGLE );
	
	if( ahrs.reference_vz[2] > 0.0f ){
		if(flag.motor_aready){
			yaw_correct = Kp *0.8f *to_180_degrees(ahrs.heading - ahrs.angle[2]);//�Ѿ�������ֻ��Ҫ���پ�����
		}else{
			yaw_correct = Kp *1.5f *to_180_degrees(ahrs.heading - ahrs.angle[2]);//û�н�������������ʱ�̣����پ���			
		}
	}	
	//�����˲�����̬���������ٶ��ϣ��������ٶȻ���Ư��	
	gyro[0] = (gyro[0] - ahrs.reference_vz[0] *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ahrs.err[0] + ahrs.err_i[0]) ) ;     //IN RADIAN
	gyro[1] = (gyro[1] - ahrs.reference_vz[1] *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ahrs.err[1] + ahrs.err_i[1]) ) ;		  //IN RADIAN
	gyro[2] = (gyro[2] - ahrs.reference_vz[2] *yaw_correct) *ANGLE_TO_RADIAN;
	
	//һ�����������������Ԫ��
	ahrs.q[0] = ahrs.q[0] +(-ahrs.q[1]*gyro[0] - ahrs.q[2]*gyro[1] - ahrs.q[3]*gyro[2])*deltaT;
	ahrs.q[1] = ahrs.q[1] + (ahrs.q[0]*gyro[0] + ahrs.q[2]*gyro[2] - ahrs.q[3]*gyro[1])*deltaT;
	ahrs.q[2] = ahrs.q[2] + (ahrs.q[0]*gyro[1] - ahrs.q[1]*gyro[2] + ahrs.q[3]*gyro[0])*deltaT;
	ahrs.q[3] = ahrs.q[3] + (ahrs.q[0]*gyro[2] + ahrs.q[1]*gyro[1] - ahrs.q[2]*gyro[0])*deltaT;  

	/* ��Ԫ����һ�� normalise quaternion */
	float norm_q = fast_sqrtf(ahrs.q[0]*ahrs.q[0] + ahrs.q[1]*ahrs.q[1] + ahrs.q[2]*ahrs.q[2] + ahrs.q[3]*ahrs.q[3]);
	ahrs.q[0] = ahrs.q[0] / norm_q;
	ahrs.q[1] = ahrs.q[1] / norm_q;
	ahrs.q[2] = ahrs.q[2] / norm_q;
	ahrs.q[3] = ahrs.q[3] / norm_q;
	
	Quart_to_euler(ahrs.angle);	
}
#endif

#if AHRS_ONLY_MPU
#define Kp 1.09f       
#define Ki 0.01f     
void ahrs_update(float a[3], float g[3], float m[3], float deltaT)
{
	float norm;
	float comp_bate;

	float acc[3];
	float gyro[3];
	
	acc[0]=a[0];acc[1]=a[1];acc[2]=a[2];
	gyro[0]=g[0];gyro[1]=g[1];gyro[2]=g[2];

	float acc_tmp = (acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])/(9.8f*9.8f);
	if(0.75f<acc_tmp && acc_tmp<1.21f){
		if(!flag.motor_flying){
			comp_bate = Kp *2;
		}else{
			comp_bate = Kp;			
		}
	}else{
			comp_bate = 0;				
	}
	//�������ٶȹ�һ��
	norm = inv_sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);       
	acc[0] = acc[0] * norm;
	acc[1] = acc[1] * norm;
	acc[2] = acc[2] * norm;

	//��ȡ��Ԫ���ĵ�Ч���Ҿ����е���������
	ahrs.reference_vz[0] = 2*(ahrs.q[1]*ahrs.q[3] - ahrs.q[0]*ahrs.q[2]);												
	ahrs.reference_vz[1] = 2*(ahrs.q[0]*ahrs.q[1] + ahrs.q[2]*ahrs.q[3]);
	ahrs.reference_vz[2] = 1 - 2*(ahrs.q[1]*ahrs.q[1] + ahrs.q[2]*ahrs.q[2]);//ahrs.reference_vz[2] = ahrs.q[0]*ahrs.q[0] - ahrs.q[1]*ahrs.q[1] - ahrs.q[2]*ahrs.q[2] + ahrs.q[3]*ahrs.q[3] ;

	//��������ó���̬���
	ahrs.err[0] = (acc[1]*ahrs.reference_vz[2] - acc[2]*ahrs.reference_vz[1]) *halfT;                           					
	ahrs.err[1] = (acc[2]*ahrs.reference_vz[0] - acc[0]*ahrs.reference_vz[2]) *halfT;
	ahrs.err[2] = (acc[0]*ahrs.reference_vz[1] - acc[1]*ahrs.reference_vz[0]) *halfT;

	//�������л���	
	ahrs.err_i[0] += ahrs.err[0] * Ki *deltaT;								  
	ahrs.err_i[1] += ahrs.err[1] * Ki *deltaT;
	ahrs.err_i[2] += ahrs.err[2] * Ki *deltaT;

	//�����˲�����̬���������ٶ��ϣ��������ٶȻ���Ư��
	gyro[0] += comp_bate*ahrs.err[0] + ahrs.err_i[0];					   						
	gyro[1] += comp_bate*ahrs.err[1] + ahrs.err_i[1];
	gyro[2] += comp_bate*ahrs.err[2] + ahrs.err_i[2];				   							

	//һ�����������������Ԫ��
	ahrs.q[0] = ahrs.q[0] + (-ahrs.q[1]*gyro[0] - ahrs.q[2]*gyro[1] - ahrs.q[3]*gyro[2])*halfT*deltaT;
	ahrs.q[1] = ahrs.q[1] + ( ahrs.q[0]*gyro[0] - ahrs.q[3]*gyro[1] + ahrs.q[2]*gyro[2])*halfT*deltaT;
	ahrs.q[2] = ahrs.q[2] + ( ahrs.q[3]*gyro[0] + ahrs.q[0]*gyro[1] - ahrs.q[1]*gyro[2])*halfT*deltaT;
	ahrs.q[3] = ahrs.q[3] + (-ahrs.q[2]*gyro[0] + ahrs.q[1]*gyro[1] + ahrs.q[0]*gyro[2])*halfT*deltaT;

	//��Ԫ����һ��
	float normalization	= inv_sqrtf(ahrs.q[0]*ahrs.q[0] + ahrs.q[1]*ahrs.q[1] + ahrs.q[2]*ahrs.q[2] + ahrs.q[3]*ahrs.q[3]);
	ahrs.q[0] = ahrs.q[0] * normalization;
	ahrs.q[1] = ahrs.q[1] * normalization;
	ahrs.q[2] = ahrs.q[2] * normalization;
	ahrs.q[3] = ahrs.q[3] * normalization;
	
	Quart_to_euler(ahrs.angle);
}
#endif

//��Ԫ��תŷ����
void Quart_to_euler(float *euler)
{
		euler[1] = fast_atan2(2.0f*(ahrs.q[0]*ahrs.q[1] + ahrs.q[2]*ahrs.q[3]),1 - 2.0f*(ahrs.q[1]*ahrs.q[1] + ahrs.q[2]*ahrs.q[2]))*57.29577f;
		euler[0] = asin(2.0f*(ahrs.q[0]*ahrs.q[2] - ahrs.q[1]*ahrs.q[3]))*57.29577951f;		
		euler[2] = fast_atan2(2.0f*(-ahrs.q[1]*ahrs.q[2] -ahrs.q[0]*ahrs.q[3]),2.0f*(ahrs.q[0]*ahrs.q[0] + ahrs.q[1]*ahrs.q[1]) -1)*57.29577f;
//		euler[2] = fast_atan2(2.0f*(ahrs.q[0]*ahrs.q[3] +ahrs.q[1]*ahrs.q[2]),1 - 2.0f*(ahrs.q[2]*ahrs.q[2] + ahrs.q[3]*ahrs.q[3]))*57.29577f;
}

//ŷ����ת��Ԫ��
void Quart_from_euler(float *euler)
{
    float cr2 = cosf(euler[0]*0.5f);
    float cp2 = cosf(euler[1]*0.5f);
    float cy2 = cosf(euler[2]*0.5f);
    float sr2 = sinf(euler[0]*0.5f);
    float sp2 = sinf(euler[1]*0.5f);
    float sy2 = sinf(euler[2]*0.5f);

    ahrs.q[0] = cr2*cp2*cy2 + sr2*sp2*sy2;
    ahrs.q[1] = sr2*cp2*cy2 - cr2*sp2*sy2;
    ahrs.q[2] = cr2*sp2*cy2 + sr2*cp2*sy2;
    ahrs.q[3] = cr2*cp2*sy2 - sr2*sp2*cy2;
}

void get_accel_ef(void)
{
	u8 i;
	static double x[3],p[3];
	float accel_ef_tmp[3];
	
	accel_ef_tmp[0] = ahrs.reference_vx[0]*mpu.accel_lpf[0] + ahrs.reference_vx[1]*mpu.accel_lpf[1] + ahrs.reference_vx[2]*mpu.accel_lpf[2] - 0.0f;	
	accel_ef_tmp[1] = ahrs.reference_vy[0]*mpu.accel_lpf[0] + ahrs.reference_vy[1]*mpu.accel_lpf[1] + ahrs.reference_vy[2]*mpu.accel_lpf[2] - 0.0f;	
	accel_ef_tmp[2] = ahrs.reference_vz[0]*mpu.accel_lpf[0] + ahrs.reference_vz[1]*mpu.accel_lpf[1] + ahrs.reference_vz[2]*mpu.accel_lpf[2] - 9.8f;	
	
	for(i=0;i<3;i++){
		ahrs.accel_ef[i] = get_kalman_filter(accel_ef_tmp[i],&x[i],&p[i],0.1f,20.1f) *100.0f;
	}
}

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
