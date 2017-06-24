#ifndef __AC_MPU6000_H
#define __AC_MPU6000_H

#include "Aircraft_Config.h"

void mpu6000_init(void);
int16_t* mpu6000_read(void);

#define MPU_IRQ	GPIOA
#define MPU_IRQ_PIN	GPIO_Pin_4
#define MPU_CS	GPIOB
#define MPU_CS_PIN	GPIO_Pin_0
#define CS_H()  MPU_CS->BSRR=MPU_CS_PIN
#define CS_L()  MPU_CS->BRR =MPU_CS_PIN

#define MPU6000_RA_SMPRT_DIV 0x19
#define MPU6000_RA_CONFIG 0x1A
#define MPU6000_RA_GYRO_CONFIG 0x1B
#define MPU6000_RA_ACCEL_CONFIG 0x1C
#define MPU6000_RA_INT_PIN_CFG 0x37
#define MPU6000_RA_INT_ENABLE 0x38
#define MPU6000_RA_ACCEL_XOUT_H 0x3B
#define MPU6000_RA_USER_CTRL 0x6A
#define MPU6000_RA_PWR_MGMT_1 0x6B
#define MPU6000_WHO_AM_I 0x68

#endif
