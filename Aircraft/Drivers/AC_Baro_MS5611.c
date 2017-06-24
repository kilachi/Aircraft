/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    ：Bmp085
**********************************************************************************/
#include "AC_Baro_MS5611.h"

uint32_t ms5611_ut;  // static result of temperature measurement
uint32_t ms5611_up;  // static result of pressure measurement
uint16_t ms5611_prom[PROM_NB];  // on-chip ROM
uint8_t t_rxbuf[3],p_rxbuf[3];

#define MOVAVG_SIZE  10	   //保存最近10组数据  5
// FIFO 队列					
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0; //队列指针

//添加一个新的值到 温度队列 进行滤波
void MS5611_NewTemp(float val) 
{
	Temp_buffer[temp_index] = val;
	temp_index = (temp_index + 1) % MOVAVG_SIZE;
}

//添加一个新的值到 气压队列 进行滤波
void MS5611_NewPress(float val)
{
	Press_buffer[press_index] = val;
	press_index = (press_index + 1) % MOVAVG_SIZE;
}

//添加一个新的值到 高度队列 进行滤波
void MS5611_NewAlt(float val) 
{
	int16_t i;
	for(i=1;i<MOVAVG_SIZE;i++)
		Alt_buffer[i-1] = Alt_buffer[i];
	Alt_buffer[MOVAVG_SIZE-1] = val;
}
//读取队列的平均值
float MS5611_getAvg(float * buff, int size) 
{
	float sum = 0.0;
	int i;
	for(i=0; i<size; i++) 
	{
		sum += buff[i];
	}
	return sum / size;
}
void baro_write(uint8_t reg,uint8_t data)
{
	BARO_CS_L();
	spi1_rw(reg);
	spi1_rw(data);
	BARO_CS_H();
}
void baro_read(uint8_t reg, uint8_t num, uint8_t* buff)
{
	BARO_CS_L();
	spi1_rw(reg);         //发送读取命令+读取数据的首寄存器地址   
	for(u8 i=0;i<num;i++)
	{ 
		buff[i]=spi1_rw(0xFF);   //循环读数 
	}
	BARO_CS_H();	
}
//复位MS5611
void MS5611_Reset()
{
	baro_write(CMD_RESET, 1);
}
u8 MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };
//	u8 check = 0;
	u8 i;

	for (i = 0; i < PROM_NB; i++)
	{
		baro_read(CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
		ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}
	return 0;
}	
void MS5611_Read_Adc_T(void)
{
	baro_read(CMD_ADC_READ, 3, t_rxbuf ); // read ADC
}

void MS5611_Read_Adc_P(void)
{
	baro_read(CMD_ADC_READ, 3, p_rxbuf); // read ADC
}

void MS5611_Start_T(void)
{
	baro_write(CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1); // D2 (temperature) conversion start!
}

void MS5611_Start_P(void)
{
  baro_write(CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1); // D1 (pressure) conversion start!
}
void MS5611_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );

	/*Config BARO Pin CS*/ 
	GPIO_InitStructure.GPIO_Pin = BARO_CS_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(BARO_CS, &GPIO_InitStructure);

	//复位MS5611
	MS5611_Reset();
	delay_ms(10);
	MS5611_Read_Prom();
	//开始读取温度
	MS5611_Start_T();	
}

//单位 [温度 度] [气压 帕]  [高度 米] 
volatile float MS5611_Temperature,MS5611_Pressure;

void MS5611_BaroAltCalculate(void)
{	
  int32_t  off2 = 0, sens2 = 0, delt;
	float temperature,Pressure;
	int32_t dT;
	int64_t off,sens;
	
	//读取的温度、压强
	ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
	ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
	
	//参数
	dT = ms5611_ut - ((uint32_t)ms5611_prom[5] << 8);
	off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
	sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
	//实际温度
	temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);

	//温度队列处理
	MS5611_NewTemp(temperature);	
	MS5611_Temperature = temperature = MS5611_getAvg(Temp_buffer,MOVAVG_SIZE);
	
	if (temperature < 2000) { // temperature lower than 20degC 
	  	delt = temperature - 2000;
			delt = delt * delt;
			off2 = (5 * delt) >> 1;
			sens2 = (5 * delt) >> 2;
			if (temperature < -1500) { // temperature lower than -15degC
					delt = temperature + 1500;
					delt = delt * delt;
					off2  += 7 * delt;
					sens2 += (11 * delt) >> 1;
			}
	}
	off  -= off2; 
	sens -= sens2;
	//温度补偿下压强
	Pressure = ((((int64_t)ms5611_up * sens ) >> 21) - off) / 32768.00f;
	
	MS5611_NewPress(Pressure);
	MS5611_Pressure = MS5611_getAvg(Press_buffer,MOVAVG_SIZE);
}

u8 MS5611_ReadData(void)
{
	static u8 state = 1;
	
	if (!state) 
	{
			MS5611_Read_Adc_P();
			MS5611_Start_T();
			MS5611_BaroAltCalculate();
			state = 1;
	} 
	else 
	{
			MS5611_Read_Adc_T();
			MS5611_Start_P();
			state = 0;
	}
	return state;
}

float get_ms5611_pressure(void){
	return MS5611_Pressure;
}
float get_ms5611_temperature(void){
	return MS5611_Temperature;
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
