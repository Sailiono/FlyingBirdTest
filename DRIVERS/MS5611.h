#ifndef __MS5611_H__
#define __MS5611_H__
//////////////////////////////////////////////////////////////////////////////////�ļ�˵��	 
//������ư�-LJT-v1
//ms5611.h   
//ljt
//��������:2020/1/8
//ʹ�õ���ԴΪI2C  PB8-SCL PB9-SDA ��mpu6050����һ��I2C
//�ṩ��APIΪ
//void MS561101BA_Init(void);  
//float ms5611_altitude(void);

//�汾��V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////ͷ�ļ�����
#include "sys.h"
#include "myiic.h"
#include "delay.h"
#include "usart.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"  
////////////////////////////////////////////////////////////////////////////////////�궨��
#define PRESSURE_STANDARD 101325
#define	MS5611CALNUM 100							//MS5611У׼ʱ������Ŀ
#define	MS561101BA_SlaveAddress 0xee  //����������IIC�����еĴӵ�ַ �Ѿ�������һλ֮��Ľ�� �����д���� 0xee|0x00 ����Ƕ����� 0xee|0x01 
#define	MS561101BA_D1 0x40 
#define	MS561101BA_D2 0x50 
#define	MS561101BA_RST 0x1E 
#define MS561101BA_T_OSR MS561101BA_D2_OSR_4096
#define MS561101BA_P_OSR MS561101BA_D1_OSR_4096

#define	MS561101BA_D1_OSR_256 0x40 
#define	MS561101BA_D1_OSR_512 0x42 
#define	MS561101BA_D1_OSR_1024 0x44 
#define	MS561101BA_D1_OSR_2048 0x46 
#define	MS561101BA_D1_OSR_4096 0x48 
#define	MS561101BA_D2_OSR_256 0x50 
#define	MS561101BA_D2_OSR_512 0x52 
#define	MS561101BA_D2_OSR_1024 0x54 
#define	MS561101BA_D2_OSR_2048 0x56 
#define	MS561101BA_D2_OSR_4096 0x58 
#define	MS561101BA_ADC_RD 0x00 
#define	MS561101BA_PROM_RD 0xA0 
#define	MS561101BA_PROM_CRC 0xAE 

////////////////////////////////////////////////////////////////////////////////////ȫ�ֱ�������


//////////////////////////////////////////////////////////////////////////////////// ȫ�ֺ�������


void MS561101BA_RESET(void); 
//void MS561101BA_PROM_READ(void); 
//u32 MS561101BA_DO_CONVERSION(u8 command); 
//void MS561101BA_GetTemperature(u8 OSR_Temp);
//void MS561101BA_GetPressure(u8 OSR_Pres);
//void MS5611_cali(void);


////////API�ӿ�
void MS561101BA_Init(void); 

float ms5611_altitude(void);
#endif




