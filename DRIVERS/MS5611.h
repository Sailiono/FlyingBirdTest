#ifndef __MS5611_H__
#define __MS5611_H__
//////////////////////////////////////////////////////////////////////////////////文件说明	 
//飞鸟控制板-LJT-v1
//ms5611.h   
//ljt
//创建日期:2020/1/8
//使用的资源为I2C  PB8-SCL PB9-SDA 和mpu6050共用一个I2C
//提供的API为
//void MS561101BA_Init(void);  
//float ms5611_altitude(void);

//版本：V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////头文件包含
#include "sys.h"
#include "myiic.h"
#include "delay.h"
#include "usart.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"  
////////////////////////////////////////////////////////////////////////////////////宏定义
#define PRESSURE_STANDARD 101325
#define	MS5611CALNUM 100							//MS5611校准时样本数目
#define	MS561101BA_SlaveAddress 0xee  //定义器件在IIC总线中的从地址 已经是左移一位之后的结果 如果是写就是 0xee|0x00 如果是读就是 0xee|0x01 
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

////////////////////////////////////////////////////////////////////////////////////全局变量声明


//////////////////////////////////////////////////////////////////////////////////// 全局函数声明


void MS561101BA_RESET(void); 
//void MS561101BA_PROM_READ(void); 
//u32 MS561101BA_DO_CONVERSION(u8 command); 
//void MS561101BA_GetTemperature(u8 OSR_Temp);
//void MS561101BA_GetPressure(u8 OSR_Pres);
//void MS5611_cali(void);


////////API接口
void MS561101BA_Init(void); 

float ms5611_altitude(void);
#endif




