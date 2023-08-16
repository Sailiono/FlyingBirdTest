#ifndef __HALL_H
#define __HALL_H	 
#include "sys.h" 
#include "main.h"
#define HALL_TEST 		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) //PB0
void HALL_Init(void);	//IO≥ı ºªØ
void getfrequency(float* get);
void hall_task(void *param);

extern float g_frequency_test;
#endif





