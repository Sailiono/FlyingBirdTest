#ifndef __MOTOR_STEERING_CONTROL_H
#define __MOTOR_STEERING_CONTROL_H
//////////////////////////////////////////////////////////////////////////////////文件说明	 
//飞鸟控制板-LJT-v1
//motor_steering.h 电机舵机驱动头文件   
//ljt
//创建日期:2020/1/7
//使用的资源为 TIM5CH1-PA0 副翼  TIM5CH2-PA1拉升舵 TIM5CH3-PA2 油门舵   
//版本：V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////头文件包含	
#include "sys.h"


extern float throttle_out,frequency_sum;
extern u8 error_is_limit_flag;
extern u32 error_is_limit_count;
void FlappingFrequencyControlTask(void *param);



#endif
