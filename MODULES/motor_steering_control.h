#ifndef __MOTOR_STEERING_CONTROL_H
#define __MOTOR_STEERING_CONTROL_H
//////////////////////////////////////////////////////////////////////////////////�ļ�˵��	 
//������ư�-LJT-v1
//motor_steering.h ����������ͷ�ļ�   
//ljt
//��������:2020/1/7
//ʹ�õ���ԴΪ TIM5CH1-PA0 ����  TIM5CH2-PA1������ TIM5CH3-PA2 ���Ŷ�   
//�汾��V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////ͷ�ļ�����	
#include "sys.h"


extern float throttle_out,frequency_sum;
extern u8 error_is_limit_flag;
extern u32 error_is_limit_count;
void FlappingFrequencyControlTask(void *param);



#endif
