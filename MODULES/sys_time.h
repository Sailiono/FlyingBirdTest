#ifndef __SYS_TIME_H
#define __SYS_TIME_H


//////////////////////////////////////////////////////////////////////////////////�ļ�˵��	 
//������ư�-LJT-v1
//sys_time.h   
//ljt
//��������:2020/1/7
//ʹ�õ���ԴΪ TIM3 ϵͳ��ʱ��
//             TIM2  ��ʱ��
//�汾��V1.0
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////ͷ�ļ�����	
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////�궨��
#define SYS_STOP 0
#define SYS_RUN  1




//////////////////////////////////////////////////////////////////////////////////ȫ�ֱ���
extern u8 g_sys_timer_flag;   //ϵͳ�жϱ�־λ







//////////////////////////////////////////////////////////////////////////////////����
void g_sys_move(void);
void g_sys_time_init(void);
void sys_time_controlIRQ_Init(void);
uint32_t Get_Time_Micros(void);

#endif
