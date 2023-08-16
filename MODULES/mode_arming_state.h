#ifndef __MODE_ARMING_STATE_H
#define __MODE_ARMING_STATE_H
//////////////////////////////////////////////////////////////////////////////////�ļ�˵��	 
//������ư�-LJT-v1
//mode_arming_state.h   
//ljt
//��������:2020/1/8
//���� ���ڷ���ģʽ�жϺͽ���״̬���ж�
//�汾��V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////ͷ�ļ�����	
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////�궨��


typedef enum
{
    MODE_ERR = 0,
    MODE_MANUAL,
    MODE_STABILIZED,
    MODE_AUTO
}enummodestatus;

typedef enum
{
    DISARMED = 0,
    ARMED,
    DISARMED_READY,
    ARMED_READY,
    CANCEL
}enumarmstatus;



void state_machine_task(void *pvParameters);
void getarmedstatus(u8 *get);
void getflymode(u8* get);
#endif


