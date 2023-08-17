#ifndef __MODE_ARMING_STATE_H
#define __MODE_ARMING_STATE_H
//////////////////////////////////////////////////////////////////////////////////文件说明	 
//飞鸟控制板-LJT-v1
//mode_arming_state.h   
//ljt
//创建日期:2020/1/8
//功能 用于飞行模式判断和解锁状态的判断
//版本：V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////头文件包含	
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////宏定义


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


