#include "watchdog.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"		 
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ���Ź���������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/



static u32 sysTickCnt=0;


void watchdogInit(u16 xms)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* 47000/32Hz => 1.47  1ms*/
	IWDG_SetReload((u16)(1.47*xms));

	watchdogReset();
	IWDG_Enable();
}

extern void xPortSysTickHandler(void);

/********************************************************
 *SysTick_Handler()
 *�δ�ʱ���жϷ�����
*********************************************************/
void  SysTick_Handler(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*ϵͳ�Ѿ�����*/
    {
        xPortSysTickHandler();	
    }else
	{
		sysTickCnt++;	/*���ȿ���֮ǰ����*/
	}
}


/********************************************************
*getSysTickCnt()
*���ȿ���֮ǰ ���� sysTickCnt
*���ȿ���֮�� ���� xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*ϵͳ�Ѿ�����*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}
