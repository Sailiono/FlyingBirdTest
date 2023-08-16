#include "watchdog.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"		 
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 看门狗驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
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
 *滴答定时器中断服务函数
*********************************************************/
void  SysTick_Handler(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
    {
        xPortSysTickHandler();	
    }else
	{
		sysTickCnt++;	/*调度开启之前计数*/
	}
}


/********************************************************
*getSysTickCnt()
*调度开启之前 返回 sysTickCnt
*调度开启之后 返回 xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}
