#ifndef __LED_H
#define __LED_H
#include "sys.h"

/**************************************************************************
Copyright

File name:LED驱动代码

Author:LJT

Data:2020.11.19

Description:该文件为LED底层驱动代码，用于配置相应GPIO口的高低电平，以驱动LED。同时提供了一些LED显示的接口。


Hardware:PC3 <-> LED1      PC2 <-> LED2      PC1 <-> LED3        PC0 <-> LED4        PB12 <-> RUN_LED 均为低电平亮

Others:

History:

1、2020.11.19,LJT根据飞鸟控制板V4_FMU的LED硬件映射关系，创建了该文件。

**************************************************************************/



 								  
//////////////////////////////////////////////////////////////////////////////////头文件包含	
#include "sys.h"




//////////////////////////////////////////////////////////////////////////////////全局函数声明
void LED_Init(void);   //LED初始化接口	
void ReverseRunningLED(void);//程序运行指示灯接口
void LEDModeIndication(u8 mode);//飞行模式指示灯接口
void LED1On(void);
void LED1Off(void);
void LED2On(void);
void LED2Off(void);
void LED3On(void);
void LED3Off(void);
void Led_Flash(int8_t a);

#endif

