#include "led.h" 

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

#define RUNNING_LED PDout(1)
#define MYRC_LOST  0        
#define MYRC_MODE1 1
#define MYRC_MODE2 2
#define MYRC_MODE3 3

/*
 * LED初始化接口
 */
void LEDInit(void)
{    	 
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2|GPIO_Pin_1|GPIO_Pin_0;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOC,GPIO_Pin_3|GPIO_Pin_2|GPIO_Pin_1|GPIO_Pin_0);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_SetBits(GPIOD,GPIO_Pin_1|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
    /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_Pin_5);*/
}

/*
 * 程序运行指示灯接口
 */
void ReverseRunningLED(void)
{
    RUNNING_LED=!RUNNING_LED;
}

void LED1On(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);
}

void LED2On(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_14);
}


void LED3On(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_15);
}

void LED1Off(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_13);
}

void LED2Off(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_14);
}

void LED3Off(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_15);
}

/*
飞行模式指示灯接口
暂时定了三种模式，可以考虑是手动、自稳和全自主 
模式名字可以在宏定义处修改
 */


