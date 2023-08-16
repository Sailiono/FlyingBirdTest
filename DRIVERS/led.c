#include "led.h" 

/**************************************************************************
Copyright

File name:LED��������

Author:LJT

Data:2020.11.19

Description:���ļ�ΪLED�ײ��������룬����������ӦGPIO�ڵĸߵ͵�ƽ��������LED��ͬʱ�ṩ��һЩLED��ʾ�Ľӿڡ�


Hardware:PC3 <-> LED1      PC2 <-> LED2      PC1 <-> LED3        PC0 <-> LED4        PB12 <-> RUN_LED ��Ϊ�͵�ƽ��

Others:

History:

1��2020.11.19,LJT���ݷ�����ư�V4_FMU��LEDӲ��ӳ���ϵ�������˸��ļ���

**************************************************************************/

#define RUNNING_LED PDout(1)
#define MYRC_LOST  0        
#define MYRC_MODE1 1
#define MYRC_MODE2 2
#define MYRC_MODE3 3

/*
 * LED��ʼ���ӿ�
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
 * ��������ָʾ�ƽӿ�
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
����ģʽָʾ�ƽӿ�
��ʱ��������ģʽ�����Կ������ֶ������Ⱥ�ȫ���� 
ģʽ���ֿ����ں궨�崦�޸�
 */


