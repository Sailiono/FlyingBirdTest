#ifndef __LED_H
#define __LED_H
#include "sys.h"

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



 								  
//////////////////////////////////////////////////////////////////////////////////ͷ�ļ�����	
#include "sys.h"




//////////////////////////////////////////////////////////////////////////////////ȫ�ֺ�������
void LED_Init(void);   //LED��ʼ���ӿ�	
void ReverseRunningLED(void);//��������ָʾ�ƽӿ�
void LEDModeIndication(u8 mode);//����ģʽָʾ�ƽӿ�
void LED1On(void);
void LED1Off(void);
void LED2On(void);
void LED2Off(void);
void LED3On(void);
void LED3Off(void);
void Led_Flash(int8_t a);

#endif

