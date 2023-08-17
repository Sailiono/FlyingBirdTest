#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stdbool.h"
extern float g_frequency_desired,g_attack_angle_desired,g_horizontal_desired,g_vertical_desired;

 void ReadOrderFromUartTask(void *param);

//如果想串口中断接收，请不要注释以下宏定义
void DebugSeriesInit(u32 bound);
void uart6_init(u32 bound);
bool uartslkGetDataWithTimout(u8 *c);
void uart1Dmasend(u8* data,u32 size);
void uart1_send_buff(u8* buf,u32 len);
void usart1_send_char(u8 temp);
#endif


