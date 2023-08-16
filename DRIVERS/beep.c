#include "beep.h" 


//////////////////////////////////////////////////////////////////////////////////	 
//飞鸟控制板-LJT-v1
//蜂鸣器驱动代码	   
//ljt
//创建日期:2020/1/7
//使用的资源为 PA4
//版本：V1.0
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////	 

	    
//BEEP IO初始化
void BEEP_Init(void)
{   
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_ResetBits(GPIOB,GPIO_Pin_7); 
}




