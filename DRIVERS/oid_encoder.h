#ifndef __OID_ENCODER_H
#define __OID_ENCODER_H
//////////////////////////////////////////////////////////////////////////////////文件说明	 
//飞鸟控制板-LJT-v1
//oid_encoder.h   
//ljt
//创建日期:2020/1/7
//使用的资源为 PA3-USART_RX
//版本：V1.0
//All rights reserved		

 

//////////////////////////////////////////////////////////////////////////////////头文件包含	
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////宏定义

//////////////////////////////////////////////////////////////////////////////////全局函数声明

void OidEncoderInit(void);
void OidEncoderTask(void *param);

//////////////////////////////////////////////////////////////////////////////////全局变量声明
extern float g_attack_angle_test;

#endif
