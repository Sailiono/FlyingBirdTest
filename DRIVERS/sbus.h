#ifndef __SBUS_H
#define __SBUS_H

#include "sys.h"
#include "DataTypeDefine.h"


/**************************************************************************
Copyright

File name:sbus驱动代码

Author:LJT

Data:2020.11.30

Description:该文件为sbus底层驱动代码，利用了串口2接收完成中断，同时结合DMA接收
一次性接收SBUS协议25个字节的数据，表示着16个通道数据


Hardware:PA3 <-> USART2_RX  DMA1_STREAM5_CHANNEL4

Others:
    futaba 14SG遥控器的摇杆信号通道说明（本工程使用到的，可以再加，同时遥控器也要相应的配置）
    CH1 aileron  滚转通道  最左是1696 中间是1024 最右是352（注：通道数据随着微调按钮会变化，下同，）
    CH2 elevator 俯仰通道  最上是1696 中间是1024 最下是352
    CH3 throttle 油门通道  最上是1696 中间是1024 最下是352
    CH4 aileron_elevator 滚转俯仰混合通道  最左是352 中间是1024 最右是1696
    CH5 mode 模式选择通道  最上是352 中间是1024 最下是1696
    CH6 calibrate 校准开关通道  最上是352 中间是1025 最下是1696
History:

1、2020.11.30,LJT根据飞鸟控制板V4_FMU的硬件映射关系，创建了该文件。

**************************************************************************/

typedef struct rc_signal
{
   u16 aileron;
   u16 elevator;
   u16 throttle;
   u16 aileron_elevator;
   u16 mode;
   u16 calibrate_num;
   u16 loggerbutton_num;
   u8 calibrate_mode;
   u8 loggerbutton;
}RcRawDataSTypeDef;



//////////////////////////////////////////////////////////////////////////////////宏定义

//////////////////////////////////////////////////////////////////////////////////全局变量声明
extern RcRawDataSTypeDef myrc;



//////////////////////////////////////////////////////////////////////////////////全局函数声明

void Sbus_Init(void);
void sbus_task(void *param);
void GetRcValue(RcRawDataSTypeDef* get);
bool GetFlightLoggerState(void);
#endif
