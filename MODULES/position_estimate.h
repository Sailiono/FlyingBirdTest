#ifndef __POSITION_ESTIMATE_H
#define __POSITION_ESTIMATE_H
//////////////////////////////////////////////////////////////////////////////////文件说明	 
//飞鸟控制板-LJT-v1
//sbus.h   
//ljt
//创建日期:2020/1/9
//位置估计 通过GPS 气压计得到的高度
//版本：V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////头文件包含	
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////宏定义
#define POSITION_ESTIMATE_STOP 0
#define POSITION_ESTIMATE_RUN  1

/////////////////////////////////////////////////////////////////////////////////结构体声明定义
//位置信息结构体变量声明
typedef struct vehicle_position_est
{
    float x;
    float y;
	float z;
	float vx;
	float vy;
	float vz;
}Position_S;


//位置传感器数据结构体变量声明
typedef struct position_sensor_combined
{
   float baro_height;
	 float gps[2];
}position_estimate_sensor_combined;

//////////////////////////////////////////////////////////////////////////////////全局变量

//取值范围是  POSITION_ESTIMATE_STOP   POSITION_ESTIMATE_RUN
extern u8 g_position_estimate_flag;




//////////////////////////////////////////////////////////////////////////////////函数
void update_height(void);


void g_position_est(void);

void Position_estimate_task(void *pvParameters);
void getheight(float *height);

#endif

