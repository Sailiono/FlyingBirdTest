#ifndef __ATTITUDE_ESTIMATE_H
#define __ATTITUDE_ESTIMATE_H

//////////////////////////////////////////////////////////////////////////////////头文件包含	
#include "sys.h"
#include "mpu6050.h"
#include "DataTypeDefine.h"

//////////////////////////////////////////////////////////////////////////////////宏定义



#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */

#define RATE_100HZ_DT 0.01f
//////////////////////////////////////////////////////////////////////////////////声明外部变量

void getImuRawData(IMURAWDATAFTypeDef* get);
void Attitude_estimate_task(void *param);
void getAttitudeData(Attitude_S* get);

#endif

