#ifndef __DATATYPEDEFINE_H__
#define __DATATYPEDEFINE_H__

#include "stdbool.h"
#include "sys.h"

typedef struct
{
    float roll;
	float pitch;
	float yaw;
    bool estisvalid;
}Attitude_S;	


typedef struct
{
    bool  ISCALIBRATED;
    float ACC_SCALE;
	short GYRO_BIASX;
	short GYRO_BIASY;
    short GYRO_BIASZ;
}CalibrateIMU_S;

//提供的全局变量为：
typedef struct 
	{
		float x;
		float y;
		float z;
	} Axis3f;


    
typedef struct
{
	float accx;
	float accy;
	float accz;
	float gyrox;
	float gyroy;
	float gyroz;
} IMURAWDATAFTypeDef;
	
typedef struct
{
	short accx;
	short accy;
	short accz;
	short gyrox;
	short gyroy;
	short gyroz;
} mpu6050_IMURAWTypeDef;

typedef struct
{
	short magx;
	short magy;
	short magz;
} magnetic_RAWTypeDef;


#endif
