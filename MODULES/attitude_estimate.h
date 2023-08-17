#ifndef __ATTITUDE_ESTIMATE_H
#define __ATTITUDE_ESTIMATE_H

//////////////////////////////////////////////////////////////////////////////////ͷ�ļ�����	
#include "sys.h"
#include "mpu6050.h"
#include "DataTypeDefine.h"

//////////////////////////////////////////////////////////////////////////////////�궨��



#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */

#define RATE_100HZ_DT 0.01f
//////////////////////////////////////////////////////////////////////////////////�����ⲿ����

void getImuRawData(IMURAWDATAFTypeDef* get);
void Attitude_estimate_task(void *param);
void getAttitudeData(Attitude_S* get);

#endif

