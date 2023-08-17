#ifndef __POSITION_ESTIMATE_H
#define __POSITION_ESTIMATE_H
//////////////////////////////////////////////////////////////////////////////////�ļ�˵��	 
//������ư�-LJT-v1
//sbus.h   
//ljt
//��������:2020/1/9
//λ�ù��� ͨ��GPS ��ѹ�Ƶõ��ĸ߶�
//�汾��V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////ͷ�ļ�����	
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////�궨��
#define POSITION_ESTIMATE_STOP 0
#define POSITION_ESTIMATE_RUN  1

/////////////////////////////////////////////////////////////////////////////////�ṹ����������
//λ����Ϣ�ṹ���������
typedef struct vehicle_position_est
{
    float x;
    float y;
	float z;
	float vx;
	float vy;
	float vz;
}Position_S;


//λ�ô��������ݽṹ���������
typedef struct position_sensor_combined
{
   float baro_height;
	 float gps[2];
}position_estimate_sensor_combined;

//////////////////////////////////////////////////////////////////////////////////ȫ�ֱ���

//ȡֵ��Χ��  POSITION_ESTIMATE_STOP   POSITION_ESTIMATE_RUN
extern u8 g_position_estimate_flag;




//////////////////////////////////////////////////////////////////////////////////����
void update_height(void);


void g_position_est(void);

void Position_estimate_task(void *pvParameters);
void getheight(float *height);

#endif

