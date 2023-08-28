#ifndef __SBUS_H
#define __SBUS_H

#include "sys.h"
#include "DataTypeDefine.h"


/**************************************************************************
Copyright

File name:sbus��������

Author:LJT

Data:2020.11.30

Description:���ļ�Ϊsbus�ײ��������룬�����˴���2��������жϣ�ͬʱ���DMA����
һ���Խ���SBUSЭ��25���ֽڵ����ݣ���ʾ��16��ͨ������


Hardware:PA3 <-> USART2_RX  DMA1_STREAM5_CHANNEL4

Others:
    futaba 14SGң������ҡ���ź�ͨ��˵����������ʹ�õ��ģ������ټӣ�ͬʱң����ҲҪ��Ӧ�����ã�
    CH1 aileron  ��תͨ��  ������1696 �м���1024 ������352��ע��ͨ����������΢����ť��仯����ͬ����
    CH2 elevator ����ͨ��  ������1696 �м���1024 ������352
    CH3 throttle ����ͨ��  ������1696 �м���1024 ������352
    CH4 aileron_elevator ��ת�������ͨ��  ������352 �м���1024 ������1696
    CH5 mode ģʽѡ��ͨ��  ������352 �м���1024 ������1696
    CH6 calibrate У׼����ͨ��  ������352 �м���1025 ������1696
History:

1��2020.11.30,LJT���ݷ�����ư�V4_FMU��Ӳ��ӳ���ϵ�������˸��ļ���

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



//////////////////////////////////////////////////////////////////////////////////�궨��

//////////////////////////////////////////////////////////////////////////////////ȫ�ֱ�������
extern RcRawDataSTypeDef myrc;



//////////////////////////////////////////////////////////////////////////////////ȫ�ֺ�������

void Sbus_Init(void);
void sbus_task(void *param);
void GetRcValue(RcRawDataSTypeDef* get);
bool GetFlightLoggerState(void);
#endif
