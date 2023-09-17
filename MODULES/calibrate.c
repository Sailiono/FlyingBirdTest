#include "calibrate.h"
#include "main.h"
#include "radio.h"
#include "DataTypeDefine.h"
#include "semphr.h"

SemaphoreHandle_t calibrateacc_binarysemaphore;		    //����һ����ֵ�ź�������
SemaphoreHandle_t calibrategyro_binarysemaphore;		//����һ����ֵ�ź�������
SemaphoreHandle_t calibratebaro_binarysemaphore;		//����һ����ֵ�ź�������

//������У׼������
void calibrate_task(void *param)
{
    calibrateacc_binarysemaphore = xSemaphoreCreateBinary();
    calibrategyro_binarysemaphore = xSemaphoreCreateBinary();
    calibratebaro_binarysemaphore = xSemaphoreCreateBinary();
    BaseType_t err	=	pdFALSE;
    static bool iscalibrate = false;
    
	while(1)
	{ 
        if(!iscalibrate)
        {
            /*�����λ���������ٶȼ�У׼����
            ʵ��������Ѽ��ٶȼƺ�������һ��У׼��*/
            if(calibrateacc_binarysemaphore	!=	NULL)
            {
                err	= xSemaphoreTake(calibrateacc_binarysemaphore, 0);;
                if(err	==	pdTRUE)
                {
                    //���ٶȼ�У׼�㷨
                    iscalibrate = true;
                    mpu6050cali();
                    
                    ANO_DT_Send_Cailibrate_Check(0x01,0x01);    //���ٶȼ�У׼�ɹ�
                    iscalibrate = false;
                }
            }
            
            /*�����λ������������У׼����*/
            if(calibrategyro_binarysemaphore	!=	NULL)
            {
                err	= xSemaphoreTake(calibrategyro_binarysemaphore, 0);;
                if(err	==	pdTRUE)
                {
                    //������У׼�㷨

                }
            }
            
            /*�����λ��������ѹ��У׼����*/
            if(calibratebaro_binarysemaphore	!=	NULL)
            {
                err	= xSemaphoreTake(calibratebaro_binarysemaphore, 0);;
                if(err	==	pdTRUE)
                {
                    //��ѹ��У׼�㷨

                }
            }
        
        }

        
        vTaskDelay(100); 
        
	
	}
}

