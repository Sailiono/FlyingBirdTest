#include "calibrate.h"
#include "main.h"
#include "radio.h"
#include "DataTypeDefine.h"
#include "semphr.h"

SemaphoreHandle_t calibrateacc_binarysemaphore;		    //定义一个二值信号量变量
SemaphoreHandle_t calibrategyro_binarysemaphore;		//定义一个二值信号量变量
SemaphoreHandle_t calibratebaro_binarysemaphore;		//定义一个二值信号量变量

//传感器校准任务函数
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
            /*如果上位机发来加速度计校准命令
            实际上这里把加速度计和陀螺仪一起校准了*/
            if(calibrateacc_binarysemaphore	!=	NULL)
            {
                err	= xSemaphoreTake(calibrateacc_binarysemaphore, 0);;
                if(err	==	pdTRUE)
                {
                    //加速度计校准算法
                    iscalibrate = true;
                    mpu6050cali();
                    
                    ANO_DT_Send_Cailibrate_Check(0x01,0x01);    //加速度计校准成功
                    iscalibrate = false;
                }
            }
            
            /*如果上位机发来陀螺仪校准命令*/
            if(calibrategyro_binarysemaphore	!=	NULL)
            {
                err	= xSemaphoreTake(calibrategyro_binarysemaphore, 0);;
                if(err	==	pdTRUE)
                {
                    //陀螺仪校准算法

                }
            }
            
            /*如果上位机发来气压计校准命令*/
            if(calibratebaro_binarysemaphore	!=	NULL)
            {
                err	= xSemaphoreTake(calibratebaro_binarysemaphore, 0);;
                if(err	==	pdTRUE)
                {
                    //气压计校准算法

                }
            }
        
        }

        
        vTaskDelay(100); 
        
	
	}
}

