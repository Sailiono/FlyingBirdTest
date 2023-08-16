/**
* @file     sensor.c
* @brief    ��ȡ����������
* @version  1.1
* @author   ljt
* @date     2020.6.11
*/

#include "attitude_estimate.h"
#include "usart.h"
#include "main.h"
#include "DataTypeDefine.h"
#include  "hall.h"






//mpu6050������
void Sensor_task(void *param)
{
	u32 lastWakeTime = xTaskGetTickCount();
  		    				
	while(1)
	{
        vTaskDelayUntil(&lastWakeTime, 10);
        
        
        if(TIM4CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			//printf("HIGH1:%d us \t",TIM4CH1_CAPTURE_VAL); //��ӡ�ܵĸߵ�ƽʱ��
			TIM4CH1_CAPTURE_STA=0;			     //������һ�β���
            
		}
        if(TIM4CH2_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			//printf("HIGH2:%d us\r\n",TIM4CH2_CAPTURE_VAL); //��ӡ�ܵĸߵ�ƽʱ��
			TIM4CH2_CAPTURE_STA=0;			     //������һ�β���
            
		}
      
        
    }
}




