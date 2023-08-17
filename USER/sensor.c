/**
* @file     sensor.c
* @brief    获取传感器数据
* @version  1.1
* @author   ljt
* @date     2020.6.11
*/

#include "attitude_estimate.h"
#include "usart.h"
#include "main.h"
#include "DataTypeDefine.h"
#include  "hall.h"






//mpu6050任务函数
void Sensor_task(void *param)
{
	u32 lastWakeTime = xTaskGetTickCount();
  		    				
	while(1)
	{
        vTaskDelayUntil(&lastWakeTime, 10);
        
        
        if(TIM4CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			//printf("HIGH1:%d us \t",TIM4CH1_CAPTURE_VAL); //打印总的高点平时间
			TIM4CH1_CAPTURE_STA=0;			     //开启下一次捕获
            
		}
        if(TIM4CH2_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			//printf("HIGH2:%d us\r\n",TIM4CH2_CAPTURE_VAL); //打印总的高点平时间
			TIM4CH2_CAPTURE_STA=0;			     //开启下一次捕获
            
		}
      
        
    }
}




