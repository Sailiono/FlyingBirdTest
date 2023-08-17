#include "position_estimate.h"
#include "ms5611.h"
#include "main.h"

static Position_S g_pos_est;



void posUpdate(Position_S *pos ,float dt)
{
    static float last_value;
    pos->z = ms5611_altitude();
    
    if( isnan(pos->z) || (pos->z - last_value > 10.0f) || (pos->z - last_value < -10.0f) )
    {
        
        pos->z = last_value;
    }
    if(pos->z == last_value)
    {
        MS561101BA_RESET();
    }
    last_value = pos->z;
}	

void getheight(float *get)
{
    *get = g_pos_est.z;
}

void Position_estimate_task(void *pvParameters)
{
																				//定时器时钟，每隔1ms +1
	u32 lastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 200);													/*1ms周期延时*/
		
			posUpdate(&g_pos_est, 1);
//             printf("%d \t",TIM_GetCounter(TIM3));
//           printf("%f\r\n",g_pos_est.z);
           
		
	}


}

