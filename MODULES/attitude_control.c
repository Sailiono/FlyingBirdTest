#include "main.h"

/**
* @file     attitude_control.c
* @brief    根据期望姿态和实际测量姿态得到不同通道的控制量
* @version  1.1
* @author   ljt
* @date     2020.4.11
*/



/*根据测量的姿态角和期望姿态角来计算舵机输出值（归一化的）*/
void attitude_control(Attitude_S euler_est,Attitude_S euler_exp)
{
    g_aileron = 0 + ROLL_DP*(euler_exp.roll - euler_est.roll);
    g_elevator = 0 + PITCH_DP*(euler_exp.pitch - euler_est.pitch);
    g_elevator = (g_elevator>0)?g_elevator:0;
    
    
}


void Attitude_control_task(void *pvParameters)
{
   																			
	u32 lastWakeTime = xTaskGetTickCount();
    u8 flymode;
    static Attitude_S euler_est;
    static Attitude_S euler_exp;
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 4);													/*4ms周期延时*/
         
            getAttitudeData(&euler_est);
            getAttitudeExpData(&euler_exp);
            getflymode(&flymode);
            
            if(flymode == MODE_STABILIZED)
            {
                if(fabs(euler_est.pitch)>70.0f) //如果俯仰角接近90度 滚转角的解算会不准确，将滚转角的解算值设为0
                {
                     euler_est.roll = 0.0f;
                }else
                {
                    euler_est.roll = euler_est.roll + ROLL_SETPOINT_OFFSET;
                }
                euler_est.pitch = euler_est.pitch - PITCH_SETPOINT_OFFSET_NORMAL;
                 
                attitude_control(euler_est,euler_exp);
            }else if(flymode == MODE_AUTO)
            {
                 if(fabs(euler_est.pitch)>70.0f) //如果俯仰角接近90度 滚转角的解算会不准确，将滚转角的解算值设为0
                {
                     euler_est.roll = 0.0f;
                }else
                {
                    euler_est.roll = euler_est.roll + ROLL_SETPOINT_OFFSET;
                }
                euler_est.pitch = euler_est.pitch - PITCH_SETPOINT_OFFSET_TAKEOFF ;
                
                attitude_control(euler_est,euler_exp);
            
            }
            
            
            
            /*根据电机的控制量期望值来发PWM波指令*/
            g_motor_run();   
        
                  
	}
}
