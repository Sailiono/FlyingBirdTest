#include "main.h"

/**
* @file     attitude_control.c
* @brief    ����������̬��ʵ�ʲ�����̬�õ���ͬͨ���Ŀ�����
* @version  1.1
* @author   ljt
* @date     2020.4.11
*/



/*���ݲ�������̬�Ǻ�������̬�������������ֵ����һ���ģ�*/
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
		vTaskDelayUntil(&lastWakeTime, 4);													/*4ms������ʱ*/
         
            getAttitudeData(&euler_est);
            getAttitudeExpData(&euler_exp);
            getflymode(&flymode);
            
            if(flymode == MODE_STABILIZED)
            {
                if(fabs(euler_est.pitch)>70.0f) //��������ǽӽ�90�� ��ת�ǵĽ���᲻׼ȷ������ת�ǵĽ���ֵ��Ϊ0
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
                 if(fabs(euler_est.pitch)>70.0f) //��������ǽӽ�90�� ��ת�ǵĽ���᲻׼ȷ������ת�ǵĽ���ֵ��Ϊ0
                {
                     euler_est.roll = 0.0f;
                }else
                {
                    euler_est.roll = euler_est.roll + ROLL_SETPOINT_OFFSET;
                }
                euler_est.pitch = euler_est.pitch - PITCH_SETPOINT_OFFSET_TAKEOFF ;
                
                attitude_control(euler_est,euler_exp);
            
            }
            
            
            
            /*���ݵ���Ŀ���������ֵ����PWM��ָ��*/
            g_motor_run();   
        
                  
	}
}
