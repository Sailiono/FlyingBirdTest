#include "step_motor_control.h"
#include "step_motor_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "projdefs.h"	
#include "math.h"
#include "delay.h"
#include "oid_encoder.h"
#include "adc.h"

/*步进电机转280圈，仰角机构转360度，即变化1度是0.7777圈

假设步进电机细分设置为 800个脉冲每圈，则转1度是622个脉冲。

*/

bool g_attack_of_angle_isok=true;
bool g_horizontal_isok=true;
bool g_vertical_isok=true;

    
float attack_of_angle_error;
float horizontal_error;
float vertical_error;

static float fabs_attack_of_angle_error;
static float fabs_horizontal_error;
static float fabs_vertical_error;

float g_vertical_test,g_horizontal_test;

//#define  g_horizontal_test (float)(Get_Adc(ADC_Channel_6))/4096.0f*350.0f //PA6 CURRENT ~ HORIZONTAL
////#define  g_vertical_test (float)(Get_Adc(ADC_Channel_7))/4096.0f*175.0f  

//#define  g_vertical_test (float)(Get_Adc(ADC_Channel_10))/4096.0f*175.0f*3.3f/3.0f-67.0f  //PA7  VOLT ~ VERTICLE 



u32 num;
void StepMotorControlTask(void *param)
{
	u32 lastWakeTime = xTaskGetTickCount();
    

	while(1)
	{  
        vTaskDelayUntil(&lastWakeTime, 10);	//10ms周期延时
        num++;
        
        g_vertical_test = (float)(Get_Adc(ADC_Channel_10))/4096.0f*175.0f*3.3f/3.0f-65.0f;  //PC0   ANGLE
        g_horizontal_test = (float)(Get_Adc(ADC_Channel_7))/4096.0f*350.0f*3.3f/3.0f-160.0f; //PA7  VOLT ~ VERTICLE 
        
        
        attack_of_angle_error = g_attack_angle_desired - g_attack_angle_test;
        fabs_attack_of_angle_error = fabs(attack_of_angle_error);
        
        horizontal_error = g_horizontal_desired - g_horizontal_test;
        fabs_horizontal_error = fabs(horizontal_error);
        
        vertical_error = g_vertical_desired - g_vertical_test;
        fabs_vertical_error = fabs(vertical_error);
       
  
        
        //确保不会一开机就运转，只有接收到上位机指令才会动
        if(g_attack_of_angle_isok == false)
        {
            //如果误差超过0.05度，则做反馈控制
             if( fabs_attack_of_angle_error>0.05f)
            {
                //朝着减少误差的方向运行
                if(attack_of_angle_error>0)
                {
                    StepMotorRunReverse(ATTACK_STEP_MOTOR);
                }else
                {
                    StepMotorRunNormal(ATTACK_STEP_MOTOR); 
                }
    
            }else
            {
                StepMotorStop(ATTACK_STEP_MOTOR);
                g_attack_of_angle_isok = true;    
            } 
        }//end of if(g_attack_of_angle_isok == false)
        
        
            // printf("x=%f\t  y= %f\r\n",g_horizontal_test,g_vertical_test);

        
        if(g_horizontal_isok == false)
        {

            //如果横轴位置误差绝对值超过2mm
             if( fabs_horizontal_error > 2.0f)
            {
                //朝着减少误差的方向运行
                if(horizontal_error > 0)
                {
                    StepMotorRunReverse(HORIZONTAL_STEP_MOTOR);
                }else
                {
                    StepMotorRunNormal(HORIZONTAL_STEP_MOTOR); 
                }
    
            }else
            {
                StepMotorStop(HORIZONTAL_STEP_MOTOR);
                g_horizontal_isok = true;    
            } 
        }//end of if(g_horizontal_isok == false)
        
        
        
       
        if(g_vertical_isok == false)
        {
           
            //如果纵轴位置误差绝对值超过2mm
             if( fabs_vertical_error > 0.5f)
            {
                //朝着减少误差的方向运行
                if(vertical_error > 0)
                {
                    StepMotorRunNormal(VERTICAL_STEP_MOTOR);         
                }else
                {
                    StepMotorRunReverse(VERTICAL_STEP_MOTOR);        
                }
            }else
            {
                StepMotorStop(VERTICAL_STEP_MOTOR);
                g_vertical_isok = true;    
            } 
        }//end of if(g_vertical_isok == false)
   
    }   
  
       
        
}    


