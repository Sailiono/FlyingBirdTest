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

/*�������ת280Ȧ�����ǻ���ת360�ȣ����仯1����0.7777Ȧ

���貽�����ϸ������Ϊ 800������ÿȦ����ת1����622�����塣

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
        vTaskDelayUntil(&lastWakeTime, 10);	//10ms������ʱ
        num++;
        
        g_vertical_test = (float)(Get_Adc(ADC_Channel_10))/4096.0f*175.0f*3.3f/3.0f-65.0f;  //PC0   ANGLE
        g_horizontal_test = (float)(Get_Adc(ADC_Channel_7))/4096.0f*350.0f*3.3f/3.0f-160.0f; //PA7  VOLT ~ VERTICLE 
        
        
        attack_of_angle_error = g_attack_angle_desired - g_attack_angle_test;
        fabs_attack_of_angle_error = fabs(attack_of_angle_error);
        
        horizontal_error = g_horizontal_desired - g_horizontal_test;
        fabs_horizontal_error = fabs(horizontal_error);
        
        vertical_error = g_vertical_desired - g_vertical_test;
        fabs_vertical_error = fabs(vertical_error);
       
  
        
        //ȷ������һ��������ת��ֻ�н��յ���λ��ָ��Żᶯ
        if(g_attack_of_angle_isok == false)
        {
            //�������0.05�ȣ�������������
             if( fabs_attack_of_angle_error>0.05f)
            {
                //���ż������ķ�������
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

            //�������λ��������ֵ����2mm
             if( fabs_horizontal_error > 2.0f)
            {
                //���ż������ķ�������
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
           
            //�������λ��������ֵ����2mm
             if( fabs_vertical_error > 0.5f)
            {
                //���ż������ķ�������
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


