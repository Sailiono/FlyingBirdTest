#include "hall.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "projdefs.h"

/*
���ܣ������˶�Ƶ��
ԭ������ģ��ÿ�ξ�������ʱ���������ƽ�仯���Ӷ�����һ���ⲿ�жϣ������жϵ�ʱ�̲Ϊ�˶����ڣ�������ΪƵ��
��Դ���ⲿ�жϣ�PB0
*/

#include "hall.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "projdefs.h"

/*
�����õ�PB0��������Ԫ�����տڣ��ⲿ�жϴ���
*/

static float frequency_last;
float g_frequency_test;
static u32 t_last;
static u32 t_now;
extern float wzhfre;
float wzhfre=0;
////SemaphoreHandle_t gs_hall_binarysemaphore;	
extern u16 mesg[5];

void EXTI0_IRQHandler(void)                 //�ж�0�жϷ�����
{
  
        if(HALL_TEST==1)
        { 
                
            t_now = FreeRTOSRunTimeTicks;

            if(t_now - t_last >= 2000) //????????150ms???
            {
                g_frequency_test = 10000.f/(t_now - t_last);
                
                if(g_frequency_test<0 || g_frequency_test>10)   //�����쳣ʱ����֮ǰһ�ε�����
                {
                    g_frequency_test = frequency_last;
                }
                frequency_last = g_frequency_test;
                t_last = t_now;
            }
                
           // printf("%f\r\n", g_frequency_test);
						wzhfre=g_frequency_test;
						mesg[2]=(u16)(g_frequency_test*1000);
            EXTI_ClearITPendingBit(EXTI_Line0);//����ж���0�ı�־λ
           
        }//end of if(HALL_TEST==0)

}

//void hall_task(void *param)
//{																																																		//��ʱ��ʱ�ӣ�ÿ��1ms +1
//	u32 lastWakeTime	=	xTaskGetTickCount();																																				//��ʱ
//	u8 nene;
//	mesg[4]=0x0a;
//	while(1)
//	{
//		vTaskDelayUntil(&lastWakeTime, 15);																		/*15ms������ʱ*/
//		if(HALL_TEST==1)
//        { 
//                
//            t_now = FreeRTOSRunTimeTicks;

//            if(t_now - t_last >= 2000) //????????150ms???
//            {
//                g_frequency_test = 10000.f/(t_now - t_last);
//                
//                if(g_frequency_test<0 || g_frequency_test>10)   //�����쳣ʱ����֮ǰһ�ε�����
//                {
//                    g_frequency_test = frequency_last;
//                }
//                frequency_last = g_frequency_test;
//                t_last = t_now;
//            }
//                
//            //printf("%f\r\n", g_frequency_test);
//						wzhfre=g_frequency_test;
//						mesg[2]=(u16)(g_frequency_test*1000);
//					}
//	}
//}
//???????
//???PC5?????.
void HALL_Init(void)
{
    EXTI_InitTypeDef   EXTI_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//�趨Ϊ��ͨ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//��Ӧ�ٶ��趨Ϊ���100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//�����������趨Ϊ����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB
    
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//SYSCFGʱ��ʹ��
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);//PB0��ӦEXTI0��
	
	  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�趨Ϊ�����ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
 
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//�ⲿ�ж�0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    t_now = FreeRTOSRunTimeTicks;
		t_last = FreeRTOSRunTimeTicks;
    //gs_hall_binarysemaphore	=	xSemaphoreCreateBinary(); 
}







