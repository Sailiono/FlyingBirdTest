#include "hall.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "projdefs.h"

/*
功能：计算扑动频率
原理：霍尔模块每次经过磁铁时，会产生电平变化，从而产生一个外部中断，两次中断的时刻差即为扑动周期，倒数即为频率
资源：外部中断，PB0
*/

#include "hall.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "projdefs.h"

/*
将不用的PB0用作霍尔元件接收口，外部中断触发
*/

static float frequency_last;
float g_frequency_test;
static u32 t_last;
static u32 t_now;
extern float wzhfre;
float wzhfre=0;
////SemaphoreHandle_t gs_hall_binarysemaphore;	
extern u16 mesg[5];

void EXTI0_IRQHandler(void)                 //中断0中断服务函数
{
  
        if(HALL_TEST==1)
        { 
                
            t_now = FreeRTOSRunTimeTicks;

            if(t_now - t_last >= 2000) //????????150ms???
            {
                g_frequency_test = 10000.f/(t_now - t_last);
                
                if(g_frequency_test<0 || g_frequency_test>10)   //数据异常时采用之前一次的数据
                {
                    g_frequency_test = frequency_last;
                }
                frequency_last = g_frequency_test;
                t_last = t_now;
            }
                
           // printf("%f\r\n", g_frequency_test);
						wzhfre=g_frequency_test;
						mesg[2]=(u16)(g_frequency_test*1000);
            EXTI_ClearITPendingBit(EXTI_Line0);//清除中断线0的标志位
           
        }//end of if(HALL_TEST==0)

}

//void hall_task(void *param)
//{																																																		//定时器时钟，每隔1ms +1
//	u32 lastWakeTime	=	xTaskGetTickCount();																																				//定时
//	u8 nene;
//	mesg[4]=0x0a;
//	while(1)
//	{
//		vTaskDelayUntil(&lastWakeTime, 15);																		/*15ms周期延时*/
//		if(HALL_TEST==1)
//        { 
//                
//            t_now = FreeRTOSRunTimeTicks;

//            if(t_now - t_last >= 2000) //????????150ms???
//            {
//                g_frequency_test = 10000.f/(t_now - t_last);
//                
//                if(g_frequency_test<0 || g_frequency_test>10)   //数据异常时采用之前一次的数据
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
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//设定为普通输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//反应速度设定为最快100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上升触发，设定为下拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB
    
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//SYSCFG时钟使能
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);//PB0对应EXTI0线
	
	  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //设定为上升沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
 
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    t_now = FreeRTOSRunTimeTicks;
		t_last = FreeRTOSRunTimeTicks;
    //gs_hall_binarysemaphore	=	xSemaphoreCreateBinary(); 
}







