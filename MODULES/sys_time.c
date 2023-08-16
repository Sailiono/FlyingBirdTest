#include "sys_time.h"
#include "mixer.h"
#include "usart.h"

//系统控制周期和系统时刻





//定时器3用于系统控制周期中断  50HZ
u8 g_sys_timer_flag=SYS_STOP;


//系统控制周期初始化
//arr：2000  psc:840    T=arr*psc/84(us)   20ms 50Hz 
void sys_time_controlIRQ_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = 5000-1; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



//定时器2用于计时器 用于PID控制 传感器时间等
void TIM2_Configuration(void) 
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2,ENABLE);	
}
uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}



void g_sys_time_init(void)
{
    TIM2_Configuration();      //计时器
    sys_time_controlIRQ_Init();//系统中断
}



////系统中断服务函数，只用来设定标志位
//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
//	{
//		g_sys_timer_flag=SYS_RUN;        
//	} 
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
//}



//系统运行函数  
void g_sys_move(void)
{
	
	
    if(g_sys_timer_flag==SYS_RUN)
		{
			//printf("time=%d\r\n",Get_Time_Micros());
		   //printf("systime");
//		  g_motor_run();
		  g_sys_timer_flag=SYS_STOP;
		}


}



