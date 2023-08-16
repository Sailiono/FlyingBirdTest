#include "sys_time.h"
#include "mixer.h"
#include "usart.h"

//ϵͳ�������ں�ϵͳʱ��





//��ʱ��3����ϵͳ���������ж�  50HZ
u8 g_sys_timer_flag=SYS_STOP;


//ϵͳ�������ڳ�ʼ��
//arr��2000  psc:840    T=arr*psc/84(us)   20ms 50Hz 
void sys_time_controlIRQ_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = 5000-1; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



//��ʱ��2���ڼ�ʱ�� ����PID���� ������ʱ���
void TIM2_Configuration(void) 
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M ��ʱ��  
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
    TIM2_Configuration();      //��ʱ��
    sys_time_controlIRQ_Init();//ϵͳ�ж�
}



////ϵͳ�жϷ�������ֻ�����趨��־λ
//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
//	{
//		g_sys_timer_flag=SYS_RUN;        
//	} 
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
//}



//ϵͳ���к���  
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



