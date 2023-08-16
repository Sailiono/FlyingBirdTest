/*步进电机驱动(括号内为板子上标注的)
调攻角电机： 脉冲：PB6(TAIL:PB6)   方向:PC2(STEP:D)
尾翼调横向电机： 脉冲：PA0(WING A)   方向:PC3(STEP:P)
尾翼调纵向电机： 脉冲：PA1(WING B)   方向:PB7(TAIL:PB7)
*/



#include "step_motor_driver.h"



void step_motor_init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能   
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM3时钟使能 
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //使能GPIOA时钟	    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTC时钟
    

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);   //GPIOA0复用为定时器5 用于控制横向步进电机
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);   //GPIOA1复用为定时器5 用于控制纵向步进电机    
    
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);   //GPIOB6复用为定时器4 用于控制攻角机构

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;               //GPIOB6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	    //速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);                   //初始化PB6
	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;    //GPIOA0,GPIOA1
	GPIO_Init(GPIOA,&GPIO_InitStructure);                   //初始化PA0 PA1    
      

	TIM_TimeBaseStructure.TIM_Prescaler=84;                 //定时器分频  psc=84
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=800;                   //自动重装载值 arr=800
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);          //初始化定时器4
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);          //初始化定时器5	
    

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //选择定时器模式:TIM5脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_Pulse=0;                              //初始化时比较值CCR
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;    
	
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据指定的参数初始化外设TIM1 3OC1
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
  
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //根据指定的参数初始化外设TIM1 3OC1
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据指定的参数初始化外设TIM1 3OC1
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器

    TIM_SetCompare1(TIM5,0);	//控制横向步进电机，让其静止
    TIM_SetCompare2(TIM5,0);	//控制纵向步进电机，让其静止    
    TIM_SetCompare1(TIM4,0);	//攻角步进电机

    
    TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
    TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPE使能     
    
   
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4	
	TIM_Cmd(TIM5, ENABLE);  //使能TIM4	



    //DIR:PC2初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
    
    GPIO_SetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_3);//GPIOC2设置高 正转
} 




void StepMotorRunNormal(u8 motor)
{
    switch (motor)
    {
        case ATTACK_STEP_MOTOR:
        ATTACK_DIR = 1;
        TIM_SetCompare1(TIM4,800/2);	//高电平脉宽为50%
        break;
        
        case HORIZONTAL_STEP_MOTOR:
        HORIZONTAL_DIR = 1; 
        TIM_SetCompare1(TIM5,800/2); 
        break;
        
        case VERTICAL_STEP_MOTOR:
        VERTICAL_DIR = 1; 
        TIM_SetCompare2(TIM5,800/2); 
        break;           
    }
}


void StepMotorRunReverse(u8 motor)
{
    switch (motor)
    {
        case ATTACK_STEP_MOTOR:
        ATTACK_DIR = 0;
        TIM_SetCompare1(TIM4,800/2);	//高电平脉宽为50%
        break;
            
        case HORIZONTAL_STEP_MOTOR:
        HORIZONTAL_DIR = 0; 
        TIM_SetCompare1(TIM5,800/2); 
        break;
        
        case VERTICAL_STEP_MOTOR:
        VERTICAL_DIR = 0; 
        TIM_SetCompare2(TIM5,800/2); 
        break;            
    }  
}


void StepMotorStop(u8 motor)
{
    switch (motor)
    {
        case ATTACK_STEP_MOTOR:
        TIM_SetCompare1(TIM4,0);	//高电平脉宽为0
        break;
        
        case HORIZONTAL_STEP_MOTOR: 
        TIM_SetCompare1(TIM5,0); 
        break;
        
        case VERTICAL_STEP_MOTOR:
        TIM_SetCompare2(TIM5,0); 
        break; 
    }  
}





