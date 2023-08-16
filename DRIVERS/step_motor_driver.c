/*�����������(������Ϊ�����ϱ�ע��)
�����ǵ���� ���壺PB6(TAIL:PB6)   ����:PC2(STEP:D)
β����������� ���壺PA0(WING A)   ����:PC3(STEP:P)
β����������� ���壺PA1(WING B)   ����:PB7(TAIL:PB7)
*/



#include "step_motor_driver.h"



void step_motor_init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4ʱ��ʹ��   
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM3ʱ��ʹ�� 
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //ʹ��GPIOAʱ��	    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTBʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTCʱ��
    

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);   //GPIOA0����Ϊ��ʱ��5 ���ڿ��ƺ��򲽽����
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);   //GPIOA1����Ϊ��ʱ��5 ���ڿ������򲽽����    
    
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);   //GPIOB6����Ϊ��ʱ��4 ���ڿ��ƹ��ǻ���

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;               //GPIOB6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	    //�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);                   //��ʼ��PB6
	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;    //GPIOA0,GPIOA1
	GPIO_Init(GPIOA,&GPIO_InitStructure);                   //��ʼ��PA0 PA1    
      

	TIM_TimeBaseStructure.TIM_Prescaler=84;                 //��ʱ����Ƶ  psc=84
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=800;                   //�Զ���װ��ֵ arr=800
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);          //��ʼ����ʱ��4
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);          //��ʼ����ʱ��5	
    

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //ѡ��ʱ��ģʽ:TIM5�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_Pulse=0;                              //��ʼ��ʱ�Ƚ�ֵCCR
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;    
	
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����ָ���Ĳ�����ʼ������TIM1 3OC1
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
  
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //����ָ���Ĳ�����ʼ������TIM1 3OC1
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //����ָ���Ĳ�����ʼ������TIM1 3OC1
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���

    TIM_SetCompare1(TIM5,0);	//���ƺ��򲽽���������侲ֹ
    TIM_SetCompare2(TIM5,0);	//�������򲽽���������侲ֹ    
    TIM_SetCompare1(TIM4,0);	//���ǲ������

    
    TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPEʹ�� 
    TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPEʹ��     
    
   
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4	
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM4	



    //DIR:PC2��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
    
    GPIO_SetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_3);//GPIOC2���ø� ��ת
} 




void StepMotorRunNormal(u8 motor)
{
    switch (motor)
    {
        case ATTACK_STEP_MOTOR:
        ATTACK_DIR = 1;
        TIM_SetCompare1(TIM4,800/2);	//�ߵ�ƽ����Ϊ50%
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
        TIM_SetCompare1(TIM4,800/2);	//�ߵ�ƽ����Ϊ50%
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
        TIM_SetCompare1(TIM4,0);	//�ߵ�ƽ����Ϊ0
        break;
        
        case HORIZONTAL_STEP_MOTOR: 
        TIM_SetCompare1(TIM5,0); 
        break;
        
        case VERTICAL_STEP_MOTOR:
        TIM_SetCompare2(TIM5,0); 
        break; 
    }  
}





