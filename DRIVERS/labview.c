#include "main.h"
#include "step_motor_control.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ư�-LJT-v1
//gps��������	   
//ljt
//��������:2020/3/1
//ʹ�õ���ԴΪ PA3-USART3_RX  DMA
//�汾��V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////	 

static u8 rec_gps_data[GPS_GNGGA_MESSAGE_MAX];


SemaphoreHandle_t labview_dma_binarysemaphore;	//����һ����ֵ�ź�������





//��ʼ�� ����3 DMA ���ڽ���GPS��Ϣ
void LabviewSeriesInit(void)
{
	//����ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;										
	
	//��������ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);   
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 
	
	//USART3IO�����ó�ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

	//DMA1ͨ������ DMA1_Stream1_Channel_4
	DMA_DeInit(DMA1_Stream1);												//׼������
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;						    //ѡ��ͨ��4����ӦӲ����
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);		//�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rec_gps_data;         //�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					//DMA���䷽��
	DMA_InitStructure.DMA_BufferSize = GPS_GNGGA_MESSAGE_MAX;		    	//����DMA�ڴ���ʱ�������ĳ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//����DMA�����ַ�Լ� ֻ��һ��������Ϊ���Լ�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //���������ֳ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�ڴ������ֳ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//����DMA�Ĵ��䷽ʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					//���ø�DMA�������ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//����ͻ�����δ���
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	
	//ʹ��ͨ��
	DMA_Cmd(DMA1_Stream1,ENABLE);											

    //��ʼ��USART3����
	USART_InitStructure.USART_BaudRate = 115200;								
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;				
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					
	USART_InitStructure.USART_Parity = USART_Parity_No;						
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;							
    USART_Init(USART3, &USART_InitStructure);								
    USART_Cmd(USART3, ENABLE);  											

	//ʹ�ܴ���3��DMA����
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);

	//Usart3 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;						
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;					
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;						
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
	NVIC_Init(&NVIC_InitStructure);											

	//����3�ж����� �������ж�
	USART_ITConfig(USART3,USART_IT_TC,DISABLE);
	USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART3,USART_IT_TXE,DISABLE);
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
	
	//ʹ�ܴ���3�ж�
    USART_Cmd(USART3, ENABLE);
		
	//����һ����ֵ�ź���
	labview_dma_binarysemaphore	=	xSemaphoreCreateBinary();																													
}


//����3�жϷ�����
void USART3_IRQHandler(void)																						
{   
	BaseType_t xHigherPriorityTaskWoken;
	
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)					        //��������˽�����һ֡���ݵ��ж�
	{	 
		USART3->SR; 															    //��������жϱ�־λ
        USART3->DR;	 	
		xSemaphoreGiveFromISR(labview_dma_binarysemaphore,	&xHigherPriorityTaskWoken);	//�ͷ�һ����ֵ�ź���
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);								//�����Ҫ�Ļ�����һ�������л�
	}	
} 





//����λ���õ���ָ��
float g_frequency_desired,g_attack_angle_desired,g_horizontal_desired,g_vertical_desired;
bool g_printfbisok=false;

float elevator_desired,aileron_desired;

//labview����
void labview_task(void *param)
{			
	u32 lastWakeTime	=	xTaskGetTickCount();    //��ʱ
	BaseType_t err	=	pdFALSE;
	while(1)
	{        
		if(labview_dma_binarysemaphore	!=	NULL)
		{
			err	=	xSemaphoreTake(labview_dma_binarysemaphore,	portMAX_DELAY);
			if(err	==	pdTRUE)
			{
				DMA_Cmd(DMA1_Stream1, DISABLE);
				
				DMA1_Stream1->NDTR = GPS_GNGGA_MESSAGE_MAX;
				
                //�����յ�һ֡����labview������
                if(rec_gps_data[0] == '$') //&& (g_attack_of_angle_isok ==true) && (g_horizontal_isok == true) && (g_vertical_isok == true))
                { 
                    
                    sscanf((const char*)rec_gps_data,"$%f,%f,%f,%f,%f,%f",&g_frequency_desired,&g_attack_angle_desired,
                                &g_horizontal_desired,&g_vertical_desired,&elevator_desired,&aileron_desired);
   
                    g_frequency_desired = g_frequency_desired<0.0f?0:g_frequency_desired;
                    g_frequency_desired = g_frequency_desired>4.0f?4.0f:g_frequency_desired;                      
                    
                    g_attack_angle_desired = g_attack_angle_desired<-5.0f?-5.0f:g_attack_angle_desired;
                    g_attack_angle_desired = g_attack_angle_desired>25.0f?25.0f:g_attack_angle_desired;   

                    
               
                    g_horizontal_desired = g_horizontal_desired<0?0:g_horizontal_desired;
                    g_horizontal_desired = g_horizontal_desired>180.0f?180.0f:g_horizontal_desired;
             
                    g_vertical_desired = g_vertical_desired<0?0:g_vertical_desired;
                    g_vertical_desired = g_vertical_desired>90.0f?90.0f:g_vertical_desired;
                    
                    
                    elevator_desired = -elevator_desired;//Ϊ�˲���Ϊ��ʱ������
                    elevator_desired  = elevator_desired>45.0f?45.0f:elevator_desired;
                    elevator_desired  = elevator_desired<-45.0f?-45.0f:elevator_desired;
                    
                    
                    aileron_desired = -aileron_desired;//Ϊ�˲���Ϊ��ʱ����ת
                    aileron_desired  = aileron_desired>45.0f?45.0f:aileron_desired;
                    aileron_desired  = aileron_desired<-45.0f?-45.0f:aileron_desired;
                    
                    g_printfbisok = false;
                    g_attack_of_angle_isok = false;
                    g_horizontal_isok = false;
                    g_vertical_isok = false;

                    printf("aa\r\n");
                    uart1Dmasend((u8*)"aa\r\n",4);
                    
                    
                    motor_control(ELEVATOR,elevator_desired/45.0f);
                    motor_control(AILERON,aileron_desired/45.0f);
                    
                    error_is_limit_flag = 0;
                    error_is_limit_count = 0;
     
                }

				DMA_Cmd(DMA1_Stream1, ENABLE);
			}
		}//end of if(labview_dma_binarysemaphore!=	NULL)

	}//end of while(1)

}//end of gps_task




