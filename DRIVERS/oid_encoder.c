#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "projdefs.h"

#include "exfuns.h"
#include "string.h"
#include "timer.h"
#include "labview.h"
#include "oid_encoder.h"

#include "DataTypeDefine.h"


u8 rec_oid_data[20]; //dma ��������
SemaphoreHandle_t oid_dma_binarysemaphore;					//����һ����ֵ�ź�������

float g_attack_angle_test;


//��ʼ�� ����2 DMA ���ڽ���SBUS��Ϣ
void OidEncoderInit(void)
{
	//����ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;						//����DMA��ʼ���ṹ��
	
	//��������ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);   //ʹ��USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	//��DMA1ʱ��
 
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
	//USART2IO�����ó�ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;           //GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);               //��ʼ��PA3
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;           //GPIOA4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);               //��ʼ��PA4
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);                   //����Ϊ����ģʽ


	//DMA1ͨ������ DMA1_Stream5_Channel_4
	DMA_DeInit(DMA1_Stream5);												//׼������
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//ѡ��ͨ��4����ӦӲ����
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);		//�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rec_oid_data;        //�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					//DMA���䷽��
	DMA_InitStructure.DMA_BufferSize = 20;				    				//����DMA�ڴ���ʱ�������ĳ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//����DMA�����ַ�Լ� ֻ��һ��������Ϊ���Լ�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //���������ֳ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�ڴ������ֳ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//����DMA�Ĵ��䷽ʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;		    		//���ø�DMA�������ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//����ͻ�����δ���
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	
	//ʹ��ͨ��
	DMA_Cmd(DMA1_Stream5,ENABLE);																					

  //��ʼ��USART2����
	USART_InitStructure.USART_BaudRate = 115200;									//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;						    //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;							    //��У��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;									//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure);										//��ʼ������2
    USART_Cmd(USART2, ENABLE);  													//ʹ�ܴ���2

	//ʹ�ܴ���2��DMA����
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);

	//Usart2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;								//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;							//��ռ���ȼ�5 ��RreeRTOS����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;								//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);													//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

	//����2�ж����� �������ж�
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	
	//ʹ�ܴ���1�ж�
    USART_Cmd(USART2, ENABLE);
		
	//����һ����ֵ�ź���
	oid_dma_binarysemaphore	=	xSemaphoreCreateBinary();																													
}




//����2�жϷ�����
void USART2_IRQHandler(void)																						
{   
	BaseType_t xHigherPriorityTaskWoken;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)												//��������˽�����һ֡���ݵ��ж�
	{	 
		USART2->SR; 																																					//��������жϱ�־λ
        USART2->DR;	 	
		xSemaphoreGiveFromISR(oid_dma_binarysemaphore,	&xHigherPriorityTaskWoken);						//�ͷ�һ����ֵ�ź���
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);													//�����Ҫ�Ļ�����һ�������л�
	}	
} 

//����У��λ
unsigned int Crc_Count(unsigned char pbuf[],unsigned char num)
{
    int i,j;
    unsigned int wcrc=0xffff;
    for(i=0;i<num;i++)
    {
        wcrc^=(unsigned int)pbuf[i];
        for(j=0;j<8;j++)
        {
            if(wcrc&0x0001)
            {
                wcrc>>=1;
                wcrc^=0xa001;
            }else
            wcrc>>=1;
        }
    }
    return wcrc;
}


//OID�������� 
float OidEncoderAnalysis(u8 * buffer_data)
{
    u16 crc_value;
    u16 circle_value,single_value;
   

    crc_value = Crc_Count(buffer_data,7);
    if(((buffer_data[8]<<8|buffer_data[7]) == crc_value))
    {       
        circle_value = buffer_data[4]<<4 | buffer_data[5]>>4;
        single_value = (buffer_data[5]&0x0f)*256 + buffer_data[6];    
        g_attack_angle_test =  (circle_value+single_value/4096.0f)/280.f*360.f-151.9f+0.537f;// - 63.8494f;        
//        printf("%d,%d,%d,",buffer_data[0],buffer_data[1],buffer_data[2]);
//        printf("%d,%d,%d,%d,%d,",buffer_data[4],buffer_data[5],buffer_data[6],circle_value,single_value);
//        printf("attack_angle=%.2f\r\n",attack_angle);
    }
   return  0;
}



//oid encoder������
void OidEncoderTask(void *param)
{																																																		//��ʱ��ʱ�ӣ�ÿ��1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();																																				//��ʱ
	BaseType_t err	=	pdFALSE;
	while(1)
	{																																										/*1ms������ʱ*/
		if(oid_dma_binarysemaphore	!=	NULL)
		{
			err	=	xSemaphoreTake(oid_dma_binarysemaphore,	portMAX_DELAY);
			if(err	==	pdTRUE)
			{        
                DMA_Cmd(DMA1_Stream5, DISABLE);
			
                OidEncoderAnalysis(rec_oid_data); 

                DMA_Cmd(DMA1_Stream5, ENABLE);
			}//end of if(err	==	pdTRUE)
                
		}//end of if(oid_dma_binarysemaphore !=	NULL)

	}

}




