#include "sys.h"
#include "usart.h"	
#include "hall.h" 
#include "motor_steering.h"
#include "string.h"


////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��FreeRTOS,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//ucos ʹ��	  
#endif
#include "queue.h"

#include "main.h"

#define UARTSLK_DATA_TIMEOUT_MS 	1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
    return 0;
} 
//__use_no_semihosting was requested, but _ttywrch was
int _ttywrch(int ch)
{
ch = ch;
    return 0;
}

//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 


static xQueueHandle uartslkDataDelivery;
//static xSemaphoreHandle waitUntilSendDone;  //��־DMA������ɵ��ź���

static DMA_InitTypeDef DMA_InitStructure;
static u8 dmaBuffer[64];



/*���ô���DMA*/
void uartslkDmaInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	

	/* USART1 TX DMA ͨ������*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)dmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


//��ʼ��IO ����1 
//bound:������
void DebugSeriesInit(u32 bound)
{
    
    uartslkDataDelivery = xQueueCreate(1024, sizeof(u8));	/*���� 1024����Ϣ*/
    
   //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

    //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    
	uartslkDmaInit();
    
    
    	/*���ô��ڷǿ��ж�*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/*���ڽ������ݼĴ����ǿ��ж�*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   
    USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
}

/*ͨ��DMA����ԭʼ����*/
void uart1Dmasend(u8* data,u32 size)
{
		while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	/*�ȴ�DMA����*/
		memcpy(dmaBuffer, data, size);		/*�������ݵ�DMA������*/
		DMA_InitStructure.DMA_BufferSize = size;
		DMA_Init(DMA2_Stream7, &DMA_InitStructure);	/*���³�ʼ��DMA������*/
		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);/*����DMA��������ж�*/		
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);/* ʹ��USART DMA TX���� */
		USART_ClearFlag(USART1, USART_FLAG_TC);		/* �����������жϱ�־λ */
		DMA_Cmd(DMA2_Stream7, ENABLE);	/* ʹ��DMA USART TX������ */
//        xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
}



 /*�ӽ��ն��ж�ȡ����(����ʱ����)*/
bool uartslkGetDataWithTimout(u8 *c)
{
	/*����uartslkDataDelivery(1024������)��Ϣ*/
	if (xQueueReceive(uartslkDataDelivery, c, UARTSLK_DATA_TIMEOUT_TICKS) == pdTRUE)	
	{
		return true;
	}
	*c = 0;
	return false;
}



void  DMA2_Stream7_IRQHandler(void)	
{
//    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
	DMA_Cmd(DMA2_Stream7, DISABLE);
    
//  xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
}






















/*������ͨ����ͳ�Ĵ��ڷ��ͷ�ʽ*/
void usart1_send_char(u8 temp)
{
    USART_SendData(USART1,(u8)temp);        
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET); 
}

void uart1_send_buff(u8* buf,u32 len)         
{    
    u32 i;    
    for(i=0;i<len;i++)    
    usart1_send_char(buf[i]);  
} 






































