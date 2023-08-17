#include "sbus.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"

#define CALIBRATION_DISABLE 0
#define ACCGYRO_CALIBRATION_ENABLE 1
#define MAG_CALIBRATION_ENABLE 2

#define CALIBRATE_NUM_TOP 400       //У׼�������ʱ�����ֵ 352
#define CALIBRATE_NUM_MIDDLE 1200   //У׼����м�ʱ�����ֵ 1024
#define CALIBRATE_FLAG_CRITICLE 20  //У׼״̬�л�ʱ�ٽ�ֵ


static u32 gs_calibrate_flag_num[3];    //У׼����״̬flag��ֵ
static u8 gs_rec_sbus_data[25];

SemaphoreHandle_t sbus_dma_binarysemaphore;     //����һ����ֵ�ź�������
RcRawDataSTypeDef myrc;


/*
 * SBUS��ʼ��
 */
void SbusInit(void)
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

	//DMA1ͨ������ DMA1_Stream5_Channel_4
	DMA_DeInit(DMA1_Stream5);												//׼������
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//ѡ��ͨ��4����ӦӲ����
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);		//�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)gs_rec_sbus_data;        //�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					//DMA���䷽��
	DMA_InitStructure.DMA_BufferSize = 25;				    				//����DMA�ڴ���ʱ�������ĳ���
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
	USART_InitStructure.USART_BaudRate = 100000;									//�����ʰ���SBUSЭ������100000
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;						//�ֳ�Ϊ9λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_2;						    //����ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;							//żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;									//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure);										//��ʼ������2
    USART_Cmd(USART2, ENABLE);  													//ʹ�ܴ���2

	//ʹ�ܴ���2��DMA����
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);

	//Usart2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;							    //����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;					        //��ռ���ȼ�5 ��RreeRTOS����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;							    //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								    //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);												    //����ָ���Ĳ�����ʼ��VIC�Ĵ�����

	//����2�ж����� �������ж�
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	
	//ʹ�ܴ���1�ж�
    USART_Cmd(USART2, ENABLE);
		
	//����һ����ֵ�ź���
	sbus_dma_binarysemaphore	=	xSemaphoreCreateBinary();																													
}





/*
����2�жϷ�����
��DMA������һ֡���ݺ󣬼�������������ж�
�ͷ�һ����ֵ�ź���
 */
void USART2_IRQHandler(void)																						
{   
	BaseType_t xHigherPriorityTaskWoken;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)						    //��������˽�����һ֡���ݵ��ж�
	{	 
		USART2->SR; 																																					//��������жϱ�־λ
        USART2->DR;	 	
		xSemaphoreGiveFromISR(sbus_dma_binarysemaphore,	&xHigherPriorityTaskWoken); //�ͷ�һ����ֵ�ź���
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);							    //�����Ҫ�Ļ�����һ�������л�
	}	
} 




/*
SBUS�������� ��Ҫ�õ�ͨ��1 2 3 5 16
 */
void SbusAnalysis(u8 * sbus_buffer_data)
{
  // printf("%d\r\n",sbus_buffer_data[0]);
	if(sbus_buffer_data[0]==0x0f)
	{
	//	printf("NOT into/r/n");
		myrc.aileron	=	((sbus_buffer_data[1]	|	sbus_buffer_data[2]<<8)	& 0x07FF);  //ͨ��1��ת  
		myrc.elevator	=	((sbus_buffer_data[2]>>3	|	sbus_buffer_data[3]<<5)	& 0x07FF);  //ͨ��2����
		myrc.throttle	=	((sbus_buffer_data[3]>>6	|	sbus_buffer_data[4]<<2 |	sbus_buffer_data[5]<<10)  & 0x07FF);    //ͨ��3����
		myrc.aileron_elevator		=	((sbus_buffer_data[5]>>1 |	sbus_buffer_data[6]<<7)	& 0x07FF);  //ͨ��4
        myrc.mode	=	((sbus_buffer_data[6]>>4	|	sbus_buffer_data[7]<<4)	& 0x07FF);  //ͨ��5����ģʽ
		myrc.calibrate_num	=	((sbus_buffer_data[7]>>7 |	sbus_buffer_data[8]<<1 |sbus_buffer_data[9]<<9)	&	0x07FF);//ͨ��6 У׼
        myrc.loggerbutton_num = ((sbus_buffer_data[10]>>5 |	sbus_buffer_data[11]<<3)	& 0x07FF); //ͨ��8 ������־��¼����
        //		channel_data[7]	=	((sbus_buffer_data[9]>>2 |	sbus_buffer_data[10]<<6)	& 0x07FF);
//		channel_data[8]	=	((sbus_buffer_data[10]>>5 |	sbus_buffer_data[11]<<3)	& 0x07FF);
//		channel_data[9]	=	(sbus_buffer_data[12]	|	sbus_buffer_data[13]<<8)	& 0x07FF);
//		channel_data[10]  =	((sbus_buffer_data[13]>>3	|	sbus_buffer_data[14]<<5)	& 0x07FF);
//		channel_data[11]  =	((sbus_buffer_data[14]>>6	|	sbus_buffer_data[15]<<2|sbus_buffer_data[16]<<10) & 0x07FF);
//		channel_data[12]  =	((sbus_buffer_data[16]>>1	|	sbus_buffer_data[17]<<7)	& 0x07FF);
//		channel_data[13]  =	((sbus_buffer_data[17]>>4	|	sbus_buffer_data[18]<<4)	& 0x07FF);
//		channel_data[14]  =	((sbus_buffer_data[18]>>7	|	sbus_buffer_data[19]<<1|sbus_buffer_data[20]<<9)	&	0x07FF);
//      channel_data[15]  =	((sbus_buffer_data[20]>>2	|	sbus_buffer_data[21]<<6)	& 0x07FF);	
//		channel_data[16]  =	((sbus_buffer_data[21]>>5	|	sbus_buffer_data[22]<<3)	& 0x07FF); 
		
        //���ں���У׼״̬�л�
//        if(myrc.calibrate_num < CALIBRATE_NUM_TOP)
//        {
//            gs_calibrate_flag_num[0] = gs_calibrate_flag_num[0] + 1;
//            gs_calibrate_flag_num[1] = 0;
//            gs_calibrate_flag_num[2] = 0;
//        }else if(myrc.calibrate_num < CALIBRATE_NUM_MIDDLE) 
//        {
//            gs_calibrate_flag_num[0] = 0;
//            gs_calibrate_flag_num[1] = gs_calibrate_flag_num[1] + 1;
//            gs_calibrate_flag_num[2] = 0; 
//        }else
//        {
//            gs_calibrate_flag_num[0] = 0;
//            gs_calibrate_flag_num[1] = 0;
//            gs_calibrate_flag_num[2] = gs_calibrate_flag_num[2] + 1; 
//        }
                
               
    }            
        
} 

bool GetFlightLoggerState(void)
{
    if( myrc.loggerbutton_num > 600)
    {
        return true;
    }else
    {
        return false;
    }
}



/*
�����������Ľӿ�
 */
void GetRcValue(RcRawDataSTypeDef* get)
{
    get->aileron = myrc.aileron;
    get->aileron_elevator = myrc.aileron_elevator;
    get->elevator = myrc.elevator;
    get->throttle = myrc.throttle;
    get->mode = myrc.mode;
	get->calibrate_num = myrc.calibrate_num;
    if(gs_calibrate_flag_num[0] > CALIBRATE_FLAG_CRITICLE)
    {
        get->calibrate_mode = CALIBRATION_DISABLE;
    }else if (gs_calibrate_flag_num[1] > CALIBRATE_FLAG_CRITICLE)
    {
        get->calibrate_mode = ACCGYRO_CALIBRATION_ENABLE;
    }else if (gs_calibrate_flag_num[2] > CALIBRATE_FLAG_CRITICLE)
    {
        get->calibrate_mode = MAG_CALIBRATION_ENABLE;
    }
 
}



/*
sbus������
ͨ���ź����������ж��������ͨ��
���ж�give��һ���ź����������ھ���̬
����sbus֡�Ľ���
 */

void sbus_task(void *param)
{																																																		//��ʱ��ʱ�ӣ�ÿ��1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();	
	//��ʱ
//	BaseType_t err	=	pdFALSE;
	while(1)
	{
				vTaskDelayUntil(&lastWakeTime, 20);																																							/*1ms������ʱ*/
	//	printf("error\r\n");
				SbusAnalysis(gs_rec_sbus_data);
//                printf("pitch=%d \t roll=%d \t throttle=%d \t aileron_elevator=%d \t mode=%d loggerbutton=%d\r\n"
//                    ,myrc.elevator,myrc.aileron,myrc.throttle,myrc.aileron_elevator,myrc.mode,myrc.loggerbutton_num);
               
			
		//printf("error7.26\r\n");

	}//end of while(1)

}




