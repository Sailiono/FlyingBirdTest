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


u8 rec_oid_data[20]; //dma 缓存数组
SemaphoreHandle_t oid_dma_binarysemaphore;					//定义一个二值信号量变量

float g_attack_angle_test;


//初始化 串口2 DMA 用于接收SBUS消息
void OidEncoderInit(void)
{
	//定义结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;						//定义DMA初始化结构体
	
	//开启外设时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);   //使能USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);	//打开DMA1时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2IO口配置初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;           //GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);               //初始化PA3
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;           //GPIOA4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);               //初始化PA4
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);                   //设置为接收模式


	//DMA1通道配置 DMA1_Stream5_Channel_4
	DMA_DeInit(DMA1_Stream5);												//准备配置
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//选择通道4（对应硬件表）
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);		//外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rec_oid_data;        //内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					//DMA传输方向
	DMA_InitStructure.DMA_BufferSize = 20;				    				//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//设置DMA外设地址自加 只有一个外设设为不自加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//设置DMA的内存递增模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//内存数据字长
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							//设置DMA的传输方式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;		    		//设置该DMA流的优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	
	//使能通道
	DMA_Cmd(DMA1_Stream5,ENABLE);																					

  //初始化USART2设置
	USART_InitStructure.USART_BaudRate = 115200;									//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;						    //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;							    //无校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;									//收发模式
    USART_Init(USART2, &USART_InitStructure);										//初始化串口2
    USART_Cmd(USART2, ENABLE);  													//使能串口2

	//使能串口2的DMA接收
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);

	//Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;								//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;							//抢占优先级5 归RreeRTOS管理
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;								//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);													//根据指定的参数初始化VIC寄存器、

	//串口2中断配置 开空闲中断
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	
	//使能串口1中断
    USART_Cmd(USART2, ENABLE);
		
	//创建一个二值信号量
	oid_dma_binarysemaphore	=	xSemaphoreCreateBinary();																													
}




//串口2中断服务函数
void USART2_IRQHandler(void)																						
{   
	BaseType_t xHigherPriorityTaskWoken;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)												//如果触发了接收完一帧数据的中断
	{	 
		USART2->SR; 																																					//用于清除中断标志位
        USART2->DR;	 	
		xSemaphoreGiveFromISR(oid_dma_binarysemaphore,	&xHigherPriorityTaskWoken);						//释放一个二值信号量
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);													//如果需要的话进行一次任务切换
	}	
} 

//计算校验位
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


//OID解析函数 
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



//oid encoder任务函数
void OidEncoderTask(void *param)
{																																																		//定时器时钟，每隔1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();																																				//定时
	BaseType_t err	=	pdFALSE;
	while(1)
	{																																										/*1ms周期延时*/
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




