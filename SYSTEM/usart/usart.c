#include "sys.h"
#include "usart.h"	
#include "hall.h" 
#include "motor_steering.h"
#include "string.h"


////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用FreeRTOS,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//ucos 使用	  
#endif
#include "queue.h"

#include "main.h"

#define UARTSLK_DATA_TIMEOUT_MS 	1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
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

//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 


static xQueueHandle uartslkDataDelivery;
//static xSemaphoreHandle waitUntilSendDone;  //标志DMA传输完成的信号量

static DMA_InitTypeDef DMA_InitStructure;
static u8 dmaBuffer[64];



/*配置串口DMA*/
void uartslkDmaInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	

	/* USART1 TX DMA 通道配置*/
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


//初始化IO 串口1 
//bound:波特率
void DebugSeriesInit(u32 bound)
{
    
    uartslkDataDelivery = xQueueCreate(1024, sizeof(u8));	/*队列 1024个消息*/
    
   //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

    //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    
	uartslkDmaInit();
    
    
    	/*配置串口非空中断*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/*串口接收数据寄存器非空中断*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
   
    USART_Cmd(USART1, ENABLE);  //使能串口1 
}

/*通过DMA发送原始数据*/
void uart1Dmasend(u8* data,u32 size)
{
		while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	/*等待DMA空闲*/
		memcpy(dmaBuffer, data, size);		/*复制数据到DMA缓冲区*/
		DMA_InitStructure.DMA_BufferSize = size;
		DMA_Init(DMA2_Stream7, &DMA_InitStructure);	/*重新初始化DMA数据流*/
		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);/*开启DMA传输完成中断*/		
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);/* 使能USART DMA TX请求 */
		USART_ClearFlag(USART1, USART_FLAG_TC);		/* 清除传输完成中断标志位 */
		DMA_Cmd(DMA2_Stream7, ENABLE);	/* 使能DMA USART TX数据流 */
//        xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
}



 /*从接收队列读取数据(带超时处理)*/
bool uartslkGetDataWithTimout(u8 *c)
{
	/*接收uartslkDataDelivery(1024个容量)消息*/
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






















/*下面是通过传统的串口发送方式*/
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






































