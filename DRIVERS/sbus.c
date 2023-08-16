#include "sbus.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"

#define CALIBRATION_DISABLE 0
#define ACCGYRO_CALIBRATION_ENABLE 1
#define MAG_CALIBRATION_ENABLE 2

#define CALIBRATE_NUM_TOP 400       //校准舵打到最上时的输出值 352
#define CALIBRATE_NUM_MIDDLE 1200   //校准舵打到中间时的输出值 1024
#define CALIBRATE_FLAG_CRITICLE 20  //校准状态切换时临界值


static u32 gs_calibrate_flag_num[3];    //校准舵量状态flag的值
static u8 gs_rec_sbus_data[25];

SemaphoreHandle_t sbus_dma_binarysemaphore;     //定义一个二值信号量变量
RcRawDataSTypeDef myrc;


/*
 * SBUS初始化
 */
void SbusInit(void)
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

	//DMA1通道配置 DMA1_Stream5_Channel_4
	DMA_DeInit(DMA1_Stream5);												//准备配置
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//选择通道4（对应硬件表）
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);		//外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)gs_rec_sbus_data;        //内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					//DMA传输方向
	DMA_InitStructure.DMA_BufferSize = 25;				    				//设置DMA在传输时缓冲区的长度
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
	USART_InitStructure.USART_BaudRate = 100000;									//波特率按照SBUS协议设置100000
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;						//字长为9位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_2;						    //两个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;							//偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;									//收发模式
    USART_Init(USART2, &USART_InitStructure);										//初始化串口2
    USART_Cmd(USART2, ENABLE);  													//使能串口2

	//使能串口2的DMA接收
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);

	//Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;							    //串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;					        //抢占优先级5 归RreeRTOS管理
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;							    //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								    //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);												    //根据指定的参数初始化VIC寄存器、

	//串口2中断配置 开空闲中断
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	
	//使能串口1中断
    USART_Cmd(USART2, ENABLE);
		
	//创建一个二值信号量
	sbus_dma_binarysemaphore	=	xSemaphoreCreateBinary();																													
}





/*
串口2中断服务函数
当DMA接收完一帧数据后，即触发接收完成中断
释放一个二值信号量
 */
void USART2_IRQHandler(void)																						
{   
	BaseType_t xHigherPriorityTaskWoken;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)						    //如果触发了接收完一帧数据的中断
	{	 
		USART2->SR; 																																					//用于清除中断标志位
        USART2->DR;	 	
		xSemaphoreGiveFromISR(sbus_dma_binarysemaphore,	&xHigherPriorityTaskWoken); //释放一个二值信号量
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);							    //如果需要的话进行一次任务切换
	}	
} 




/*
SBUS解析函数 主要用到通道1 2 3 5 16
 */
void SbusAnalysis(u8 * sbus_buffer_data)
{
  // printf("%d\r\n",sbus_buffer_data[0]);
	if(sbus_buffer_data[0]==0x0f)
	{
	//	printf("NOT into/r/n");
		myrc.aileron	=	((sbus_buffer_data[1]	|	sbus_buffer_data[2]<<8)	& 0x07FF);  //通道1滚转  
		myrc.elevator	=	((sbus_buffer_data[2]>>3	|	sbus_buffer_data[3]<<5)	& 0x07FF);  //通道2俯仰
		myrc.throttle	=	((sbus_buffer_data[3]>>6	|	sbus_buffer_data[4]<<2 |	sbus_buffer_data[5]<<10)  & 0x07FF);    //通道3油门
		myrc.aileron_elevator		=	((sbus_buffer_data[5]>>1 |	sbus_buffer_data[6]<<7)	& 0x07FF);  //通道4
        myrc.mode	=	((sbus_buffer_data[6]>>4	|	sbus_buffer_data[7]<<4)	& 0x07FF);  //通道5飞行模式
		myrc.calibrate_num	=	((sbus_buffer_data[7]>>7 |	sbus_buffer_data[8]<<1 |sbus_buffer_data[9]<<9)	&	0x07FF);//通道6 校准
        myrc.loggerbutton_num = ((sbus_buffer_data[10]>>5 |	sbus_buffer_data[11]<<3)	& 0x07FF); //通道8 飞行日志记录开关
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
		
        //用于后面校准状态切换
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
给其它函数的接口
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
sbus任务函数
通过信号量来进行中断与任务的通信
当中断give出一个信号量后，任务处于就绪态
进行sbus帧的解析
 */

void sbus_task(void *param)
{																																																		//定时器时钟，每隔1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();	
	//定时
//	BaseType_t err	=	pdFALSE;
	while(1)
	{
				vTaskDelayUntil(&lastWakeTime, 20);																																							/*1ms周期延时*/
	//	printf("error\r\n");
				SbusAnalysis(gs_rec_sbus_data);
//                printf("pitch=%d \t roll=%d \t throttle=%d \t aileron_elevator=%d \t mode=%d loggerbutton=%d\r\n"
//                    ,myrc.elevator,myrc.aileron,myrc.throttle,myrc.aileron_elevator,myrc.mode,myrc.loggerbutton_num);
               
			
		//printf("error7.26\r\n");

	}//end of while(1)

}




