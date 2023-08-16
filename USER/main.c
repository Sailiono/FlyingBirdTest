#include "main.h"
#include "watchdog.h"
#include "sbus.h"
#include "sdio_sdcard.h"
#include "flight_log.h"
#include "bsp_spi_ad7606.h"
#include "led.h"
#include "imu.h"
TaskHandle_t StartTask_Handler;//创建任务用任务句柄
void start_task(void *pvParameters);//创建任务用任务函数声明
void RunTimeStats_task(void *pvParameters);
//static FIL flight_log_fil;
//static UINT flight_log_bww;
//static char flight_log_buf[200];
FIFO_T wzh;   

//int64_t temp =0;
//int64_t result =0;

int main(void)
{ 
//	int i,j,p,k=0;
//	s16 Rome[1000][8]={-1};   //用来储存100次转化结果
//	static s16 LL [8]={-1};   //储存单次结果
//	char logname_buf[15] = "test.txt";
//	int temp= 4;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
  
	delay_init(168);
	
	DebugSeriesInit(115200); 
	
	delay_ms(500);
	LEDInit();
    //SbusInit();
   	// AdcInit();
	while(1)
	{printf("aa");
	delay_ms(500);
	//printf("led");
	break;
	};
	LED1On();
	delay_ms(1000);
	LED1Off();
	bsp_InitAD7606();
	ad7606_Reset();
	//printf("sssssssssssss");
	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	ad7606_Reset();
	AT24CXX_Init();			//IIC初始化
	//HALL_Init();							

   // motor_steering_init();  //初始化油门电机控制信号 
	while(SD_Init()!=0){printf("sd");};                 //初始化SD卡
	while(exfuns_init()!=0){printf("02");};			    //为fatfs相关变量申请内存				 
	while(f_mount(fs[0],"0:",1)!=0){printf("03");}; 	//挂载SD卡 
 	
	//LED1On();
	//delay_ms(1000);
	//LED1Off();
	//delay_ms(1000);
	//printf("dd");
	IMU_Init();
	//GPIO_ResetBits(GPIOD,GPIO_Pin_13);
	//delay_ms(1000);
	//GPIO_SetBits(GPIOD,GPIO_Pin_13);
	delay_ms(500);
	xTaskCreate(start_task,"start_task",500,NULL,2,&StartTask_Handler);
	vTaskStartScheduler();
	return 0;
}
 

//开始任务函数
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	   
	// xTaskCreate(FlappingFrequencyControlTask, "FlappingFrequencyControlTask", 500, NULL, 3, NULL);  //控制扑动频率
	//   
	//    
	xTaskCreate(ad_task, "ad_task", 500, NULL, 6, NULL);
	xTaskCreate(imu_task, "imu_task", 500, NULL, 5, NULL);
	//xTaskCreate(mag_task, "mag_task", 500, NULL, 5, NULL);
	//xTaskCreate(sbus_task, "sbus_task", 500, NULL, 8, NULL);
	//xTaskCreate(hall_task, "hall_task", 500, NULL, 7, NULL);
	//xTaskCreate(twostm_task, "twostm_task", 1000, NULL,9, NULL);
	//xTaskCreate(windspeed_task, "windspeed_task", 500, NULL, 7, NULL);
	
	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}



void vApplicationIdleHook( void )
{
    __WFI();	/*进入低功耗模式*/
}









