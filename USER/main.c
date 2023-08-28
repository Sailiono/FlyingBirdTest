#include "main.h"
#include "watchdog.h"
#include "sbus.h"
#include "sdio_sdcard.h"
#include "flight_log.h"
#include "bsp_spi_ad7606.h"
#include "led.h"
#include "imu.h"
#include "imu2.h"
#include "adxrs290.h"



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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4

	delay_init(168);
	DebugSeriesInit(115200); 
	delay_ms(7000);
	LED_Init();
	//Sbus_Init();
	//AdcInit();
	Led_Flash(1);
	bsp_InitAD7606();
	ad7606_Reset();
	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	ad7606_Reset();
	AT24CXX_Init();			//IIC初始化
	//HALL_Init();

	//motor_steering_init();  //初始化油门电机控制信号 
	while(SD_Init()!=0){printf("sd");};                 //初始化SD卡
	while(exfuns_init()!=0){printf("02");};			    //为fatfs相关变量申请内存
	while(f_mount(fs[0],"0:",1)!=0){printf("03");}; 	//挂载SD卡 
	IMU_Init();
	IMU2_Init();
	delay_ms(500);
	xTaskCreate(start_task,"start_task",500,NULL,2,&StartTask_Handler);
	vTaskStartScheduler();
	return 0;
}
 

//开始任务函数
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区

//	xTaskCreate(FlappingFrequencyControlTask, "FlappingFrequencyControlTask", 500, NULL, 3, NULL);  //控制扑动频率

	xTaskCreate(ad_task, "ad_task", 500, NULL, 6, NULL);
	xTaskCreate(IMU_Task, "imu_task", 500, NULL, 5, NULL);
	xTaskCreate(IMU2_Task, "imu2_task", 500, NULL, 7, NULL);
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



