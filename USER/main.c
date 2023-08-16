#include "main.h"
#include "watchdog.h"
#include "sbus.h"
#include "sdio_sdcard.h"
#include "flight_log.h"
#include "bsp_spi_ad7606.h"
#include "led.h"
#include "imu.h"
TaskHandle_t StartTask_Handler;//����������������
void start_task(void *pvParameters);//��������������������
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
//	s16 Rome[1000][8]={-1};   //��������100��ת�����
//	static s16 LL [8]={-1};   //���浥�ν��
//	char logname_buf[15] = "test.txt";
//	int temp= 4;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
  
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
	AT24CXX_Init();			//IIC��ʼ��
	//HALL_Init();							

   // motor_steering_init();  //��ʼ�����ŵ�������ź� 
	while(SD_Init()!=0){printf("sd");};                 //��ʼ��SD��
	while(exfuns_init()!=0){printf("02");};			    //Ϊfatfs��ر��������ڴ�				 
	while(f_mount(fs[0],"0:",1)!=0){printf("03");}; 	//����SD�� 
 	
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
 

//��ʼ������
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();           //�����ٽ���
	   
	// xTaskCreate(FlappingFrequencyControlTask, "FlappingFrequencyControlTask", 500, NULL, 3, NULL);  //�����˶�Ƶ��
	//   
	//    
	xTaskCreate(ad_task, "ad_task", 500, NULL, 6, NULL);
	xTaskCreate(imu_task, "imu_task", 500, NULL, 5, NULL);
	//xTaskCreate(mag_task, "mag_task", 500, NULL, 5, NULL);
	//xTaskCreate(sbus_task, "sbus_task", 500, NULL, 8, NULL);
	//xTaskCreate(hall_task, "hall_task", 500, NULL, 7, NULL);
	//xTaskCreate(twostm_task, "twostm_task", 1000, NULL,9, NULL);
	//xTaskCreate(windspeed_task, "windspeed_task", 500, NULL, 7, NULL);
	
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	taskEXIT_CRITICAL();            //�˳��ٽ���
}



void vApplicationIdleHook( void )
{
    __WFI();	/*����͹���ģʽ*/
}









