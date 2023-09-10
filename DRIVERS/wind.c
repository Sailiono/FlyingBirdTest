#include "wind.h"
#include "sys.h"
#include "delay.h"
#include "flight_log.h"
#include "myiic.h"
#include "led.h"
#include "exfuns.h"
#include "string.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h"
#endif


void windspeed_Task(void *param)
{
	u32 lastWakeTime = xTaskGetTickCount();
//	int resultP = 0;
	int *result;
	float gyro_data = 0;
	float temperature = 0;
	u16 save_flag=0; 
	static FIL flight_log_fil;
	static UINT flight_log_bww;
	static char flight_log_buf[200];
	char logname_buf[15] = "test.txt";
	static bool file_not_open_flag = true;
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 20);
		delay_us(2);
		if(file_not_open_flag)
		{
			sprintf((char*)logname_buf,"wind_%lld.txt",FreeRTOSRunTimeTicks);
			if(f_open(&flight_log_fil,logname_buf,FA_OPEN_APPEND|FA_WRITE)==0)
			{
				file_not_open_flag = false;
			}
		}
//		resultP = SM3041_Data_FetchBaro();
		result = SM3041_Data_FetchAll();
		gyro_data = (float)*result / 32768 * 100;        //caculate corrected windpressure
		temperature = (float)*(result+1) / 32768 * 100;
		if(!file_not_open_flag == true)
		// if(!file_not_open_flag && flightloggerbutton == true)
		{
//			sprintf((char*)flight_log_buf,"%.4f\r\n", windspeed);
			sprintf((char*)flight_log_buf,"%.4f,%.4f\r\n", gyro_data, temperature);
			f_write(&flight_log_fil,flight_log_buf,strlen(flight_log_buf),&flight_log_bww);
			LED2On();
			if(save_flag  %200==0)
			{
				f_sync(&flight_log_fil);
				save_flag=0;
			}
			save_flag++;

			}
	}//end of while(1)

}

uint8_t SM3041_Init(void)
{
	IIC_Wind_Init();
	delay_ms(20);
	return 0;
}

//Data Fetch 2  
//只读取空速数据，共两个Bit
//返回值:读到的数据
int SM3041_Data_FetchBaro(void)
{
	uint16_t resP;
	uint8_t status = 0;
	uint16_t temp = 0;
	IIC_Wind_Start();
	IIC_Wind_Send_Byte((SM3041_ADDR << 1) | 0x01);
	IIC_Wind_Wait_Ack();
	temp = IIC_Wind_Read_Byte(1);
	status = temp >>6;
	if(status == 0)
	{
		resP = IIC_Wind_Read_Byte(0);
		resP |= ((temp & 0x3f)<<8);
	}
	IIC_Wind_Stop();
	return resP;
}

//Data Fetch 4  
//读取空速和温度数据，共四个字节
//返回值:读到的数据
int * SM3041_Data_FetchAll(void)
{
	static int res[2];
	u8 temp = 0;
	u8 status = 0;
	IIC_Wind_Start();
	IIC_Wind_Send_Byte((SM3041_ADDR << 1) | 0x01);
	IIC_Wind_Wait_Ack();
	temp = IIC_Wind_Read_Byte(1);
	status = temp >>6;
	if(status == 0)
	{
		res[0] = IIC_Wind_Read_Byte(1);
		res[0] |= ((int)temp & 0x3f)<<8;
	}
	temp = IIC_Wind_Read_Byte(1);
	res[1] = IIC_Wind_Read_Byte(0);
	res[1] = (res[1]>>5) | (temp<<3);
	IIC_Wind_Stop();
	return res;
}

