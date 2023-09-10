#include "adxrs290.h"
#include "sys.h"
#include "delay.h"
#include "flight_log.h"
#include "spi.h"
#include "led.h"
#include "exfuns.h"
#include "string.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h"
#endif



void GYRO2_Task(void *param)
{
	u32 lastWakeTime = xTaskGetTickCount();
//	int resultP = 0;
	s16 *result;
	float gyro_data_x = 0;
	float gyro_data_y = 0;
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
			sprintf((char*)logname_buf,"gyro2_%lld.txt",FreeRTOSRunTimeTicks);
			if(f_open(&flight_log_fil,logname_buf,FA_OPEN_APPEND|FA_WRITE)==0)
			{
				file_not_open_flag = false;
			}
		}
		result = Adxrs290_ReadAllData();
		gyro_data_x = ((float)(*result))/32768*100;											//caculate x axis result
		gyro_data_y = ((float)(*(result+1)))/32768*100;											//caculate y axis result
		temperature = ((float)(*(result+2)))/10;
		if(!file_not_open_flag == true)
		// if(!file_not_open_flag && flightloggerbutton == true)
		{
//			sprintf((char*)flight_log_buf,"%.4f\r\n", windspeed);
			sprintf((char*)flight_log_buf,"%.4f, %.4f, %.4f\r\n", gyro_data_x, gyro_data_y, temperature);
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

void Adxrs290_Init(void)
{ 
	
	GPIO_InitTypeDef  GPIO_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		//使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;					//PB12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;				//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//初始化
	SPI2_Init();
	
	ADXRS290_CS = 1;											//片选输出1
	
	Adxrs290_Reg_Write(ADXRS290_REG_POWER_CTL, ADXRS290_MEASUREMENT | ADXRS290_TSM);
	delay_ms(10);
	u8 test;
	test = Adxrs290_Reg_Read(ADXRS290_REG_DEV_ID);
	
	while(test == ADXRS290_DEV_ID){
		Led_Flash(2);
		printf("%.4f",(float)test);
	}
	delay_ms(500);
}


u8 Adxrs290_Reg_Write(u8 regAddr,u8 data)						//寄存器写入
{
	u8 command = regAddr;
    ADXRS290_CS = 0;                          					//使能器件  
    SPI2_ReadWriteByte(command);
    SPI2_ReadWriteByte(data);
    ADXRS290_CS = 1;
    return 0;
}

s8 Adxrs290_Reg_Read(u8 regAddr)								//寄存器读取
{
	static u8 data;
    u8 command = ADXRS290_READ_REG(regAddr);
	ADXRS290_CS = 0;                           					//使能器件  
	SPI2_ReadWriteByte(command);
	data = SPI2_ReadWriteByte(0x00);
	ADXRS290_CS = 1;
	return data;
}


s8 * Adxrs290_Multi_Reg_Read(u8 regAddr,u8 bytes_to_read)		//多字节读取
{
	s8* read_out_data;
	u8 i =0;
    u8 command = ADXRS290_READ_REG(regAddr);
	ADXRS290_CS = 0;                          					//使能器件  
	SPI2_ReadWriteByte(command);
	*read_out_data = SPI2_ReadWriteByte(0x00);
	bytes_to_read--;
	while (bytes_to_read > 0)
    {
		i++;
        *(read_out_data + i) = SPI2_ReadWriteByte(0x00);
        bytes_to_read--;
    }
	ADXRS290_CS = 1;
	return read_out_data;
}

/*int64_t SPI2_Read(u8 regAddr,u8 bytes_to_read)//寄存器读取
{
	uint8_t in_byte = 0;
    int64_t result = 0;
    u8 command = regAddr | 1;
	ADXRS290_CS =0;                         				//使能器件  
	SPI2_ReadWriteByte(command);
	result = SPI2_ReadWriteByte(0x00);
	bytes_to_read--;
	 while (bytes_to_read > 0)
    {
        result = result << 8;
        in_byte = SPI2_ReadWriteByte(0x00);
        result |= in_byte;
        bytes_to_read--;
    }
	ADXRS290_CS =0;
	return result;
}*/

s16 * Adxrs290_ReadXYAxisResultData(void)
{
	static s8 * data;
	static s16* result;
	static s16 temp;
	data = Adxrs290_Multi_Reg_Read(ADXRS290_REG_DATAX0, 4);
	*result = (u16)*(data +1) << 8 | *data;
	*(result+1) = (u16)*(data +3) << 8 | *(data+2);
	temp = *result;
	temp = *(result+1);
	return result;
}

s16 Adxrs290_ReadTempData(void)
{
	static s8 * data;
	static int temp;
	data = Adxrs290_Multi_Reg_Read(ADXRS290_REG_TEMP0, 2);
	temp = ((s16)*(data +1) << 8 | *data) & 0x0FFF;
	return temp;
}
s16 * Adxrs290_ReadAllData(void)
{
	static s8* data;
	static s16* result;
	static s16 temp;

	data = Adxrs290_Multi_Reg_Read(ADXRS290_REG_DATAX0, 6);
	*result = (s16)*(data +1) << 8 | *data;			//X axis data
	*(result+1) = (s16)*(data +3) << 8 | *(data+2);	//Y axis data
	*(result+2) = (s16)*(data +5) << 8 | *(data+4); //temp data
	temp = *result;
	temp = *(result+1);
	temp = *(result+2);
	return result;
}
