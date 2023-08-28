#include "imu.h"
#include "delay.h"
#include "spi.h"
#include "flight_log.h"
#include <math.h>
#include <stdio.h>
#include "led.h"



//static bool file_not_open_flag = true;

void IMU_Init(void)
{ 
	GPIO_InitTypeDef  GPIO_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);			//使能GPIOE时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;	//PE0~2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;					//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);							//初始化
	ACC_CS=1;														//PE0输出1,不使用自带加速度计
	GYRO_CS=1;														//陀螺仪片选置1
	MAG_CS=1;														//磁力计片选置1
	SPI1_Init();		   											//初始化SPI
	
	SPI_Write(BMX055_GYRO_BGW_SOFTRESET,0xB6,GYRO);
	delay_ms(10);
	SPI_Write(BMX055_GYRO_RANGE,G_500DPS,GYRO);
	SPI_Write(BMX055_GYRO_BW, G_100Hz12Hz,GYRO);
	SPI_Write(BMX055_GYRO_LPM1,0x00,GYRO);
	SPI_Write(BMX055_GYRO_RATE_HBW,0x00,GYRO);//config Gyro
	
	SPI_Write(BMX055_MAG_PWR_CNTL1,0x82,MAG);  // softreset magnetometer
	delay_ms(10);
	SPI_Write(BMX055_MAG_PWR_CNTL1,0x01,MAG); // set to sleep mode, it's needed for state transit
	delay_ms(10);
	SPI_Write(BMX055_MAG_PWR_CNTL2,MODR_30Hz + 0x00,MAG); // Normal mode, 30hz
	Led_Flash(3);
	//delay_ms(500);
	int64_t a;
	a= SPI_Read(BMX055_GYRO_WHOAMI,1,GYRO);
	if(a==0x0F){Led_Flash(2);}
	delay_ms(10);
	a= SPI_Read(BMX055_MAG_WHOAMI,1,MAG);
	if(a==0x32){Led_Flash(2);}
	delay_ms(500);
	
//	SPI1_SetSpeed(SPI_BaudRatePrescaler_16);						//设置为7.5M时钟,高速模式
//	GYR_BIST();
}


//写入IMU的数据，数据来源（可配置为GYR、MAG)
/*void IMU_Write(u8 regAddr,u8 Value,u8 SensorSelec)   
{    
	u8 command;	
	command = regAddr | Write;
	if(SensorSelec == GYRO)GYRO_CS=0;else if(SensorSelec == MAG)MAG_CS=0;                          				//使能器件   
	SPI1_ReadWriteByte(command);//发送寄存器号 
	SPI1_ReadWriteByte(Value);      //写入寄存器的值
	GYRO_CS=1; 
	MAG_CS=1;
}*/

//读取IMU的数据，数据来源（可配置为GYR、MAG)
/*void IMU_Read(u8 regAddr,u8 *readData,u8 SensorSelec)   
{    		
	u8 command;
	command = regAddr | Read;
	if(SensorSelec == GYRO)GYRO_CS=0;else if(SensorSelec == MAG)MAG_CS=0;                         //使能器件   
	SPI1_ReadWriteByte(command);			//发送寄存器号 
	*readData =SPI1_ReadWriteByte(0x00);      //读取寄存器的值   
	GYRO_CS=1; 
	MAG_CS=1;
}  */

void SPI_Write(u8 regAddr,u8 data,u8 SensorSelec)//寄存器写入
{
	u8 command = regAddr;
    if(SensorSelec == GYRO)GYRO_CS=0;else if(SensorSelec == MAG)MAG_CS=0;                          				//使能器件  
    SPI1_ReadWriteByte(command);
    SPI1_ReadWriteByte(data);
    GYRO_CS=1;
	MAG_CS=1;
    return;
}


int64_t SPI_Read(u8 regAddr,u8 bytes_to_read,u8 SensorSelec)//寄存器读取
{
	uint8_t in_byte = 0;
    int64_t result = 0;
    u8 command = regAddr | Read;
	if(SensorSelec == GYRO)GYRO_CS=0;else if(SensorSelec == MAG)MAG_CS=0;                          				//使能器件  
	SPI1_ReadWriteByte(command);
	result = SPI1_ReadWriteByte(0x00);
	bytes_to_read--;
	 while (bytes_to_read > 0)
    {
        result = result << 8;
        in_byte = SPI1_ReadWriteByte(0x00);
        result |= in_byte;
        bytes_to_read--;
    }
	GYRO_CS=1;
	MAG_CS=1;
	return result;
}


// gyr info read

/*void Gyro_Read_Data(unsigned char *x1, unsigned char *y, unsigned char *z){
	IMU_Read(BMX055_GYRO_RATE_X_LSB, x,GYRO);
	IMU_Read(BMX055_GYRO_RATE_X_MSB, x+1,GYRO);
	IMU_Read(BMX055_GYRO_RATE_Y_LSB, y,GYRO);
	IMU_Read(BMX055_GYRO_RATE_Y_MSB, y+1,GYRO);
	IMU_Read(BMX055_GYRO_RATE_Z_LSB, z,GYRO);
	IMU_Read(BMX055_GYRO_RATE_Z_MSB, z+1,GYRO);
}*/

// mag info read
/*void Mag_Read_Data(unsigned char *x, unsigned char *y, unsigned char *z){
	Mag_Read_Register(BMX055_MAG_XOUT_LSB, x);
	Mag_Read_Register(BMX055_MAG_XOUT_MSB, x+1);
	Mag_Read_Register(BMX055_MAG_YOUT_LSB, y);
	Mag_Read_Register(BMX055_MAG_YOUT_MSB, y+1);
	Mag_Read_Register(BMX055_MAG_ZOUT_LSB, z);
	Mag_Read_Register(BMX055_MAG_ZOUT_MSB, z+1);
}*/


//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
/*u8 IMU_Read_Buf(8 reg,u8 *pBuf,u8 len,u8 SensorSelec)   
{    		
    u8 status,u8_ctr;		
	if(SensorSelec == 1)GYR_CS=0;else if(SensorSelec == 2)MAG_CS=0;else return 0;                           //使能器件   
	status=SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI1_ReadWriteByte(0XFF);//读出数据
	GYR_CS=1; 
	MAG_CS=1;	
	return status;        //返回读到的状态值

}  */

//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
/*u8 IMU_Write_Buf(u8 reg,u8 *pBuf,u8 len,u8 SensorSelec)
{
	u8 status,u8_ctr;	
	if(SensorSelec == GYR)GYR_CS=0;else if(SensorSelec == MAG)MAG_CS=0;else return 0;                            //使能器件    
	SPI1_ReadWriteByte(W25X_PageProgram);      //发送写页命令   
	status = SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI1_ReadWriteByte(*pBuf++); //写入数据	 
	GYR_CS=1; 
	MAG_CS=1;
	return status;          //返回读到的状态值
} */

/*u8 GYR_BIST(void)
{
	u8 reg,result;
//	static char bist_log_buf[200];
//	static FIL bist_log_fil;
//	static UINT bist_log_bww;
//	char logname_buf[15] = "test.txt";
//	u16 save_flag=0;
	IMU_Write(GYR_BIST_REG,GYR_Trig_BIST,GYR);
	
	reg = IMU_Read(GYR_BIST_REG,GYR);
	if(reg == GYR_BIST_OK){result=1;printf("GYR_BIST OK");}
	else{result = 0;printf("GYR_BIST NOT OK,Error:%4d",reg);}
	return result;
}*/

		


/*void GYR_SetRange(void)
{
	u8 reg,result;
	IMU_Write(GYR_BIST_REG,GYR_Trig_BIST,GYR);
	result = IMU_Read(GYR_BIST_REG,GYR);
	while(reg == GYR_BIST_OK){}else{printf("GYR_BIST NOT OK,Error:%4d",reg)}
	return result;
}*/
		




void IMU_Task(void *param)
{																																																		//定时器时钟，每隔1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();//定时 
	int8_t tempXLSB=0;
	int8_t tempXMSB=0;
	int8_t tempYLSB=0;
	int8_t tempYMSB=0;
	int8_t tempZLSB=0;
	int8_t tempZMSB=0;
//	u8 i,j;
	
	struct Sensor sensor = {
		.gyro_reso = 500.0/32768.0,	// dafault use 500dps
		//.accel_reso = 4.0/2048.0,		// default use 4g/s
		.mag_reso = 0.3,					// fixed 0.3 uT/count
};
	

	u16 save_flag=0;
//	float gg[3]={-1};
   
	static FIL flight_log_fil;
    static UINT flight_log_bww;
    static char flight_log_buf[400];
  
    char logname_buf[15] = "test.txt";
	static bool file_not_open_flag = true;
                       

//  static float volt,current;
//  static bool flightloggerbutton;
//	static RcRawDataSTypeDef myrc;
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 50);																		/*10ms周期延时*/
//		flightloggerbutton=GetFlightLoggerState();
//		if(flightloggerbutton == false)
//		{
//		GPIO_SetBits(GPIOA,GPIO_Pin_5);
//		}
		delay_us(2);
		if(file_not_open_flag) 		
        {
			//printf("123");
           // if(flightloggerbutton == true)
            {
				
                sprintf((char*)logname_buf,"imu_%lld.txt",FreeRTOSRunTimeTicks);//打印数据文件名
				//sprintf((char*)logname_buf,"ui.txt");
                if(f_open(&flight_log_fil,logname_buf,FA_OPEN_APPEND|FA_WRITE)==0)
                {
                    file_not_open_flag = false;
                }
           // }
//            if(armedstatus == ARMED && f_open(&flight_log_fil,logname_buf,FA_OPEN_APPEND|FA_WRITE)==0)
//            {
//                file_not_open_flag = false;
//                printf("create success");
//            }
            
        }
			}
	
				
		/*tempXLSB = IMU_Read(GYR_X_LSB_REG,GYR);
		ResultGyrX |= tempXLSB;
		ResultGyrX = ResultGyrX<<8;
		while(tempXLSB != 0)				 
		{
			tempXMSB = IMU_Read(GYR_X_MSB_REG,GYR);
			ResultGyrX |=tempXMSB;
			break;
		}	
		
		tempYLSB = IMU_Read(GYR_Y_LSB_REG,GYR);	
		ResultGyrY |= tempXLSB;
		ResultGyrY = ResultGyrX<<8;		
		while(tempYLSB != 0)				 
		{
			tempYMSB = IMU_Read(GYR_Y_MSB_REG,GYR);
			ResultGyrY |=tempYMSB;
			break;
		}	
		
		tempZLSB = IMU_Read(GYR_Z_LSB_REG,GYR);	
		ResultGyrY |= tempXLSB;
		ResultGyrY = ResultGyrX<<8;			
		while(tempZLSB != 0)
		{
			tempZMSB = IMU_Read(GYR_Z_MSB_REG,GYR);
			ResultGyrZ |=tempZMSB;
			break;
		}	
		gg[0] = (float)ResultGyrX/32767*2000;
		gg[1] = (float)ResultGyrY/32767*2000;
		gg[2] = (float)ResultGyrZ/32767*2000;*/
		int64_t temp =0;
		temp = SPI_Read(BMX055_GYRO_RATE_X_LSB,6,GYRO);
		//printf("%s\r\n",(char*)temp);
		tempXLSB=temp>>40;
		tempXMSB=temp>>32;
		tempYLSB=temp>>24;
		tempYMSB=temp>>16;
		tempZLSB=temp>>8;
		tempZMSB=temp;//Gyroscope每个轴向的高位和低位数据
		
		sensor.cov_tmp = ((int)tempXMSB<<8)|tempXLSB;//先转化为MSB|LSB格式的临时数据
		sensor.gyro_x_float = (float)sensor.cov_tmp * sensor.gyro_reso;//根据设置的量程和数据转化为十进制浮点数
		sensor.cov_tmp = ((int)tempYMSB<<8)|tempYLSB;
		sensor.gyro_y_float = (float)sensor.cov_tmp * sensor.gyro_reso;
		sensor.cov_tmp = ((int)tempZMSB<<8)|tempZLSB;
		sensor.gyro_z_float = (float)sensor.cov_tmp * sensor.gyro_reso;
		delay_us(10);
		
		temp = SPI_Read(BMX055_MAG_XOUT_LSB,6,MAG);
		tempXLSB=temp>>40;
		tempXMSB=temp>>32;
		tempYLSB=temp>>24;
		tempYMSB=temp>>16;
		tempZLSB=temp>>8;
		tempZMSB=temp;//Magnet每个轴向的高位和低位数据
		
		sensor.cov_tmp = (((int)tempXMSB<<8)|tempXLSB)>>3;//先转化为MSB|LSB格式的临时数据
		sensor.mag_x_float = (float)sensor.cov_tmp * sensor.mag_reso;//根据设置的量程和数据转化为十进制浮点数
		sensor.cov_tmp = (((int)tempYMSB<<8)|tempYLSB)>>3;
		sensor.mag_y_float = (float)sensor.cov_tmp * sensor.mag_reso;
		sensor.cov_tmp = (((int)tempZMSB<<8)|tempZLSB)>>3;
		sensor.mag_z_float = (float)sensor.cov_tmp * sensor.mag_reso;
		
		if(!file_not_open_flag == true)
		// if(!file_not_open_flag && flightloggerbutton == true)
        {

			sprintf((char*)flight_log_buf,"%.4f ,%.4f ,%.4f,%.4f ,%.4f ,%.4f\r\n",sensor.gyro_x_float,sensor.gyro_y_float,sensor.gyro_z_float,sensor.mag_x_float,sensor.mag_y_float,sensor.mag_z_float);//写入flightlog
			f_write(&flight_log_fil,flight_log_buf,strlen(flight_log_buf),&flight_log_bww); //写入内存卡
			LED1On();
//			printf("%d\r\n",FLAPPINGANGLE_POSITION);
							
            /*if(file_not_open_flag == false && flightloggerbutton == false)
            {
				f_sync(&flight_log_fil);
				file_not_open_flag = true;
			}*/
            if(save_flag  %200==0)
            {
				f_sync(&flight_log_fil);
				save_flag=0;
            }
				save_flag++;
        
        }
	}//end of while(1)

}

/*void Bmx_Convert_Data(struct Sensor *sensor){
	// gyro convert, 16 bits
	sensor->cov_tmp = ((int)sensor->gyro_x[1] << 8) | sensor->gyro_x[0];    // [0] is high bit, [1] is low bit
	sensor->gyro_x_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	sensor->cov_tmp = ((int)sensor->gyro_y[1] << 8) | sensor->gyro_y[0];    // [0] is high bit, [1] is low bit
	sensor->gyro_y_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	sensor->cov_tmp = ((int)sensor->gyro_z[1] << 8) | sensor->gyro_z[0];    // [0] is high bit, [1] is low bit
	sensor->gyro_z_float = (float)sensor->cov_tmp * sensor->gyro_reso;
	// accel convert, 12 bits
	sensor->cov_tmp = (((int)sensor->accel_x[1] << 8) | sensor->accel_x[0]) >> 4;    // [0] is high bit, [1] is low bit
	sensor->accel_x_float = (float)sensor->cov_tmp * sensor->accel_reso;
	sensor->cov_tmp = (((int)sensor->accel_y[1] << 8) | sensor->accel_y[0]) >> 4;    // [0] is high bit, [1] is low bit
	sensor->accel_y_float = (float)sensor->cov_tmp * sensor->accel_reso;
	sensor->cov_tmp = (((int)sensor->accel_z[1] << 8) | sensor->accel_z[0]) >> 4;    // [0] is high bit, [1] is low bit
	sensor->accel_z_float = (float)sensor->cov_tmp * sensor->accel_reso;
	//mag convert, 13 bits for xy, 15 bits for z
	sensor->cov_tmp = (((int)sensor->mag_x[1] << 8) | sensor->mag_x[0]) >> 3;    // [0] is high bit, [1] is low bit
	sensor->mag_x_float = (float)sensor->cov_tmp * sensor->mag_reso;
	sensor->cov_tmp = (((int)sensor->mag_y[1] << 8) | sensor->mag_y[0]) >> 3;    // [0] is high bit, [1] is low bit
	sensor->mag_y_float = (float)sensor->cov_tmp * sensor->mag_reso;
	sensor->cov_tmp = (((int)sensor->mag_z[1] << 8) | sensor->mag_z[0]) >> 1;    // [0] is high bit, [1] is low bit
	sensor->mag_z_float = (float)sensor->cov_tmp * sensor->mag_reso;
}*/
