#include "imu2.h"

/** begin(),i2c init & set default I2C address.
    @param set i2c_address
**/
int32_t acc_Init(u8 dev_addr) {
    u8 ID = 0;
    IIC_Imu_Init();
    _dev_addr = dev_addr;
    ID = imu2_Read_Byte(ACC_I2C_ADDRESS,DEV_ID_REG_ADDR);
    if (ID != 0xed) {
        return -1;
    }
    acc_Write_Byte(ACC_I2C_ADDRESS,RESET_REG_ADDR, 0x52);
    delay_ms(200);
    accSetRange(TEN_G);

    return 0;
}


int32_t accSetRange(Acc_Range range) {
    u8 orig = 0;
    orig = imu2_Read_Byte(ACC_I2C_ADDRESS,SET_RANGE_REG_ADDR);
    orig |= range;
    return imu2_Write_Byte(ACC_I2C_ADDRESS,SET_RANGE_REG_ADDR, orig);
}


int32_t accGetActiveCnt(void) {
    u8 cnt = 0;
    if (i2cReadByte(GET_ACTIVE_COUNT_REG_ADDR, cnt)) {
        return -1;
    }
    return cnt;
}

int32_t accSetActThreshold(float acc_g, float factory) {
    int16_t thres = 0;
    thres = (int16_t)(acc_g / factory);
    thres >>= 3;
    // thres = 13300;
    // thres >>= 3;

    return i2cWriteU16(SET_THRESHOLD_REG_ADDR, (uint16_t)thres);
}

/** set action enable.
    @param enable_x enable x axis action.When the x axis result above threshold,trigger event.
    @param enable_y enable y axis action.When the y axis result above threshold,trigger event.
    @param enable_z enable z axis action.When the z axis result above threshold,trigger event.
 **/
int32_t accSetActEnable(bool enable_x, bool enable_y, bool enable_z) {
    u8 val = 0;
    val = val | (enable_z << 2) | (enable_y << 1) | enable_x;
    return i2cWriteByte(ACTION_ENABLE_REG_ADDR, val);
}

/** Config ADXL357 mode.
    bit2 - DRDY_OFF ,
    bit1 - TEMP-OFF ,
    bit0 - running mode,0 for measurement mode,1 for standby mode.
 **/
int32_t accSetPowerCtr(u8 val) {
    return i2cWriteByte(POWER_CTR_REG_ADDR, val);
}


int32_t accReadTemperature(float& T) {
    int32_t ret = 0;
    int16_t temperature = 0;
    uint16_t val = 0;
    if (i2cReadU16(TEMPERATURE_REG_ADDR, val)) {
        return -1;
    }

    temperature = val;
    T = 25 + (temperature - 1852) / (-9.05);
    return 0;
}

int32_t accSetFilter(void) {
    /*011 - 15.545*10^-3*ODR, 0011 - 500HZ&125HZ*/
    return i2cWriteByte(FILTER_REG_ADDR, 0x33);
}

int32_t accReadXYZAxisResultDataFromFIFO(int32_t& x, int32_t& y, int32_t& z) {
    u8 data[9] = {0};
    x = y = z = 0;

    if (imu2_Read_Len(ACC_I2C_ADDRESS,FIFO_DATA_REG_ADDR, 9, data)) {
        return -1;
    }
    x = ((u32)data[0] << 12) | ((u32)data[1] << 4) | ((u32)data[2] >> 4);
    y = ((u32)data[3] << 12) | ((u32)data[4] << 4) | ((u32)data[5] >> 4);
    z = ((u32)data[6] << 12) | ((u32)data[7] << 4) | ((u32)data[8] >> 4);

    if (x & 0x80000) {
        x = (x & 0x7ffff) - 0x80000;
    }
    if (y & 0x80000) {
        y = (y & 0x7ffff) - 0x80000;
    }
    if (z & 0x80000) {
        z = (z & 0x7ffff) - 0x80000;
    }

    return 0;
}



int32_t accReadXYZAxisResultData(int32_t x, int32_t y, int32_t z) {
    u8 data[9] = {0};
    x = y = z = 0;

    if (imu2_Write_Len(ACC_I2C_ADDRESS,FIFO_DATA_REG_ADDR, 9, data)) {
        return -1;
    }
    x = ((u32)data[0] << 12) | ((u32)data[1] << 4) | ((u32)data[2] >> 4);
    y = ((u32)data[3] << 12) | ((u32)data[4] << 4) | ((u32)data[5] >> 4);
    z = ((u32)data[6] << 12) | ((u32)data[7] << 4) | ((u32)data[8] >> 4);

    if (x & 0x80000) {
        x = (x & 0x7ffff) - 0x80000;
    }
    if (y & 0x80000) {
        y = (y & 0x7ffff) - 0x80000;
    }
    if (z & 0x80000) {
        z = (z & 0x7ffff) - 0x80000;
    }

    return 0;
}

int32_t accGetStatus(u8 byte) {
    return byte = acc_Read_Byte(STATUS_REG_ADDR);
}

bool accCheckDataReady(void) {
    u8 stat = 0;
    accGetStatus(stat);
    return stat & 0x01;
}

/*
IMU includes the following functions: Accelerator,Barometer,Altimeter.
With the following function, MCU gets data from imu2 in set frequency and store them in sd cards.
 */
void imu2_task(void *param)
{																																																		//定时器时钟，每隔1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();																																				//定时
	u8 temp= 4;
	s16 pp [8]={-1};
	float gg[8]={-1};
	u8 i,j;
	u16 save_flag=0; 
   
	static FIL flight_log_fil;
    static UINT flight_log_bww;
    static char flight_log_buf[200];
  
    char logname_buf[15] = "test.txt";
	static bool file_not_open_flag = true;  //????????
                       //?????? ???+1 ????????????

//    static float volt,current;
//    static bool flightloggerbutton;
//	static RcRawDataSTypeDef myrc;
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 10);																		/*10ms周期延时*/
	//	flightloggerbutton=GetFlightLoggerState();
//    if(flightloggerbutton == false)
//		{
//		GPIO_SetBits(GPIOA,GPIO_Pin_5);
//		}
    delay_us(2);
		if(file_not_open_flag)
        {
					//printf("123");
           // if(flightloggerbutton == true)
            {
							//printf("fg");
							//LED1On();
                sprintf((char*)logname_buf,"imu2_%lld.txt",FreeRTOSRunTimeTicks);
					//sprintf((char*)logname_buf,"ui.txt");
//                sprintf((char*)logname_buf,"%02d.%02d.%02d.txt",gpsdata.gps_time.hours,gpsdata.gps_time.minutes,gpsdata.gps_time.seconds);
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
				
				
		ad7606_StartConv();
		temp = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);			 // ?? BUSY??? 
	  while(temp == 1)				//?busy?????,??????,???????? 
		{
			temp = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);		// ?? BUSY??? 	 
		}	
   AD7606_read_data(pp);
		
		for(i=0;i<8;i++)
		{
		gg[i]=(float)((float)pp[i]/32768*5000)/1000;
			//printf("%f",gg[i]);
		}
		for(j=0;j<2;j++)     //???????
		{
		mesg[j]=(u16)(gg[j]*1000);
		}
		//printf("\r\n");
		 if(!file_not_open_flag == true)
		// if(!file_not_open_flag && flightloggerbutton == true)
        {
//			printf("ssss");
//            sprintf((char*)flight_log_buf,"%f %f %f %.2f %.2f %.2f %.2f,%.2f %.2f,%.2f,%.2f %.2f,%d,%lld\r\n",
//                                euler_est.roll,euler_est.pitch,euler_est.yaw,
//                                gpsdata.latitude,gpsdata.longitude,gpsdata.height,
//								g_pos_est.x,g_pos_est.y,g_pos_est.z,
//								g_pos_est.vx,g_pos_est.vy,g_pos_est.vz,
//								flymode,
//                                FreeRTOSRunTimeTicks); 
					
              //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6507)/0.05575,wzhfre,wzhspeed);   //黄底
              //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6680)/0.0582,wzhfre,wzhspeed);   //白底带胶
							//sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6450)/0.0580,wzhfre,wzhspeed);  //坏底无胶
					    //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.65)/0.0571,wzhfre,wzhspeed);  //L1
					    //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6537)/0.0570,wzhfre,wzhspeed);   //黄底N3
				    	 // sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6464)/0.0562,wzhfre,wzhspeed);   //2 HAO
			sprintf((char*)flight_log_buf,"%.4f ,%.4f\r\n",gg[0]*4.92f,(gg[1]-1.6565f)/0.0563f);   //输入采集数据到飞行日志
            f_write(&flight_log_fil,flight_log_buf,strlen(flight_log_buf),&flight_log_bww);       //写入SD卡
			LED2On();
//            printf("%d\r\n",FLAPPINGANGLE_POSITION);
							
            //if(file_not_open_flag == false && flightloggerbutton == false)
            //{
                 //f_sync(&flight_log_fil);	//????????????,????
                // file_not_open_flag = true;
           // }                
            if(save_flag  %200==0)
            {
                f_sync(&flight_log_fil);	//?????????? ????????
                save_flag=0;    
            }	
            save_flag++;
        
        }
	}//end of while(1)

}



//IIC_Imu连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 imu2_Write_Len(u8 dev_Addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Imu_Start(); 
	IIC_Imu_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Imu_Wait_Ack())	//等待应答
	{
		IIC_Imu_Stop();		 
		return 1;		
	}
    IIC_Imu_Send_Byte(reg);	//写寄存器地址
    IIC_Imu_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Imu_Send_Byte(buf[i]);	//发送数据
		if(IIC_Imu_Wait_Ack())		//等待ACK
		{
			IIC_Imu_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Imu_Stop();	 
	return 0;	
} 
//IIC_Imu连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 imu2_Read_Len(u8 dev_Addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Imu_Start(); 
	IIC_Imu_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Imu_Wait_Ack())	//等待应答
	{
		IIC_Imu_Stop();		 
		return 1;		
	}
    IIC_Imu_Send_Byte(reg);	//写寄存器地址
    IIC_Imu_Wait_Ack();		//等待应答
    IIC_Imu_Start();
	IIC_Imu_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Imu_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Imu_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Imu_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}
    IIC_Imu_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC_Imu写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 imu2_Write_Byte(u8 dev_Addr,u8 reg,u8 data) 				 
{ 
    IIC_Imu_Start(); 
	IIC_Imu_Send_Byte((dev_Addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Imu_Wait_Ack())	//等待应答
	{
		IIC_Imu_Stop();		 
		return 1;		
	}
    IIC_Imu_Send_Byte(reg);	//写寄存器地址
    IIC_Imu_Wait_Ack();		//等待应答 
	IIC_Imu_Send_Byte(data);//发送数据
	if(IIC_Imu_Wait_Ack())	//等待ACK
	{
		IIC_Imu_Stop();	 
		return 1;
	}
    IIC_Imu_Stop();	 
	return 0;
}
//IIC_Imu读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 imu2_Read_Byte(u8 dev_Addr,u8 reg)
{
	u8 res;
    IIC_Imu_Start();
	IIC_Imu_Send_Byte((dev_Addr<<1)|0);//发送器件地址+写命令	
	IIC_Imu_Wait_Ack();		//等待应答 
    IIC_Imu_Send_Byte(reg);	//写寄存器地址
    IIC_Imu_Wait_Ack();		//等待应答
    IIC_Imu_Start();
	IIC_Imu_Send_Byte((dev_Addr<<1)|1);//发送器件地址+读命令	
    IIC_Imu_Wait_Ack();		//等待应答 
	res=IIC_Imu_Read_Byte(0);//读取数据,发送nACK 
    IIC_Imu_Stop();			//产生一个停止条件 
	return res;		
}
