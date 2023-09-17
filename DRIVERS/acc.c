#include "acc.h"
#include "sys.h"
#include "delay.h"
#include "flight_log.h"
#include "myiic.h"
#include "led.h"
#include "exfuns.h"
#include "string.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSÊ¹ÓÃ		  
#include "task.h"
#endif

#define return_err -1


/*
With the following function, MCU gets datas from Accelerator in set frequency and store them in sd cards(or send them to functions in need).
 */
void ACC_Task(void *param)
{
	u32 lastWakeTime = xTaskGetTickCount();
	int32_t* result;
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
			sprintf((char*)logname_buf,"acc_%lld.txt",FreeRTOSRunTimeTicks);
			if(f_open(&flight_log_fil,logname_buf,FA_OPEN_APPEND|FA_WRITE)==0)
			{
				file_not_open_flag = false;
			}
		}
		result = ADXL357B_ReadXYZAxisResultData();
		
		printf("x:%#.6x\r\n",*result);
		printf("y:%#.6x\r\n",*(result+1));
		printf("z:%#.6x\r\n",*(result+2));
		if(!file_not_open_flag == true)
		// if(!file_not_open_flag && flightloggerbutton == true)
		{
			sprintf((char*)flight_log_buf,"%.4f, %.4f, %.4f\r\n",(float)(*result)/51200,(float)(*(result+1))/51200,(float)(*(result+2))/51200);
			f_write(&flight_log_fil,flight_log_buf,strlen(flight_log_buf),&flight_log_bww);
			LED3On();
			if(save_flag  %200==0)
			{
				f_sync(&flight_log_fil);
				save_flag=0;
			}
			save_flag++;

		}
	}//end of while(1)
}


/** begin(),i2c init & set default I2C address.
	@param set i2c_address
**/


u8 ACC_Init(void){
	u8 ID = 0;
	extern bool imuInitBit;
	if (imuInitBit ==0)
	{
		IIC_Imu_Init();
	}
	imuInitBit =1;
	ID = 0x11;
	ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS,DEV_MEMS_REG_ADDR);
	
	printf("i2cid:%#.2x\r\n",ID);
	ID = 0x11;
	ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS,DEV_MEMS_REG_ADDR);
	
	printf("i2cid:%#.2x\r\n",ID);
	ID = 0x11;
	ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS,DEV_ID_REG_ADDR);
	
	printf("devid:%#.2x\r\n",ID);
	ADXL357B_Reset();
	ID = 0x11;
	ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS,POWER_CTR_REG_ADDR);
	printf("powermode:%#.2x\r\n",ID);
	
	ADXL357B_SetPowerCtr(0x00);//set to normal mode
	ID = 0x11;
	ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS,POWER_CTR_REG_ADDR);
	printf("powermode:%#.2x\r\n",ID);
	
	ID = 0x11;
	ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS,STATUS_REG_ADDR);
	
	printf("status:%#.2x\r\n",ID);

	ADXL357B_SetRange(TEN_G);

	ADXL357B_SetFilter(0x33);//011--1.55*10^-4;0011--500Hz

	ADXL357B_SetActEnable(1, 1, 1);//enable x,y,z 3 axis
	ID = 0x11;
	ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS,ACTION_ENABLE_REG_ADDR);
	printf("powermode:%#.2x\r\n",ID);
	
	float t =0;
	ADXL357B_ReadTemperature(t);
	printf("temperture:%.2f\r\n",t);
	
	/*ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS, DEV_ID_REG_ADDR);
	if (ID != 0xed) {
		Led_Flash(2);
		printf("%.4f\r\n",(double)ID);
		return 1;
	}*/

	return 0;
}

int ADXL357B_ReadTemperature(float T) {
	int16_t temperature = 0;
	u8 *val = 0;
	IIC_Imu_Read_Len(ACC_I2C_ADDRESS, TEMPERATURE_REG_ADDR, 2, val);
	temperature = ((uint16_t)(val[0] & 0xf0)) | val[1];
	if(T != 25 + (temperature - 1885) / (-9.05)){
		T = 25 + (temperature - 1885) / (-9.05);
	};
	return 0;
}

int * ADXL357B_ReadXYZAxisResultDataFromFIFO(void) {
	u8 data[9] = {0};
	u8 ID =0;
	int i =0;
	static int res[3];
	ID = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS, FIFO_ENTRY_REG_ADDR);
	if (ID>0) {
		printf("fifo_number:%#.2x\r\n",ID);
	}
	for(i =0;i<9;i++)
	{
	data[i] = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS, FIFO_DATA_REG_ADDR);
	}

	res[0] = (((u32)data[0]) << 12) | (((u32)data[1]) << 4) | (((u32)data[2]) >> 4);
	res[1] = (((u32)data[3]) << 12) | (((u32)data[4]) << 4) | (((u32)data[5]) >> 4);
	res[2] = (((u32)data[6]) << 12) | (((u32)data[7]) << 4) | (((u32)data[8]) >> 4);

	if (res[0] & 0x80000) {
		res[0] = (res[0] & 0x7ffff) - 0x80000;
	}
	if (res[1] & 0x80000) {
		res[1] = (res[1] & 0x7ffff) - 0x80000;
	}
	if (res[2] & 0x80000) {
		res[2] = (res[2] & 0x7ffff) - 0x80000;
	}

	return res;
}

int32_t * ADXL357B_ReadXYZAxisResultData(void)
{
	u8 data[9] = {0};
	int i =0;
	static int res[3];
	for(i =0;i<9;i++)
	{
	IIC_Imu_Read_Len(ACC_I2C_ADDRESS, X_DATA_REG_ADDR, 9, data);
	}

	res[0] = (((u32)data[0]) << 12) | (((u32)data[1]) << 4) | (((u32)data[2]) >> 4);
	res[1] = (((u32)data[3]) << 12) | (((u32)data[4]) << 4) | (((u32)data[5]) >> 4);
	res[2] = (((u32)data[6]) << 12) | (((u32)data[7]) << 4) | (((u32)data[8]) >> 4);

	if (res[0] & 0x80000) {
		res[0] = (res[0] & 0x7ffff) - 0x80000;
	}
	if (res[1] & 0x80000) {
		res[1] = (res[1] & 0x7ffff) - 0x80000;
	}
	if (res[2] & 0x80000) {
		res[2] = (res[2] & 0x7ffff) - 0x80000;
	}

	return res;
}



/*int32_t ADXL357B_ReadXYZAxisResultData(int32_t *x, int32_t *y, int32_t *z) {
    u8 data[9];
	*x =*y =*z =0;
	int i =0;
	for(i =0;i<9;i++)
	{
		data[i] = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS, X_DATA_REG_ADDR);

	}
    if (IIC_Imu_Read_Len(ACC_I2C_ADDRESS, X_DATA_REG_ADDR, 9, rawData)) {
        return -1;
    }
	for(i =0;i<9;i++)
	{
		data[i] = *(rawData+i);
	}
	
    *x = ((u32)data[0] << 12) | ((u32)data[1] << 4) | ((u32)data[2] >> 4);
    *y = ((u32)data[3] << 12) | ((u32)data[4] << 4) | ((u32)data[5] >> 4);
    *z = ((u32)data[6] << 12) | ((u32)data[7] << 4) | ((u32)data[8] >> 4);
	
    if (*x & 0x80000) {
        *x = (*x & 0x7ffff) - 0x80000;
    }
    if (*y & 0x80000) {
        *y = (*y & 0x7ffff) - 0x80000;
    }
    if (*z & 0x80000) {
        *z = (*z & 0x7ffff) - 0x80000;
    }
    return 0;
}*/



u8 ADXL357B_Reset(void)
{
	u8 reset_code = 0x52;  // Datasheet P.40

	return IIC_Imu_Write_Reg(ACC_I2C_ADDRESS, RESET_REG_ADDR, reset_code);
}

u8 ADXL357B_SetRange(Acc_Range range) {
	u8 orig = 0;
	orig = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS, SET_RANGE_REG_ADDR);
	orig |= range;
	return IIC_Imu_Write_Reg(ACC_I2C_ADDRESS, SET_RANGE_REG_ADDR, orig);
}

u8 ADXL357B_SetFilter(u8 val)
{
	u8 filter_setcode = val;  // Datasheet P.38

	return IIC_Imu_Write_Reg(ACC_I2C_ADDRESS, FILTER_REG_ADDR, filter_setcode);
}


//Config ADXL357 mode.bit2 - DRDY_OFF ,
//bit1 - TEMP-OFF ,bit0 - running mode,0 for measurement mode,1 for standby mode.
u8 ADXL357B_SetPowerCtr(u8 val)
{
	u8 powerctr_setcode = val;  // Datasheet P.38

	return IIC_Imu_Write_Reg(ACC_I2C_ADDRESS, POWER_CTR_REG_ADDR, powerctr_setcode);
}

u8 ADXL357B_GetActiveCnt(void) {
	u8 cnt = 0;
	cnt = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS, GET_ACTIVE_COUNT_REG_ADDR);
	return cnt;
}

u8 ADXL357B_SetActThreshold(float acc_g, float factory) {
	int16_t thres = 0;
	thres = (int16_t)(acc_g / factory);
	thres >>= 3;
	// thres = 13300;
	// thres >>= 3;
	u8* buf ={0};
	buf[0] = (u8)(thres >>8);
	buf[1] = (u8)(thres);
	return IIC_Imu_Write_Len(ACC_I2C_ADDRESS, SET_THRESHOLD_REG_ADDR, 2, buf);
}

/** set action enable.
	@param enable_x enable x axis action.When the x axis result above threshold,trigger event.
	@param enable_y enable y axis action.When the y axis result above threshold,trigger event.
	@param enable_z enable z axis action.When the z axis result above threshold,trigger event.
 **/
u8 ADXL357B_SetActEnable(bool enable_x, bool enable_y, bool enable_z) {
	u8 val = 0;
	val = val | (enable_z << 2) | (enable_y << 1) | enable_x;
	return IIC_Imu_Write_Len(ACC_I2C_ADDRESS,ACTION_ENABLE_REG_ADDR, 1, &val);
}



u8 ADXL357B_GetStatus(void) {
	static u8 byte =0;
	byte = IIC_Imu_Read_Reg(ACC_I2C_ADDRESS,STATUS_REG_ADDR);
	return byte;
}

bool ADXL357B_CheckDataReady(void) {
	u8 stat = 0;
	stat = ADXL357B_GetStatus();
	return stat & 0x01;
}

