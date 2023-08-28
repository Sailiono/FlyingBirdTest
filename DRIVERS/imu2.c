#include "imu2.h"




/* PUBLIC VARIABLES ================================================================================================================================ */
HP203x_TH_TypeDef		HP203B_TH_Struct;
HP203x_CR_TypeDef		HP203B_CR_Struct;
HP203x_Data_TypeDef		HP203B_Data;

/** begin(),i2c init & set default I2C address.
	@param set i2c_address
**/

int IMU2_Init(void) {
	if(ADXL357B_USAGE){
		ADXL357B_Init();
	}
	if(HP203B_USAGE){
		HP203B_Init(&HP203B_CR_Struct, &HP203B_TH_Struct);
	}
	return 0;
}





u8 ADXL357B_Reset(void)
{
	u8 reset_code = 0x52;  // Datasheet P.40

	return IMU2_Write_Reg(ACC_I2C_ADDRESS,RESET_REG_ADDR,reset_code);
}

s32 ADXL357B_SetRange(Acc_Range range) {
	u8 orig = 0;
	orig = IMU2_Read_Reg(ACC_I2C_ADDRESS,SET_RANGE_REG_ADDR);
	orig |= range;
	return IMU2_Write_Reg(ACC_I2C_ADDRESS,SET_RANGE_REG_ADDR, orig);
}

u8 ADXL357B_SetFilter(u8 val)
{
	u8 filter_setcode = val;  // Datasheet P.38

	return IMU2_Write_Reg(ACC_I2C_ADDRESS,FILTER_REG_ADDR, filter_setcode);
}


//Config ADXL357 mode.bit2 - DRDY_OFF ,
//bit1 - TEMP-OFF ,bit0 - running mode,0 for measurement mode,1 for standby mode.
u8 ADXL357B_SetPowerCtr(u8 val)
{
	u8 powerctr_setcode = val;  // Datasheet P.38

	return IMU2_Write_Reg(ACC_I2C_ADDRESS,POWER_CTR_REG_ADDR, powerctr_setcode);
}

s32 ADXL357B_GetActiveCnt(void) {
	u8 cnt = 0;
	cnt = IMU2_Read_Reg(ACC_I2C_ADDRESS,GET_ACTIVE_COUNT_REG_ADDR);
	return cnt;
}

s32 ADXL357B_SetActThreshold(float acc_g, float factory) {
	int16_t thres = 0;
	thres = (int16_t)(acc_g / factory);
	thres >>= 3;
	// thres = 13300;
	// thres >>= 3;
	u8* buf ={0};
	buf[0] = (u8)(thres >>8);
	buf[1] = (u8)(thres);
	return IMU2_Write_Len(ACC_I2C_ADDRESS,SET_THRESHOLD_REG_ADDR,2,buf);
}

/** set action enable.
	@param enable_x enable x axis action.When the x axis result above threshold,trigger event.
	@param enable_y enable y axis action.When the y axis result above threshold,trigger event.
	@param enable_z enable z axis action.When the z axis result above threshold,trigger event.
 **/
s32 ADXL357B_SetActEnable(bool enable_x, bool enable_y, bool enable_z) {
	u8 val = 0;
	val = val | (enable_z << 2) | (enable_y << 1) | enable_x;
	return IMU2_Write_Len(ACC_I2C_ADDRESS,ACTION_ENABLE_REG_ADDR, 1, &val);
}

s32 ADXL357B_ReadTemperature(float T) {
	int16_t temperature = 0;
	u8 *val = 0;
	IMU2_Read_Len(ACC_I2C_ADDRESS,TEMPERATURE_REG_ADDR, 2, val);
	temperature = ((uint16_t)(val[0] & 0xf0)) | val[1];
	if(T != 25 + (temperature - 1885) / (-9.05)){
		T = 25 + (temperature - 1885) / (-9.05);
	};
	return 0;
}

s32 ADXL357B_ReadXYZAxisResultDataFromFIFO(s32 *x, s32 *y, s32 *z) {
	u8 data[9] = {0};
	*x = *y = *z = 0;

	if (IMU2_Read_Len(ACC_I2C_ADDRESS,FIFO_DATA_REG_ADDR, 9, data)) {
		return -1;
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
}

s32 ADXL357B_ReadXYZAxisResultData(s32 *x, s32 *y, s32 *z) {
	u8 data[9] = {0};
	*x = *y = *z = 0;

	if (IMU2_Write_Len(ACC_I2C_ADDRESS,FIFO_DATA_REG_ADDR, 9, data)) {
		return -1;
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
}

s32 ADXL357B_GetStatus(u8 byte) {
	byte = IMU2_Read_Reg(ACC_I2C_ADDRESS,STATUS_REG_ADDR);
	return byte;
}

bool ADXL357B_CheckDataReady(void) {
	u8 stat = 0;
	ADXL357B_GetStatus(stat);
	return stat & 0x01;
}




//Following: HP203B Functions
//Private functions
/************************************************************
 * Initialize the structure of altitude offset and thresholds
 * - read comments for understanding values metric
 ***********************************************************/
static void HP203B_TH_StructInit(HP203x_TH_TypeDef *HP203B_TH_InitStruct) {
	HP203B_TH_InitStruct->ALT_OFF = 0;		// Altitude offset (cm)
	HP203B_TH_InitStruct->P_H_TH  = 0;		// Pressure upper bound threshold (Pa)
	HP203B_TH_InitStruct->P_M_TH  = 0;		// Pressure middle bound threshold (Pa)
	HP203B_TH_InitStruct->P_L_TH  = 0;		// Pressure lower bound threshold (Pa)
	HP203B_TH_InitStruct->A_H_TH  = 0;		// Altitude upper bound threshold (m)
	HP203B_TH_InitStruct->A_M_TH  = 0;		// Altitude middle bound threshold (m)
	HP203B_TH_InitStruct->A_L_TH  = 0;		// Altitude lower bound threshold (m)
	HP203B_TH_InitStruct->T_H_TH  = 0;		// Temperature upper bound threshold (°C)
	HP203B_TH_InitStruct->T_M_TH  = 0;		// Temperature middle bound threshold (°C)
	HP203B_TH_InitStruct->T_L_TH  = 0;		// Temperature lower bound threshold (°C)
	HP203B_TH_InitStruct->P_or_A  = 1;		// 0 for pressure, 1 for altitude
}

/************************************************************
 * Initialize the HP203B control registers structure
 * - read comments for understanding changes which you are can adjust
 ***********************************************************/
static void HP203B_CR_StructInit(HP203x_CR_TypeDef *HP203B_CR_InitStruct, HP203x_TH_TypeDef *HP203B_TH_Struct) {
	HP203B_TH_StructInit(HP203B_TH_Struct);												// Configure the structure of HP203B thresholds
	HP203B_CR_InitStruct->ALT_OFF_LSB = (u8)(HP203B_TH_Struct->ALT_OFF);			// Write LSB of altitude offset (1cm per count)
	HP203B_CR_InitStruct->ALT_OFF_MSB = (u8)(HP203B_TH_Struct->ALT_OFF >> 8);		// Write MSB of altitude offset
	if(!HP203B_TH_Struct->P_or_A) {														// If P_or_A = 0 (pressure)
		HP203B_CR_InitStruct->PA_H_TH_LSB = (u8)(HP203B_TH_Struct->P_H_TH >> 1);	// Write LSB for upper bound of pressure (0.02mbar or 2Pa per count)
		HP203B_CR_InitStruct->PA_H_TH_MSB = (u8)(HP203B_TH_Struct->P_H_TH >> 9);	// Write MSB for upper bound of pressure
		HP203B_CR_InitStruct->PA_M_TH_LSB = (u8)(HP203B_TH_Struct->P_M_TH >> 1);	// Write LSB for middle bound of pressure (0.02mbar or 2Pa per count)
		HP203B_CR_InitStruct->PA_M_TH_MSB = (u8)(HP203B_TH_Struct->P_M_TH >> 9);	// Write MSB for middle bound of pressure
		HP203B_CR_InitStruct->PA_L_TH_LSB = (u8)(HP203B_TH_Struct->P_L_TH >> 1);	// Write LSB for lower bound of pressure (0.02mbar or 2Pa per count)
		HP203B_CR_InitStruct->PA_L_TH_MSB = (u8)(HP203B_TH_Struct->P_L_TH >> 9);	// Write MSB for lower bound of pressure
	}
	else {																				// else if P_or_A = 1 (altitude)
		HP203B_CR_InitStruct->PA_H_TH_LSB = (u8)(HP203B_TH_Struct->A_H_TH);		// Write LSB for upper bound of altitude (1m per count)
		HP203B_CR_InitStruct->PA_H_TH_MSB = (u8)(HP203B_TH_Struct->A_H_TH >> 8);	// Write MSB for upper bound of altitude
		HP203B_CR_InitStruct->PA_M_TH_LSB = (u8)(HP203B_TH_Struct->A_M_TH);		// Write LSB for middle bound of altitude (1m per count)
		HP203B_CR_InitStruct->PA_M_TH_MSB = (u8)(HP203B_TH_Struct->A_M_TH >> 8);	// Write MSB for middle bound of altitude
		HP203B_CR_InitStruct->PA_L_TH_LSB = (u8)(HP203B_TH_Struct->A_L_TH);		// Write LSB for lower bound of altitude (1m per count)
		HP203B_CR_InitStruct->PA_L_TH_MSB = (u8)(HP203B_TH_Struct->A_L_TH >> 8);	// Write MSB for lower bound of altitude
	}
	HP203B_CR_InitStruct->T_H_TH = HP203B_TH_Struct->T_H_TH;							// Write upper bound of temperature (1°C per count)
	HP203B_CR_InitStruct->T_M_TH = HP203B_TH_Struct->T_M_TH;							// Write middle bound of temperature (1°C per count)
	HP203B_CR_InitStruct->T_L_TH = HP203B_TH_Struct->T_L_TH;							// Write lower bound of temperature (1°C per count)

	HP203B_CR_InitStruct->INT_EN =  // Disable/enable interrupt (0:disable,1:enable)
				1 << PA_RDY_EN	|	// Pressure/altitude is ready to reading
				0 << T_RDY_EN	|	// Temperature is ready to reading
				0 << PA_TRAV_EN	|	// Pressure/altitude traversed the middle threshold
				0 << T_TRAV_EN	|	// Temperature traversed the middle threshold
				0 << PA_WIN_EN	|	// Pressure/altitude is outside predefined window
				0 << T_WIN_EN;		// Temperature is outside predefined window

	HP203B_CR_InitStruct->INT_CFG =	// Select pin for interrupt output (0:INT0,1:INT1)
				HP203B_TH_Struct->P_or_A << PA_MODE |	// Selects whether the event detection parameters and the interrupts registers prefixed by
														// a ‘PA_’ corresponds to the pressure or the altitude measurement (0:pressure,1:altitude)
				0 << PA_RDY_CFG	|	// Pressure/altitude is ready to reading
				0 << T_RDY_CFG	|	// Temperature is ready to reading
				0 << PA_TRAV_CFG|	// Pressure/altitude traversed the middle threshold
				0 << T_TRAV_CFG	|	// Temperature traversed the middle threshold
				0 << PA_WIN_CFG	|	// Pressure/altitude is outside predefined window
				0 << T_WIN_CFG;		// Temperature is outside predefined window

	HP203B_CR_InitStruct->INT_SRC =	// Flags that indicates interrupt status (read only)
				0 << TH_ERR		|	// Indicates that improper settings for thresholds are set (lower bound above higher bound for example)
				0 << DEV_RDY	|	// Indicates whether the HP203B is ready (in the sleep state and not performing any operation) or not (0:busy,1:ready)
				0 << PA_RDY		|	// Pressure/altitude is ready to reading
				0 << T_RDY		|	// Temperature is ready to reading
				0 << PA_TRAV	|	// Pressure/altitude traversed the middle threshold
				0 << T_TRAV		|	// Temperature traversed the middle threshold
				0 << PA_WIN		|	// Pressure/altitude is outside predefined window
				0 << T_WIN;			// Temperature is outside predefined window

	HP203B_CR_InitStruct->INT_DIR =	// Check details of traversal or window interrupt events (read only)
				0 << CMPS_EN	|	// The data compensation enabled or disabled (0:disabled,1:enabled)
				0 << P_TRAV_DIR	|	// 1 if pressure rising from low to high, 0 if pressure falling from high to low (through middle level)
				0 << T_TRAV_DIR	|	// 1 if temperature rising from low to high, 0 if temperature falling from high to low (through middle level)
				0 << P_WIN_DIR	|	// 1 if pressure above the window, 0 if pressure below the window
				0 << T_WIN_DIR;		// 1 if temperature above the window, 0 if temperature below the window

	HP203B_CR_InitStruct->PARA = 1 << CMPS_EN;	// Enable/disable data compensation (0:disable,1:enable)
}

/* PUBLIC FUNCTIONS ================================================================================================================================= */
/************************************************************
 * Soft reset
 * - executed immediately and no matter what it has doing
 * - all memories reset to their default values
 * - automatically performed power-up sequence
 ***********************************************************/
void HP203B_SoftReset(void){
	IMU2_Write_Byte(HP203B_Addr,HP203B_SOFT_RST);

}

/************************************************************
 * Start convert
 * - start convert temperature/pressure/altitude to digital value
 * - OSR must be one of following:
					OSR4096
					OSR2048
					OSR1024
					OSR512
					OSR256
					OSR128
 * - Channel must be one of following:
 * 					PT_CHANNEL
					T_CHANNEL
 ***********************************************************/
void HP203B_StartConvert(u8 OSR, u8 Channel){
	u8 command = HP203B_ADC_CVT | OSR<<2 | Channel;
	IMU2_Write_Byte(HP203B_Addr,command);
}

/************************************************************
 * Get pressure and temperature
 * - read the temperature and the pressure values
 * - 3 bytes for temperature MSB first
 * - 3 bytes for pressure MSB first
 ***********************************************************/
void HP203B_ReadPT(void) {
	u8* buf = {0};
	IMU2_Read_Len(HP203B_Addr, HP203B_READ_PT, 6, buf);
	HP203B_Data.T = (s32)buf[0]<<16 | (s32)buf[1]<<8 | (s32)buf[2]; // Save the temperature value to the data structure
	if(HP203B_Data.T & (1<<23))	// If temperature below 0
		HP203B_Data.T |= 0xFF000; // Rewrite 8 MSBs by 1
	HP203B_Data.P = (u32)buf[3]<<16 | (u32)buf[4]<<8 | (u32)buf[5]; // Save the pressure value to the data structure
}

/************************************************************
 * Get altitude and temperature
 * - read the temperature and the altitude values
 * - 3 bytes for temperature MSB first
 * - 3 bytes for altitude MSB first
 ***********************************************************/
void HP203B_ReadAT(void) {
	u8* buf = {0};
	IMU2_Read_Len(HP203B_Addr, HP203B_READ_AT, 6, buf);
	HP203B_Data.T = (s32)buf[0]<<16 | (s32)buf[1]<<8 | (s32)buf[2]; // Save the temperature values to the data structure
	if(HP203B_Data.T & (1<<23)) // If temperature below 0
		HP203B_Data.T |= 0xFF000; // Rewrite 8 MSBs by 1
	HP203B_Data.H = (s32)buf[3]<<16 | (s32)buf[4]<<8 | (s32)buf[5]; // Save the altitude values ti the data structure
	if(HP203B_Data.H & (1<<23)) // If altitude below 0
		HP203B_Data.H |= 0xFF000; // Rewrite 8 MSBs by 1
}

/************************************************************
 * Get pressure
 * - read the pressure value
 * - 3 bytes for pressure MSB first
 ***********************************************************/
void HP203B_ReadP(void) {
	u8* buf = {0};
	IMU2_Read_Len(HP203B_Addr, HP203B_READ_P, 3, buf);
	HP203B_Data.P = (u32)buf[0]<<16 | (u32)buf[1]<<8 | (u32)buf[2]; // Save the pressure value to the data structure
}

/************************************************************
 * Get altitude
 * - read the altitude value
 * - 3 bytes for altitude MSB first
 ***********************************************************/
void HP203B_ReadA(void) {
	u8* buf = {0};
	IMU2_Read_Len(HP203B_Addr, HP203B_READ_A, 3, buf);
	HP203B_Data.H = (s32)buf[0]<<16 | (s32)buf[1]<<8 | (s32)buf[2]; // Save the altitude value to the data structure
	if(HP203B_Data.H & (1<<23)) // If altitude below 0
		HP203B_Data.H |= 0xFF000; // Rewrite 8 MSBs by 1
}

/************************************************************
 * Get temperature
 * - read the temperature value
 * - 3 bytes for temperature MSB first
 ***********************************************************/
void HP203B_ReadT(void) {
	u8* buf = {0};
	IMU2_Read_Len(HP203B_Addr, HP203B_READ_T, 3, buf);
	HP203B_Data.T = (s32)buf[0]<<16 | (s32)buf[1]<<8 | (s32)buf[2]; // Save the temperature value to the data structure
	if(HP203B_Data.T & (1<<23)) // If temperature below 0
		HP203B_Data.T |= 0xFF000; // Rewrite 8 MSBs by 1
}

/************************************************************
 * Start internal calibration
 * - re-calibrate internal circuits
 * - use when environment rapidly changed
 * - this command allows increase the accurate
 * - no need execute it if environment is stable
 * - after finishes sensor enters to sleep mode
 ***********************************************************/
void HP203B_Calibration(HP203x_CR_TypeDef *HP203B_CR_Struct) {
	IMU2_Write_Byte(HP203B_Addr, HP203B_ANA_CAL);
	while(1) { // Checking DEV_RDY bit of the INT_SRC register and wait until it will set to 1
	IMU2_Read_Reg(HP203B_Addr,HP203B_READ_REG | INT_SRC); // Read INT_SRC register from HP203B
	if(HP203B_CR_Struct->INT_SRC & (1<<DEV_RDY)) // Check DEV_RDY bit
		break; // Break if HP203B is ready (DEV_RDY = 1)
	}
}








/*
imu2 includes the following functions: Accelerator,Barometer,Altimeter.
With the following function, MCU gets data from imu2 in set frequency and store them in sd cards.
 */
void IMU2_Task(void *param)
{
	u32 lastWakeTime = xTaskGetTickCount();
	s32 pp [3]={-1};
	float gg[3]={-1};
	u8 i;
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
			sprintf((char*)logname_buf,"imu2_%lld.txt",FreeRTOSRunTimeTicks);
			if(f_open(&flight_log_fil,logname_buf,FA_OPEN_APPEND|FA_WRITE)==0)
			{
				file_not_open_flag = false;
			}
		}
		ADXL357B_ReadXYZAxisResultData(&pp[0] , &pp[1], &pp[2]);
		
		for(i=0;i<3;i++)
		{
		gg[i]=(float)((float)pp[i]/524288*10);
		}
		 if(!file_not_open_flag == true)
		// if(!file_not_open_flag && flightloggerbutton == true)
		{
			sprintf((char*)flight_log_buf,"%.4f ,%.4f,%.4f\r\n",gg[0],gg[1],gg[2]);
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

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 IMU2_Write_Len(u8 dev_Addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
	IIC_Imu_Start(); 
	IIC_Imu_Send_Byte((dev_Addr<<1)|0);
	if(IIC_Imu_Wait_Ack())
	{
		IIC_Imu_Stop();
		return 1;
	}
	IIC_Imu_Send_Byte(reg);
	IIC_Imu_Wait_Ack();
	for(i=0;i<len;i++)
	{
		IIC_Imu_Send_Byte(buf[i]);
		if(IIC_Imu_Wait_Ack())
		{
			IIC_Imu_Stop();
			return 1;
		}		
	}
	IIC_Imu_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 IMU2_Read_Len(u8 dev_Addr,u8 reg,u8 len,u8 *buf)
{ 
	IIC_Imu_Start(); 
	IIC_Imu_Send_Byte((dev_Addr<<1)|0);
	if(IIC_Imu_Wait_Ack())
	{
		IIC_Imu_Stop();
		return 1;
	}
	IIC_Imu_Send_Byte(reg);
	IIC_Imu_Wait_Ack();
	IIC_Imu_Start();
	IIC_Imu_Send_Byte((dev_Addr<<1)|1);
	IIC_Imu_Wait_Ack();
	while(len)
	{
		if(len==1)*buf=IIC_Imu_Read_Byte(0); 
		else *buf=IIC_Imu_Read_Byte(1);
		len--;
		buf++; 
	}
	IIC_Imu_Stop();
	return 0;
}
//IIC写一个字节到寄存器 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 IMU2_Write_Reg(u8 dev_Addr,u8 reg,u8 data)
{ 
	IIC_Imu_Start(); 
	IIC_Imu_Send_Byte((dev_Addr << 1) | 0x00);
	if(IIC_Imu_Wait_Ack())
	{
		IIC_Imu_Stop();
		return 1;
	}
	IIC_Imu_Send_Byte(reg);
	IIC_Imu_Wait_Ack();
	IIC_Imu_Send_Byte(data);
	if(IIC_Imu_Wait_Ack())
	{
		IIC_Imu_Stop();
		return 1;
	}
	IIC_Imu_Stop();
	return 0;
}
//IIC从寄存器读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 IMU2_Read_Reg(u8 dev_Addr,u8 reg)
{
	u8 res;
	IIC_Imu_Start();
	IIC_Imu_Send_Byte((dev_Addr << 1) | 0x00);
	IIC_Imu_Wait_Ack();
	IIC_Imu_Send_Byte(reg);
	IIC_Imu_Wait_Ack();
	IIC_Imu_Start();
	IIC_Imu_Send_Byte((dev_Addr << 1) | 0x01);
	IIC_Imu_Wait_Ack();
	res = IIC_Imu_Read_Byte(0);
	IIC_Imu_Stop();
	return res;
}

//直接读取一个字节
u8 IMU2_Write_Byte(u8 dev_Addr,u8 command)
{ 
	IIC_Imu_Start(); 
	IIC_Imu_Send_Byte((dev_Addr << 1) | 0x00);
	if(IIC_Imu_Wait_Ack())
	{
		IIC_Imu_Stop();
		return 1;
	}
	IIC_Imu_Send_Byte(command);
	if(IIC_Imu_Wait_Ack())
	{
		IIC_Imu_Stop();
		return 1;
	}
	IIC_Imu_Stop();
	return 0;
}

/************************************************************
 * Write all registers from the structure to HP203B
 ***********************************************************/
void HP203B_WriteAllReg(HP203x_CR_TypeDef *HP203B_CR_Struct) {
	u8 address;
	for(address = ALT_OFF_LSB; address <= INT_SRC; address++) // For addresses from LT_OFF_LSB to INT_SRC
		IMU2_Write_Reg(HP203B_Addr, HP203B_WRITE_REG | address, *((u8*)HP203B_CR_Struct + address)); // Write data from the structure to appropriate register by address
	IMU2_Write_Reg(HP203B_Addr, HP203B_WRITE_REG | PARA, *((u8*)HP203B_CR_Struct + address)); // Write data from the structure to PARA register
}

u8 ADXL357B_Init(void){
		u8 ID = 0;
		IIC_Imu_Init();
		ADXL357B_Reset();
		delay_ms(20);
		ADXL357B_SetPowerCtr(0x00);//set to normal mode
		ID = IMU2_Read_Reg(ACC_I2C_ADDRESS,POWER_CTR_REG_ADDR);
		if (ID != 0x00) {
			Led_Flash(1);
			printf("%.4f\r\n",(double)ID);
			//return -1;
		}
		delay_ms(20);
		ADXL357B_SetRange(TEN_G);
		delay_ms(20);
		ADXL357B_SetFilter(0x33);//011--1.55*10^-4;0011--500Hz
		delay_ms(20);
		ADXL357B_SetActEnable(1, 1, 1);//enable x,y,z 3 axis
		ADXL357B_GetStatus(ID);
		printf("%.4f\r\n",(double)ID);
		ID = IMU2_Read_Reg(ACC_I2C_ADDRESS,DEV_ID_REG_ADDR);
		if (ID != 0xed) {
			Led_Flash(1);
			printf("%.4f\r\n",(double)ID);
			//return -1;
		}
		delay_ms(20);
		return 0;
	}

/* Initialization sequence for HP203B sensor */
void HP203B_Init(HP203x_CR_TypeDef *HP203B_CR_Struct, HP203x_TH_TypeDef *HP203B_TH_Struct) {
//	INT0_GPIO->IO0IntEnR |= 1<<INT0_PIN;					// Enable rise interrupt for INT0
//	INT1_GPIO->IO0IntEnR |= 1<<INT1_PIN;					// Enable rise interrupt for INT1
	IIC_Imu_Init();
	HP203B_SoftReset();
	while(1) {												// Checking DEV_RDY bit of the INT_SRC register and wait until it will set to 1
		IMU2_Read_Reg(HP203B_Addr,HP203B_READ_REG | INT_SRC); // Read INT_SRC register from HP203BB
		if(HP203B_CR_Struct->INT_SRC & (1<<DEV_RDY))		// Check DEV_RDY bit
			break;											// Break if HP203B is ready (DEV_RDY = 1)
	}
	HP203B_CR_StructInit(HP203B_CR_Struct, HP203B_TH_Struct);// Configure the structure of HP203B registers
	HP203B_WriteAllReg(HP203B_CR_Struct);					// Copy this structure into internal registers of HP203B
}
