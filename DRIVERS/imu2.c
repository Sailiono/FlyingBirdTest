#include "imu2.h"
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

#define return_err -1

/* PUBLIC VARIABLES ================================================================================================================================ */
HP203x_TH_TypeDef		HP203B_TH_Struct;
HP203x_CR_TypeDef		HP203B_CR_Struct;
HP203x_Data_TypeDef		HP203B_Data;




//Following: HP203B Functions
//Private functions
/************************************************************
 * Initialize the structure of altitude offset and thresholds
 * - read comments for understanding values metric
 ***********************************************************/
/*static void HP203B_TH_StructInit(HP203x_TH_TypeDef *HP203B_TH_InitStruct) {
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
}*/

/************************************************************
 * Initialize the HP203B control registers structure
 * - read comments for understanding changes which you are can adjust
 ***********************************************************/
/*static void HP203B_CR_StructInit(HP203x_CR_TypeDef *HP203B_CR_InitStruct, HP203x_TH_TypeDef *HP203B_TH_Struct) {
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
}*/

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
	int *pp;
	float output[3]={-1};
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
		
		for(i=0;i<3;i++)
		{
		output[i]=(float)(*(pp + i))/524288*10;
		}
		if(!file_not_open_flag == true)
		// if(!file_not_open_flag && flightloggerbutton == true)
		{
			sprintf((char*)flight_log_buf,"%.4f , %.4f, %.4f\r\n",output[0],output[1],output[2]);
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



/************************************************************
 * Write all registers from the structure to HP203B
 ***********************************************************/
void HP203B_WriteAllReg(HP203x_CR_TypeDef *HP203B_CR_Struct) {
	u8 address;
	for(address = ALT_OFF_LSB; address <= INT_SRC; address++) // For addresses from LT_OFF_LSB to INT_SRC
		IMU2_Write_Reg(HP203B_Addr, HP203B_WRITE_REG | address, *((u8*)HP203B_CR_Struct + address)); // Write data from the structure to appropriate register by address
	IMU2_Write_Reg(HP203B_Addr, HP203B_WRITE_REG | PARA, *((u8*)HP203B_CR_Struct + address)); // Write data from the structure to PARA register
}


/* Initialization sequence for HP203B sensor */
/*void HP203B_Init(HP203x_CR_TypeDef *HP203B_CR_Struct, HP203x_TH_TypeDef *HP203B_TH_Struct) {
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
}*/
