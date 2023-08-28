#ifndef __IMU2_H
#define __IMU2_H
#endif

#include "sys.h"
#include "delay.h"
#include "flight_log.h"
#include "myiic.h"
#include "led.h"

// Using of HP203B sensor (0:don't use,1:use)
#define ADXL357B_USAGE	1

//Accelerator Register Map

#define ACC_I2C_ADDRESS   		  	0x1d

#define DEV_MEMS_REG_ADDR         	0x01
#define DEV_ID_REG_ADDR           	0x02
#define DEV_VERSION_ID_REG_ADDR   	0x03
#define STATUS_REG_ADDR           	0x04
#define FIFO_ENTRY_REG_ADDR       	0x05

#define ACTION_ENABLE_REG_ADDR    	0x24
#define SET_THRESHOLD_REG_ADDR    	0x25
#define GET_ACTIVE_COUNT_REG_ADDR 	0x26
#define SET_INT_PIN_MAP_REG_ADDR  	0x2a

#define X_DATA_REG_ADDR           	0x08
#define FIFO_DATA_REG_ADDR        	0x11

#define POWER_CTR_REG_ADDR        	0x2d

#define TEMPERATURE_REG_ADDR      	0x06
#define FILTER_REG_ADDR           	0x28

#define SET_RANGE_REG_ADDR        	0x2c
#define RESET_REG_ADDR            	0x2f

//Accelerator value define
typedef enum {
    X_DIR,
    Y_DIR,
    Z_DIR,
} AxisPos;


typedef enum {
    TEN_G = 1,
    TWENTY_G,
    FOURTY_G,
} Acc_Range;


#ifndef HP203B_H_
#define HP203B_H_
//Aux Barometer and Altimeter define
#define HP203B_USAGE	1		// Using of HP203B sensor (0:don't use,1:use)

#define HP203B_Addr  				0x77 // CSB low,the 7bit address of HP203B

/* INT_EN bits define */
#define PA_RDY_EN		5
#define T_RDY_EN		4
#define PA_TRAV_EN		3
#define T_TRAV_EN		2
#define PA_WIN_EN		1
#define T_WIN_EN		0

/* INT_CFG bits define */
#define PA_MODE			6
#define PA_RDY_CFG		5
#define T_RDY_CFG		4
#define PA_TRAV_CFG		3
#define T_TRAV_CFG		2
#define PA_WIN_CFG		1
#define T_WIN_CFG		0

/* INT_SRC bits define */
#define TH_ERR			7
#define DEV_RDY			6		// Indicates that the sensor is ready (don't do any operation and in a sleep mode)
#define PA_RDY			5		// Indicates that the pressure (or altitude) measurement is done and the result is ready to read
#define T_RDY			4		// Indicate that the temperature measurement is done and the result is ready to read
#define PA_TRAV			3		// Indicate that the pressure (or altitude) value has traversed the middle threshold during the last measurement
#define T_TRAV			2		// Indicate that the temperature value has traversed the middle threshold during the last measurement
#define PA_WIN			1		// Indicate that the pressure (or altitude) value locates outside the pre-defined window (the value in between the upper bound and lower bound thresholds) during the last measurement
#define T_WIN			0		// Indicate that the temperature value locates outside the pre-defined window (the value in between the upper bound and lower bound thresholds) during the last measurement

/* INT_DIR bits define */
#define CMPS_EN			7
#define P_TRAV_DIR		3
#define T_TRAV_DIR		2
#define P_WIN_DIR		1
#define T_WIN_DIR		0

/* PARA bits define */
#define CMPS_EN			7

/* The 2-bit channel (CHNL) parameter tells the device the data from which channel(s) shall be converted by the internal ADC */
typedef enum {
	PT_CHANNEL	= 0x00,		// Sensor pressure and temperature channel
	T_CHANNEL	= 0x01		// Temperature channel
} HP203x_Channel;

/* The 3-bit OSR defines the decimation rate of the internal digital filter */
typedef enum {
	OSR4096		= 0x00,		// 4096
	OSR2048		= 0x01,		// 2048
	OSR1024		= 0x02,		// 1024
	OSR512		= 0x03,		// 512
	OSR256		= 0x04,		// 256
	OSR128		= 0x05		// 128
} HP203x_OSR;

/* Command set for HP203B sensor */
typedef enum {
	HP203B_SOFT_RST		= 0x06,		// Soft reset the device
	HP203B_READ_PT		= 0x10,		// Read the temperature and pressure values
	HP203B_READ_AT		= 0x11,		// Read the temperature and altitude values
	HP203B_ANA_CAL		= 0x28,		// Re-calibrate the internal analog blocks
	HP203B_READ_P		= 0x30,		// Read the pressure value only
	HP203B_READ_A		= 0x31,		// Read the altitude value only
	HP203B_READ_T		= 0x32,		// Read the temperature value only
	HP203B_ADC_CVT		= 0x40,		// Perform ADC conversion
	HP203B_READ_REG		= 0x80,		// Read out the control registers
	HP203B_WRITE_REG	= 0xC0		// Write in the control registers
} HP203x_Command;

typedef enum {
	ALT_OFF_LSB	= 0x00,
	ALT_OFF_MSB	= 0x01,
	PA_H_TH_LSB	= 0x02,
	PA_H_TH_MSB	= 0x03,
	PA_M_TH_LSB	= 0x04,
	PA_M_TH_MSB	= 0x05,
	PA_L_TH_LSB	= 0x06,
	PA_L_TH_MSB	= 0x07,
	T_H_TH		= 0x08,
	T_M_TH		= 0x09,
	T_L_TH		= 0x0A,
	INT_EN		= 0x0B,
	INT_CFG		= 0x0C,
	INT_SRC		= 0x0D,
	INT_DIR		= 0x0E,
	PARA		= 0x0F
} HP203x_Registers;

/* HP203B control registers */
typedef struct {
	uint8_t ALT_OFF_LSB;	// Altitude Offset Compensation
	uint8_t ALT_OFF_MSB;
	uint8_t PA_H_TH_LSB;	// Set pressure/altitude upper bound threshold
	uint8_t PA_H_TH_MSB;
	uint8_t PA_M_TH_LSB;	// Set pressure/altitude middle bound threshold
	uint8_t PA_M_TH_MSB;
	uint8_t PA_L_TH_LSB;	// Set pressure/altitude lower bound threshold
	uint8_t PA_L_TH_MSB;
	uint8_t T_H_TH;			// Set temperature upper bound threshold
	uint8_t T_M_TH;			// Set temperature middle bound threshold
	uint8_t T_L_TH;			// Set temperature lower bound threshold
	uint8_t INT_EN;			// Interrupts enable/disable
	uint8_t INT_CFG;		// Interrupts configure
	uint8_t INT_SRC;		// Interrupts status
	uint8_t INT_DIR;
	uint8_t PARA;			// Data compensation enable/disable
} HP203x_CR_TypeDef;

typedef struct {
	int16_t  ALT_OFF;		// Altitude offset (cm)
	uint16_t P_H_TH;		// Pressure upper bound threshold (Pa)
	uint16_t P_M_TH;		// Pressure middle bound threshold (Pa)
	uint16_t P_L_TH;		// Pressure lower bound threshold (Pa)
	int16_t  A_H_TH;		// Altitude upper bound threshold (m)
	int16_t  A_M_TH;		// Altitude middle bound threshold (m)
	int16_t  A_L_TH;		// Altitude lower bound threshold (m)
	int8_t   T_H_TH;		// Temperature upper bound threshold (°„C)
	int8_t   T_M_TH;		// Temperature middle bound threshold (°„C)
	int8_t   T_L_TH;		// Temperature lower bound threshold (°„C)
	int8_t   P_or_A;		// 0 for pressure, 1 for altitude
} HP203x_TH_TypeDef;

/* HP203B measuring data */
typedef struct {
	int32_t  T;				// Temperature
	uint32_t P;				// Pressure
	int32_t  H;				// Altitude
} HP203x_Data_TypeDef;


#endif

//Usage define 
//Accelerator
#if ADXL357B_USAGE

uint8_t ADXL357B_Reset(void);

uint8_t ADXL357B_Init(void);

/** set action enable.
    @param enable_x enable x axis action.When the x axis result above threshold,trigger event.
    @param enable_y enable y axis action.When the y axis result above threshold,trigger event.
    @param enable_z enable z axis action.When the z axis result above threshold,trigger event.
 **/
int32_t ADXL357B_SetActEnable(bool enable_x, bool enable_y, bool enable_z);

/** Read x/y/z axis data from register.

 **/
int32_t ADXL357B_ReadXYZAxisResultData(int32_t *x, int32_t *y, int32_t *z);
/** Config ADXL357 mode.
    bit2 - DRDY_OFF ,
    bit1 - TEMP-OFF ,
    bit0 - running mode,0 for measurement mode,1 for standby mode.
 **/
uint8_t ADXL357B_SetPowerCtr(u8 val);

int32_t ADXL357B_ReadTemperature(float T);

/** Get status register data.
    bit4 - NVM-BUSY
    bit3 - Activity
    bit2 - FIFO_OVR
    bit1 - FIFO_FULL
    bit0 - DATA_RDY
 **/
int32_t ADXL357B_GetStatus(u8 byte);
/** Check whether the  status register's first bit is 1.
 **/
bool ADXL357B_CheckDataReady(void);

uint8_t ADXL357B_SetFilter(u8 val);
/** Set ADXL357's full scale range.
    °¿10g,°¿20g,°¿40g
 **/
int32_t ADXL357B_SetRange(Acc_Range range);
/** Read x/y/z axis data from FIFO.

 **/
int32_t ADXL357B_ReadXYZAxisResultDataFromFIFO(int32_t *x, int32_t *y, int32_t *z) ;

/** Set threshold,when measured result over than threshold,trigger event,if corresponding INT function has enabled,trigger interruct event.
    @param acc_g The threshold value,unit is g.
    @param factory Acceleration value corresponding to one acceleration result value,the result value is read from register.

 **/
int32_t ADXL357B_setActThreshold(float acc_g, float factory);

/** Event count.event count incresed when axis result value over than threshold.

 **/
int32_t ADXL357B_GetActiveCnt(void);

#endif

//Barometer and Altimeter
#if HP203B_USAGE
/* HP203B internal commands */
void HP203B_SoftReset(void);
void HP203B_StartConvert(uint8_t, uint8_t);
void HP203B_ReadPT(void);
void HP203B_ReadAT(void);
void HP203B_ReadP(void);
void HP203B_ReadA(void);
void HP203B_ReadT(void);
void HP203B_Calibration(HP203x_CR_TypeDef*);
//void HP203B_ReadReg(HP203x_CR_TypeDef*, uint8_t);
//void HP203B_WriteReg(HP203x_CR_TypeDef*, uint8_t);

/* HP203B general commands */
//void HP203B_ReadAllReg(HP203x_CR_TypeDef*);
void HP203B_WriteAllReg(HP203x_CR_TypeDef*);
void HP203B_Init(HP203x_CR_TypeDef*, HP203x_TH_TypeDef*);
#endif /* HP203B_H_ */

int IMU2_Init(void);

void IMU2_Task(void *param);

u8 IMU2_Write_Len(u8 dev_Addr,u8 reg,u8 len,u8 *buf);

u8 IMU2_Read_Len(u8 dev_Addr,u8 reg,u8 len,u8 *buf);

u8 IMU2_Write_Reg(u8 dev_Addr,u8 reg,u8 data);

u8 IMU2_Read_Reg(u8 dev_Addr,u8 reg);

u8 IMU2_Write_Byte(u8 dev_Addr,u8 command);


