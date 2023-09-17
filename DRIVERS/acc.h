#ifndef __ACC_H
#define __ACC_H


#include "stm32f4xx.h" 
#include <stdbool.h>
#include "sys.h"

#define u8 									uint8_t


// Using of Acc sensor (0:don't use,1:use)
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
}Acc_Range;

//Usage define 
//Accelerator
#if ADXL357B_USAGE

void ACC_Task(void *param);

u8 ADXL357B_Reset(void);

u8 ACC_Init(void);

/** set action enable.
    @param enable_x enable x axis action.When the x axis result above threshold,trigger event.
    @param enable_y enable y axis action.When the y axis result above threshold,trigger event.
    @param enable_z enable z axis action.When the z axis result above threshold,trigger event.
 **/

/** Read x/y/z axis data from register.

 **/

int32_t * ADXL357B_ReadXYZAxisResultData(void);
//int32_t ADXL357B_ReadXYZAxisResultData(int32_t *x, int32_t *y, int32_t *z);
/** Config ADXL357 mode.
    bit2 - DRDY_OFF ,
    bit1 - TEMP-OFF ,
    bit0 - running mode,0 for measurement mode,1 for standby mode.
 **/
u8 ADXL357B_SetPowerCtr(u8 val);

int ADXL357B_ReadTemperature(float T);

/** Get status register data.
    bit4 - NVM-BUSY
    bit3 - Activity
    bit2 - FIFO_OVR
    bit1 - FIFO_FULL
    bit0 - DATA_RDY
 **/
u8 ADXL357B_GetStatus(void);
/** Check whether the  status register's first bit is 1.
 **/
bool ADXL357B_CheckDataReady(void);

u8 ADXL357B_SetFilter(u8 val);
/** Set ADXL357's full scale range.
    ¡À10g,¡À20g,¡À40g
 **/
u8 ADXL357B_SetRange(Acc_Range range);
/** Read x/y/z axis data from FIFO.

 **/
int * ADXL357B_ReadXYZAxisResultDataFromFIFO(void)  ;

/** Set threshold,when measured result over than threshold,trigger event,if corresponding INT function has enabled,trigger interruct event.
    @param acc_g The threshold value,unit is g.
    @param factory Acceleration value corresponding to one acceleration result value,the result value is read from register.

 **/
u8 ADXL357B_SetActEnable(bool enable_x, bool enable_y, bool enable_z);

/** Event count.event count incresed when axis result value over than threshold.

 **/
u8 ADXL357B_GetActiveCnt(void);

#endif


#endif
