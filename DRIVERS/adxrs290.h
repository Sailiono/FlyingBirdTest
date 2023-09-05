#ifndef __ADXRS290_H
#define __ADXRS290_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>
#include "spi.h"
#include <stdio.h>
#include <math.h>
#include "delay.h"
#include "led.h"
#include "flight_log.h"
#include "sbus.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

//define const

// Using of ADXRS290 sensor (0:don't use,1:use)
#define ADXRS290_USAGE							1
#define	ADXRS290_CS 							PBout(12)  		//加速度计的片选信号


//Accelerator Register Map

/* ADXRS290 ID */
#define ADXRS290_ADI_ID							0xAD
#define ADXRS290_MEMS_ID						0x1D
#define ADXRS290_DEV_ID							0x92

/* ADXRS290 Registers */
#define ADXRS290_REG_ADI_ID						0x00
#define ADXRS290_REG_MEMS_ID					0x01
#define ADXRS290_REG_DEV_ID						0x02
#define ADXRS290_REG_REV_ID						0x03
#define ADXRS290_REG_SN0						0x04
#define ADXRS290_REG_SN1						0x05
#define ADXRS290_REG_SN2						0x06
#define ADXRS290_REG_SN3						0x07
#define ADXRS290_REG_DATAX0						0x08
#define ADXRS290_REG_DATAX1						0x09
#define ADXRS290_REG_DATAY0						0x0A
#define ADXRS290_REG_DATAY1						0x0B
#define ADXRS290_REG_TEMP0						0x0C
#define ADXRS290_REG_TEMP1						0x0D

#define ADXRS290_REG_POWER_CTL					0x10
#define ADXRS290_REG_FILTER						0x11
#define ADXRS290_REG_DATA_READY					0x12

#define ADXRS290_READ							0x80
#define ADXRS290_TSM							0x01
#define ADXRS290_MEASUREMENT					0x02
#define ADXRS290_DATA_RDY_OUT					0x01
#define ADXRS290_SYNC_MASK						0x03
#define ADXRS290_SYNC(x)						(x) & ADXRS290_SYNC_MASK
#define ADXRS290_LPF_MASK						0x07
#define ADXRS290_LPF(x)							(x) & ADXRS290_LPF_MASK
#define ADXRS290_HPF_MASK						0xF0
#define ADXRS290_HPF(x)							((x) & ADXRS290_HPF_MASK )>>4

#define ADXRS290_READ_REG(reg)					ADXRS290_READ | (reg)

#define ADXRS290_MAX_TRANSITION_TIME_MS 		100
/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @enum adxrs290_mode
 * @brief Mode of the adxrs290
 */
enum adxrs290_mode {
	/** Standby mode */
	ADXRS290_MODE_STANDBY,
	/** Measurement mode */
	ADXRS290_MODE_MEASUREMENT,
};

/**
* @enum adxrs290_channel
* @brief Channel of teh adxrs290 data rate
*/
enum adxrs290_channel {
	/** X-Axis */
	ADXRS290_CHANNEL_X,
	/** Y-Axis */
	ADXRS290_CHANNEL_Y,
	/** Temp */
	ADXRS290_CHANNEL_TEMP,
 };

/**
 * @enum adxrs290_lpf
 * @brief Low-Pass filter pole location
 */
enum adxrs290_lpf {
	ADXRS290_LPF_480HZ,
	ADXRS290_LPF_320HZ,
	ADXRS290_LPF_160HZ,
	ADXRS290_LPF_80HZ,
	ADXRS290_LPF_56HZ6,
	ADXRS290_LPF_40HZ,
	ADXRS290_LPF_28HZ3,
	ADXRS290_LPF_20HZ
};

 /**
  * @enum adxrs290_hpf
  * @brief High-Pass filter pole location
  */
 enum adxrs290_hpf {
	ADXRS290_HPF_ALL_PASS,
 	ADXRS290_HPF_0HZ011,
	ADXRS290_HPF_0HZ022,
	ADXRS290_HPF_0HZ044,
	ADXRS290_HPF_0HZ087,
	ADXRS290_HPF_0HZ175,
	ADXRS290_HPF_0HZ350,
	ADXRS290_HPF_0HZ700,
	ADXRS290_HPF_1HZ400,
	ADXRS290_HPF_2HZ800,
	ADXRS290_HPF_11HZ30
 };

/**
 * @struct adxrs290_init_param
 * @brief Device driver initialization structure
 */
struct adxrs290_init_param {
	/** SPI Initialization structure. */
	//struct spi_init_param	spi_init;
	/** Sync pin initialization */
	//struct gpio_init_param sync;
	/** Initial Mode */
	enum adxrs290_mode mode;
	/** Initial lpf settings */
	enum adxrs290_lpf lpf;
	/** Initial hpf settings */
	enum adxrs290_hpf hpf;

};

/**
 * @struct adxrs290_dev
 * @brief Device driver handler
 */
struct adxrs290_dev {
	/** SPI handler */
	struct spi_desc *spi_desc;
	/** Sync handler */
	struct gpio_desc *sync;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

void Adxrs290_Init(void);
void Adxrs290_Reg_Write(u8 regAddr,u8 data);
int64_t Adxrs290_Multi_Reg_Read(u8 regAddr,u8 bytes_to_read,s16* read_out_data);
void ADXRS290_task(void *param);
s8 Adxrs290_Reg_Read(u8 regAddr,s8 data);

#endif /* ADXRS290_H_ */
