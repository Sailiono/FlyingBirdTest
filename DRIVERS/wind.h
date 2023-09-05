#ifndef __WIND_H
#define __WIND_H
#include "stm32f4xx.h" 

#define  SM30_BARO_X_N  				1  /* multiplying power of pressure data. X1,X10,X100...*/
#define  SM30_TEMP_X_N  				10  /* multiplying power of temperture data. X1,X10,X100...*/


/* i2c slave address */
#define SM3041_ADDR             		0x28 	          

/* sm3041 register value */
#define SM3041_MAXCOUNT         		14745
#define SM3041_MINCOUNT        			1638
    
/* sm3041 device info for RT-Thread sensor device*/
#define SM3041_PRESSURE_MAX     		34474
#define SM3041_PRESSURE_MIN     		0
#define SM3041_PRESSURE_PERIOD   		2   /* read 500 times in 1 second */


void windspeed_Task(void *param);
int SM3041_Data_FetchBaro(void);
int* SM3041_Data_FetchAll(void);
uint8_t SM3041_Init(void);


#endif
