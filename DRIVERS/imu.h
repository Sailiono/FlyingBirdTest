#ifndef __IMU_H
#define __IMU_H			    
#include "sys.h"
#include "main.h" 
#include <stdio.h>

void imu_Init(void);
#define	ACC_CS 								PEout(0)  		//加速度计的片选信号
#define	GYRO_CS 							PEout(1)  		//陀螺仪的片选信号
#define	MAG_CS 								PEout(2)  		//磁力计的片选信号
#define	GYRO		    					1  		        //选择陀螺仪
#define	MAG		    						2  		        //选择磁力计
#define u8 									uint8_t
#define	Read		    					0x80
#define	Write		    					0x00

// BMX055 Gyroscope Registers, i2c address is 0x68
#define BMX055_GYRO_WHOAMI           0x00  // should return 0x0F
//#define BMX055_GYRO_Reserved       0x01
#define BMX055_GYRO_RATE_X_LSB       0x02
#define BMX055_GYRO_RATE_X_MSB       0x03
#define BMX055_GYRO_RATE_Y_LSB       0x04
#define BMX055_GYRO_RATE_Y_MSB       0x05
#define BMX055_GYRO_RATE_Z_LSB       0x06
#define BMX055_GYRO_RATE_Z_MSB       0x07
//#define BMX055_GYRO_Reserved       0x08
#define BMX055_GYRO_INT_STATUS_0  0x09
#define BMX055_GYRO_INT_STATUS_1  0x0A
#define BMX055_GYRO_INT_STATUS_2  0x0B
#define BMX055_GYRO_INT_STATUS_3  0x0C
//#define BMX055_GYRO_Reserved    0x0D
#define BMX055_GYRO_FIFO_STATUS   0x0E
#define BMX055_GYRO_RANGE         0x0F
#define BMX055_GYRO_BW            0x10
#define BMX055_GYRO_LPM1          0x11
#define BMX055_GYRO_LPM2          0x12
#define BMX055_GYRO_RATE_HBW      0x13
#define BMX055_GYRO_BGW_SOFTRESET 0x14
#define BMX055_GYRO_INT_EN_0      0x15
#define BMX055_GYRO_INT_EN_1      0x16
#define BMX055_GYRO_INT_MAP_0     0x17
#define BMX055_GYRO_INT_MAP_1     0x18
#define BMX055_GYRO_INT_MAP_2     0x19
#define BMX055_GYRO_INT_SRC_1     0x1A
#define BMX055_GYRO_INT_SRC_2     0x1B
#define BMX055_GYRO_INT_SRC_3     0x1C
//#define BMX055_GYRO_Reserved    0x1D
#define BMX055_GYRO_FIFO_EN       0x1E
//#define BMX055_GYRO_Reserved    0x1F
//#define BMX055_GYRO_Reserved    0x20
#define BMX055_GYRO_INT_RST_LATCH 0x21
#define BMX055_GYRO_HIGH_TH_X     0x22
#define BMX055_GYRO_HIGH_DUR_X    0x23
#define BMX055_GYRO_HIGH_TH_Y     0x24
#define BMX055_GYRO_HIGH_DUR_Y    0x25
#define BMX055_GYRO_HIGH_TH_Z     0x26
#define BMX055_GYRO_HIGH_DUR_Z    0x27
//#define BMX055_GYRO_Reserved    0x28
//#define BMX055_GYRO_Reserved    0x29
//#define BMX055_GYRO_Reserved    0x2A
#define BMX055_GYRO_SOC           0x31
#define BMX055_GYRO_A_FOC         0x32
#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
#define BMX055_GYRO_BGW_SPI3_WDT  0x34
//#define BMX055_GYRO_Reserved    0x35
#define BMX055_GYRO_OFC1          0x36
#define BMX055_GYRO_OFC2          0x37
#define BMX055_GYRO_OFC3          0x38
#define BMX055_GYRO_OFC4          0x39
#define BMX055_GYRO_TRIM_GP0      0x3A
#define BMX055_GYRO_TRIM_GP1      0x3B
#define BMX055_GYRO_BIST          0x3C
#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
#define BMX055_GYRO_FIFO_CONFIG_1 0x3E

// BMX055 magnetometer registers, i2c address is 0x10
#define BMX055_MAG_WHOAMI         0x40  // should return 0x32
#define BMX055_MAG_Reserved       0x41
#define BMX055_MAG_XOUT_LSB       0x42
#define BMX055_MAG_XOUT_MSB       0x43
#define BMX055_MAG_YOUT_LSB       0x44
#define BMX055_MAG_YOUT_MSB       0x45
#define BMX055_MAG_ZOUT_LSB       0x46
#define BMX055_MAG_ZOUT_MSB       0x47
#define BMX055_MAG_ROUT_LSB       0x48
#define BMX055_MAG_ROUT_MSB       0x49
#define BMX055_MAG_INT_STATUS     0x4A
#define BMX055_MAG_PWR_CNTL1      0x4B
#define BMX055_MAG_PWR_CNTL2      0x4C
#define BMX055_MAG_INT_EN_1       0x4D
#define BMX055_MAG_INT_EN_2       0x4E
#define BMX055_MAG_LOW_THS        0x4F
#define BMX055_MAG_HIGH_THS       0x50
#define BMX055_MAG_REP_XY         0x51
#define BMX055_MAG_REP_Z          0x52
/* Trim Extended Registers */


// Using the Teensy Mini Add-On board, SDO1 = SDO2 = CSB3 = GND as designed
// Seven-bit device addresses are ACC = 0x18, GYRO = 0x68, MAG = 0x10
#define BMX055_ACC_ADDRESS  0x18   // Address of BMX055 accelerometer
#define BMX055_GYRO_ADDRESS 0x68   // Address of BMX055 gyroscope
#define BMX055_MAG_ADDRESS  0x10   // Address of BMX055 magnetometer
#define MS5637_ADDRESS      0x76   // Address of altimeter




enum Gscale {
  G_2000DPS = 0,
  G_1000DPS,
  G_500DPS,
  G_250DPS,
  G_125DPS
};


enum GODRBW {
  G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
  G_2000Hz230Hz,
  G_1000Hz116Hz,
  G_400Hz47Hz,
  G_200Hz23Hz,		// 200hz sample rate and filtered with 23hz bandwidth
  G_100Hz12Hz,
  G_200Hz64Hz,
  G_100Hz32Hz  // 100 Hz ODR and 32 Hz bandwidth
};

enum MODR {
  MODR_10Hz = 0x00,   // 10 Hz ODR
  MODR_2Hz  = 0x08   ,   // 2 Hz ODR
  MODR_6Hz  = 0x10   ,   // 6 Hz ODR
  MODR_8Hz  = 0x18   ,   // 8 Hz ODR
  MODR_15Hz = 0x20   ,   // 15 Hz ODR
  MODR_20Hz = 0x28   ,   // 20 Hz ODR
  MODR_25Hz = 0x30   ,   // 25 Hz ODR
  MODR_30Hz = 0x38       // 30 Hz ODR
};



void IMU_Init(void);
void IMU_Write(unsigned char RegAddr,unsigned char Value,unsigned char SensorSelec);
void IMU_Read(unsigned char regAddr,unsigned char *readData,unsigned char SensorSelec);
u8 IMU_Write_Buf(u8 reg,u8 *pBuf,u8 len,u8 SensorSelec);
u8 IMU_Read_Buf(u8 reg,u8 *pBuf,u8 len,u8 SensorSelec);
u8 GYR_BIST(void);
void imu_task(void *param);
//void gyro_task(void *param);
//void mag_task(void *param);
//void Bmx_Convert_Data(struct Sensor *sensor);
void Gyro_Read_Data(unsigned char *x, unsigned char *y, unsigned char *z);

//void GYR_SetRange(void);
void IMU_Led_Flash(int8_t a);

int64_t SPI_Read(u8 regAddr,u8 bytes_to_read,u8 SensorSelec);
void SPI_Write(u8 regAddr,u8 data,u8 SensorSelec);

struct Sensor{
	//int64_t gyro_x[2];
	//int64_t gyro_y[2];
	//int64_t gyro_z[2];
	//int64_t accel_x[2];
	//int64_t accel_y[2];
	//int64_t accel_z[2];
	//int64_t mag_x[2];
	//int64_t mag_y[2];
	//int64_t mag_z[2];
	float gyro_reso;	// dafault use 1000dps
	float accel_reso;		// default use 4g/s
	float mag_reso; 	// default mag 16bit reso
	float gyro_x_float;
	float gyro_y_float;
	float gyro_z_float;
	float accel_x_float;
	float accel_y_float;
	float accel_z_float;
	float mag_x_float;
	float mag_y_float;
	float mag_z_float;
	int16_t cov_tmp;   		// for data convert from char -> int -> float, middle int tmp
	//void (*convert)(struct Sensor *sensor);			// function for convert data from char -> int -> float, need to be registered to Mpu_Convert_Data
	//void (*convert_asa)(struct Sensor *sensor);		// convert mag asa data
};
	
	

#endif
