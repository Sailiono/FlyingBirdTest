//////////////////////////////////////////////////////////////////////////////////文件说明	 
//飞鸟控制板-LJT-v1
//mpu6050.c  
//ljt
//创建日期:2020/1/8
//使用的资源为I2C  PB8-SCL PB9-SDA 和ms5611共用一个I2C
/////////////////////////////////////////////////////////
//提供的API为
//u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
//u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
//提供的全局变量为：

/////////////////////////////////////////////////////////
//版本：V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////头文件包含
#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"   
#include "beep.h"
//#include "timer.h"
#include "main.h"
#include "watchdog.h"
#include "w25qxx.h" 

static mpu6050_IMURAWTypeDef IMURawdata_;
CalibrateIMU_S calibrate;
/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];


//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_RegInit(void)
{ 
	IIC_Init();
	//确定总线上的挂载的是MPU6050
	while(IIC_Read_One_Byte(MPU_ADDR,MPU_DEVICE_ID_REG)!=MPU_ADDR)    
	{
	    printf("MPU_RegInit_error_00 %d\r\n",IIC_Read_One_Byte(MPU_ADDR,MPU_DEVICE_ID_REG));
		  return 0xff;
	}
	//写0x6B 复位MPU6050寄存器
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x80)!=0)   
	{ 
	    printf("MPU_RegInit_error_01\r\n");            
		  return 0xff;
	}
	delay_ms(100);
	//写0x6B 唤醒MPU6050
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x00)!=0)   
	{ 
	    printf("MPU_RegInit_error_02\r\n");            
		  return 0xff;
	}
	//写0x1A配置寄存器  配置数字低通滤波 acc 10Hz gyro10Hz 1Khz采样
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_CFG_REG,0x05)!=0)   
	{
	    printf("MPU_RegInit_error_03\r\n");            
		  return 0xff;
	}
	//写0x1B配置陀螺仪满量程范围 ±500°/s  00001000
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_GYRO_CFG_REG,0x08)!=0)   
	{
	    printf("MPU_RegInit_error_04\r\n");            
		  return 0xff;
	}
	//写0x1C配置加速度计满量程范围 ±8g 00010000  对应着原始值为±32767
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_ACCEL_CFG_REG,0x10)!=0)   
	{
	    printf("MPU_RegInit_error_05\r\n");            
		  return 0xff;
	}
	//写0x37 中断旁路配置寄存器  允许直接访问AUXI2C接口   v1.1暂时用不�
//	while(IIC_Write_One_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0x02)!=0)   � 
//	{
//	    printf("MPU_RegInit_error_06\r\n");            
//		  return 0xff;
//	}
	//写0x38配置允许数据使能中断  关闭
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_INT_EN_REG,0x00)!=0)   
	{
	    printf("MPU_RegInit_error_07\r\n");            
		  return 0xff;
	}
	//写0x6A 配置用户配置寄存器  关闭FIFO 关闭MPU6050主模式
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_USER_CTRL_REG,0x00)!=0)   
	{
	    printf("MPU_RegInit_error_08\r\n");            
		  return 0xff;
	}
	//写0x6B电源管理1寄存器 设置CLKSEL,PLL X轴为参考
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x01)!=0)   
	{ 
	    printf("MPU_RegInit_error_09\r\n");            
		  return 0xff;
	}
	//写0x6B电源管理2寄存器 加速度与陀螺仪都工作
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT2_REG,0x00)!=0)  
	{ 
	    printf("MPU_RegInit_error_10\r\n");            
		  return 0xff;
	}
	//写0x19 采样频率分频器 1000Hz
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_SAMPLE_RATE_REG,0x00)!=0)   
	{ 
	    printf("MPU_RegInit_error_11\r\n");            
		  return 0xff;
	}
	
//	IIC_Write_One_Byte(MPU_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
//	IIC_Write_One_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效

    for (u8 i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}
	return 0;
}




/*读取MPU6050寄存器的值 得到acc和gyro的采样值  -32768~32768*/
u8 mpu6050_RawData(void)
{
    mpu6050_IMURAWTypeDef IMURawDatatemp;
	u8 buf[14],res;
	res=IIC_Read_Len_Byte(MPU_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
  if(res==0)
	{
		IMURawDatatemp.accx = ((u16)buf[0]<<8)|buf[1];  
		IMURawDatatemp.accy = ((u16)buf[2]<<8)|buf[3];  
		IMURawDatatemp.accz = ((u16)buf[4]<<8)|buf[5];
		IMURawDatatemp.gyrox = ((u16)buf[8]<<8)|buf[9];  
		IMURawDatatemp.gyroy = ((u16)buf[10]<<8)|buf[11];  
		IMURawDatatemp.gyroz = ((u16)buf[12]<<8)|buf[13];
        
        IMURawdata_.gyrox = IMURawDatatemp.gyrox;
        IMURawdata_.gyroy = -IMURawDatatemp.gyroy;
        IMURawdata_.gyroz = -IMURawDatatemp.gyroz;
        IMURawdata_.accx = IMURawDatatemp.accx;
        IMURawdata_.accy = -IMURawDatatemp.accy;
        IMURawdata_.accz = -IMURawDatatemp.accz;
        
        
        //printf("%d,%d,%d=\r\n",IMURawdata_.gyrox,IMURawdata_.gyroy,IMURawdata_.gyroz);
	} 	
    return res;
}

void MPU_Get_Gyroscope(short*getgyrox,short*getgyroy,short* getgyroz)
{
    *getgyrox = IMURawdata_.gyrox - calibrate.GYRO_BIASX;
    *getgyroy = IMURawdata_.gyroy - calibrate.GYRO_BIASY;
    *getgyroz = IMURawdata_.gyroz - calibrate.GYRO_BIASZ;
//    printf("%d,%d,%d\r\n",*getgyrox,*getgyroy,*getgyroz);
}
void MPU_Get_Accelerometer(short*getaccx,short*getaccy, short*getaccz)
{
    *getaccx = IMURawdata_.accx / calibrate.ACC_SCALE;
    *getaccy = IMURawdata_.accy / calibrate.ACC_SCALE;
    *getaccz = IMURawdata_.accz / calibrate.ACC_SCALE;
//    printf("%d,%d,%d\r\n",*getaccx,*getaccy,*getaccz);
}



void mpu6050cali(void)
{
	Axis3f gyro_sum;
	float acc_sum;
	u16 i_sum;
	while(i_sum<MPU6050CALINUM)
	{
        printf("cc");
		if(mpu6050_RawData()==0)
		{
			//如果某个轴动作过大  超过了1°/s  说明没有静止 全部重来
			if(	abs(IMURawdata_.gyrox<GYROSAMPLETODEG)&&\
					(abs(IMURawdata_.gyroy)<GYROSAMPLETODEG)&&\
					(abs(IMURawdata_.gyroz)<GYROSAMPLETODEG)	)   
			{
				acc_sum=acc_sum+sqrtf(powf((float)IMURawdata_.accx/ACCSAMPLETODEG,2)+\
															powf((float)IMURawdata_.accy/ACCSAMPLETODEG,2)+\
															powf((float)IMURawdata_.accz/ACCSAMPLETODEG,2));
				gyro_sum.x=IMURawdata_.gyrox/GYROSAMPLETODEG+gyro_sum.x;
				gyro_sum.y=IMURawdata_.gyroy/GYROSAMPLETODEG+gyro_sum.y;
				gyro_sum.z=IMURawdata_.gyroz/GYROSAMPLETODEG+gyro_sum.z;
				
				i_sum++;  
			}
			else
			{
				delay_ms(100);
                printf("dd");
				//如果其中有一组数据抖动过大，则全部重新来过
				gyro_sum.x=0;
				gyro_sum.y=0; 
				gyro_sum.z=0;
				i_sum=0;
				acc_sum=0;  
			}
		}
	}
	
    calibrate.ISCALIBRATED = true;
    calibrate.ACC_SCALE = acc_sum/MPU6050CALINUM;
	calibrate.GYRO_BIASX = gyro_sum.x/MPU6050CALINUM * GYROSAMPLETODEG;  //BIAS是short类型变量，为采样原始值的偏差
	calibrate.GYRO_BIASY = gyro_sum.y/MPU6050CALINUM * GYROSAMPLETODEG; 
	calibrate.GYRO_BIASZ = gyro_sum.z/MPU6050CALINUM * GYROSAMPLETODEG;
	
    W25QXX_Write((u8 *)&calibrate,FLASH_SIZE-100,20);	
}



//对mpu6050进行初始化 如果从Flash中读出的校准值为0则重新校准
void mpu6050_init(void)
{
	u8 MPU_RegInit_check_num=0;
	while(MPU_RegInit())   //如果初始化不成功就会返回非0
	{
	   MPU_RegInit_check_num++;
		 delay_ms(100);              //每过0.1s再次尝试初始化一遍
		if(MPU_RegInit_check_num>=30)  //如果超过3s仍然未初始化成功，打开beep 同时串口打印错误信息
		{ 
		     BEEP=1;
			 printf("mpu6050地址错误\r\n");
		}
	}
	printf("mpu6050初始化成功");

//    mpu6050cali();
    W25QXX_Read((u8*)&calibrate,FLASH_SIZE-100,20);
    printf("%d,%f,%d,%d,%d",calibrate.ISCALIBRATED,calibrate.ACC_SCALE,calibrate.GYRO_BIASX,calibrate.GYRO_BIASY,calibrate.GYRO_BIASZ);
        
    if(calibrate.ISCALIBRATED == true)//之前校准过就不用校准了
    {
        printf("已校准\r\n");
        	
    }else
    {
        printf("待校准\r\n");
        mpu6050cali();
    }
	 
}


/*二阶低通滤波*/
 void applyAxis3fLpf(lpf2pData *data, mpu6050_IMUTypeDef* in)
{
		for (u8 i = 0; i < 3; i++) 
	{
		in->accx = lpf2pApply(&data[i], in->accx);
        in->accy = lpf2pApply(&data[i], in->accy);
        in->accz = lpf2pApply(&data[i], in->accz);
	}

}

/*读取加速度计陀螺仪的数据,保存在结构体指针中*/
void mpu6050_ReadData(mpu6050_IMUTypeDef * mpu6050_IMUTypeStructure)
{
	/*将mpu6050测得的adc值转化为物理单位*/
	if(mpu6050_RawData()==0)
	{
		mpu6050_IMUTypeStructure->accx = (float)(IMURawdata_.accx) / ACCSAMPLETODEG / calibrate.ACC_SCALE;
		mpu6050_IMUTypeStructure->accy = (float)(IMURawdata_.accy) / ACCSAMPLETODEG / calibrate.ACC_SCALE;
		mpu6050_IMUTypeStructure->accz = (float)(IMURawdata_.accz) / ACCSAMPLETODEG / calibrate.ACC_SCALE;
        //applyAxis3fLpf(accLpf, mpu6050_IMUTypeStructure);
        
		mpu6050_IMUTypeStructure->gyrox = (float)(IMURawdata_.gyrox - calibrate.GYRO_BIASX)/GYROSAMPLETODEG ;
		mpu6050_IMUTypeStructure->gyroy = (float)(IMURawdata_.gyroy - calibrate.GYRO_BIASY)/GYROSAMPLETODEG ;
		mpu6050_IMUTypeStructure->gyroz = (float)(IMURawdata_.gyroz - calibrate.GYRO_BIASZ)/GYROSAMPLETODEG;
	}
	 
}




