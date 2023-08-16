//////////////////////////////////////////////////////////////////////////////////�ļ�˵��	 
//������ư�-LJT-v1
//mpu6050.c  
//ljt
//��������:2020/1/8
//ʹ�õ���ԴΪI2C  PB8-SCL PB9-SDA ��ms5611����һ��I2C
/////////////////////////////////////////////////////////
//�ṩ��APIΪ
//u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
//u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
//�ṩ��ȫ�ֱ���Ϊ��

/////////////////////////////////////////////////////////
//�汾��V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////ͷ�ļ�����
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
/*��ͨ�˲�����*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];


//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_RegInit(void)
{ 
	IIC_Init();
	//ȷ�������ϵĹ��ص���MPU6050
	while(IIC_Read_One_Byte(MPU_ADDR,MPU_DEVICE_ID_REG)!=MPU_ADDR)    
	{
	    printf("MPU_RegInit_error_00 %d\r\n",IIC_Read_One_Byte(MPU_ADDR,MPU_DEVICE_ID_REG));
		  return 0xff;
	}
	//д0x6B ��λMPU6050�Ĵ���
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x80)!=0)   
	{ 
	    printf("MPU_RegInit_error_01\r\n");            
		  return 0xff;
	}
	delay_ms(100);
	//д0x6B ����MPU6050
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x00)!=0)   
	{ 
	    printf("MPU_RegInit_error_02\r\n");            
		  return 0xff;
	}
	//д0x1A���üĴ���  �������ֵ�ͨ�˲� acc 10Hz gyro10Hz 1Khz����
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_CFG_REG,0x05)!=0)   
	{
	    printf("MPU_RegInit_error_03\r\n");            
		  return 0xff;
	}
	//д0x1B���������������̷�Χ ��500��/s  00001000
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_GYRO_CFG_REG,0x08)!=0)   
	{
	    printf("MPU_RegInit_error_04\r\n");            
		  return 0xff;
	}
	//д0x1C���ü��ٶȼ������̷�Χ ��8g 00010000  ��Ӧ��ԭʼֵΪ��32767
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_ACCEL_CFG_REG,0x10)!=0)   
	{
	    printf("MPU_RegInit_error_05\r\n");            
		  return 0xff;
	}
	//д0x37 �ж���·���üĴ���  ����ֱ�ӷ���AUXI2C�ӿ�   v1.1��ʱ�ò��
//	while(IIC_Write_One_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0x02)!=0)   � 
//	{
//	    printf("MPU_RegInit_error_06\r\n");            
//		  return 0xff;
//	}
	//д0x38������������ʹ���ж�  �ر�
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_INT_EN_REG,0x00)!=0)   
	{
	    printf("MPU_RegInit_error_07\r\n");            
		  return 0xff;
	}
	//д0x6A �����û����üĴ���  �ر�FIFO �ر�MPU6050��ģʽ
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_USER_CTRL_REG,0x00)!=0)   
	{
	    printf("MPU_RegInit_error_08\r\n");            
		  return 0xff;
	}
	//д0x6B��Դ����1�Ĵ��� ����CLKSEL,PLL X��Ϊ�ο�
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x01)!=0)   
	{ 
	    printf("MPU_RegInit_error_09\r\n");            
		  return 0xff;
	}
	//д0x6B��Դ����2�Ĵ��� ���ٶ��������Ƕ�����
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT2_REG,0x00)!=0)  
	{ 
	    printf("MPU_RegInit_error_10\r\n");            
		  return 0xff;
	}
	//д0x19 ����Ƶ�ʷ�Ƶ�� 1000Hz
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_SAMPLE_RATE_REG,0x00)!=0)   
	{ 
	    printf("MPU_RegInit_error_11\r\n");            
		  return 0xff;
	}
	
//	IIC_Write_One_Byte(MPU_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
//	IIC_Write_One_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч

    for (u8 i = 0; i < 3; i++)// ��ʼ�����ټƺ����ݶ��׵�ͨ�˲�
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}
	return 0;
}




/*��ȡMPU6050�Ĵ�����ֵ �õ�acc��gyro�Ĳ���ֵ  -32768~32768*/
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
			//���ĳ���ᶯ������  ������1��/s  ˵��û�о�ֹ ȫ������
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
				//���������һ�����ݶ���������ȫ����������
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
	calibrate.GYRO_BIASX = gyro_sum.x/MPU6050CALINUM * GYROSAMPLETODEG;  //BIAS��short���ͱ�����Ϊ����ԭʼֵ��ƫ��
	calibrate.GYRO_BIASY = gyro_sum.y/MPU6050CALINUM * GYROSAMPLETODEG; 
	calibrate.GYRO_BIASZ = gyro_sum.z/MPU6050CALINUM * GYROSAMPLETODEG;
	
    W25QXX_Write((u8 *)&calibrate,FLASH_SIZE-100,20);	
}



//��mpu6050���г�ʼ�� �����Flash�ж�����У׼ֵΪ0������У׼
void mpu6050_init(void)
{
	u8 MPU_RegInit_check_num=0;
	while(MPU_RegInit())   //�����ʼ�����ɹ��ͻ᷵�ط�0
	{
	   MPU_RegInit_check_num++;
		 delay_ms(100);              //ÿ��0.1s�ٴγ��Գ�ʼ��һ��
		if(MPU_RegInit_check_num>=30)  //�������3s��Ȼδ��ʼ���ɹ�����beep ͬʱ���ڴ�ӡ������Ϣ
		{ 
		     BEEP=1;
			 printf("mpu6050��ַ����\r\n");
		}
	}
	printf("mpu6050��ʼ���ɹ�");

//    mpu6050cali();
    W25QXX_Read((u8*)&calibrate,FLASH_SIZE-100,20);
    printf("%d,%f,%d,%d,%d",calibrate.ISCALIBRATED,calibrate.ACC_SCALE,calibrate.GYRO_BIASX,calibrate.GYRO_BIASY,calibrate.GYRO_BIASZ);
        
    if(calibrate.ISCALIBRATED == true)//֮ǰУ׼���Ͳ���У׼��
    {
        printf("��У׼\r\n");
        	
    }else
    {
        printf("��У׼\r\n");
        mpu6050cali();
    }
	 
}


/*���׵�ͨ�˲�*/
 void applyAxis3fLpf(lpf2pData *data, mpu6050_IMUTypeDef* in)
{
		for (u8 i = 0; i < 3; i++) 
	{
		in->accx = lpf2pApply(&data[i], in->accx);
        in->accy = lpf2pApply(&data[i], in->accy);
        in->accz = lpf2pApply(&data[i], in->accz);
	}

}

/*��ȡ���ٶȼ������ǵ�����,�����ڽṹ��ָ����*/
void mpu6050_ReadData(mpu6050_IMUTypeDef * mpu6050_IMUTypeStructure)
{
	/*��mpu6050��õ�adcֵת��Ϊ����λ*/
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




