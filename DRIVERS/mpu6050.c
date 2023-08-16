//////////////////////////////////////////////////////////////////////////////////ÎÄ¼şËµÃ÷	 
//·ÉÄñ¿ØÖÆ°å-LJT-v1
//mpu6050.c  
//ljt
//´´½¨ÈÕÆÚ:2020/1/8
//Ê¹ÓÃµÄ×ÊÔ´ÎªI2C  PB8-SCL PB9-SDA ºÍms5611¹²ÓÃÒ»¸öI2C
/////////////////////////////////////////////////////////
//Ìá¹©µÄAPIÎª
//u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
//u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
//Ìá¹©µÄÈ«¾Ö±äÁ¿Îª£º

/////////////////////////////////////////////////////////
//°æ±¾£ºV1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////Í·ÎÄ¼ş°üº¬
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
/*µÍÍ¨ÂË²¨²ÎÊı*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];


//³õÊ¼»¯MPU6050
//·µ»ØÖµ:0,³É¹¦
//    ÆäËû,´íÎó´úÂë
u8 MPU_RegInit(void)
{ 
	IIC_Init();
	//È·¶¨×ÜÏßÉÏµÄ¹ÒÔØµÄÊÇMPU6050
	while(IIC_Read_One_Byte(MPU_ADDR,MPU_DEVICE_ID_REG)!=MPU_ADDR)    
	{
	    printf("MPU_RegInit_error_00 %d\r\n",IIC_Read_One_Byte(MPU_ADDR,MPU_DEVICE_ID_REG));
		  return 0xff;
	}
	//Ğ´0x6B ¸´Î»MPU6050¼Ä´æÆ÷
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x80)!=0)   
	{ 
	    printf("MPU_RegInit_error_01\r\n");            
		  return 0xff;
	}
	delay_ms(100);
	//Ğ´0x6B »½ĞÑMPU6050
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x00)!=0)   
	{ 
	    printf("MPU_RegInit_error_02\r\n");            
		  return 0xff;
	}
	//Ğ´0x1AÅäÖÃ¼Ä´æÆ÷  ÅäÖÃÊı×ÖµÍÍ¨ÂË²¨ acc 10Hz gyro10Hz 1Khz²ÉÑù
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_CFG_REG,0x05)!=0)   
	{
	    printf("MPU_RegInit_error_03\r\n");            
		  return 0xff;
	}
	//Ğ´0x1BÅäÖÃÍÓÂİÒÇÂúÁ¿³Ì·¶Î§ ¡À500¡ã/s  00001000
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_GYRO_CFG_REG,0x08)!=0)   
	{
	    printf("MPU_RegInit_error_04\r\n");            
		  return 0xff;
	}
	//Ğ´0x1CÅäÖÃ¼ÓËÙ¶È¼ÆÂúÁ¿³Ì·¶Î§ ¡À8g 00010000  ¶ÔÓ¦×ÅÔ­Ê¼ÖµÎª¡À32767
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_ACCEL_CFG_REG,0x10)!=0)   
	{
	    printf("MPU_RegInit_error_05\r\n");            
		  return 0xff;
	}
	//Ğ´0x37 ÖĞ¶ÏÅÔÂ·ÅäÖÃ¼Ä´æÆ÷  ÔÊĞíÖ±½Ó·ÃÎÊAUXI2C½Ó¿Ú   v1.1ÔİÊ±ÓÃ²»É
//	while(IIC_Write_One_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0x02)!=0)   Ï 
//	{
//	    printf("MPU_RegInit_error_06\r\n");            
//		  return 0xff;
//	}
	//Ğ´0x38ÅäÖÃÔÊĞíÊı¾İÊ¹ÄÜÖĞ¶Ï  ¹Ø±Õ
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_INT_EN_REG,0x00)!=0)   
	{
	    printf("MPU_RegInit_error_07\r\n");            
		  return 0xff;
	}
	//Ğ´0x6A ÅäÖÃÓÃ»§ÅäÖÃ¼Ä´æÆ÷  ¹Ø±ÕFIFO ¹Ø±ÕMPU6050Ö÷Ä£Ê½
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_USER_CTRL_REG,0x00)!=0)   
	{
	    printf("MPU_RegInit_error_08\r\n");            
		  return 0xff;
	}
	//Ğ´0x6BµçÔ´¹ÜÀí1¼Ä´æÆ÷ ÉèÖÃCLKSEL,PLL XÖáÎª²Î¿¼
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x01)!=0)   
	{ 
	    printf("MPU_RegInit_error_09\r\n");            
		  return 0xff;
	}
	//Ğ´0x6BµçÔ´¹ÜÀí2¼Ä´æÆ÷ ¼ÓËÙ¶ÈÓëÍÓÂİÒÇ¶¼¹¤×÷
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_PWR_MGMT2_REG,0x00)!=0)  
	{ 
	    printf("MPU_RegInit_error_10\r\n");            
		  return 0xff;
	}
	//Ğ´0x19 ²ÉÑùÆµÂÊ·ÖÆµÆ÷ 1000Hz
	while(IIC_Write_One_Byte(MPU_ADDR,MPU_SAMPLE_RATE_REG,0x00)!=0)   
	{ 
	    printf("MPU_RegInit_error_11\r\n");            
		  return 0xff;
	}
	
//	IIC_Write_One_Byte(MPU_ADDR,MPU_FIFO_EN_REG,0X00);	//¹Ø±ÕFIFO
//	IIC_Write_One_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0X80);	//INTÒı½ÅµÍµçÆ½ÓĞĞ§

    for (u8 i = 0; i < 3; i++)// ³õÊ¼»¯¼ÓËÙ¼ÆºÍÍÓÂİ¶ş½×µÍÍ¨ÂË²¨
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}
	return 0;
}




/*¶ÁÈ¡MPU6050¼Ä´æÆ÷µÄÖµ µÃµ½accºÍgyroµÄ²ÉÑùÖµ  -32768~32768*/
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
			//Èç¹ûÄ³¸öÖá¶¯×÷¹ı´ó  ³¬¹ıÁË1¡ã/s  ËµÃ÷Ã»ÓĞ¾²Ö¹ È«²¿ÖØÀ´
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
				//Èç¹ûÆäÖĞÓĞÒ»×éÊı¾İ¶¶¶¯¹ı´ó£¬ÔòÈ«²¿ÖØĞÂÀ´¹ı
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
	calibrate.GYRO_BIASX = gyro_sum.x/MPU6050CALINUM * GYROSAMPLETODEG;  //BIASÊÇshortÀàĞÍ±äÁ¿£¬Îª²ÉÑùÔ­Ê¼ÖµµÄÆ«²î
	calibrate.GYRO_BIASY = gyro_sum.y/MPU6050CALINUM * GYROSAMPLETODEG; 
	calibrate.GYRO_BIASZ = gyro_sum.z/MPU6050CALINUM * GYROSAMPLETODEG;
	
    W25QXX_Write((u8 *)&calibrate,FLASH_SIZE-100,20);	
}



//¶Ômpu6050½øĞĞ³õÊ¼»¯ Èç¹û´ÓFlashÖĞ¶Á³öµÄĞ£×¼ÖµÎª0ÔòÖØĞÂĞ£×¼
void mpu6050_init(void)
{
	u8 MPU_RegInit_check_num=0;
	while(MPU_RegInit())   //Èç¹û³õÊ¼»¯²»³É¹¦¾Í»á·µ»Ø·Ç0
	{
	   MPU_RegInit_check_num++;
		 delay_ms(100);              //Ã¿¹ı0.1sÔÙ´Î³¢ÊÔ³õÊ¼»¯Ò»±é
		if(MPU_RegInit_check_num>=30)  //Èç¹û³¬¹ı3sÈÔÈ»Î´³õÊ¼»¯³É¹¦£¬´ò¿ªbeep Í¬Ê±´®¿Ú´òÓ¡´íÎóĞÅÏ¢
		{ 
		     BEEP=1;
			 printf("mpu6050µØÖ·´íÎó\r\n");
		}
	}
	printf("mpu6050³õÊ¼»¯³É¹¦");

//    mpu6050cali();
    W25QXX_Read((u8*)&calibrate,FLASH_SIZE-100,20);
    printf("%d,%f,%d,%d,%d",calibrate.ISCALIBRATED,calibrate.ACC_SCALE,calibrate.GYRO_BIASX,calibrate.GYRO_BIASY,calibrate.GYRO_BIASZ);
        
    if(calibrate.ISCALIBRATED == true)//Ö®Ç°Ğ£×¼¹ı¾Í²»ÓÃĞ£×¼ÁË
    {
        printf("ÒÑĞ£×¼\r\n");
        	
    }else
    {
        printf("´ıĞ£×¼\r\n");
        mpu6050cali();
    }
	 
}


/*¶ş½×µÍÍ¨ÂË²¨*/
 void applyAxis3fLpf(lpf2pData *data, mpu6050_IMUTypeDef* in)
{
		for (u8 i = 0; i < 3; i++) 
	{
		in->accx = lpf2pApply(&data[i], in->accx);
        in->accy = lpf2pApply(&data[i], in->accy);
        in->accz = lpf2pApply(&data[i], in->accz);
	}

}

/*¶ÁÈ¡¼ÓËÙ¶È¼ÆÍÓÂİÒÇµÄÊı¾İ,±£´æÔÚ½á¹¹ÌåÖ¸ÕëÖĞ*/
void mpu6050_ReadData(mpu6050_IMUTypeDef * mpu6050_IMUTypeStructure)
{
	/*½«mpu6050²âµÃµÄadcÖµ×ª»¯ÎªÎïÀíµ¥Î»*/
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




