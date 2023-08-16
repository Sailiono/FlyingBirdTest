#include "ms5611.h"
//////////////////////////////////////////////////////////////////////////////////	 
//飞鸟控制板-LJT-v1
//ms5611驱动代码 
//ljt
//创建日期:2020/1/8
//使用的资源为I2C  PB8-SCL PB9-SDA 和mpu6050共用一个I2C
//版本：V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////	

static u16 Cal_C[7];												//用于存放PROM中的7组数据（实际上一共有8组 最后一组是校验） 第一组数据为reserved 从第二组开始一共6组有效数据
static u32 D1_Pres,D2_Temp;									//存放数字压力和温度
static float HEIGHT_COMPENSATOR;						//存放高度校准补偿值
static float Pressure;											//测量大气压
static float dT,	Temperature,	Temperature2;		//实际和参考温度之间的差异,实际温度,中间值
static double OFF,	SENS;											//实际温度抵消,实际温度灵敏度
												
//=========================================================
//******MS561101BA程序********
//=========================================================

//MS5611初始化必须操作	//如数据读不出来，或程序跑不下去，请注释掉此处while(IIC_Wait_Ack());
void MS561101BA_RESET(void)		
{	
	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress);
	IIC_Wait_Ack();
	delay_us(100);
	IIC_Send_Byte(MS561101BA_RST);
	IIC_Wait_Ack();
	delay_us(100);
	IIC_Stop();
}

//从PROM读取出厂校准数据
void MS561101BA_PROM_READ(void)
{
	u16 d1,d2;
	u8 i;
	for(i	=	0; i <= 6; i++)
	{
		IIC_Start();
		IIC_Send_Byte(MS561101BA_SlaveAddress);									//向IIC发送写命令
		IIC_Wait_Ack();
		IIC_Send_Byte((MS561101BA_PROM_RD+i*2));								//读PROM值的命令 
		IIC_Wait_Ack();

		IIC_Start();
		IIC_Send_Byte(MS561101BA_SlaveAddress+0x01);						//向IIC发送读命令 读两个字节 
		IIC_Wait_Ack();
		d1=IIC_Read_Byte(1);
		d2=IIC_Read_Byte(0);
		IIC_Stop();
		
		delay_ms(10);
		Cal_C[i]=(d1<<8)| d2;
	}
//	打印PROM读取出厂校准数据，检测数据传输是否正常
//	printf("C1 =%d\n",Cal_C[1]);
//	printf("C2 =%d\n",Cal_C[2]);
//	printf("C3 =%d\n",Cal_C[3]);
//	printf("C4 =%d\n",Cal_C[4]);
//	printf("C5 =%d\n",Cal_C[5]);
//	printf("C6 =%d\n",Cal_C[6]);
}

//MS5611内部转换代码
u32 MS561101BA_DO_CONVERSION(u8 command)
{
	u32 conversion = 0;
	u32 conv1, conv2, conv3; 
	
	IIC_Start();								//IIC写命令
	IIC_Send_Byte(MS561101BA_SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(command);						//根据输入来定 MS561101BA_P_OSR_4096（气压）    MS561101BA_T_OSR_4096（温度）
    IIC_Wait_Ack();
	IIC_Stop();

	delay_ms(10);		    					//OSR=4096时 要等10ms OSR=2048则要等5s  必须等待转换完成 否则会读不到

	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress);		//向MS5611发送 MS561101BA_ADC_RD命令
	IIC_Wait_Ack();
	IIC_Send_Byte(MS561101BA_ADC_RD);
	IIC_Wait_Ack();
	IIC_Stop();

	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress + 1);	//IIC读命令 24位ADC 三个字节 
	IIC_Wait_Ack();
	conv1 = IIC_Read_Byte(1);
	conv2 = IIC_Read_Byte(1);
	conv3 = IIC_Read_Byte(0);
	IIC_Stop();
	conversion = (conv1<<16) + (conv2<<8) + conv3;		

	return conversion;
}


//读取数字温度 2000=20度
void MS561101BA_GetTemperature(u8 OSR_Temp)
{  
	D2_Temp	= MS561101BA_DO_CONVERSION(OSR_Temp);	
	delay_ms(10);
	dT	=	D2_Temp - (((u32)Cal_C[5])<<8);
	Temperature	=	2000.0f	+	dT	*	((u32)Cal_C[6])	/	8388608.0f;
}



//读取数字气压 单位Pa 公式见数据手册
void MS561101BA_GetPressure(u8 OSR_Pres)
{
	float Aux,OFF2,SENS2;  //温度校验值	
	
	D1_Pres	= MS561101BA_DO_CONVERSION(OSR_Pres);
	OFF	= (u32)(Cal_C[2]<<16) + ((u32)Cal_C[4] * dT) / 128.0f;
	SENS = (u32)(Cal_C[1]<<15) + ((u32)Cal_C[3] * dT) / 256.0f;
	//温度补偿 second order temperature compensation when under 20 degrees C
	if(Temperature < 2000)
	{
		Temperature2 = (dT * dT) / 0x80000000;
		Aux = (Temperature - 2000) * (Temperature -	2000);
		OFF2 = 2.5f	* Aux;
		SENS2 = 1.25f *	Aux;
	}
	else  //(Temperature > 2000)
	{
		Temperature2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	Temperature = Temperature - Temperature2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;	
	Pressure = (D1_Pres	* SENS / 2097152.0f	- OFF) / 32768.0f;						
}



//校准气压计，得到当前高度的平均值，以便于后面求高度的时候减掉，使得起飞时的高度为0
void MS5611_cali(void)
{
	u8 i;
	float height = 0;
	float height_sum = 0;
	for(i =	0; i < MS5611CALNUM; i++)
	{ 
        MS561101BA_GetTemperature(MS561101BA_T_OSR);
        MS561101BA_GetPressure(MS561101BA_P_OSR); 
//      printf("p=%f\r\n",Pressure);
		height	= 44330.f * (powf((1015.7f / Pressure), 0.190295f) - 1.0f);				//经验公式
		height_sum	=	height_sum+height;
		delay_ms(10);
	}
	HEIGHT_COMPENSATOR = height_sum	/ MS5611CALNUM;
}



//MS5611初始化接口
void MS561101BA_Init(void)
{
	MS561101BA_RESET();
	delay_ms(100);
	MS561101BA_PROM_READ();
	delay_ms(100);
	MS5611_cali();
} 

#define FILTER_NUM	5
#define FILTER_A	5.0f
/*限幅平均滤波法*/
void heightFilter(float* in, float* out)
{	
	static u8 i=0;
	static float filter_buf[FILTER_NUM]={0.0};
	double filter_sum=0.0;
	u8 cnt=0;	
	float deta;		
	
	if(filter_buf[i] == 0.0f)
	{
		filter_buf[i]=*in;
		*out=*in;
		if(++i>=FILTER_NUM)	i=0;
	} else 
	{
		if(i) deta=*in-filter_buf[i-1];
		else deta=*in-filter_buf[FILTER_NUM-1];
		
		if(fabs(deta)<FILTER_A)
		{
			filter_buf[i]=*in;
			if(++i>=FILTER_NUM)	i=0;
		}
		for(cnt=0;cnt<FILTER_NUM;cnt++)
		{
			filter_sum+=filter_buf[cnt];
		}
		*out=filter_sum /FILTER_NUM;
	}
}


//获取气压计高度
float ms5611_altitude(void)
{
    static float raw_altitude;	
//    static float altitude_filterout;
	MS561101BA_GetTemperature(MS561101BA_T_OSR);
	MS561101BA_GetPressure(MS561101BA_P_OSR);
	raw_altitude = 44330.f * (powf((1015.7f / Pressure), 0.190295f) - 1.0f) - HEIGHT_COMPENSATOR;			//通过气压值（Pa）求高度（m） 经验公式
//	heightFilter(&raw_altitude,&altitude_filterout);
	
    return raw_altitude;
}



		 


