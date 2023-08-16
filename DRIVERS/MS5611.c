#include "ms5611.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ư�-LJT-v1
//ms5611�������� 
//ljt
//��������:2020/1/8
//ʹ�õ���ԴΪI2C  PB8-SCL PB9-SDA ��mpu6050����һ��I2C
//�汾��V1.0
//All rights reserved		
//////////////////////////////////////////////////////////////////////////////////	

static u16 Cal_C[7];												//���ڴ��PROM�е�7�����ݣ�ʵ����һ����8�� ���һ����У�飩 ��һ������Ϊreserved �ӵڶ��鿪ʼһ��6����Ч����
static u32 D1_Pres,D2_Temp;									//�������ѹ�����¶�
static float HEIGHT_COMPENSATOR;						//��Ÿ߶�У׼����ֵ
static float Pressure;											//��������ѹ
static float dT,	Temperature,	Temperature2;		//ʵ�ʺͲο��¶�֮��Ĳ���,ʵ���¶�,�м�ֵ
static double OFF,	SENS;											//ʵ���¶ȵ���,ʵ���¶�������
												
//=========================================================
//******MS561101BA����********
//=========================================================

//MS5611��ʼ���������	//�����ݶ���������������ܲ���ȥ����ע�͵��˴�while(IIC_Wait_Ack());
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

//��PROM��ȡ����У׼����
void MS561101BA_PROM_READ(void)
{
	u16 d1,d2;
	u8 i;
	for(i	=	0; i <= 6; i++)
	{
		IIC_Start();
		IIC_Send_Byte(MS561101BA_SlaveAddress);									//��IIC����д����
		IIC_Wait_Ack();
		IIC_Send_Byte((MS561101BA_PROM_RD+i*2));								//��PROMֵ������ 
		IIC_Wait_Ack();

		IIC_Start();
		IIC_Send_Byte(MS561101BA_SlaveAddress+0x01);						//��IIC���Ͷ����� �������ֽ� 
		IIC_Wait_Ack();
		d1=IIC_Read_Byte(1);
		d2=IIC_Read_Byte(0);
		IIC_Stop();
		
		delay_ms(10);
		Cal_C[i]=(d1<<8)| d2;
	}
//	��ӡPROM��ȡ����У׼���ݣ�������ݴ����Ƿ�����
//	printf("C1 =%d\n",Cal_C[1]);
//	printf("C2 =%d\n",Cal_C[2]);
//	printf("C3 =%d\n",Cal_C[3]);
//	printf("C4 =%d\n",Cal_C[4]);
//	printf("C5 =%d\n",Cal_C[5]);
//	printf("C6 =%d\n",Cal_C[6]);
}

//MS5611�ڲ�ת������
u32 MS561101BA_DO_CONVERSION(u8 command)
{
	u32 conversion = 0;
	u32 conv1, conv2, conv3; 
	
	IIC_Start();								//IICд����
	IIC_Send_Byte(MS561101BA_SlaveAddress);
	IIC_Wait_Ack();
	IIC_Send_Byte(command);						//������������ MS561101BA_P_OSR_4096����ѹ��    MS561101BA_T_OSR_4096���¶ȣ�
    IIC_Wait_Ack();
	IIC_Stop();

	delay_ms(10);		    					//OSR=4096ʱ Ҫ��10ms OSR=2048��Ҫ��5s  ����ȴ�ת����� ����������

	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress);		//��MS5611���� MS561101BA_ADC_RD����
	IIC_Wait_Ack();
	IIC_Send_Byte(MS561101BA_ADC_RD);
	IIC_Wait_Ack();
	IIC_Stop();

	IIC_Start();
	IIC_Send_Byte(MS561101BA_SlaveAddress + 1);	//IIC������ 24λADC �����ֽ� 
	IIC_Wait_Ack();
	conv1 = IIC_Read_Byte(1);
	conv2 = IIC_Read_Byte(1);
	conv3 = IIC_Read_Byte(0);
	IIC_Stop();
	conversion = (conv1<<16) + (conv2<<8) + conv3;		

	return conversion;
}


//��ȡ�����¶� 2000=20��
void MS561101BA_GetTemperature(u8 OSR_Temp)
{  
	D2_Temp	= MS561101BA_DO_CONVERSION(OSR_Temp);	
	delay_ms(10);
	dT	=	D2_Temp - (((u32)Cal_C[5])<<8);
	Temperature	=	2000.0f	+	dT	*	((u32)Cal_C[6])	/	8388608.0f;
}



//��ȡ������ѹ ��λPa ��ʽ�������ֲ�
void MS561101BA_GetPressure(u8 OSR_Pres)
{
	float Aux,OFF2,SENS2;  //�¶�У��ֵ	
	
	D1_Pres	= MS561101BA_DO_CONVERSION(OSR_Pres);
	OFF	= (u32)(Cal_C[2]<<16) + ((u32)Cal_C[4] * dT) / 128.0f;
	SENS = (u32)(Cal_C[1]<<15) + ((u32)Cal_C[3] * dT) / 256.0f;
	//�¶Ȳ��� second order temperature compensation when under 20 degrees C
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



//У׼��ѹ�ƣ��õ���ǰ�߶ȵ�ƽ��ֵ���Ա��ں�����߶ȵ�ʱ�������ʹ�����ʱ�ĸ߶�Ϊ0
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
		height	= 44330.f * (powf((1015.7f / Pressure), 0.190295f) - 1.0f);				//���鹫ʽ
		height_sum	=	height_sum+height;
		delay_ms(10);
	}
	HEIGHT_COMPENSATOR = height_sum	/ MS5611CALNUM;
}



//MS5611��ʼ���ӿ�
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
/*�޷�ƽ���˲���*/
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


//��ȡ��ѹ�Ƹ߶�
float ms5611_altitude(void)
{
    static float raw_altitude;	
//    static float altitude_filterout;
	MS561101BA_GetTemperature(MS561101BA_T_OSR);
	MS561101BA_GetPressure(MS561101BA_P_OSR);
	raw_altitude = 44330.f * (powf((1015.7f / Pressure), 0.190295f) - 1.0f) - HEIGHT_COMPENSATOR;			//ͨ����ѹֵ��Pa����߶ȣ�m�� ���鹫ʽ
//	heightFilter(&raw_altitude,&altitude_filterout);
	
    return raw_altitude;
}



		 


