#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#define SCL_PIN GPIO_Pin_8
#define SDA_PIN GPIO_Pin_9

//��ʼ��IIC
void IIC_Imu_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,9��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	IIC_Imu_SCL=1;
	IIC_Imu_SDA=1;
}

void IIC_Wind_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��

  //GPIOE5,6��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��

	IIC_Wind_SCL=1;
	IIC_Wind_SDA=1;
}

void IIC_Rom_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIODʱ��

  //GPIOD3,4��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��

	IIC_Rom_SCL=1;
	IIC_Rom_SDA=1;
}

//����IIC��ʼ�ź�
void IIC_Imu_Start(void)
{
	SDA_Imu_OUT();     //sda�����
	IIC_Imu_SDA=1;
	IIC_Imu_SCL=1;
	delay_us(4);
 	IIC_Imu_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_Imu_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  

void IIC_Wind_Start(void)
{
	SDA_Wind_OUT();     //sda�����
	IIC_Wind_SDA=1;
	IIC_Wind_SCL=1;
	delay_us(4);
 	IIC_Wind_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_Wind_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  

void IIC_Rom_Start(void)
{
	SDA_Rom_OUT();     //sda�����
	IIC_Rom_SDA=1;
	IIC_Rom_SCL=1;
	delay_us(4);
 	IIC_Rom_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_Rom_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  

//����IICֹͣ�ź�
void IIC_Imu_Stop(void)
{
	SDA_Imu_OUT();//sda�����
	IIC_Imu_SCL=1;
	IIC_Imu_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_Imu_SCL=1; 
	IIC_Imu_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}

void IIC_Wind_Stop(void)
{
	SDA_Wind_OUT();//sda�����
	IIC_Wind_SCL=0;
	IIC_Wind_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_Wind_SCL=1; 
	IIC_Wind_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}

void IIC_Rom_Stop(void)
{
	SDA_Rom_OUT();//sda�����
	IIC_Rom_SCL=0;
	IIC_Rom_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_Rom_SCL=1; 
	IIC_Rom_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Imu_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_Imu_IN();      //SDA����Ϊ����
	IIC_Imu_SDA=1;delay_us(1);
	IIC_Imu_SCL=1;delay_us(1);
	while(READ_Imu_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Imu_Stop();
			return 1;
		}
	}
	IIC_Imu_SCL=0;//ʱ�����0 	   
	return 0;  
} 

u8 IIC_Wind_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_Wind_IN();      //SDA����Ϊ����
	IIC_Wind_SDA=1;delay_us(1);	
	IIC_Wind_SCL=1;delay_us(1);	
	while(READ_Wind_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Wind_Stop();
			return 1;
		}
	}
	IIC_Wind_SCL=0;//ʱ�����0 	   
	return 0;  
} 

u8 IIC_Rom_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_Rom_IN();      //SDA����Ϊ����  
	IIC_Rom_SDA=1;delay_us(1);	   
	IIC_Rom_SCL=1;delay_us(1);	 
	while(READ_Rom_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Rom_Stop();
			return 1;
		}
	}
	IIC_Rom_SCL=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
void IIC_Imu_Ack(void)
{
	//IIC_Imu_SCL=0;
	SDA_Imu_OUT();
	IIC_Imu_SDA=0;
	delay_us(2);
	IIC_Imu_SCL=1;
	delay_us(2);
	IIC_Imu_SCL=0;
}

void IIC_Wind_Ack(void)
{
	IIC_Wind_SCL=0;
	SDA_Wind_OUT();
	IIC_Wind_SDA=0;
	delay_us(2);
	IIC_Wind_SCL=1;
	delay_us(2);
	IIC_Wind_SCL=0;
}

void IIC_Rom_Ack(void)
{
	IIC_Rom_SCL=0;
	SDA_Rom_OUT();
	IIC_Rom_SDA=0;
	delay_us(2);
	IIC_Rom_SCL=1;
	delay_us(2);
	IIC_Rom_SCL=0;
}

//������ACKӦ��		    
void IIC_Imu_NAck(void)
{
	//IIC_Imu_SCL=0;
	SDA_Imu_OUT();
	IIC_Imu_SDA=1;
	delay_us(2);
	IIC_Imu_SCL=1;
	delay_us(2);
	IIC_Imu_SCL=0;
}		

void IIC_Wind_NAck(void)
{
	IIC_Wind_SCL=0;
	SDA_Wind_OUT();
	IIC_Wind_SDA=1;
	delay_us(2);
	IIC_Wind_SCL=1;
	delay_us(2);
	IIC_Wind_SCL=0;
}	

void IIC_Rom_NAck(void)
{
	IIC_Rom_SCL=0;
	SDA_Rom_OUT();
	IIC_Rom_SDA=1;
	delay_us(2);
	IIC_Rom_SCL=1;
	delay_us(2);
	IIC_Rom_SCL=0;
}

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Imu_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_Imu_OUT(); 	    
    IIC_Imu_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
		IIC_Imu_SDA=(txd&0x80)>>7;
		txd<<=1; 	  
		delay_us(2);
		IIC_Imu_SCL=1;
		delay_us(2); 
		IIC_Imu_SCL=0;	
		delay_us(2);
    }	 
}

void IIC_Wind_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_Wind_OUT(); 	    
    IIC_Wind_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_Wind_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);    
		IIC_Wind_SCL=1;
		delay_us(2); 
		IIC_Wind_SCL=0;	
		delay_us(2);
    }	 
} 

void IIC_Rom_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_Rom_OUT(); 	    
    IIC_Rom_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_Rom_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);    
		IIC_Rom_SCL=1;
		delay_us(2); 
		IIC_Rom_SCL=0;	
		delay_us(2);
    }
} 

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Imu_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_Imu_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_Imu_SCL=0; 
        delay_us(2);
	    IIC_Imu_SCL=1;
        receive<<=1;
        if(READ_Imu_SDA)receive++;
		delay_us(1); 
    }					 
    if (!ack)
        IIC_Imu_NAck();//����nACK
    else
        IIC_Imu_Ack(); //����ACK   
    return receive;
}

u8 IIC_Wind_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_Wind_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_Wind_SCL=0; 
        delay_us(2);
	    IIC_Wind_SCL=1;
        receive<<=1;
        if(READ_Wind_SDA)receive++;
		delay_us(1); 
    }					 
    if (!ack)
        IIC_Wind_NAck();//����nACK
    else
        IIC_Wind_Ack(); //����ACK   
    return receive;
}

u8 IIC_Rom_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_Rom_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_Rom_SCL=0; 
        delay_us(2);
	    IIC_Rom_SCL=1;
        receive<<=1;
        if(READ_Rom_SDA)receive++;   
		delay_us(1); 
    }
    if (!ack)
        IIC_Rom_NAck();//����nACK
    else
        IIC_Rom_Ack(); //����ACK   
    return receive;
}


u8 IIC_Imu_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf)
{  	IIC_Imu_Start();
	IIC_Imu_Send_Byte((addr<<1)|1); ////����������ַ+������  24λADC �����ֽ� 
  	if(IIC_Imu_Wait_Ack())  //�ȴ�Ӧ��
	{
		IIC_Imu_Stop();		 
		return 1;		
	}
	while(len)
	{
		if(len==1)*buf=IIC_Imu_Read_Byte(0);    //������,����nACK 
		else *buf=IIC_Imu_Read_Byte(1); //������,����ACK  
		len--;
		buf++;
	}    
	IIC_Imu_Stop(); //����һ��ֹͣ���� 
	return 0;	
}

u8 IIC_Wind_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf)
{  	IIC_Wind_Start();
	IIC_Wind_Send_Byte((addr<<1)|1); ////����������ַ+������  24λADC �����ֽ� 
  	if(IIC_Wind_Wait_Ack())  //�ȴ�Ӧ��
	{
		IIC_Wind_Stop();		 
		return 1;		
	}
	while(len)
	{
		if(len==1)*buf=IIC_Wind_Read_Byte(0);    //������,����nACK 
		else *buf=IIC_Wind_Read_Byte(1); //������,����ACK  
		len--;
		buf++;
	}    
	IIC_Wind_Stop(); //����һ��ֹͣ���� 
	return 0;	
}

u8 IIC_Rom_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf)
{  	IIC_Rom_Start();
	IIC_Rom_Send_Byte((addr<<1)|1); ////����������ַ+������  24λADC �����ֽ� 
  	if(IIC_Rom_Wait_Ack())  //�ȴ�Ӧ��
	{
		IIC_Rom_Stop();		 
		return 1;		
	}
	while(len)
	{
		if(len==1)*buf=IIC_Rom_Read_Byte(0);    //������,����nACK 
		else *buf=IIC_Rom_Read_Byte(1); //������,����ACK  
		len--;
		buf++;
	}    
	IIC_Rom_Stop(); //����һ��ֹͣ���� 
	return 0;	
	
}























