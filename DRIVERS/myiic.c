#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//IIC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#define SCL_PIN GPIO_Pin_8
#define SDA_PIN GPIO_Pin_9

//初始化IIC
void IIC_Imu_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,9初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	IIC_Imu_SCL=1;
	IIC_Imu_SDA=1;
}

void IIC_Wind_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟

  //GPIOE5,6初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化

	IIC_Wind_SCL=1;
	IIC_Wind_SDA=1;
}

void IIC_Rom_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOD时钟

  //GPIOD3,4初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化

	IIC_Rom_SCL=1;
	IIC_Rom_SDA=1;
}

//产生IIC起始信号
void IIC_Imu_Start(void)
{
	SDA_Imu_OUT();     //sda线输出
	IIC_Imu_SDA=1;
	IIC_Imu_SCL=1;
	delay_us(4);
 	IIC_Imu_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_Imu_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  

void IIC_Wind_Start(void)
{
	SDA_Wind_OUT();     //sda线输出
	IIC_Wind_SDA=1;
	IIC_Wind_SCL=1;
	delay_us(4);
 	IIC_Wind_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_Wind_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  

void IIC_Rom_Start(void)
{
	SDA_Rom_OUT();     //sda线输出
	IIC_Rom_SDA=1;
	IIC_Rom_SCL=1;
	delay_us(4);
 	IIC_Rom_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_Rom_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  

//产生IIC停止信号
void IIC_Imu_Stop(void)
{
	SDA_Imu_OUT();//sda线输出
	IIC_Imu_SCL=1;
	IIC_Imu_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_Imu_SCL=1; 
	IIC_Imu_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}

void IIC_Wind_Stop(void)
{
	SDA_Wind_OUT();//sda线输出
	IIC_Wind_SCL=0;
	IIC_Wind_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_Wind_SCL=1; 
	IIC_Wind_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}

void IIC_Rom_Stop(void)
{
	SDA_Rom_OUT();//sda线输出
	IIC_Rom_SCL=0;
	IIC_Rom_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_Rom_SCL=1; 
	IIC_Rom_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Imu_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_Imu_IN();      //SDA设置为输入
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
	IIC_Imu_SCL=0;//时钟输出0 	   
	return 0;  
} 

u8 IIC_Wind_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_Wind_IN();      //SDA设置为输入
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
	IIC_Wind_SCL=0;//时钟输出0 	   
	return 0;  
} 

u8 IIC_Rom_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_Rom_IN();      //SDA设置为输入  
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
	IIC_Rom_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
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

//不产生ACK应答		    
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

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Imu_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_Imu_OUT(); 	    
    IIC_Imu_SCL=0;//拉低时钟开始数据传输
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
    IIC_Wind_SCL=0;//拉低时钟开始数据传输
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
    IIC_Rom_SCL=0;//拉低时钟开始数据传输
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

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Imu_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_Imu_IN();//SDA设置为输入
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
        IIC_Imu_NAck();//发送nACK
    else
        IIC_Imu_Ack(); //发送ACK   
    return receive;
}

u8 IIC_Wind_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_Wind_IN();//SDA设置为输入
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
        IIC_Wind_NAck();//发送nACK
    else
        IIC_Wind_Ack(); //发送ACK   
    return receive;
}

u8 IIC_Rom_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_Rom_IN();//SDA设置为输入
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
        IIC_Rom_NAck();//发送nACK
    else
        IIC_Rom_Ack(); //发送ACK   
    return receive;
}


u8 IIC_Imu_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf)
{  	IIC_Imu_Start();
	IIC_Imu_Send_Byte((addr<<1)|1); ////发送器件地址+读命令  24位ADC 三个字节 
  	if(IIC_Imu_Wait_Ack())  //等待应答
	{
		IIC_Imu_Stop();		 
		return 1;		
	}
	while(len)
	{
		if(len==1)*buf=IIC_Imu_Read_Byte(0);    //读数据,发送nACK 
		else *buf=IIC_Imu_Read_Byte(1); //读数据,发送ACK  
		len--;
		buf++;
	}    
	IIC_Imu_Stop(); //产生一个停止条件 
	return 0;	
}

u8 IIC_Wind_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf)
{  	IIC_Wind_Start();
	IIC_Wind_Send_Byte((addr<<1)|1); ////发送器件地址+读命令  24位ADC 三个字节 
  	if(IIC_Wind_Wait_Ack())  //等待应答
	{
		IIC_Wind_Stop();		 
		return 1;		
	}
	while(len)
	{
		if(len==1)*buf=IIC_Wind_Read_Byte(0);    //读数据,发送nACK 
		else *buf=IIC_Wind_Read_Byte(1); //读数据,发送ACK  
		len--;
		buf++;
	}    
	IIC_Wind_Stop(); //产生一个停止条件 
	return 0;	
}

u8 IIC_Rom_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf)
{  	IIC_Rom_Start();
	IIC_Rom_Send_Byte((addr<<1)|1); ////发送器件地址+读命令  24位ADC 三个字节 
  	if(IIC_Rom_Wait_Ack())  //等待应答
	{
		IIC_Rom_Stop();		 
		return 1;		
	}
	while(len)
	{
		if(len==1)*buf=IIC_Rom_Read_Byte(0);    //读数据,发送nACK 
		else *buf=IIC_Rom_Read_Byte(1); //读数据,发送ACK  
		len--;
		buf++;
	}    
	IIC_Rom_Stop(); //产生一个停止条件 
	return 0;	
	
}























