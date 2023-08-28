#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
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
   	   		   
//IO方向设置
#define SDA_Imu_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	    //PB9输入模式
#define SDA_Wind_IN()  {GPIOE->MODER&=~(3<<(6*2));GPIOE->MODER|=0<<6*2;}    //PE6输入模式
#define SDA_Rom_IN()  {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=0<<4*2;}	    //PD4输入模式
#define SDA_Imu_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;}     //PB9输出模式
#define SDA_Wind_OUT() {GPIOE->MODER&=~(3<<(6*2));GPIOE->MODER|=1<<6*2;}    //PE6输出模式
#define SDA_Rom_OUT() {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=1<<4*2;}     //PD4输出模式

//IO操作函数	 
#define IIC_Imu_SCL     PBout(8)                                            
#define IIC_Imu_SDA     PBout(9)                                            
#define READ_Imu_SDA    PBin(9)                                             

#define IIC_Wind_SCL    PEout(5)                                            
#define IIC_Wind_SDA    PEout(6)                                            
#define READ_Wind_SDA   PEin(6)                                             

#define IIC_Rom_SCL     PDout(3)                                            //SCL
#define IIC_Rom_SDA     PDout(4)                                            //SDA	 
#define READ_Rom_SDA    PDin(4)                                             //输入SDA

//IIC所有操作函数
void IIC_Imu_Init(void);                
void IIC_Rom_Init(void);
void IIC_Wind_Init(void);                                                   //初始化IIC的IO口

void IIC_Imu_Start(void);				
void IIC_Wind_Start(void);
void IIC_Rom_Start(void);                                                   //发送IIC开始信号

void IIC_Imu_Stop(void);	  			
void IIC_Wind_Stop(void);	
void IIC_Rom_Stop(void);	                                                //发送IIC停止信号

void IIC_Imu_Send_Byte(u8 txd);
void IIC_Wind_Send_Byte(u8 txd);
void IIC_Rom_Send_Byte(u8 txd);			                                    //IIC发送一个字节

u8 IIC_Imu_Read_Byte(unsigned char ack);
u8 IIC_Wind_Read_Byte(unsigned char ack);
u8 IIC_Rom_Read_Byte(unsigned char ack);                                    //IIC读取一个字节

u8 IIC_Imu_Wait_Ack(void); 
u8 IIC_Wind_Wait_Ack(void);
u8 IIC_Rom_Wait_Ack(void); 				                                    //IIC等待ACK信号

void IIC_Imu_Ack(void);
void IIC_Wind_Ack(void);
void IIC_Rom_Ack(void);					                                    //IIC发送ACK信号

void IIC_Imu_NAck(void);
void IIC_Wind_NAck(void);
void IIC_Rom_NAck(void);				                                    //IIC不发送ACK信号

u8 IIC_Imu_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf);
u8 IIC_Wind_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf);
u8 IIC_Rom_ReadLenByteWithoutReg(u8 addr,u8 len,u8 *buf);

void IIC_Imu_Write_One_Byte(u8 daddr,u8 addr,u8 data);
void IIC_Imu_Write_One_Byte(u8 daddr,u8 addr,u8 data);
void IIC_Imu_Write_One_Byte(u8 daddr,u8 addr,u8 data);

u8 IIC_Imu_Read_One_Byte(u8 daddr,u8 addr);
u8 IIC_Wind_Read_One_Byte(u8 daddr,u8 addr);
u8 IIC_Rom_Read_One_Byte(u8 daddr,u8 addr);
#endif
















