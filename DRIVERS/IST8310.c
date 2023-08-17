#include "IST8310.h"


u8 IST8310_Init(void)
{ 
	IIC_Init();
	//确定总线上的挂载的是IST8310
	while(IIC_Read_One_Byte(IST8310_ADDR,IST8310_WAI)!=WHOAMI)    
	{
	    printf("IST8310_error_00\r\n");
		return 0xff;
	}
    printf("aa");
     //复位
	while(IIC_Write_One_Byte(IST8310_ADDR,IST8310_CNTL2,0x09)!=0)   
	{ 
	    printf("IST8310_error_01\r\n");            
		  return 0xff;
	}
    delay_ms(10);
    //写IST8310_CNTL1
	while(IIC_Write_One_Byte(IST8310_ADDR,IST8310_CNTL1,0x01)!=0)   
	{ 
	    printf("IST8310_error_02\r\n");            
		  return 0xff;
	}
     delay_ms(10);
    
    //写IST8310_CNTL2
	while(IIC_Write_One_Byte(IST8310_ADDR,IST8310_CNTL2,0x08)!=0)   
	{ 
	    printf("IST8310_error_03\r\n");            
		  return 0xff;
	}
     delay_ms(10);
    //写IST8310_AVGCNTL  默认是0x12 数据在硬件中进行4次平均后输出  0x24是16次平均后输出 此时两次读取数据的间隔不能小于6ms
	while(IIC_Write_One_Byte(IST8310_ADDR,IST8310_AVGCNTL,0x12)!=0)   
	{ 
	    printf("IST8310_error_04\r\n");            
		  return 0xff;
	}
     delay_ms(10);
    //写IST8310_AVGCNTL
	while(IIC_Write_One_Byte(IST8310_ADDR,IST8310_PDCNTL,0xc0)!=0)   
	{ 
	    printf("IST8310_error_05\r\n");            
		return 0xff;
	}
     delay_ms(10);
    
    //不自检0x00  自检0x40
	while(IIC_Write_One_Byte(IST8310_ADDR,IST8310_STR,0x00)!=0)   
	{ 
	    printf("IST8310_error_05\r\n");            
		return 0xff;
	}
     delay_ms(10);
    
	return 0;
}

//读IST8310的IST8310_STAT1寄存器 判断数据是否准备好
bool IST8310dataisready(void)
{
    
    if( (IIC_Read_One_Byte(IST8310_ADDR,IST8310_STAT1) & 0x01) == 0x01 )
    {
        return true;
    }
    //printf("ss=%d\r\n",IIC_Read_One_Byte(IST8310_ADDR,IST8310_STAT1)); //调试用
    return false;
}

//重置IST8310_CNTL1 恢复准备读取数据的状态
u8 IST8310_SETCNTL1(void)
{
	while(IIC_Write_One_Byte(IST8310_ADDR,IST8310_CNTL1,0x01)!=0)   
	{ 
	    printf("IST8310_error_01\r\n");            
		return 0xff;
	}
    return 0;
}

//读IST8310_DATA寄存器，获取xyz轴数据
u8 IST8310_RawData(magnetic_RAWTypeDef *RawDataS)
{
	u8 buf[6],res;
    static magnetic_RAWTypeDef temp;
    short tempforchange;
	res=IIC_Read_Len_Byte(IST8310_ADDR,IST8310_DATAXL,6,buf);
    if(res==0)
	{
        temp.magx = ((u16)buf[1]<<8)|buf[0];  
		temp.magy = ((u16)buf[3]<<8)|buf[2];  
		temp.magz = ((u16)buf[5]<<8)|buf[4];
		RawDataS->magx = ((u16)buf[1]<<8)|buf[0];  
		RawDataS->magy = ((u16)buf[3]<<8)|buf[2];  
		RawDataS->magz = ((u16)buf[5]<<8)|buf[4];
        
        IST8310_SETCNTL1();
        Filterx(&temp.magx,&RawDataS->magx);
        Filtery(&temp.magy,&RawDataS->magy);
        Filterz(&temp.magz,&RawDataS->magz);
        tempforchange = RawDataS->magy;
        RawDataS->magy = RawDataS->magx;
        RawDataS->magx = tempforchange;
        
	} 	
    return res;
}

#define FILTER_NUM 5
#define FILTER_A 20
void Filterx(short* in, short* out)
{	
	static u8 i=0;
	static short filter_buf[5]={0};
	short filter_sum=0;
	u8 cnt=0;	
	short deta;		
	
	if(filter_buf[i] == 0)
	{
		filter_buf[i]=*in;
		*out=*in;
		if(++i>=5)	i=0;
	} else 
	{
		if(i) deta=*in-filter_buf[i-1];
		else deta=*in-filter_buf[5-1];
		
		if(abs(deta)<FILTER_A)
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
void Filtery(short* in, short* out)
{	
	static u8 i=0;
	static short filter_buf[5]={0};
	short filter_sum=0;
	u8 cnt=0;	
	short deta;		
	
	if(filter_buf[i] == 0)
	{
		filter_buf[i]=*in;
		*out=*in;
		if(++i>=5)	i=0;
	} else 
	{
		if(i) deta=*in-filter_buf[i-1];
		else deta=*in-filter_buf[5-1];
		
		if(abs(deta)<FILTER_A)
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
void Filterz(short* in, short* out)
{	
	static u8 i=0;
	static short filter_buf[5]={0};
	short filter_sum=0;
	u8 cnt=0;	
	short deta;		
	
	if(filter_buf[i] == 0)
	{
		filter_buf[i]=*in;
		*out=*in;
		if(++i>=5)	i=0;
	} else 
	{
		if(i) deta=*in-filter_buf[i-1];
		else deta=*in-filter_buf[5-1];
		
		if(abs(deta)<FILTER_A)
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

