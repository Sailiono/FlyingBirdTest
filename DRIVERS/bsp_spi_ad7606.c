/*
*********************************************************************************************************
*	                                  
*	模块名称 : AD7606驱动模块
*	文件名称 : bsp_spi_ad7606.c
*	说    明 : 驱动AD7606 ADC转换器 SPI接口
*	创    建 ：WZH
*
*********************************************************************************************************
*/

#include "spi.h"
#include <stdio.h>
#include "bsp_spi_ad7606.h"
#include <math.h>
#include "delay.h"
#include "led.h"
#include "flight_log.h"
#include "sbus.h"
#include "string.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h"
#endif
#include "exfuns.h"

extern FIFO_T wzh;
extern float wzhspeed;
extern float wzhfre;
// 定义一个交换缓冲区，用于存储AD采集数据，并用于写入SD
u16 mesg [5] ={0};
extern u16 mesg[5];

bool napo = 0;
extern bool napo;

/*
*********************************************************************************************************
*	函 数 名: bsp_InitAD7606
*	功能说明: 初始化AD7606 SPI口线
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitAD7606(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE ,ENABLE);    /* 使能GPIO时钟 */
	
	
		

	//RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE ,ENABLE);    /* 使能GPIO时钟 */
   
	 
    //RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPIW, ENABLE);//使能SPI2时钟
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟    PE4为SPIWsck信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PE复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟    PE3为SPIW miso信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;       	//PE3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//默认
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOD时钟    PD0为SPIW片选信号
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化
	
	
	/* 配置其它的GPIO */

	/* 配置RESET GPIO */
	  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;               /* PA8用作RESET*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 配置BUSY GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;               /* PC13用作BUSY*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//默认
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	//GPIO_InitStructure.GPIO_Pin = AD_CONVST_PIN;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_Init(AD_CONVST_GPIO_PORT, &GPIO_InitStructure);
 
 	/* 配置CONVST GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;               /* PA11用作CONVST*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 配置OS0-2 GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉		/* PC3用作OS0*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		 
		 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* PC4用作OS1*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* PC5用作OS2*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	/* 设置过采样模式 */
	ad7606_SetOS(4);

	/* 设置GPIO的初始状态 */
	ad7606_Reset();				/* 硬件复位复AD7606 */
	/* 设置过采样模式 */
	
	ad7606_SetOS(4);
	
	//AD_CONVST_HIGH();		
	GPIO_SetBits(GPIOD,GPIO_Pin_0); /* CS脚设置为高电平 PD0*/	
	GPIO_SetBits(GPIOA,GPIO_Pin_11);	/* CONVST脚设置为高电平 PA11*/	
	GPIO_SetBits(GPIOE,GPIO_Pin_4); /* clk脚设置为高电平 PE4*/
	// ad7606_StartConv();
	ad7606_Reset();
	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	ad7606_Reset();

		
}

/*
*********************************************************************************************************
*	函 数 名: ad7606_Reset
*	功能说明: 硬件复位AD7606
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_Reset(void)
{
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	//printf("%d",GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8));
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);     //复位至少50ns
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	delay_us(1);
	//printf("%d",GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8));
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	printf("%d",GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8));
}	
	
/*
*********************************************************************************************************
*	函 数 名: ad7606_SetOS
*	功能说明: 设置过采样模式（数字滤波，硬件求平均值)
*	形    参：_ucMode : 0-6  0表示无过采样，1表示2倍，2表示4倍，3表示8倍，4表示16倍
*				5表示32倍，6表示64倍
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_SetOS(uint8_t _ucMode)
{
	if (_ucMode == 1)
	{
		//AD_OS2_0();
		//AD_OS1_0();
		//AD_OS0_1();
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_ResetBits(GPIOC,GPIO_Pin_4);
		GPIO_SetBits(GPIOC,GPIO_Pin_3);
	}
	else if (_ucMode == 2)
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);//AD_OS2_0();
		GPIO_SetBits(GPIOC,GPIO_Pin_4);//AD_OS1_1();
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);//AD_OS0_0();
	}
	else if (_ucMode == 3)
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);//AD_OS2_0();
		GPIO_SetBits(GPIOC,GPIO_Pin_4);//AD_OS1_1();
		GPIO_SetBits(GPIOC,GPIO_Pin_3);//AD_OS0_1();
	}
	else if (_ucMode == 4)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_5);//AD_OS2_1();
		GPIO_ResetBits(GPIOC,GPIO_Pin_4);//AD_OS1_0();
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);//AD_OS0_0();
	}
	else if (_ucMode == 5)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_5);//AD_OS2_1();
		GPIO_ResetBits(GPIOC,GPIO_Pin_4);//AD_OS1_0();
		GPIO_SetBits(GPIOC,GPIO_Pin_3);//AD_OS0_1();
	}
	else if (_ucMode == 6)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_5);//AD_OS2_1();
		GPIO_SetBits(GPIOC,GPIO_Pin_4);//AD_OS1_1();
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);//AD_OS0_0();
	}
	else if	(_ucMode == 0)   /* 按0处理 */
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);//AD_OS2_0();
	  GPIO_ResetBits(GPIOC,GPIO_Pin_4);	//AD_OS1_0();
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);//AD_OS0_0();
	}
}

/*
*********************************************************************************************************
*	函 数 名: ad7606_StartConv
*	功能说明: 启动AD7606的ADC转换
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_StartConv(void)
{
	/* 上升沿开始转换，低电平持续时间至少25ns  */
	GPIO_SetBits(GPIOA,GPIO_Pin_11);
	
	//printf("%d",GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7));
	GPIO_ResetBits(GPIOA,GPIO_Pin_11); //AD_CONVST_LOW();
	GPIO_ResetBits(GPIOA,GPIO_Pin_11); //AD_CONVST_LOW();
 	// printf("%d",GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11));	//AD_CONVST_LOW();	/* 连续执行2次，低电平约50ns */
	GPIO_SetBits(GPIOA,GPIO_Pin_11); 
	// printf("%d",GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1));	

}


void AD7606_read_data(s16 * DB_data) 
{  
	u8 i,j; 	
	GPIO_ResetBits(GPIOD,GPIO_Pin_0);      //拉低PD0 cs信号开始接受数据
	GPIO_ResetBits(GPIOD,GPIO_Pin_0);
	for(i=0;i<8;i++)
	{
		
		u16 DB_data1 = 0;
		//GPIO_SetBits(GPIOE,GPIO_Pin_4);
   		//  delay_us (4);	
		//  GPIO_ResetBits(GPIOE,GPIO_Pin_4);       //clk周期约为100~200ns,先给一个周期
	 	//delay_us (4);	
		for(j=0;j<16;j++)
		{		
		GPIO_SetBits(GPIOE,GPIO_Pin_4);   //接收时钟，16次因为有16个帧	，默认为正
		GPIO_SetBits(GPIOE,GPIO_Pin_4);			
	  	GPIO_ResetBits(GPIOE,GPIO_Pin_4);    //接收时钟
		GPIO_ResetBits(GPIOE,GPIO_Pin_4);
		DB_data1 = ((u16)((GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)))*pow(2,(15-j))) + DB_data1 ;     //接收时钟，16次因为有16个帧(二进制补码)，最高位是正负先忽略	
		//printf("%d",GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3));	
		
    
			
		}			
		DB_data[i] = (u16)DB_data1;
	}
	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	GPIO_SetBits(GPIOD,GPIO_Pin_0);   //拉高PA4 cs信号结束接收数据
	GPIO_SetBits(GPIOE,GPIO_Pin_4);  //拉高时钟信号
	GPIO_SetBits(GPIOE,GPIO_Pin_4);
} 

void ad_task(void *param)
{																																																		//定时器时钟，每隔1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();																																				//定时
	u8 temp= 4;
	s16 pp [8]={-1};
	float gg[8]={-1};
	u8 i,j;
	u16 save_flag=0; 
   
	static FIL flight_log_fil;
    static UINT flight_log_bww;
    static char flight_log_buf[200];
  
    char logname_buf[15] = "test.txt";
	static bool file_not_open_flag = true;  //????????
                       //?????? ???+1 ????????????

//    static float volt,current;
//    static bool flightloggerbutton;
//	static RcRawDataSTypeDef myrc;
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 10);																		/*10ms周期延时*/
	//	flightloggerbutton=GetFlightLoggerState();
//    if(flightloggerbutton == false)
//		{
//		GPIO_SetBits(GPIOA,GPIO_Pin_5);
//		}
    delay_us(2);
		if(file_not_open_flag) 		
        {
					//printf("123");
           // if(flightloggerbutton == true)
            {
							//printf("fg");
							//LED1On();
                sprintf((char*)logname_buf,"ad_%lld.txt",FreeRTOSRunTimeTicks);
					//sprintf((char*)logname_buf,"ui.txt");
//                sprintf((char*)logname_buf,"%02d.%02d.%02d.txt",gpsdata.gps_time.hours,gpsdata.gps_time.minutes,gpsdata.gps_time.seconds);
                if(f_open(&flight_log_fil,logname_buf,FA_OPEN_APPEND|FA_WRITE)==0)
                {
                    file_not_open_flag = false;
                }
           // }
//            if(armedstatus == ARMED && f_open(&flight_log_fil,logname_buf,FA_OPEN_APPEND|FA_WRITE)==0)
//            {
//                file_not_open_flag = false;
//                printf("create success");
//            }
            
        }
			}
				
				
		ad7606_StartConv();
		temp = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);			 // ?? BUSY??? 
		while(temp == 1)				//?busy?????,??????,???????? 
		{
			temp = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);		// ?? BUSY??? 	 
		}	
		AD7606_read_data(pp);
		
		for(i=0;i<8;i++)
		{
		gg[i]=(float)((float)pp[i]/32768*5000)/1000;
			//printf("%f",gg[i]);
		}
		for(j=0;j<2;j++)     //???????
		{
		mesg[j]=(u16)(gg[j]*1000);
		}
		//printf("\r\n");
		 if(!file_not_open_flag == true)
		// if(!file_not_open_flag && flightloggerbutton == true)
        {
//			printf("ssss");
//       	sprintf((char*)flight_log_buf,"%f %f %f %.2f %.2f %.2f %.2f,%.2f %.2f,%.2f,%.2f %.2f,%d,%lld\r\n",
//                                	uler_est.roll,euler_est.pitch,euler_est.yaw,
//                               	gpsdata.latitude,gpsdata.longitude,gpsdata.height,
//									g_pos_est.x,g_pos_est.y,g_pos_est.z,
//									g_pos_est.vx,g_pos_est.vy,g_pos_est.vz,
//									flymode,
//                                	FreeRTOSRunTimeTicks); 
					
              //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6507)/0.05575,wzhfre,wzhspeed);   //黄底
              //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6680)/0.0582,wzhfre,wzhspeed);   //白底带胶
							//sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6450)/0.0580,wzhfre,wzhspeed);  //坏底无胶
					    //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.65)/0.0571,wzhfre,wzhspeed);  //L1
					    //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6537)/0.0570,wzhfre,wzhspeed);   //黄底N3
				    	 // sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6464)/0.0562,wzhfre,wzhspeed);   //2 HAO
			sprintf((char*)flight_log_buf,"%.4f ,%.4f\r\n",gg[0]*4.92f,(gg[1]-1.6565f)/0.0563f);   //输入采集数据到飞行日志
            f_write(&flight_log_fil,flight_log_buf,strlen(flight_log_buf),&flight_log_bww);       //写入SD卡
			LED2On();
//          printf("%d\r\n",FLAPPINGANGLE_POSITION);
							
            //if(file_not_open_flag == false && flightloggerbutton == false)
            //{
                 //f_sync(&flight_log_fil);	//????????????,????
                // file_not_open_flag = true;
           // }                
            if(save_flag  %200==0)
            {
                f_sync(&flight_log_fil);	//?????????? ????????
                save_flag=0;    
            }	
            save_flag++;
        
        }
	}//end of while(1)

}

