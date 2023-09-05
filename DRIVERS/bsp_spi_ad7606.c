/*
*********************************************************************************************************
*	                                  
*	ģ������ : AD7606����ģ��
*	�ļ����� : bsp_spi_ad7606.c
*	˵    �� : ����AD7606 ADCת���� SPI�ӿ�
*	��    �� ��WZH
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
#include "FreeRTOS.h"					//FreeRTOSʹ��		  
#include "task.h"
#endif
#include "exfuns.h"

extern FIFO_T wzh;
extern float wzhspeed;
extern float wzhfre;
// ����һ�����������������ڴ洢AD�ɼ����ݣ�������д��SD
u16 mesg [5] ={0};
extern u16 mesg[5];

bool napo = 0;
extern bool napo;

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitAD7606
*	����˵��: ��ʼ��AD7606 SPI����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitAD7606(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE ,ENABLE);    /* ʹ��GPIOʱ�� */
	
	
		

	//RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE ,ENABLE);    /* ʹ��GPIOʱ�� */
   
	 
    //RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPIW, ENABLE);//ʹ��SPI2ʱ��
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��    PE4ΪSPIWsck�ź�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PE���ù������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��    PE3ΪSPIW miso�ź�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;       	//PE3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//���빦��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//Ĭ��
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIODʱ��    PD0ΪSPIWƬѡ�ź�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA���ù������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��
	
	
	/* ����������GPIO */

	/* ����RESET GPIO */
	  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;               /* PA8����RESET*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* ����BUSY GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;               /* PC13����BUSY*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//Ĭ��
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	//GPIO_InitStructure.GPIO_Pin = AD_CONVST_PIN;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_Init(AD_CONVST_GPIO_PORT, &GPIO_InitStructure);
 
 	/* ����CONVST GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;               /* PA11����CONVST*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* ����OS0-2 GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����		/* PC3����OS0*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		 
		 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* PC4����OS1*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* PC5����OS2*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	/* ���ù�����ģʽ */
	ad7606_SetOS(4);

	/* ����GPIO�ĳ�ʼ״̬ */
	ad7606_Reset();				/* Ӳ����λ��AD7606 */
	/* ���ù�����ģʽ */
	
	ad7606_SetOS(4);
	
	//AD_CONVST_HIGH();		
	GPIO_SetBits(GPIOD,GPIO_Pin_0); /* CS������Ϊ�ߵ�ƽ PD0*/	
	GPIO_SetBits(GPIOA,GPIO_Pin_11);	/* CONVST������Ϊ�ߵ�ƽ PA11*/	
	GPIO_SetBits(GPIOE,GPIO_Pin_4); /* clk������Ϊ�ߵ�ƽ PE4*/
	// ad7606_StartConv();
	ad7606_Reset();
	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	ad7606_Reset();

		
}

/*
*********************************************************************************************************
*	�� �� ��: ad7606_Reset
*	����˵��: Ӳ����λAD7606
*	��    �Σ���
*	�� �� ֵ: ��
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
	GPIO_SetBits(GPIOA,GPIO_Pin_8);     //��λ����50ns
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
*	�� �� ��: ad7606_SetOS
*	����˵��: ���ù�����ģʽ�������˲���Ӳ����ƽ��ֵ)
*	��    �Σ�_ucMode : 0-6  0��ʾ�޹�������1��ʾ2����2��ʾ4����3��ʾ8����4��ʾ16��
*				5��ʾ32����6��ʾ64��
*	�� �� ֵ: ��
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
	else if	(_ucMode == 0)   /* ��0���� */
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);//AD_OS2_0();
	  GPIO_ResetBits(GPIOC,GPIO_Pin_4);	//AD_OS1_0();
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);//AD_OS0_0();
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ad7606_StartConv
*	����˵��: ����AD7606��ADCת��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ad7606_StartConv(void)
{
	/* �����ؿ�ʼת�����͵�ƽ����ʱ������25ns  */
	GPIO_SetBits(GPIOA,GPIO_Pin_11);
	
	//printf("%d",GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7));
	GPIO_ResetBits(GPIOA,GPIO_Pin_11); //AD_CONVST_LOW();
	GPIO_ResetBits(GPIOA,GPIO_Pin_11); //AD_CONVST_LOW();
 	// printf("%d",GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11));	//AD_CONVST_LOW();	/* ����ִ��2�Σ��͵�ƽԼ50ns */
	GPIO_SetBits(GPIOA,GPIO_Pin_11); 
	// printf("%d",GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1));	

}


void AD7606_read_data(s16 * DB_data) 
{  
	u8 i,j; 	
	GPIO_ResetBits(GPIOD,GPIO_Pin_0);      //����PD0 cs�źſ�ʼ��������
	GPIO_ResetBits(GPIOD,GPIO_Pin_0);
	for(i=0;i<8;i++)
	{
		
		u16 DB_data1 = 0;
		//GPIO_SetBits(GPIOE,GPIO_Pin_4);
   		//  delay_us (4);	
		//  GPIO_ResetBits(GPIOE,GPIO_Pin_4);       //clk����ԼΪ100~200ns,�ȸ�һ������
	 	//delay_us (4);	
		for(j=0;j<16;j++)
		{		
		GPIO_SetBits(GPIOE,GPIO_Pin_4);   //����ʱ�ӣ�16����Ϊ��16��֡	��Ĭ��Ϊ��
		GPIO_SetBits(GPIOE,GPIO_Pin_4);			
	  	GPIO_ResetBits(GPIOE,GPIO_Pin_4);    //����ʱ��
		GPIO_ResetBits(GPIOE,GPIO_Pin_4);
		DB_data1 = ((u16)((GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)))*pow(2,(15-j))) + DB_data1 ;     //����ʱ�ӣ�16����Ϊ��16��֡(�����Ʋ���)�����λ�������Ⱥ���	
		//printf("%d",GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3));	
		
    
			
		}			
		DB_data[i] = (u16)DB_data1;
	}
	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	GPIO_SetBits(GPIOD,GPIO_Pin_0);   //����PA4 cs�źŽ�����������
	GPIO_SetBits(GPIOE,GPIO_Pin_4);  //����ʱ���ź�
	GPIO_SetBits(GPIOE,GPIO_Pin_4);
} 

void ad_task(void *param)
{																																																		//��ʱ��ʱ�ӣ�ÿ��1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();																																				//��ʱ
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
		vTaskDelayUntil(&lastWakeTime, 10);																		/*10ms������ʱ*/
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
					
              //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6507)/0.05575,wzhfre,wzhspeed);   //�Ƶ�
              //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6680)/0.0582,wzhfre,wzhspeed);   //�׵״���
							//sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6450)/0.0580,wzhfre,wzhspeed);  //�����޽�
					    //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.65)/0.0571,wzhfre,wzhspeed);  //L1
					    //sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6537)/0.0570,wzhfre,wzhspeed);   //�Ƶ�N3
				    	 // sprintf((char*)flight_log_buf,"%.4f ,%.4f ,flapping speed is %.4f ,airspeed is %.4f \r\n",gg[0]*4.92,(gg[1]-1.6464)/0.0562,wzhfre,wzhspeed);   //2 HAO
			sprintf((char*)flight_log_buf,"%.4f ,%.4f\r\n",gg[0]*4.92f,(gg[1]-1.6565f)/0.0563f);   //����ɼ����ݵ�������־
            f_write(&flight_log_fil,flight_log_buf,strlen(flight_log_buf),&flight_log_bww);       //д��SD��
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

