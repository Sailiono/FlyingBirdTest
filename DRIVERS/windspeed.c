#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "24cxx.h"
#include "math.h"
#include "windspeed.h"
//ALIENTEK ̽����STM32F407������ ʵ��24
//IIC ʵ�� --�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com  
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK
float wzhspeed=0;
extern float wzhspeed;
extern u16 mesg[5];
static float airspeed[3];
//void windspeed_task(void *param)
//{																																																		//��ʱ��ʱ�ӣ�ÿ��1ms +1
//	u32 lastWakeTime	=	xTaskGetTickCount();																																				//��ʱ
//	u8 buf[4],res;
//	u32 press;
//	u32 upress[10];
//	float delta_press;
//	float rho = 1.255;  //�����ܶ�
//	
//	float wwww;
//	float lxh;
//	float jj=2;
//	u8 i,j,k,m;
//	m=0;

//		
//		
//	while(1)
//	{
//		vTaskDelayUntil(&lastWakeTime, 800);																		/*250ms������ʱ*/
//	while (jj>=1 &&  m==0)
//	{
//			
//		res = IICReadLenByteWithoutReg(0x28,4,buf);
//		if(res == 0)
//		{
//			
//     
//	
//		
//			press = ((buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3]) & 0x3FFF0000;
//			press>>=16;
//			//printf("press = %d",press);	
//      delta_press = ((float)press - lxh)*2*6894.757/(0.8*16383);  //���㶯ѹ
//			jj = sqrt(2*fabs(delta_press)/rho);
//			lxh=press;
//		//	printf("   airspeed = %f\r\n",airspeed);
//			
//			
//		}
//	
//		printf("jj = %f",jj);
//		
//		
//		
//	}
//	m=1;
//		//printf("aa\r\n");
//				for(i=0;i<100;i++)
//		{
//		res = IICReadLenByteWithoutReg(0x28,4,buf);
//		if(res == 0)
//		{
//			
//     
//	
//		
//			press = ((buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3]) & 0x3FFF0000;
//			press>>=16;
//			//printf("press = %d",press);	
//      delta_press = ((float)press - lxh)*2*6894.757/(0.8*16383);  //���㶯ѹ
//			airspeed[1] = sqrt(2*fabs(delta_press)/rho);  //�������
//			 
//		//	printf("   airspeed = %f\r\n",airspeed);
//			if(airspeed[1]>=airspeed[0])
//			{
//			airspeed[0]=airspeed[1];
//			}
//			else
//			{
//			airspeed[1]=airspeed[0];
//			}
//			delay_ms(4);
//		}
//	}
//   // printf("airspeed=%f\r\n",airspeed[0]);
//	if(airspeed[0]<5)
//	{
//		wzhspeed=0;
//		mesg[3]=0;
//		airspeed[0]=0;
//		airspeed[1]=0;
//	}
//	 if(airspeed[0]>5 & airspeed[0]<=20)
//	 {
//	  wzhspeed=airspeed[0];
//		airspeed[2]=airspeed[0];
//	  mesg[3]=(u16)(airspeed[0]*1000);
//		airspeed[0]=0;
//		airspeed[1]=0;
//	 }
//	 
//	 if(airspeed[0]>20)
//	 {
//	  wzhspeed=airspeed[2];
//	  mesg[3]=(u16)(airspeed[2]*1000);
//		airspeed[0]=0;
//		airspeed[1]=0;
//		 
//	 }
//		//delay_ms(50);
//		    


//		}
//	}