#include "2stm.h"
extern bool napo;
extern u16 mesg[5];












void twostm_task(void *param)
{																																																		//��ʱ��ʱ�ӣ�ÿ��1ms +1
	u32 lastWakeTime	=	xTaskGetTickCount();																																				//��ʱ
	u8 nene;
	mesg[4]=0x0a;
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 1000);																		/*1s������ʱ*/
		
		//if(GetFlightLoggerState()==1)
		
		 
			nene = (u8)(mesg[4] )&0xFF;  
      usart1_send_char(nene);
      // delay_us(100);
			delay_us(100);
      nene = (u8)(mesg[0] >>8)&0xFF;  
      usart1_send_char(nene);
		  delay_us(100);
      // delay_ms(2);
      nene = (mesg[0]&0xFF);
			usart1_send_char(nene);
			// delay_ms(2);
			delay_us(100);
			nene = (u8)(mesg[1] >>8)&0xFF;  
      usart1_send_char(nene);
      //delay_ms(2);
		  delay_us(100);
      nene = (mesg[1]&0xFF);
			usart1_send_char(nene);
			delay_us(100);
			// delay_ms(2);
			
			nene = (u8)(mesg[2] >>8)&0xFF;  
      usart1_send_char(nene);
			delay_us(100);
      // delay_ms(2);
      nene = (mesg[2]&0xFF);
			usart1_send_char(nene);
			delay_us(100);
			// delay_ms(2);
			
			
		
		
		
		}//end of while(1)

}
