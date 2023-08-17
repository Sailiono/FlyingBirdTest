#include "radio.h"
#include "usart.h"
#include "main.h"
#include "position_estimate.h"
#include "w25qxx.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )



u8 data_to_send[50];	//�������ݻ���

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange��������������ݷ������� �˺���Ӧ���û�ÿ1ms����һ��
void ANO_DT_Data_Exchange(void)
{
	static u32 cnt = 0; 

/*û�п���v4�汾�ĵ���վ������Ӳ���汾����ʾ���� ������*/
//	if((cnt % VERSION_CNT) == 0) 
//    {
//		ANO_DT_Send_Version(4,300,100,400,0);
//    }           
/*��������վֻ�ܽ���s16��ʽ�Ĵ�����ԭʼ���ݣ����й�����������
    ��������ԭʼ���ݻ��Թ��굥λ��¼����־�ļ���*/	    
//	if((cnt % SENSOR_CNT) == 0)
//    {
////      ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar);
//    }

    /*���ͷ���״̬����  ����
    ��̬��  �߶� ����ģʽ ����״̬*/	
	if((cnt % STATUS_CNT) == 0)
	{
        Attitude_S attitude;
        float height;
        u8 flymode;
        u8 armedstatus;
        
        getAttitudeData(&attitude);
        getheight(&height);
        getflymode(&flymode);
        getarmedstatus(&armedstatus);
       
        ANO_DT_Send_Status(attitude.roll,attitude.pitch,attitude.yaw,(s32)(height*100),flymode,armedstatus);
    }	

    /*����ң������ԭʼͨ��ֵ*/
	if((cnt % RCDATA_CNT) == 0)
	{
        RC PWM_US;
        
        getpwmus(&PWM_US);
        
        /*void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);*/
        ANO_DT_Send_RCData(PWM_US.throttle,PWM_US.aileron_elevator,PWM_US.aileron,PWM_US.elevator,\
                                PWM_US.mode,0,0,0,0,0);
    }
	
//	if((cnt % MOTORPWM_CNT) == 0)
//	{
//        ANO_DT_Send_MotoPWM(1,2,3,4,5,6,7,8);
//  }	
	
//	if((cnt % POWER_CNT) == 0)
//	{
//        ANO_DT_Send_Power(123,456);
//  }	
	cnt++;
}

/*///////////////////////////////////////////////////////////////////////////////////
//Send_Data ������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���
*/
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	uart1Dmasend(data_to_send, length);
}

/*�ɿؽ��յ���λ��������ĳЩ֡���ݺ�Ҫ����һ��У��֡*/
static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}



/*////////////////////////////////////////////////////////////////////////////////////
ANO_DT_Data_Receive_Prepare������Э��Ԥ���������������յ���һ��һ���ֽ�ƴ��һ֡����
ÿ�յ�һ���ֽڵ���һ��������������ռ������ֽ����չ�һ֡ʱ�������е������ݽ�������
*/
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}



pid_s ctrl_1,ctrl_4;
static float DP,DP_rate;

void getPID(float*dp,float*dp_rate)
{
    W25QXX_Read((u8*)&DP_rate,FLASH_SIZE-4096-100,4);
    W25QXX_Read((u8*)&DP,FLASH_SIZE-4096-100-4,4);
    *dp = DP;
    *dp_rate = DP_rate;
}

extern SemaphoreHandle_t calibrateacc_binarysemaphore;		//���ٶȼƶ�ֵ�ź�������
extern SemaphoreHandle_t calibrategyro_binarysemaphore;		//�����Ƕ�ֵ�ź�������
extern SemaphoreHandle_t calibratebaro_binarysemaphore;		//��ѹ�ƶ�ֵ�ź�������
/*
ANO_DT_Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡
��Щ����֡��һ����ָ��
�ú������ȶ���һ֡���ݽ���У�飨��ͣ� 
У��ͨ��������ݽ��н�����ʵ����Ӧ����  ��ָ����в�����У׼�����ز�������ʼ�����õȣ�
�˺������Բ����û����е��ã��ɺ���ANO_DT_Data_Receive_Prepare�Զ�����
*/
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ AA AF ����λ�����ɿص�����
	
    

    
    //������=0X01 ������֡Ϊ��λ��������У׼����֡
	if(*(data_buf+2)==0X01)             
	{
        u8 armedstatus;
        getarmedstatus(&armedstatus);
        if(armedstatus != ARMED)
        {
            if(*(data_buf+4)==0X01)         //����Ϊ 0x01 ACCУ׼
            {
                xSemaphoreGive(calibrateacc_binarysemaphore);	  
            }
            if(*(data_buf+4)==0X02)         //����Ϊ 0x02 GYROУ׼
            {
                ANO_DT_Send_Cailibrate_Check(0x02,0xE1);    //������У׼�ɹ�
            }

            if(*(data_buf+4)==0X04)         //����Ϊ 0x04   //MAGУ׼
            {
                ANO_DT_Send_Cailibrate_Check(0x03,0x01);    //����У׼�ɹ�
            }
            if(*(data_buf+4)==0X05)         //����Ϊ 0x04   //BAROУ׼
            {
                ANO_DT_Send_Cailibrate_Check(0x03,0x01);    //��ѹ��У׼�ɹ�
            }
        }

        
	}
	
    
	if(*(data_buf+2)==0X02)             //������=0x02 ������֡Ϊ��λ����������������֡
	{
		if(*(data_buf+4)==0X01)         //����Ϊ 0x01 ��ȡPID����
		{
            
            W25QXX_Read((u8*)&DP_rate,FLASH_SIZE-4096-100,4);
            W25QXX_Read((u8*)&DP,FLASH_SIZE-4096-100-4,4);
            ANO_DT_Send_PID(1, DP_rate,0,0,0,0,0,0 ,0,0);
            ANO_DT_Send_PID(2, DP,0,0,0,0,0,0 ,0,0);
		}
		if(*(data_buf+4)==0X02)         //����Ϊ0x02 ��ȡ����ģʽ��������
		{
			
		}
		if(*(data_buf+4)==0XA0)		    //����Ϊ0xA0 ��ȡ��λ���汾��Ϣ
		{
//			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		    //����Ϊ0xA1 �ָ�Ĭ�ϲ���
		{
//			Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X10)			    //������=0x10 ������֡Ϊ��λ���������úõ�PID123����
    {
        ctrl_1.kp = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
       
        W25QXX_Write((u8 *)&ctrl_1.kp,FLASH_SIZE-4096-100,4);
        
        ctrl_4.kp = 0.001*( (vs16)(*(data_buf+22)<<8)|*(data_buf+23) );
        W25QXX_Write((u8 *)&ctrl_4.kp,FLASH_SIZE-4096-100-4,4);
        ANO_DT_Send_Check(*(data_buf+2),sum);

    }
    if(*(data_buf+2)==0X11)				//������=0x11 ������֡Ϊ��λ���������úõ�PID456����
    {
        ctrl_4.kp = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        W25QXX_Write((u8 *)&ctrl_4.kp,FLASH_SIZE-4096-100-4,4);
        ANO_DT_Send_Check(*(data_buf+2),sum);	
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
        ANO_DT_Send_Check(*(data_buf+2),sum);		
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}




void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}


void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


/*�ɿؽ��յ���λ��������ĳЩ֡���ݺ�Ҫ����һ��У��֡*/
void ANO_DT_Send_Cailibrate_Check(u8 msg_id, u8 msg_data)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=msg_id;
	data_to_send[5]=msg_data;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}






/*������λ�������������ݲ����һ֡
ANO_DT_Data_Receive_Prepare() �������֡��У���
�����е������Ľ�������*/
void Radiorx_task(void *pvParameters)
{
    u8 charrx;
    while(1)
    {
        if (uartslkGetDataWithTimout(&charrx)) //��������������һ������
        {
            ANO_DT_Data_Receive_Prepare(charrx);
        }
        
    }
}


//ͨ����������λ���������ݵ�����
void Radiotx_task(void *pvParameters)
{
    u32 lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, 1);	
        ANO_DT_Data_Exchange();
        
    }
}
