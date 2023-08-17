#include "main.h"


static enumarmstatus armstatus = DISARMED;
static enummodestatus modestatus = MODE_ERR;
static bool armedchangeflag = false; 

//��ʱ��������ģʽ �ֶ� ���� �� ������Ҫɶ���ܿ�������������ӣ�
//����ģʽ�л��ж� ���������µķ���ģʽ
void flightmode_judge(void)
{
//	static enummodestatus flight_mode_pre = MODE_ERR;
	enummodestatus flight_mode_cur = MODE_ERR;
	
    /*���myrc modeͨ��������ռ�ձȵĴ�С����Ӧ��ͬң�����ƶ�����ģʽ*/
	if(myrc.mode<10)
	{
        flight_mode_cur = MODE_ERR;
	}else 
	if((myrc.mode>10)&&(myrc.mode<500))
	{
        flight_mode_cur = MODE_MANUAL;
	}else
    if((myrc.mode>500)&&(myrc.mode<1200))
	{
        flight_mode_cur = MODE_STABILIZED;
	}else
	{
        flight_mode_cur = MODE_AUTO;
	}    
    
    /*�����ǰģʽҪ��ȫ�������У���Ҫ���һ����û��GPS�źţ�
    ����У�������л������û�� ��ǰģʽ�˻ص���һ��ģʽ
    �����ǰģʽ��ȫ�������У���ͨ����ǰ�л� ģʽת���ɹ�*/
//	if(flight_mode_cur >= MODE_AUTO)       
//	{
//        if(!isgpsvalid)
//        {  
//            flight_mode_cur = flight_mode_pre; 
//        }
//     
//	} 
    modestatus = flight_mode_cur;
}



/*���ڽ��������ж�
�������������� ��5��״̬ ���� DISARMED ARMED DISARMED_READY��ģʽ�л��� ARMED_READY��ģʽ�л��� CANCEL
��ʾ���յĽ�� ֻ��2��״̬ ���� DISARMED�������� ARMED�������� 
�����armstatus_temp armedchangeflag ֻ��Ϊ�˸��õظ���״̬��ͬ������BEEP
���յĽ����  �����ڽ���ʱ ����������  ��ģʽ�л�����ʱ һֱ��ֱ���л���� ����������״̬ʱ ÿ��1����һ��
*/
void armstatus_judge(void)
{
    static enumarmstatus armstatus_temp = DISARMED;
    static u32 armedreadytime;
    
    /*���� ҡ�������´��Ҵ�ʱ�ɿ�״̬Ϊ �ǽ���״̬ ���ܹ������һ����
    1�����ȴ򿪷����� ��ʾΣ��    ͬʱ��״̬ת������ʱ��־λ��1  ��ֹ��������ͣ�л�
    2���ж�armstatus_temp����������� ARMED_READY ״̬�������ý���׼����ʼʱ��ֵarmedreadytime
                  ����ǣ����жϵ�ǰʱ�̺ͽ���׼����ʼʱ��ֵ֮�� �Ƿ����2s ��������� ���������
    3��armstatus_temp = ARMED_READY
    */
    /*���� ҡ�������´��Ҵ�ʱ�ɿ�״̬Ϊ ������״̬ ���ܹ�����ڶ�����
    1�����ȴ򿪷����� ��ʾΣ��    ͬʱ��״̬ת������ʱ��־λ��1  ��ֹ��������ͣ�л�
    2���ж�armstatus_temp����������� DISARMED_READY ״̬������������׼����ʼʱ��ֵarmedreadytime
                  ����ǣ����жϵ�ǰʱ�̺�����׼����ʼʱ��ֵ֮�� �Ƿ����2s ���������� ���������
    3��armstatus_temp = DISARMED_READY
    */
    if((myrc.throttle<=(THROTTLE_MIN+10))&&(myrc.aileron_elevator>(AILERON_ELEVATOR_MAX-10))&&(armstatus != ARMED))
    {
        BEEPON;
        armedchangeflag = true;
        if(armstatus_temp != ARMED_READY)
        {
            armedreadytime = FreeRTOSRunTimeTicks;
        }else if((FreeRTOSRunTimeTicks-armedreadytime)>=20000)
        {
            TIM_Cmd(TIM5, ENABLE);  //ʹ�ܵ��pwm�����ɵĶ�ʱ��
            armstatus_temp = ARMED;
            armstatus = armstatus_temp;
            BEEPOFF;
        }
        armstatus_temp = ARMED_READY;
    }else if((myrc.throttle<=(THROTTLE_MIN+10))&&(myrc.aileron_elevator<(AILERON_ELEVATOR_MIN+10))&&(armstatus != DISARMED))
    {
        BEEPON;
        armedchangeflag = true;
        if(armstatus_temp != DISARMED_READY)
        {
            armedreadytime = FreeRTOSRunTimeTicks;
        }else if((FreeRTOSRunTimeTicks-armedreadytime)>=20000)
        {
            TIM_Cmd(TIM5, DISABLE);  //��ֹ���pwm�����ɵĶ�ʱ��
            armstatus_temp = DISARMED;
            armstatus = armstatus_temp;
            BEEPOFF;
            armedchangeflag = false;
        }
        armstatus_temp = DISARMED_READY;
    }else if( myrc.throttle > (THROTTLE_MIN+10) || 
    (  myrc.aileron_elevator>(AILERON_ELEVATOR_MIN+10) && myrc.aileron_elevator<(AILERON_ELEVATOR_MAX-10)     )  )
    
    {
        //���ʱ��ģʽ�л��Ѿ�ͣ�ˣ����������չ涨ȥ��
        armedchangeflag = false;
        /*ֻ�е�����Ԥ���׶� ͻȻ�˳� ���п��ܱ�Ϊȡ��״̬*/
        if(armstatus_temp==ARMED_READY || armstatus_temp==DISARMED_READY)
        {
            armstatus_temp = CANCEL; 
            BEEPOFF;
        }

    }
}

void getarmedstatus(u8* get)
{
    *get = armstatus;
}
void getflymode(u8* get)
{
    *get = modestatus;
}


void state_machine_task(void *pvParameters)
{
    u8 tick = 0;										//��ʱ��ʱ�ӣ�ÿ��1ms +1
	u32 lastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 600);				/*50ms������ʱ*/
           
        armstatus_judge(); /*�жϵ������״̬ Ĭ����������DISARMED�� ���յõ�������״̬�ḳֵ��ȫ�־�̬����armstatus*/   
        flightmode_judge();/*�жϷ���ģʽ Ĭ���Ǵ���ģʽ��MODE_ERR�� ���յõ��ķ���ģʽ�ḳֵ��ȫ�־�̬����modestatus*/    
        led_mode(modestatus);  
        
        
        /*�����ʱ����δ����״̬�����Ҳ��Ǵ���ģʽ�л��� ����������-ͣ�л�*/
        if(((tick % 1) == 0) && (armstatus == DISARMED) &&(!armedchangeflag))                  												//����4ms 250Hz
        {    
            tick = 0;
            BEEP = !BEEP;
        }
        tick++;
                      
	}
}
 

