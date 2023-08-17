#include "main.h"


float g_aileron;     //-1~1
float g_elevator;    //-1~1
float g_throttle;  //��Χ0-1

static Attitude_S euler_exp;


//ң����ʧ������
void position_control_mode0(void)
{
    g_throttle = 0;   //����Ϊ0
    euler_exp.roll = 0;   //��ת�汣��ƽ��
    euler_exp.pitch = -1; //���ջ���ϵ���������������̵����
}


//���ֶ�ģʽ ֱ�ӽ�172~1811��Χ�Ķ���ֵ���й�һ��ֱ����Ϊ��������
void position_control_mode1(void)
{
	g_throttle = (float)(myrc.throttle-THROTTLE_MIN)/(THROTTLE_MAX-THROTTLE_MIN);
	//������ֵΪ172ʱ�����������ף�aileronΪ-1  ������ֵΪ1811ʱ�����������ף� aileronΪ1    
	g_aileron = (float)(AILERON_MIN-myrc.aileron)*2.0f/(AILERON_MAX-AILERON_MIN)+1.0f;   
    //������ֵΪ172ʱ������������ͣ�elevatorΪ1  ������ֵΪ1811ʱ�������ƣ� elevatorΪ-1
    g_elevator = (float)(ELEVATOR_MIN-myrc.elevator)*2.0f/(ELEVATOR_MAX-ELEVATOR_MIN)+1.0f + 0.3*fabs(g_aileron); 
}


//���ȷ���ģʽ  ��172~1811��Χ�Ķ���ֵ��һ����ת��Ϊ�����Ƕ�
void position_control_mode2(void)
{
	g_throttle = (float)(myrc.throttle-THROTTLE_MIN)/(THROTTLE_MAX-THROTTLE_MIN);  //���ſ���ֵ
    euler_exp.roll = (float)(AILERON_MIN-myrc.aileron)*2.0f/(AILERON_MAX-AILERON_MIN)+1.0f;
	euler_exp.pitch = (float)(ELEVATOR_MIN-myrc.elevator)*2.0f/(ELEVATOR_MAX-ELEVATOR_MIN)+1.0f + 0.3*fabs(euler_exp.roll); 
	
    euler_exp.roll = euler_exp.roll * 50;
    euler_exp.pitch = euler_exp.pitch * 30;
}




void position_control_mode3(void)
{
//	  u8 flight_mode
//	  float height_exp,height_est;
//	  g_position_est();  //�õ������߶�ֵ
//    position_exp_get();//�õ������߶ȵ�ֵ ��ȫ�ֱ���g_vehicle_pos_exp�ṹ�������
//	  height_est=g_vehicle_pos_est_cur.pos_height_est;
//	  height_exp=g_vehicle_pos_exp.pos_height_exp;

	g_throttle = (float)(myrc.throttle-THROTTLE_MIN)/(THROTTLE_MAX-THROTTLE_MIN);  //���ſ���ֵ
	euler_exp.roll = (float)(AILERON_MIN-myrc.aileron)*2.0f/(AILERON_MAX-AILERON_MIN)+1.0f;
    euler_exp.pitch = (float)(ELEVATOR_MIN-myrc.elevator)*2.0f/(ELEVATOR_MAX-ELEVATOR_MIN)+1.0f + 0.3*fabs(euler_exp.roll); 
    
    euler_exp.roll = euler_exp.roll * 50;
    euler_exp.pitch = euler_exp.pitch * 30;
}


void getAttitudeExpData(Attitude_S* get)
{
    get->roll = euler_exp.roll;
	get->pitch = euler_exp.pitch;
	get->yaw = euler_exp.yaw;
}


void Position_control_task(void *pvParameters)
{
    //u8 flymode;  																			
	u32 lastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 5);													/*4ms������ʱ*/
		position_control_mode1();
        g_motor_run();
//            getflymode(&flymode);
//            
//            switch(flymode)
//            {
//            case MODE_ERR:
//                position_control_mode0();
//                break;
//            case MODE_MANUAL:
//                position_control_mode1();  
//                break;
//            case MODE_STABILIZED: 
//                position_control_mode2();
//                break;
//            case MODE_AUTO:
//                position_control_mode3();
//                break;
//            }
		
	}
    
}




