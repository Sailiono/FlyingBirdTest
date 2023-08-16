#include "mixer.h"
#include "position_control.h"
#include "motor_steering.h"
#include "usart.h"




//��֤g_aileron g_elevator��Χ�� -1 1    g_throttle��Χ��0-1
void amlitude_limit(void)
{
    g_aileron = g_aileron>1?1:g_aileron;
    g_aileron = g_aileron<-1?-1:g_aileron;

    g_elevator = g_elevator>1?1:g_elevator;
    g_elevator = g_elevator<-1?-1:g_elevator;
	
    g_throttle = g_throttle>1?1:g_throttle;
    g_throttle = g_throttle<0?0:g_throttle;	
}


void g_motor_run(void)
{
    amlitude_limit(); //�޷�
    // printf("g_aileron=%f\t",g_aileron);
    //printf("g_elevator=%f\r\n",g_elevator);
    motor_control(AILERON,-g_aileron);  //�Ӹ�������Ϊ�˷���
    motor_control(ELEVATOR,-g_elevator);//�Ӹ�������Ϊ�˷���
    motor_control(THROTTLE,g_throttle);
    
}

