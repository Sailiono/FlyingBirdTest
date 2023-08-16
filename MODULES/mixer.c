#include "mixer.h"
#include "position_control.h"
#include "motor_steering.h"
#include "usart.h"




//保证g_aileron g_elevator范围在 -1 1    g_throttle范围在0-1
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
    amlitude_limit(); //限幅
    // printf("g_aileron=%f\t",g_aileron);
    //printf("g_elevator=%f\r\n",g_elevator);
    motor_control(AILERON,-g_aileron);  //加个负号是为了反向
    motor_control(ELEVATOR,-g_elevator);//加个负号是为了反向
    motor_control(THROTTLE,g_throttle);
    
}

