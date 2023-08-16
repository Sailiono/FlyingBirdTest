#include "main.h"


float g_aileron;     //-1~1
float g_elevator;    //-1~1
float g_throttle;  //范围0-1

static Attitude_S euler_exp;


//遥控器失联保护
void position_control_mode0(void)
{
    g_throttle = 0;   //油门为0
    euler_exp.roll = 0;   //滚转舵保持平衡
    euler_exp.pitch = -1; //按照机体系来看俯仰舵往上翘到最高
}


//纯手动模式 直接将172~1811范围的舵量值进行归一化直接作为舵机的输出
void position_control_mode1(void)
{
	g_throttle = (float)(myrc.throttle-THROTTLE_MIN)/(THROTTLE_MAX-THROTTLE_MIN);
	//当舵量值为172时（往左拉到底）aileron为-1  当舵量值为1811时（往右拉到底） aileron为1    
	g_aileron = (float)(AILERON_MIN-myrc.aileron)*2.0f/(AILERON_MAX-AILERON_MIN)+1.0f;   
    //当舵量值为172时（往下拉到最低）elevator为1  当舵量值为1811时（往上推） elevator为-1
    g_elevator = (float)(ELEVATOR_MIN-myrc.elevator)*2.0f/(ELEVATOR_MAX-ELEVATOR_MIN)+1.0f + 0.3*fabs(g_aileron); 
}


//自稳飞行模式  将172~1811范围的舵量值归一化后转化为期望角度
void position_control_mode2(void)
{
	g_throttle = (float)(myrc.throttle-THROTTLE_MIN)/(THROTTLE_MAX-THROTTLE_MIN);  //油门控制值
    euler_exp.roll = (float)(AILERON_MIN-myrc.aileron)*2.0f/(AILERON_MAX-AILERON_MIN)+1.0f;
	euler_exp.pitch = (float)(ELEVATOR_MIN-myrc.elevator)*2.0f/(ELEVATOR_MAX-ELEVATOR_MIN)+1.0f + 0.3*fabs(euler_exp.roll); 
	
    euler_exp.roll = euler_exp.roll * 50;
    euler_exp.pitch = euler_exp.pitch * 30;
}




void position_control_mode3(void)
{
//	  u8 flight_mode
//	  float height_exp,height_est;
//	  g_position_est();  //得到测量高度值
//    position_exp_get();//得到期望高度的值 在全局变量g_vehicle_pos_exp结构体变量中
//	  height_est=g_vehicle_pos_est_cur.pos_height_est;
//	  height_exp=g_vehicle_pos_exp.pos_height_exp;

	g_throttle = (float)(myrc.throttle-THROTTLE_MIN)/(THROTTLE_MAX-THROTTLE_MIN);  //油门控制值
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
		vTaskDelayUntil(&lastWakeTime, 5);													/*4ms周期延时*/
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




