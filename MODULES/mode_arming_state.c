#include "main.h"


static enumarmstatus armstatus = DISARMED;
static enummodestatus modestatus = MODE_ERR;
static bool armedchangeflag = false; 

//暂时就设三个模式 手动 自稳 和 自主（要啥功能可以在自主里面加）
//飞行模式切换判断 并返回最新的飞行模式
void flightmode_judge(void)
{
//	static enummodestatus flight_mode_pre = MODE_ERR;
	enummodestatus flight_mode_cur = MODE_ERR;
	
    /*检测myrc mode通道，根据占空比的大小来对应不同遥控器制定飞行模式*/
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
    
    /*如果当前模式要求全自主飞行，则要检查一下有没有GPS信号，
    如果有，则可以切换，如果没有 则当前模式退回到上一个模式
    如果当前模式非全自主飞行，则通过当前切换 模式转换成功*/
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



/*用于解锁上锁判断
解锁上锁过程中 有5个状态 包括 DISARMED ARMED DISARMED_READY（模式切换） ARMED_READY（模式切换） CANCEL
表示最终的结果 只有2个状态 包括 DISARMED（上锁） ARMED（解锁） 
定义的armstatus_temp armedchangeflag 只是为了更好地根据状态不同来驱动BEEP
最终的结果是  当处于解锁时 蜂鸣器不响  当模式切换过程时 一直响直到切换完成 当处于上锁状态时 每隔1秒响一次
*/
void armstatus_judge(void)
{
    static enumarmstatus armstatus_temp = DISARMED;
    static u32 armedreadytime;
    
    /*解锁 摇杆往右下打，且此时飞控状态为 非解锁状态 才能够进入第一部分
    1、首先打开蜂鸣器 提示危险    同时将状态转换进行时标志位置1  防止蜂鸣器响停切换
    2、判断armstatus_temp，如果它不是 ARMED_READY 状态，则重置解锁准备初始时刻值armedreadytime
                  如果是，则判断当前时刻和解锁准备初始时刻值之差 是否大于2s 大于则解锁 否则继续等
    3、armstatus_temp = ARMED_READY
    */
    /*上锁 摇杆往左下打，且此时飞控状态为 非上锁状态 才能够进入第二部分
    1、首先打开蜂鸣器 提示危险    同时将状态转换进行时标志位置1  防止蜂鸣器响停切换
    2、判断armstatus_temp，如果它不是 DISARMED_READY 状态，则重置上锁准备初始时刻值armedreadytime
                  如果是，则判断当前时刻和上锁准备初始时刻值之差 是否大于2s 大于则上锁 否则继续等
    3、armstatus_temp = DISARMED_READY
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
            TIM_Cmd(TIM5, ENABLE);  //使能电机pwm波生成的定时器
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
            TIM_Cmd(TIM5, DISABLE);  //禁止电机pwm波生成的定时器
            armstatus_temp = DISARMED;
            armstatus = armstatus_temp;
            BEEPOFF;
            armedchangeflag = false;
        }
        armstatus_temp = DISARMED_READY;
    }else if( myrc.throttle > (THROTTLE_MIN+10) || 
    (  myrc.aileron_elevator>(AILERON_ELEVATOR_MIN+10) && myrc.aileron_elevator<(AILERON_ELEVATOR_MAX-10)     )  )
    
    {
        //这个时候模式切换已经停了，蜂鸣器按照规定去响
        armedchangeflag = false;
        /*只有当处于预备阶段 突然退出 才有可能变为取消状态*/
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
    u8 tick = 0;										//定时器时钟，每隔1ms +1
	u32 lastWakeTime = xTaskGetTickCount();

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 600);				/*50ms周期延时*/
           
        armstatus_judge(); /*判断电机锁定状态 默认是上锁（DISARMED） 最终得到的锁定状态会赋值给全局静态变量armstatus*/   
        flightmode_judge();/*判断飞行模式 默认是错误模式（MODE_ERR） 最终得到的飞行模式会赋值给全局静态变量modestatus*/    
        led_mode(modestatus);  
        
        
        /*如果此时处于未解锁状态，而且不是处于模式切换中 蜂鸣器就响-停切换*/
        if(((tick % 1) == 0) && (armstatus == DISARMED) &&(!armedchangeflag))                  												//周期4ms 250Hz
        {    
            tick = 0;
            BEEP = !BEEP;
        }
        tick++;
                      
	}
}
 

