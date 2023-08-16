#include "motor_steering.h"
#include "delay.h"



/*
电机控制:产生PWM波   这里只用到了throttle通道

PB4(TIM3_CH1) <-> roll   PB5(TIM3_CH2) <-> pitch   PB0(TIM3_CH3) <-> throttle

//arr：自动重装值   16位定时器 最大为65535
//psc：时钟预分频数

*/




