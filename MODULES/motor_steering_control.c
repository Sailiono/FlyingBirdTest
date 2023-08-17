#include "motor_steering_control.h"
#include "motor_steering.h"
#include "usart.h"
#include "hall.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "projdefs.h"
#include "math.h"
#include "step_motor_control.h"
#include "oid_encoder.h"
#include "sbus.h"

#define LIMIT_OF_TIMES 2    //误差连续满足要求的次数
#define LIMIT_OF_TIMES_AVERAGE 2    //停止调整PWM并求平均值的次数

#define LIMIT_OF_FRE_ERROR 0.1f



