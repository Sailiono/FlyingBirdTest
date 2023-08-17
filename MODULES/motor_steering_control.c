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

#define LIMIT_OF_TIMES 2    //�����������Ҫ��Ĵ���
#define LIMIT_OF_TIMES_AVERAGE 2    //ֹͣ����PWM����ƽ��ֵ�Ĵ���

#define LIMIT_OF_FRE_ERROR 0.1f



