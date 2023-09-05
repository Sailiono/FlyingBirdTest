#ifndef __MAIN_H
#define __MAIN_H
#if defined(__CC_ARM) 
	#pragma anon_unions
#endif




//#include "motor_steering.h"
//#include "motor_steering_control.h"

#define THROTTLE_MIN 352.0f
#define THROTTLE_MAX 1696.0f
#define AILERON_MIN 352.0f
#define AILERON_MAX 1696.0f
#define ELEVATOR_MIN 352.0f
#define ELEVATOR_MAX 1696.0f

#define AILERON_ELEVATOR_MAX 1696.0f
#define AILERON_ELEVATOR_MIN 352.0f
extern float wzhspeed;
extern float wzhfre;
#endif


