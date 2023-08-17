#ifndef __STEP_MOTOR_DRIVER_H
#define __STEP_MOTOR_DRIVER_H	


#include "sys.h"

#define ATTACK_DIR PCout(2)	        //ATTACK_DIR
#define HORIZONTAL_DIR PCout(3)	    //HORIZONTAL_DIR
#define VERTICAL_DIR PBout(7)       //VERTICAL_DIR


#define ATTACK_STEP_MOTOR 0
#define HORIZONTAL_STEP_MOTOR 1
#define VERTICAL_STEP_MOTOR 2

void StepMotorRunNormal(u8 motor);
void StepMotorRunReverse(u8 motor);
void StepMotorStop(u8 motor);

void step_motor_init(void);

#endif

