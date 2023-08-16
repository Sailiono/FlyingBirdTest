#ifndef STEP_MOTOR_CONTROL
#define STEP_MOTOR_CONTROL

#include "sys.h"
#include "timer.h"

#include "stdbool.h"

extern float attack_of_angle_error;
extern float horizontal_error;
extern float vertical_error;

extern bool g_attack_of_angle_isok;
extern bool g_horizontal_isok;
extern bool g_vertical_isok;

void StepMotorControlTask(void *param);

#endif

