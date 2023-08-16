#ifndef __ALTITUDE_CONTROL_H
#define __ALTITUDE_CONTROL_H
#include "sys.h"
#include "position_control.h"
//////////////////////////////////////////////////////////////////////////////////∫Í∂®“Â
#define PITCH_SETPOINT_OFFSET_NORMAL 8.0f
#define PITCH_SETPOINT_OFFSET_TAKEOFF 35.0f
#define ROLL_SETPOINT_OFFSET 0.0f
#define PITCH_DP 0.025f
#define ROLL_DP 0.02f

void Attitude_control_task(void *pvParameters);

#endif

