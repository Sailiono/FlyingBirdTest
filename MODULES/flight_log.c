//////////////////////////////////////////////////////////////////////////////////	 
//������ư�-LJT-v1
//flight_log
//ljt
//��������:2020/2/28
//�汾��V1.0
//All rights reserved		
//
//////////////////////////////////////////////////////////////////////////////////
 
 

#include "stdbool.h"
#include "flight_log.h"
#include "hall.h"
#include "w25qxx.h"
#include "position_estimate.h"
#include "sbus.h"
#include "adc.h"
#include "bsp_spi_ad7606.h"


extern float frequency_desired;
extern float throttle_out;     //???????,0~1

