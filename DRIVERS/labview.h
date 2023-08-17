#ifndef _LABVIEW_H
#define _LABVIEW_H

#include "sys.h"

#define GPS_GNGGA_MESSAGE_MAX 50





void usart3_send_char(u8 temp);

void uart3_send_buff(u8* buf,u32 len);          
void LabviewSeriesInit(void);
void labview_task(void *param);


#endif


