/*
*********************************************************************************************************
*	                                  
*	模块名称 : AD7606驱动模块 
*	文件名称 : bsp_ad7606.h
* 编    辑 : WZH
*********************************************************************************************************
*/
#include "main.h" 
#ifndef __BSP_AD7606_H
#define __BSP_AD7606_H

/* 每个样本2字节，采集通道 */
#define CH_NUM			2				/* 采集2通道 */
#define FIFO_SIZE		8*1024*2		/* 大小不要超过48K (CPU内部RAM 只有64K) */



/* AD数据采集缓冲区 */
typedef struct
{
	uint16_t usRead;
	uint16_t usWrite;
	uint16_t usCount;
	uint16_t usBuf[FIFO_SIZE];
}FIFO_T;

/* 供外部调用的函数声明 */
void ad7606_Reset(void);
void ad7606_SetOS(uint8_t _ucMode);
void bsp_SET_TIM4_FREQ(uint32_t _ulFreq);
void bsp_InitAD7606(void);
void ad7606_StartRecord(uint32_t _ulFreq);
void ad7606_StopRecord(void);
uint8_t GetAdcFormFifo(uint16_t *_usReadAdc);
void bsp_TIM4_Configuration(void);
extern FIFO_T wzh;
void AD7606_read_data(s16 * DB_data) ;
void ad7606_StartConv(void);
void ad_task(void *param);
#endif


