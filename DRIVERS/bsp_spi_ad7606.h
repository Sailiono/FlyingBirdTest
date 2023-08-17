/*
*********************************************************************************************************
*	                                  
*	ģ������ : AD7606����ģ�� 
*	�ļ����� : bsp_ad7606.h
* ��    �� : WZH
*********************************************************************************************************
*/
#include "main.h" 
#ifndef __BSP_AD7606_H
#define __BSP_AD7606_H

/* ÿ������2�ֽڣ��ɼ�ͨ�� */
#define CH_NUM			2				/* �ɼ�2ͨ�� */
#define FIFO_SIZE		8*1024*2		/* ��С��Ҫ����48K (CPU�ڲ�RAM ֻ��64K) */



/* AD���ݲɼ������� */
typedef struct
{
	uint16_t usRead;
	uint16_t usWrite;
	uint16_t usCount;
	uint16_t usBuf[FIFO_SIZE];
}FIFO_T;

/* ���ⲿ���õĺ������� */
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


