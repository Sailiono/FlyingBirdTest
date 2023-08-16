#ifndef __IST8310_H
#define __IST8310_H
#include "sys.h"
#include "main.h"

#define WHOAMI 0x10


#define IST8310_ADDR 0x0e
#define IST8310_WAI 0x00
#define IST8310_STAT1 0X02
#define IST8310_DATAXL 0X03
#define IST8310_DATAXH 0X04
#define IST8310_DATAYL 0X05
#define IST8310_DATAYH 0X06
#define IST8310_DATAZL 0X07
#define IST8310_DATAZH 0X08
#define IST8310_STAT2 0X09

#define IST8310_CNTL1 0X0A
#define IST8310_CNTL2 0X0B
#define IST8310_STR 0X0C
#define IST8310_TEMPL 0X1C
#define IST8310_TEMPH 0X1D
#define IST8310_AVGCNTL 0X41

/*This register controls the times of average done in the circuit to lower the noise. Higher average times
leads to lower noise
bit5:3 for y sensor data 000 no average;  001 2times; 010 4times(default); 011 8times; 100 16times; 
bit2:0 for x&z sensor data 000 no average;  001 2times; 010 4times(default); 011 8times; 100 16times; 
0x12 is default value
*/
#define IST8310_PDCNTL 0X42

/*0x0c is default value*/




bool IST8310dataisready(void);
u8 IST8310_Init(void);
u8 IST8310_RawData(magnetic_RAWTypeDef *RawDataS);
void Filterx(short* in, short* out);
void Filtery(short* in, short* out);
void Filterz(short* in, short* out);
#endif

