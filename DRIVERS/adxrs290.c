#include "adxrs290.h"

void Adxrs290_Init(void)
{ 
	
	GPIO_InitTypeDef  GPIO_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		//使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;					//PB12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;				//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//初始化
	SPI2_Init();
	
	ADXRS290_CS = 1;											//片选输出1
	
	Adxrs290_Reg_Write(ADXRS290_REG_POWER_CTL, ADXRS290_MEASUREMENT | ADXRS290_TSM);
	delay_ms(10);
	u8 dev_id;
	Adxrs290_Reg_Read(ADXRS290_REG_DEV_ID, dev_id);
	
	while(dev_id == ADXRS290_DEV_ID){
		Led_Flash(2);
		printf("%.4f",(float)dev_id);
	}
	delay_ms(500);
}


void Adxrs290_Reg_Write(u8 regAddr,u8 data)						//寄存器写入
{
	u8 command = regAddr;
    ADXRS290_CS = 0;                          					//使能器件  
    SPI2_ReadWriteByte(command);
    SPI2_ReadWriteByte(data);
    ADXRS290_CS = 1;
    return;
}

s8 Adxrs290_Reg_Read(u8 regAddr,s8 data)						//寄存器读取
{
    u8 command = ADXRS290_READ_REG(regAddr);
	ADXRS290_CS = 0;                           					//使能器件  
	SPI1_ReadWriteByte(command);
	data = SPI1_ReadWriteByte(0x00);
	ADXRS290_CS = 1;
	return data;
}


int64_t Adxrs290_Multi_Reg_Read(u8 regAddr,u8 bytes_to_read,s16* read_out_data)	//多字节读取
{
	uint8_t in_byte = 0;
    int64_t result = 0;
    u8 command = ADXRS290_READ_REG(regAddr);
	ADXRS290_CS = 0;                          									//使能器件  
	SPI2_ReadWriteByte(command);
	read_out_data[0] |= SPI2_ReadWriteByte(0x00);
	bytes_to_read--;
	read_out_data++;
	 while (bytes_to_read > 0)
    {
        result = result << 8;
        in_byte = SPI2_ReadWriteByte(0x00);
        result |= in_byte;
        bytes_to_read--;
		read_out_data++;
    }
	ADXRS290_CS = 1;
	return 0;
}
