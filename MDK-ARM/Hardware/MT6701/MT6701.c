#include "MT6701.h"
#include "spi.h"

static const uint8_t tx_buff[2] =  {0xFF,0xFF};

//读取一帧24位完整数据
static void MT6701_Read(uint8_t* data)
{
	MT6701_CS_ON();
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)tx_buff,data,2,1);
	MT6701_CS_OFF();
}

uint16_t MT6701_ReadRaw()
{
	uint8_t data[2];
	uint16_t angle_raw;
		
	MT6701_Read(data);
	angle_raw = data[1] >> 2;
	angle_raw |= data[0] << 6;
	
	return angle_raw;
}

