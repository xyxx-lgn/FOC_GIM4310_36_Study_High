#ifndef __MT6701_H
#define __MT6701_H

#include "main.h"

#define MT6701_CS_ON()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define MT6701_CS_OFF()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

static void MT6701_Read(uint8_t* data);
uint16_t MT6701_ReadRaw(void);

#endif
