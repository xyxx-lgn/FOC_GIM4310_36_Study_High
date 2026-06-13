#ifndef __USERMAIN_H
#define __USERMAIN_H

#include "type.h"


void Data_Init(void);
void ADC_Task(ADCTask_Param* adctask_param,uint16_t* adc_raw);


void Encoder_NLCal_PrintTask(EncoderNLCal_Param* p);

void usermain(void);


#endif
