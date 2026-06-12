#ifndef __MOTORID_H
#define __MOTORID_H

#include "type.h"

void RsID_Task(MotorID_Param* rsparam);      //相电阻辨析任务
void LdqID_Task(MotorID_Param* ldqparam);    //相电感辨析任务

void ScanFrequence_Init(ScanFre_Param* sf_p);
void ScanFrequence_Start(ScanFre_Param* sf_p,float iq_bias,float iq_amp,float fre_hz,float lock_angle);
void ScanFrequence_Task(ScanFre_Param* sf_p);
void ScanFrequence_PrintBuff(void);
int generate_bode_frequencies(float start_freq_Hz,float end_freq_Hz,int num_points,float *freq_array_Hz);
void print_and_verify_frequencies(const float *freq_array_Hz, int num_points);
void SpeedID_Kw_Task(PID_Param* p);
#endif
