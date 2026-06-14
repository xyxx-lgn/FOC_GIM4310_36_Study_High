#ifndef __PID_H
#define __PID_H

#include "type.h"

void PID_I_Control(PID_Param* pid_i);
void PID_Speed_Control(PID_Param* pid_sp);   //速度环PID运算
//DOB扰动观测器
void Speed_DOB_Init(SpeedDOB_Param* dob, float kw, float ts, float fo_hz, float zeta, float iq_ff_limit);
void Speed_DOB_Reset(SpeedDOB_Param* dob, float omega_init);
void Speed_DOB_Task(SpeedDOB_Param* dob, float omega_meas);

#endif
