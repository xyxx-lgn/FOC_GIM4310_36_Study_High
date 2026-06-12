#ifndef __PID_H
#define __PID_H

#include "type.h"

void PID_I_Control(PID_Param* pid_i);
void PID_Speed_Control(PID_Param* pid_sp);   //速度环PID运算

#endif
