#ifndef __MOTORID_H
#define __MOTORID_H

#include "type.h"

void RsID_Task(MotorID_Param* rsparam);      //相电阻辨析任务
void LdqID_Task(MotorID_Param* ldqparam);    //相电感辨析任务
#endif
