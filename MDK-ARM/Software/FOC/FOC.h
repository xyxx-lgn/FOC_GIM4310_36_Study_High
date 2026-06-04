#ifndef __FOC_H
#define __FOC_H

#include "type.h"

float Angle_Limit(float input_angle,float limit);
void Set_SPWM(float Uq,float Ud,SPWM_Param* spwm_param);
void Getdq(float* Iq,float* Id);
void Set_Svpwm(float Uq,float Ud,float ElectAngle,SVPWM_Param* svpwm_param);

#endif
