#ifndef __TYPE_H
#define __TYPE_H

#include "main.h"
#include "math.h"
#include "arm_math.h"         //DSP库头文件

#define PI_F 3.14159265358979323846f

/*
函数功能:
	包含通用头文件，并且定义结构体和全局变量
*/

//电机参数结构体
typedef struct
{
	float supply_Udc;      //供电电压
	uint8_t pole;          //电机极对数
	uint16_t Tpwm;         //定时器计数最大值
}Motor_Param;  

typedef struct
{
	float Ua,Ub,Uc;        //ABC三相电压值
	float Ualpha,Ubeta;     
	float Uq,Ud;			
	float virtual_step;    //每次自增的虚拟角度步长
	float virtual_angle;   //当前虚拟角度
	float supply_Udc;        //母线电压
	float vf_k;            //vf系数
	uint16_t Tpwm;         //定时器计数最大值
}SPWM_Param;

typedef struct
{
	float Udc;             //母线电压大小
	uint16_t Ts;         //定时器计数最大值
	float vf_k;            //vf系数
}SVPWM_Param;

#endif
