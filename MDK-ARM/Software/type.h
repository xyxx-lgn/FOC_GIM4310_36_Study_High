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
	float Rs;              //采样电阻大小
	float Gain;            //运放增益大小
}Motor_Param;  

typedef struct  //所有标志位默认为0
{
	uint8_t Error_Flag;            //为1表示过压，2为欠压，3表示过流
	uint8_t Adc_OffectOver_Flag;   //1代表ADC校准完成
}Motor_Flag;


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

typedef struct
{
	float Ia_Sample,Ib_Sample,Ic_Sample;    //ADC采样原始值，12位ADC：0-4095
	float Ia_offect,Ib_offect,Ic_offect;    //ADC三相电流偏置值，正常是3.3V/2 = 1.65V
	float Ia,Ib,Ic,Udc;
	uint16_t Iadc_offect_counts;            //三相电流偏置值计次
	float IGain;                            //运算放大器和采样电阻组合的电流增益大小：IGain = Rs(采样电阻大小)*运放倍数
}ADCTask_Param;



#endif
