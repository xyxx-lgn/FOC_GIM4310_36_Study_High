#ifndef __TYPE_H
#define __TYPE_H

#include "main.h"
#include "math.h"
#include "arm_math.h"         //DSP库头文件

#define PI_F 3.14159265358979323846f

#define Limit(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/*
函数功能:
	包含通用头文件，并且定义结构体和全局变量
*/

//电机参数结构体
typedef struct
{
	float supply_Udc;      //供电电压
	uint8_t pole;          //电机极对数
	float motor_gear;      //电机减速比
	float motor_phaseL;    //电机相电感
	float motor_phaseR;    //电机相电阻
	uint16_t Tpwm;         //定时器计数最大值
	float Rs;              //采样电阻大小
	float Gain;            //运放增益大小
}Motor_Param;  

//电机运行标志位结构体
typedef struct  //所有标志位默认为0
{
	uint8_t Error_Flag;            //为1表示过压，2为欠压，3表示过流
	uint8_t Adc_OffectOver_Flag;   //1代表ADC校准完成
	uint8_t Zero_Flag;             //1代表零偏校准完成，默认为1，需要校准时再零偏校准
	uint8_t Econder_Mode;          //编码器模式，1为开环自增角度，2为闭环真实角度
	uint8_t Mode_Select;           //电机运行模式选择，1为SPWM运行，2为SVPWM运行，3为电流环运行，4为速度-电流环运行，5为位置-速度-电流环运行
}Motor_Flag;


typedef struct
{
	float Ua,Ub,Uc;        //ABC三相电压值
	float Ualpha,Ubeta;     
	float supply_Udc;        //母线电压
	uint16_t Tpwm;         //定时器计数最大值
}SPWM_Param;

typedef struct
{
	float Udc;             //母线电压大小
	uint16_t Ts;         //定时器计数最大值
}SVPWM_Param;

typedef struct
{
	float Ia_Sample,Ib_Sample,Ic_Sample;    //ADC采样原始值，12位ADC：0-4095
	float Ia_offect,Ib_offect,Ic_offect;    //ADC三相电流偏置值，正常是3.3V/2 = 1.65V
	float Ia,Ib,Ic,Udc;
	uint16_t Iadc_offect_counts;            //三相电流偏置值计次
	float IGain;                            //运算放大器和采样电阻组合的电流增益大小：IGain = Rs(采样电阻大小)*运放倍数
}ADCTask_Param;

typedef struct
{
	uint16_t Encoder_raw;           //编码器原始数据，0-16383
	uint8_t motordir;               //编码器旋转方向
	float Shaft_Angle;              //机械角度
	float Elect_Angle;              //电角度，电角度=机械角度*极对数
	float Zero_Angle_Sum;           //零偏校准角度和
	float Zero_Angle;               //零偏角度，机械0与电角度0的差值
	uint16_t Zero_counts;           //零偏校准次数
	float Return_Angle;             //真实程序使用的角度值
	float vf_v;                     //vf强拖的系数v
	float vf_k;                     //vf系数
	float virtual_step;             //每次自增的虚拟角度步长
	float sin_dsp,cos_dsp;          //用Return_Angle
}EncoderTask_Param;

typedef struct
{
	float Kp_I,Ki_I;
	float Iq_aim,Id_aim;
	float Iq_now,Id_now;
	float Iqd_Max;
	float Ki_I_SumMax;
	float erro_iq_sum,erro_id_sum;
	float Uq,Ud;
}PID_Param;


#endif
