#include "FOC.h"

#define _1_sqrt3  0.5773502691896258f

extern SPWM_Param spwm_param;         //SPWM生成的过程参数
extern SVPWM_Param svpwm_param;        //SPWM生成的过程参数

//角度限幅处理,将角度限幅到[-limit,limit)
static float Angle_Limit(float input_angle,float limit)   //耗时510ns
{
	float _2_limit = 2.0f*limit;
	//常见小越界，通过加减解决
	if(input_angle < limit && input_angle>=-limit) 
		return input_angle;
	if(input_angle>=limit && input_angle<limit+_2_limit) 
		return input_angle - _2_limit;
	if(input_angle<-limit && input_angle>=-limit-_2_limit)
		return input_angle + _2_limit;
	
	//输入值大越界,用floorf函数对浮点数向下取整，如5.23f->5.0f -3.7f->-4.0f
	float k = floorf((input_angle+limit)/_2_limit);
	input_angle -= k*_2_limit;
	
	if(input_angle>=limit) 
		input_angle -= _2_limit;
	else if(input_angle<-limit)
		input_angle += _2_limit;
	
	return input_angle;
}

void Set_SPWM(SPWM_Param* spwm_param)     //耗时3.2us
{
	float sin_dsp,cos_dsp;
	//VF拖动自增补偿计算
	spwm_param->virtual_step = spwm_param->Uq*spwm_param->vf_k;  //VF强托系数，即step = k*Uq
	
	//虚拟角度自增  20K的执行频率
	spwm_param->virtual_angle += spwm_param->virtual_step;    //14极对数，360/20000*14 = 0.252
	spwm_param->virtual_angle = Angle_Limit(spwm_param->virtual_angle,180.0f);
	
	//注意arm_sin_cos_f32这个函数，角度传参是角度值  ，arm_sin_f32和arm_cos_f32传参是弧度值
	arm_sin_cos_f32(spwm_param->virtual_angle,&sin_dsp,&cos_dsp);        //DSP库计算三角，在电流环帕克变换处计算一次即可
	
	//帕克逆变化
	arm_inv_park_f32(spwm_param->Ud,spwm_param->Uq,&spwm_param->Ualpha,&spwm_param->Ubeta,sin_dsp,cos_dsp);
	
	//克拉克逆变化
    float Ua = spwm_param->Ualpha;
    float Ub = -0.5f * spwm_param->Ualpha + 0.8660254039f * spwm_param->Ubeta;
	float Uc = -0.5f * spwm_param->Ualpha +- 0.8660254039f * spwm_param->Ubeta;
	
	float half_Udc = spwm_param->supply_Udc*0.5f;
	
	//对求解得到的三相电压值往上平移最大电压的一半，确保无负电压
	spwm_param->Ua = Ua+half_Udc;
	spwm_param->Ub = Ub+half_Udc;
	spwm_param->Uc = Uc+half_Udc;
	
	//计算三相PWM寄存器值
	float Ta = (spwm_param->Ua/spwm_param->supply_Udc)*(float)spwm_param->Tpwm;
	float Tb = (spwm_param->Ub/spwm_param->supply_Udc)*(float)spwm_param->Tpwm;
	float Tc = (spwm_param->Uc/spwm_param->supply_Udc)*(float)spwm_param->Tpwm;
	
	//输出到定时器PWM的寄存器通道
	TIM1->CCR1 = (uint32_t)Ta;
	TIM1->CCR2 = (uint32_t)Tb;
	TIM1->CCR3 = (uint32_t)Tc;
}


void Set_Svpwm(float Uq,float Ud,float ElectAngle,SVPWM_Param* svpwm_param)
{
	float sin_dsp,cos_dsp;
	arm_sin_cos_f32(ElectAngle,&sin_dsp,&cos_dsp);        //角度传参为角度值,求解三角函数值
	
	float gain = svpwm_param->Udc*_1_sqrt3;                              //先标幺化处理
	float Umax = svpwm_param->Udc * _1_sqrt3;   //判断电压矢量是否超出Udc/sqrt3
	
	
	float Ualpha,Ubeta;
	//反帕克变化，将Uq，Ud ->Ualpha，Ubeta
	arm_inv_park_f32(Ud,Uq,&Ualpha,&Ubeta,sin_dsp,cos_dsp);
	
	
}
