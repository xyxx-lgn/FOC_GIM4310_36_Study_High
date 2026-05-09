#include "FOC.h"



extern SPWM_Param spwm_param;         //SPWM生成的过程参数
extern SVPWM_Param svpwm_param;        //SPWM生成的过程参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体

//角度限幅处理,将角度限幅到[-limit,limit)
float Angle_Limit(float input_angle,float limit)   //耗时510ns
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

void Set_SPWM(float Uq,float Ud,SPWM_Param* spwm_param)     //耗时3.2us
{
	//帕克逆变化
	arm_inv_park_f32(Ud,Uq,&spwm_param->Ualpha,&spwm_param->Ubeta,encodertask_param.sin_dsp,encodertask_param.cos_dsp);
	
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


void Set_Svpwm(float Uq,float Ud,float ElectAngle,SVPWM_Param* svpwm_param) //用时3.3us
{
	const float sqrt3     = 1.73205080757f;
	const float _1_sqrt3  = 0.5773502691896258f;
	
	float Ualpha,Ubeta;
	
	//1、先求三角函数值
	float sin_dsp,cos_dsp;
	arm_sin_cos_f32(ElectAngle,&sin_dsp,&cos_dsp);        //角度传参为角度值,求解三角函数值
	
	//2、反帕克变化，将Uq，Ud ->Ualpha，Ubeta
	arm_inv_park_f32(Ud,Uq,&Ualpha,&Ubeta,sin_dsp,cos_dsp);
	
	//3、线性圆限幅，最大相电压 = Udc/√3
	float Umax = svpwm_param->Udc * _1_sqrt3;   //判断电压矢量是否超出Udc/sqrt3
	float Uqd2 = Ualpha*Ualpha+Ubeta*Ubeta;
	if(Uqd2>Umax*Umax)
	{
		float k = Umax / sqrtf(Uqd2);
		Ualpha *= k;
		Ubeta *= k;
	}
	
	//4、时间定标，将电压映射到时间
	//最大相电压 = Udc/√3 ，电压 × 系数 = 时间，系数 = Ts / (Udc/√3) = √3 × Ts / Udc
	float gain = sqrt3 * (float)svpwm_param->Ts / svpwm_param->Udc;     //先标幺化处理
	Ualpha *= gain;
	Ubeta *= gain;
	
	//5、扇区判断
	uint8_t A = (Ubeta>0.0f) ? 1 : 0;
	uint8_t B = ((sqrt3*Ualpha-Ubeta)>0.0f) ? 1 : 0;
	uint8_t C = ((-sqrt3*Ualpha-Ubeta)>0.0f) ? 1 : 0;
	uint8_t N = ((C << 2) | (B << 1) | A);
	
	//6、矢量作用时间T1、T2；X、Y、Z计算与分配
	float X = Ubeta;
	float Y = sqrt3*0.5f*Ualpha + 0.5f*Ubeta;
	float Z = -sqrt3*0.5f*Ualpha + 0.5f*Ubeta;
	
	float T1 = 0.0f,T2 = 0.0f;
	switch(N)//扇区1-6对应编码值N为3、1、5、4、6、2
	{
		case 1: T1 = Z;T2 = Y;break;
		case 2: T1 = Y;T2 = -X;break;
		case 3: T1 = -Z;T2 = X;break;
		case 4: T1 = -X;T2 = Z;break;
		case 5: T1 = X;T2 = -Y;break;
		case 6: T1 = -Y;T2 = -Z;break;
	}
	
	//7、过调制保护，确保T1+T2 <= Ts
	if((T1+T2)>svpwm_param->Ts)
	{
		float Ts_k = (float)svpwm_param->Ts / (T1+T2);
		T1 *= Ts_k;
		T2 *= Ts_k;
	}
	
	//8、计算并映射三通道Ta、Tb、Tc三通道比较值
	float Ta = (svpwm_param->Ts - T1 - T2)*0.25f;  //T0 = Ts - T1 - T2
	float Tb = Ta + 0.5f*T1;
	float Tc = Tb + 0.5f*T2;
	
	float ccrA,ccrB,ccrC;
	switch(N)//扇区1-6对应编码值N为3、1、5、4、6、2
	{
		case 1: ccrA = Tb;ccrB = Ta;ccrC = Tc;break;
		case 2: ccrA = Ta;ccrB = Tc;ccrC = Tb;break;
		case 3: ccrA = Ta;ccrB = Tb;ccrC = Tc;break;
		case 4: ccrA = Tc;ccrB = Tb;ccrC = Ta;break;
		case 5: ccrA = Tc;ccrB = Ta;ccrC = Tb;break;
		case 6: ccrA = Tb;ccrB = Tc;ccrC = Ta;break;
		default:
			ccrA=ccrB=ccrC = svpwm_param->Ts/4;   //2100/4200，等于50%占空比
			break;
	}
	
	//输出到定时器PWM的寄存器通道
	TIM1->CCR1 = (uint32_t)ccrA;
	TIM1->CCR2 = (uint32_t)ccrB;
	TIM1->CCR3 = (uint32_t)ccrC;
}
