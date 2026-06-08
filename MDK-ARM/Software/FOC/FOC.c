#include "FOC.h"


extern ADCTask_Param adctask_param;          //ADC采样任务参数
extern SPWM_Param spwm_param;         //SPWM生成的过程参数
extern SVPWM_Param svpwm_param;        //SPWM生成的过程参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体
extern Motor_Flag motor_flag;                //电机标志位结构体


extern uint8_t adc_map[3];
extern uint8_t pwm_map[3];
extern int8_t  i_sign[3];

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


//死区补偿平滑符号函数，带一个[-th,th]的死区,避免
//零点附近因为噪声导致补偿符号来回跳
static float sign_smooth(float x,float th)
{
    if (x >= th)  return 1.0f;
    if (x <= -th) return -1.0f;
    return x / th;   // 在[-th, th]内线性过渡
}

//死区补偿一阶低通滤波器，使用滤波后电流进行方向判断，系数k越小滤波效果越强
//y[n] = α * x[n] + (1-α) * y[n-1]= y[n-1] + α * (x[n] - y[n-1])
static float Low_Pass_Fliter_Death(float in,float *last_out,float k)
{
	*last_out += k*(in - *last_out);
	return *last_out;
}


void Set_SPWM(float Uq,float Ud,SPWM_Param* spwm_param)     //耗时3.2us
{
	//帕克逆变化
	arm_inv_park_f32(Ud,Uq,&spwm_param->Ualpha,&spwm_param->Ubeta,encodertask_param.sin_dsp,encodertask_param.cos_dsp);
	
	//克拉克逆变化
  float Ua = spwm_param->Ualpha;
  float Ub = -0.5f * spwm_param->Ualpha + 0.8660254039f * spwm_param->Ubeta;
	float Uc = -0.5f * spwm_param->Ualpha - 0.8660254039f * spwm_param->Ubeta;
	
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
	TIM1->CCR1 = spwm_param->Tpwm-(uint32_t)Ta;
	TIM1->CCR2 = spwm_param->Tpwm-(uint32_t)Tb;
	TIM1->CCR3 = spwm_param->Tpwm-(uint32_t)Tc;
}



//通过三相电流，求得实时的Id
void Getdq(float* Iq,float* Id)
{
	//1、克拉克变化和反帕克变化
	const float _1_sqrt3 = 0.5773502691896258f;  //1/sqrt3
	const float _2_1_sqrt3 = 1.15470053838f;     //2/sqrt3
	
	//(1)克拉克变化（已做过等幅值处理，及乘2/3）Iabc->alhpa、beta
	float Ialpha = adctask_param.Ia;
	float Ibeta = _1_sqrt3 * adctask_param.Ia + _2_1_sqrt3 * adctask_param.Ib;
	
	//(2)帕克逆变化 alhpa、beta->Iq、Id
	arm_park_f32(Ialpha,Ibeta,Id,Iq,encodertask_param.sin_dsp,encodertask_param.cos_dsp);
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
	
	if(motor_flag.Death_Compensation_Enable == 1)  //死区补偿开启
	{
		//(1)先进行参数初始化，并对三相电流进行轻微滤波
		static float ia_filt = 0.0f;    //三相滤波后电流
		static float ib_filt = 0.0f;
		static float ic_filt = 0.0f;
		const float k_lpfd = 0.10f;     //死区补偿电流一阶低通滤波系数k
		const float i_th = 0.05f;      //零点平滑阈值(A)
		const float u_dt = 300.0f*1e-6*Current_ISR_FRE*24.0f; //等效死区补偿电压(V)，算出来约为0.15~0.25直接，也可以自行计算，或者直接给值补偿或者乘个系数放大
		
		ia_filt = Low_Pass_Fliter_Death(adctask_param.Ia, &ia_filt, k_lpfd);
		ib_filt = Low_Pass_Fliter_Death(adctask_param.Ib, &ib_filt, k_lpfd);
		ic_filt = Low_Pass_Fliter_Death(adctask_param.Ic, &ic_filt, k_lpfd);
		
		//(2)三相电流方向判定
		float signa = sign_smooth(ia_filt, i_th);
		float signb = sign_smooth(ib_filt, i_th);
		float signc = sign_smooth(ic_filt, i_th);
		
		//(3)将三相符号映射回alpha-beta补偿矢量,并赋值
		float u_alpha_comp = (2.0f * signa - signb - signc) * 0.3333333333f * u_dt;   //0.333=1/3
		float u_beta_comp  = (signb - signc) * 0.57735026919f * u_dt;    //1/sqrt(3) = 0.57735
		
		Ualpha += u_alpha_comp;
		Ubeta  += u_beta_comp;
	}
	
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
	
	ccrA = Limit(ccrA,300,4190);
	ccrB = Limit(ccrB,300,4190);
	ccrC = Limit(ccrC,300,4190);
	
	//输出到定时器PWM的寄存器通道
//	TIM1->CCR1 = (uint32_t)ccrA;
//	TIM1->CCR2 = (uint32_t)ccrB;
//	TIM1->CCR3 = (uint32_t)ccrC;
	
	float duty_logic[3] = {ccrA, ccrB, ccrC}; // 逻辑A/B/C
	uint32_t ccr_out[4] = {0};

	ccr_out[pwm_map[0]] = (uint32_t)duty_logic[0]; // A
	ccr_out[pwm_map[1]] = (uint32_t)duty_logic[1]; // B
	ccr_out[pwm_map[2]] = (uint32_t)duty_logic[2]; // C

	TIM1->CCR1 = ccr_out[1];         //输出坐标系校准，通过电流环判定
	TIM1->CCR2 = ccr_out[2];
	TIM1->CCR3 = ccr_out[3];
}
