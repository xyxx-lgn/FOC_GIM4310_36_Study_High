#include "usermain.h"
#include "MT6701.h"
#include "FOC.h"
#include "MT6701.h"
#include "PID.h"
#include "MotorID.h"






// 逻辑相序索引: 0->A, 1->B, 2->C
// ADC原始通道(JDR1,JDR2,JDR3) -> 逻辑相(A,B,C)
// 例如 {0,1,2} 表示 A=JDR1, B=JDR2, C=JDR3
uint8_t adc_map[3] = {2, 1, 0};        //电流反馈坐标系

// 逻辑相(A,B,C) -> PWM硬件通道(CCR1,CCR2,CCR3)
// 例如 {1,2,3} 表示 A->CCR1, B->CCR2, C->CCR3
uint8_t pwm_map[3] = {1, 2, 3};        //输出坐标系

// 每相电流符号修正（+1或-1）
int8_t i_sign[3] = {-1, -1, -1};




extern TIM_HandleTypeDef htim1;

extern Motor_Param motor_param;              //电机参数结构体
extern SPWM_Param spwm_param;                //SPWM生成的过程参数
extern ADCTask_Param adctask_param;          //ADC采样任务参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体
extern Motor_Flag motor_flag;                //电机标志位结构体
extern SVPWM_Param svpwm_param;              //SPWM生成的过程参数
extern PID_Param pid_param;                  //PID参数结构体
extern MotorID_Param rsid_param;                //电阻辨析结构体参数
extern ScanFre_Sample scanfre_buff[1200];       //扫频法数据存储区
extern ScanFre_Param scanfre_param;             //扫频法测带宽结构体
extern EncoderNLCal_Param enc_nlcal_param;      //编码器非线性化校准结构体

extern uint16_t ADC1InjectDate[4];     //注入组采样数组

void Data_Init()
{
	//电机参数结构体
	motor_param.supply_Udc = 24.0f;                   //供电电压
	motor_param.pole = 14;                            //电机极对数
	motor_param.motor_gear=36.0f;         	          //电机减速比
	motor_param.motor_phaseL=0.000144f;                //电机相电感   0.000405f实测       0.00075f是电机厂家参数
	motor_param.motor_phaseR=0.897f;                   //电机相电阻    0.897f实测          1.89f是电机厂家参数
	motor_param.motor_Ld = motor_param.motor_phaseL;  //电机d轴电感
	motor_param.motor_Lq = motor_param.motor_phaseL;  //电机q轴电感
	motor_param.motor_filed_link = 0.0f;              //永磁体磁链大小，还未测量
	motor_param.Tpwm = 8400;                          //定时器计数最大值
	motor_param.Rs = 0.01;                            //采样电阻值大小
	motor_param.Gain = 50;                            //运算放大器增益
	motor_param.I_Width = 600.0f;                     //电机电流环带宽大小
	motor_param.Kw = 2400.0f;                         //Kw = Kt/J,单位：rad/s2/A，用于速度环PI整定使用
	motor_param.Speed_Width = 8.0f;                  //速度环带宽大小
	
	//电机运行标志位结构体
	motor_flag.Zero_Flag = 1;                          //默认零偏校准标志位为1，避免每次启动都校准
	motor_flag.Econder_Mode = 2;                       //编码器模式，1为开环自增角度，2为闭环真实角度
	motor_flag.Mode_Select = 3; 
	motor_flag.v_limit_mode = V_LIMIT_VECTOR;          //3种电压限幅，分别为q轴优先，d轴优先，等比例限幅
	motor_flag.dec_mode = FOC_CC_DECOUPLING_DISABLED;  //电流环前馈补偿选择，分别为不补偿，dq轴补偿，反电动势补偿，都补偿
	motor_flag.Death_Compensation_Enable = 0;          //死区补偿开始，1为开启，0为关闭，在GIM4310-36这个板子和电机上使用时发现效果不佳，因此关闭死区补偿
	
	//SPWM生成的过程参数
	spwm_param.supply_Udc = motor_param.supply_Udc;      //SPWM电压抬升
	spwm_param.Tpwm = motor_param.Tpwm/2;                  //定时器计数最大值
	
	//SVPWM生成的过程参数
	svpwm_param.Ts = motor_param.Tpwm;                  //SVPWM里面的时间系数Ts
	svpwm_param.Udc = motor_param.supply_Udc;           //SVPWM母线电压
	
	//ADC采样任务的参数
	adctask_param.Ia_offect = 1.65f;
	adctask_param.Ib_offect = 1.65f;
	adctask_param.Ic_offect = 1.65f;
	adctask_param.IGain = motor_param.Rs*motor_param.Gain;
	
	//Encoder编码器任务参数
	encodertask_param.Encoder_Max = 16384;    //编码器最大值，14位磁编最大值16384
	encodertask_param.motordir = 0;
	encodertask_param.Zero_Angle = 14.6338f;                    //14.7656f   1.86768f  6.9873f   
	encodertask_param.virtual_step = 0.252f;                    //自增步长，14极对数，360/20000*14 = 0.252相当于1圈每秒
	encodertask_param.vf_v = 1.0f;                              //VF强托系数里面的V
	encodertask_param.vf_k = 0.252f;                            //VF强托系数，即Uq = k*F; 得到step = k*Uq
	//PLL锁相环参数
	encodertask_param.pll_theta_hat = 0.0f;
	encodertask_param.pll_omega_hat = 0.0f;
	encodertask_param.pll_zeta = 1.2f;
	encodertask_param.pll_wn = 300.0f;
	encodertask_param.pll_kp = 2.0f * encodertask_param.pll_zeta * encodertask_param.pll_wn;    //pll_kp = 2 * zeta * wn
	encodertask_param.pll_ki = encodertask_param.pll_wn * encodertask_param.pll_wn;             //pll_ki = wn^2
	
	//PID参数
	//电流环参数
	pid_param.Iqd_Max = 1.0f; //电流限幅3.3A*0.9 = 2.97，大于这个值会削顶（1.65/50/0.01 = 3.3A）
	pid_param.Ki_I_SumMax = 0.6f*motor_param.supply_Udc/sqrtf(3);   //最大矢量圆限幅，给90%控制裕度 
	
	pid_param.Kp_I = 2.0f*PI_F*motor_param.I_Width*motor_param.motor_phaseL;            //2*PI*f_bw*L
	pid_param.Ki_I = 2.0f*PI_F*motor_param.I_Width*motor_param.motor_phaseR/Current_ISR_FRE;            //2*PI*f_bw*R*dt

	motor_flag.pid_param_flag = 1;   //电流环PI参数变更标志位
	
	pid_param.Iq_aim = 0.0f;
	pid_param.Id_aim = 0.0f;
		
	
	
	
	//速度环参数
	/*
		速度环PI参数整定式子：
			Kp = 2ζωn / Kω其中，ωn = 2π*f_bw_speed ，阻尼比ζ取0.707
			Ki = ωn^2 / Kω ，Ki离散化还要乘执行周期Ts_v,即速度环执行周期，Ki = ωn^2 / Kω*Ts_v
	*/
	pid_param.Kp_S =  2.0f*0.707f*2.0f*PI_F*motor_param.Speed_Width/motor_param.Kw;  
	pid_param.Ki_S =  (2.0f*PI_F*motor_param.Speed_Width)*(2.0f*PI_F*motor_param.Speed_Width)/motor_param.Kw*1.0f/Speed_ISR_FRE; 
	
	pid_param.Motor_Speed_aim = 0.0f;
	/*速度单位换算
		1 rpm = 0.10471975511965977 rad/s
		1 rad/s = 9.549296585513721 rpm
		1 rad/s = 6°/s
	*/
	pid_param.Speed_Max = 150.7964f;  //约为40rpm，4.18879 rad/s为输出轴限幅最大值  ,电机轴限幅值1440rpm，150.7964f rad/s
	pid_param.Speed_KISumMax = 1.0f; 
	
	pid_param.Speed_lpf_k = 0.467f;   //速度滤波系数，先偏稳一点 k = 1 - e^{-2*pi*f_c*T_s}，其中f_c为滤波器截止频率，一般取速度环得5~10倍，我取100，T_s为速度环执行频率
	
	pid_param.Speed_Div = 20;       //分频系数，电流环执行频率/Speed_Div = 速度环执行频率
	pid_param._1_Ts = Speed_ISR_FRE;   //速度环周期倒数（1/s），速度计算进行乘法
	
	//电机参数辨析参数
	rsid_param.Ud_Set = 1.0f;            //相电阻辨析Ud电压设置（1.0-2.0之间合适，过小会有采样以及逆变器的误差干扰）
	rsid_param.Rs_result = 0.938f;       //默认没进行测量时，使用之前测过的一个相电阻数据值
	rsid_param.RsID_Start = 0;           //将其设置为0，且Mode_Select == 6时再次进行相电阻辨析
	
	//电感参数辨析
	rsid_param.Udq_inject = 1.0f;
	rsid_param.frequency_inject = 400;   //电感辨析注入频率，可以更改，默认400，过小，电机电阻压降影响大，过大，采样点过少
	rsid_param.Ldq_select = 1;           //dq电感辨析选择标志位，默认为0辨析q轴电感，1辨析d轴电感
	rsid_param.Ldq_half_cnt = 0;         //计算出半个周期内当前计次值是多少
	rsid_param.half_index = 0;           ///半个周期计次数，看完成了多少半个周期
	rsid_param.LdqID_Start = 0;          //   Ld 0.0004164f      Lq 0.0003968f   取Lq=Ld=0.000405f
	
	//扫频法测电流环带宽
	ScanFrequence_Init(&scanfre_param);          //扫频法初始化参数
	scanfre_param.scanfre_buff = scanfre_buff;   //扫频法过程数据存储区，就是通过这里面的4类数据发给matlab进行频域分析
	scanfre_param.frequence_hz = 100;            //默认设置扫频当前频率，可根据扫频处函数去设定起始和最高频率，并中间对数插值处理
	scanfre_param.start_flag=0;                  //scanfre_param.start_flag==0 && scanfre_param.done_flag==0 && scanfre_param.scanfre_state == SCANFRE_IDLE
	scanfre_param.done_flag=0;                   //即上面三种情况同时成立下重新进行一次进行扫频，默认完成一次扫频done_flag置1
	scanfre_param.scanfre_state = SCANFRE_IDLE;  //当scanfre_state不是SCANFRE_IDLE意味着扫频进行中
	
	// 编码器非线性连续扫描参数
	enc_nlcal_param.start_flag = 0;
	enc_nlcal_param.done_flag = 0;
	enc_nlcal_param.dir = 1;                 // 默认正转

	enc_nlcal_param.div_cnt = 0;
	enc_nlcal_param.div_num = 100;           // 20kHz / 100 = 200Hz -> 5ms采一点

	enc_nlcal_param.ud = 4.0f;               // 学长方案
	enc_nlcal_param.step_deg = 0.02f * 57.2957795f;
	enc_nlcal_param.cmd_deg = 0.0f;
	enc_nlcal_param.mech_tick_sum = 0.0f;

	enc_nlcal_param.sample_idx = 0;
	enc_nlcal_param.fifo_wr = 0;
	enc_nlcal_param.fifo_rd = 0;
}


void ADC_Task(ADCTask_Param* adctask_param,uint16_t* adc_raw)
{
	//1、先得到母线电压  电压系数：0.0088623046875f = 1/4096*3.3*(20+2)/2
	adctask_param->Udc = (float)adc_raw[3]*0.0088623f; 
	
	//2、过压、欠压判断
	if(adctask_param->Udc>motor_param.supply_Udc+2.0f)       //如果母线电压大于26V报警过压
	{
		motor_flag.Error_Flag = 1;
	}
	else if(adctask_param->Udc<motor_param.supply_Udc-2.0f) //如果母线电压低于22V报警欠压
	{
		motor_flag.Error_Flag = 2;
	}
	else if(motor_flag.Error_Flag == 1 || motor_flag.Error_Flag == 2)
	{
		motor_flag.Error_Flag = 0;
//		motor_flag.Adc_OffectOver_Flag = 0; //如果电压报警错误一次就重新进行电流校准，避免错误
	}
	
	svpwm_param.Udc = adctask_param->Udc;
	pid_param.Ki_I_SumMax = 0.6f*adctask_param->Udc*0.57735026919f;
	pid_param.Uout_Max = 0.9f*adctask_param->Udc*0.57735026919f;
	
	//3、采集三相电流数据
	adctask_param->Ia_Sample = (float)adc_raw[0]*3.3f/4096.0f;
	adctask_param->Ib_Sample = (float)adc_raw[1]*3.3f/4096.0f;
	adctask_param->Ic_Sample = (float)adc_raw[2]*3.3f/4096.0f;
	
	//4、进行三相电流校准(母线电压正常时才进行电流计算，只要母线电压异常就重新校准)
	if(motor_flag.Adc_OffectOver_Flag == 0 && motor_flag.Error_Flag != 1 && motor_flag.Error_Flag != 2)
	{
		//进行20k次校准，用时1.0s
		if(adctask_param->Iadc_offect_counts<20000)
		{
			if(fabs(adctask_param->Ia_Sample-1.65f)<0.3f)
				adctask_param->Ia_offect = adctask_param->Ia_offect*0.95f + adctask_param->Ia_Sample*0.05f;
			if(fabs(adctask_param->Ib_Sample-1.65f)<0.3f)
				adctask_param->Ib_offect = adctask_param->Ib_offect*0.95f + adctask_param->Ib_Sample*0.05f;
			if(fabs(adctask_param->Ic_Sample-1.65f)<0.3f)
				adctask_param->Ic_offect = adctask_param->Ic_offect*0.95f + adctask_param->Ic_Sample*0.05f;
		}
		else
		{ 
			if (fabs(adctask_param->Ia_offect-1.65f)<0.3f && fabs(adctask_param->Ib_offect-1.65f)<0.3f && fabs(adctask_param->Ic_offect-1.65f)<0.3f)
			{
				motor_flag.Error_Flag = 0; 
				//确保校准后的有效性，如果离1.65过远，认为有问题
				adctask_param->Iadc_offect_counts = 0;
				motor_flag.Adc_OffectOver_Flag = 1;
			}
			else  //如果电流校准值异常，则一直校准直到正常
			{
				motor_flag.Error_Flag = 4;                  //ADC三通道电流校准有问题
				adctask_param->Iadc_offect_counts = 0;
				motor_flag.Adc_OffectOver_Flag = 0;
			}
		}
		adctask_param->Iadc_offect_counts++;
	}
	
	//5、计算真实三相电流值(运行在此处以后只会出报错Error_Flag = 3，即过流)
	if(motor_flag.Adc_OffectOver_Flag == 1)
	{
//		adctask_param->Ia = (adctask_param->Ia_Sample - adctask_param->Ia_offect)/adctask_param->IGain;
//		adctask_param->Ib = (adctask_param->Ib_Sample - adctask_param->Ib_offect)/adctask_param->IGain;
//		adctask_param->Ic = (adctask_param->Ic_Sample - adctask_param->Ic_offect)/adctask_param->IGain;
		
		float i_raw[3];
		i_raw[0] = (adctask_param->Ia_Sample - adctask_param->Ia_offect)/adctask_param->IGain; // JDR1  先拿到实际采集到的3相电流反馈
		i_raw[1] = (adctask_param->Ib_Sample - adctask_param->Ib_offect)/adctask_param->IGain; // JDR2
		i_raw[2] = (adctask_param->Ic_Sample - adctask_param->Ic_offect)/adctask_param->IGain; // JDR3

		adctask_param->Ia = i_sign[0] * i_raw[adc_map[0]];              //对到真实反馈坐标系进行检查，确保电流环检测时极性正常
		adctask_param->Ib = i_sign[1] * i_raw[adc_map[1]]; 
		adctask_param->Ic = i_sign[2] * i_raw[adc_map[2]];
	}
	
	//6、进行是否过流判断
	if(fabsf(adctask_param->Ia)>3.3f || fabsf(adctask_param->Ib)>3.3f || fabsf(adctask_param->Ic)>3.3f)
		motor_flag.Error_Flag = 3; 
	else if(motor_flag.Error_Flag==0 || motor_flag.Error_Flag==3)
		motor_flag.Error_Flag = 0;
		
	
}



//编码器采样任务
void Encoder_Task(EncoderTask_Param* encodertask_param)
{
	//1、首先读读取MT6701原始值(用时6.8us)
	uint16_t encoder_raw = MT6701_ReadRaw();    
	if(encodertask_param->motordir == 0) 
		encodertask_param->Encoder_raw = encoder_raw;
	else
		encodertask_param->Encoder_raw = 16384-encoder_raw;
		

	//2、转化角度值，将编码值转化为机械角度、电角度
		////0.02197265625 = 1/16384*360
	encodertask_param->Shaft_Angle = (float)encodertask_param->Encoder_raw * 0.02197265625f - encodertask_param->Zero_Angle; 
	encodertask_param->Elect_Angle = Angle_Limit(encodertask_param->Shaft_Angle * (float)motor_param.pole,180.0f);
	
	//3、零偏校准程序
	if(motor_flag.Zero_Flag == 0 && motor_flag.Adc_OffectOver_Flag == 1)
	{
		Set_Svpwm(0,1.0f,0,&svpwm_param);
		if(encodertask_param->Zero_counts>=20000)
		{
			if(encodertask_param->Zero_counts<21000)
			{
				encodertask_param->Zero_n ++;
				encodertask_param->Zero_Angle_Sum += encodertask_param->Shaft_Angle;
			}
			else
			{
				motor_flag.Zero_Flag = 1;
				encodertask_param->Zero_Angle = encodertask_param->Zero_Angle_Sum/(float)encodertask_param->Zero_n;
				encodertask_param->Zero_Angle_Sum = 0;
				encodertask_param->Zero_counts = 0;
				encodertask_param->Zero_n = 0;
			}
		}			
		encodertask_param->Zero_counts++;
	}
	
	//4、返回角度选择
	if(motor_flag.Econder_Mode == 1)  //开环自增角度
	{
		//VF拖动自增补偿计算
		encodertask_param->virtual_step = encodertask_param->vf_k*encodertask_param->vf_v;  //VF强托系数，即step = k*Uq
		//虚拟角度自增  20K的执行频率
		encodertask_param->Return_Angle = Angle_Limit(encodertask_param->Return_Angle+encodertask_param->virtual_step,180.0f);    //14极对数，360/20000*14 = 0.252
		encodertask_param->Return_Rads = encodertask_param->Return_Angle*0.01745329f;  //将角度值转化为弧度值，1/180*PI = 0.01745329f
	}
	else if(motor_flag.Econder_Mode == 2)  //闭环角度处理
	{
		encodertask_param->Return_Angle = encodertask_param->Elect_Angle;
	}
	else if(motor_flag.Econder_Mode == 3)  //角度定点模式
	{
//		encodertask_param->Return_Angle = 0.0f;
	}
	
	//5、计算正余弦值
	//注意arm_sin_cos_f32这个函数，角度传参是角度值  ，arm_sin_f32和arm_cos_f32传参是弧度值
	arm_sin_cos_f32(encodertask_param->Return_Angle,&encodertask_param->sin_dsp,&encodertask_param->cos_dsp);  //DSP库计算三角，在电流环帕克变换处计算一次即可
}

//速度计算任务，差分法，运行频率1KHz，和PLL测速2选1
void Speed_Measure_Task(PID_Param* sp,EncoderTask_Param* encodertask_param)
{
	sp->Speed_cnt++;
	if(sp->Speed_cnt<sp->Speed_Div)
		return;

	sp->Speed_cnt = 0;
	
	float angle_now = encodertask_param->Shaft_Angle;  //deg
	
	float dtheta_deg = angle_now - sp->Shaft_Angle_Last; 
	
	//处理机械角跨越 0/360 度
	if (dtheta_deg > 180.0f)
			dtheta_deg -= 360.0f;
	else if (dtheta_deg < -180.0f)
			dtheta_deg += 360.0f;
		
	sp->Shaft_Angle_Last = 	angle_now;
	
	//电机轴机械角速度 rad/s  0.01745329 = PI/180.0f
	sp->Motor_Speed_now = dtheta_deg * 0.01745329f * sp->_1_Ts;
	//输出轴机械角速度 rad/s
	sp->Speed_now = sp->Motor_Speed_now / motor_param.motor_gear;
	
	//一阶低通滤波
	sp->Motor_Speed_filt_now += sp->Speed_lpf_k * (sp->Motor_Speed_now - sp->Motor_Speed_filt_now);
	sp->Speed_filt += sp->Speed_lpf_k * (sp->Speed_now - sp->Speed_filt);
}

//PLL把角度误差限制到 [-pi, pi]
static float wrap_pm_pi(float x)
{
    while (x > PI_F)  x -= PI2_F;
    while (x < -PI_F) x += PI2_F;
    return x;
}
//PLL把估计角度限制到 [0, 2pi)
static float wrap_0_2pi(float x)
{
    while (x >= PI2_F) x -= PI2_F;
    while (x < 0.0f)   x += PI2_F;
    return x;
}
//PLL锁相环测速，运行频率 = Speed_ISR_FRE(即1kHz)，和差分法测速2选1
void Speed_PLL_Task(EncoderTask_Param* ep)
{
	pid_param.Speed_cnt++;
	if(pid_param.Speed_cnt < pid_param.Speed_Div)
			return;

	pid_param.Speed_cnt = 0;
	
	// 速度任务周期 Ts = 1 / Speed_ISR_FRE
	const float Ts = 1.0f / Speed_ISR_FRE;
	
	// 直接使用编码器原始值构造机械角 [0, 2pi)
	// 这样最干净，不受零偏影响，速度估算不用管零偏
	float theta_meas = ((float)ep->Encoder_raw) * (PI2_F / 16384.0f);

	// 第一次进入PLL时，对齐内部状态
	if(ep->pll_init_flag == 0)
	{
		ep->pll_theta_hat = theta_meas;
		ep->pll_omega_hat = 0.0f;
		ep->pll_init_flag = 1;
	}
	
	// 相位误差，限制到[-pi, pi]
	float err = wrap_pm_pi(theta_meas - ep->pll_theta_hat);
	
	// ===== 两状态PLL =====
	// 1) 用当前速度预测位置，并用比例项快速校正位置
	ep->pll_theta_hat += (ep->pll_omega_hat + ep->pll_kp * err) * Ts;
	ep->pll_theta_hat = wrap_0_2pi(ep->pll_theta_hat);

	// 2) 用积分项修正速度估计
	ep->pll_omega_hat += ep->pll_ki * err * Ts;
	
	// 写回现有变量
	pid_param.Motor_Speed_now = ep->pll_omega_hat;       //电机轴机械速度 rad/s
	pid_param.Motor_Speed_filt_now = ep->pll_omega_hat;  //先测试，让滤波速度和实际速度一致，如果效果不好再加
	
	pid_param.Speed_now = ep->pll_omega_hat / motor_param.motor_gear;      //输出轴速度 rad/s
	pid_param.Speed_filt = pid_param.Speed_now;                        //先不再叠加额外低通
}



static uint8_t EncoderNL_Push(EncoderNLCal_Param* p, uint32_t idx, int32_t cmd_mech_tick, uint16_t raw)
{
    uint16_t next = p->fifo_wr + 1;
    if(next >= ENC_NLCAL_FIFO_LEN)
        next = 0;

    if(next == p->fifo_rd)
        return 0;   // FIFO满，直接丢掉本点

    p->fifo[p->fifo_wr].idx = idx;
    p->fifo[p->fifo_wr].cmd_mech_tick = cmd_mech_tick;
    p->fifo[p->fifo_wr].encoder_raw = raw;
    p->fifo_wr = next;

    return 1;
}

static uint8_t EncoderNL_Pop(EncoderNLCal_Param* p, EncoderNLCal_Frame* out)
{
    if(p->fifo_rd == p->fifo_wr)
        return 0;

    *out = p->fifo[p->fifo_rd];

    p->fifo_rd++;
    if(p->fifo_rd >= ENC_NLCAL_FIFO_LEN)
        p->fifo_rd = 0;

    return 1;
}

static void EncoderNL_Start(EncoderNLCal_Param* p, int8_t dir)
{
    p->start_flag = 1;
    p->done_flag = 0;
    p->dir = dir;

    p->div_cnt = 0;
    p->cmd_deg = 0.0f;
    p->mech_tick_sum = 0.0f;
    p->sample_idx = 0;
    p->fifo_wr = 0;
    p->fifo_rd = 0;
}

static void EncoderNL_Task(EncoderNLCal_Param* p, EncoderTask_Param* ep)
{
    float mech_step_tick;
    int32_t cmd_mech_tick;

    if(p->start_flag == 0 || p->done_flag == 1)
        return;

    p->div_cnt++;
    if(p->div_cnt < p->div_num)
        return;

    p->div_cnt = 0;

    // 1. 推进电角度命令
    p->cmd_deg += (float)p->dir * p->step_deg;
    p->cmd_deg = Angle_Limit(p->cmd_deg, 180.0f);
    ep->Return_Angle = p->cmd_deg;

    // 2. 按你工程当前用法，恢复理想机械角增量
    mech_step_tick = ((p->step_deg / 360.0f) / motor_param.pole) * 16384.0f;
    p->mech_tick_sum += (float)p->dir * mech_step_tick;

    cmd_mech_tick = (int32_t)(p->mech_tick_sum + 0.5f);
    while(cmd_mech_tick < 0) cmd_mech_tick += 16384;
    while(cmd_mech_tick >= 16384) cmd_mech_tick -= 16384;

    // 3. 记录一帧
    EncoderNL_Push(p, p->sample_idx, cmd_mech_tick, ep->Encoder_raw);
    p->sample_idx++;

    // 4. 扫满一圈机械角就停
    if(fabsf(p->mech_tick_sum) >= 16384.0f)
    {
        p->start_flag = 0;
        p->done_flag = 1;
    }
}

void Encoder_NLCal_PrintTask(EncoderNLCal_Param* p)
{
    static uint8_t header_printed = 0;
    static uint8_t done_printed = 0;
    EncoderNLCal_Frame frame;

    if(p->start_flag == 1 && header_printed == 0)
    {
        printf("#ENC_NLCAL_START,dir=%d,step_deg=%.6f,pole=%.3f\r\n",
               p->dir, p->step_deg, motor_param.pole);
        printf("idx,cmd_mech_tick,encoder_raw\r\n");
        header_printed = 1;
        done_printed = 0;
    }

    while(EncoderNL_Pop(p, &frame))
    {
        printf("%lu,%ld,%u\r\n",
               (unsigned long)frame.idx,
               (long)frame.cmd_mech_tick,
               frame.encoder_raw);
    }

    if(p->done_flag == 1 && done_printed == 0 && p->fifo_rd == p->fifo_wr)
    {
        printf("#ENC_NLCAL_DONE,total=%lu\r\n", (unsigned long)p->sample_idx);
        done_printed = 1;
        header_printed = 0;
    }
}

//电机运行模式任务         
void Mode_Task()
{
	if(motor_flag.Mode_Select == 1)       //SPWM运行模式
	{
		//虚拟自增角度，使用SPWM强驱电机旋转
		motor_flag.Econder_Mode = 1;
		Set_SPWM(encodertask_param.vf_v,0,&spwm_param);
	}
	else if(motor_flag.Mode_Select == 2)  //SVPWM运行模式
	{
		motor_flag.Econder_Mode = 2;
		Set_Svpwm(encodertask_param.vf_v,0,encodertask_param.Return_Angle,&svpwm_param);   //用时3.3us
	}
	else if(motor_flag.Mode_Select == 3)   //电流环运行模式
	{
		motor_flag.Econder_Mode = 2;
		
//		//1kHz更新一次测试目标，Kw辨析
//		if(pid_param.Speed_cnt == 0)  //每次完成速度计算都会置0
//		{
//			SpeedID_Kw_Task(&pid_param);
//		}
		
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
		PID_I_Control(&pid_param);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
		Set_Svpwm(pid_param.Uq,pid_param.Ud,encodertask_param.Return_Angle,&svpwm_param);   //用时3.3us
	}
	else if(motor_flag.Mode_Select == 4)   //电流-速度环运行模式
	{
		motor_flag.Econder_Mode = 2;
		
		//1kHz执行速度环
		if(pid_param.Speed_cnt == 0)  //每次完成速度计算都会置0
		{
			PID_Speed_Control(&pid_param);
		}
		
		//20kHz电流环执行
		PID_I_Control(&pid_param);
		Set_Svpwm(pid_param.Uq,pid_param.Ud,encodertask_param.Return_Angle,&svpwm_param);
	}
	else if(motor_flag.Mode_Select == 5)   //电流-速度-位置环运行模式
	{
		motor_flag.Econder_Mode = 2;
	}
	else if(motor_flag.Mode_Select == 6)   //参数辨析模式
	{ 
		motor_flag.Econder_Mode = 3;        //角度定点模式
//		encodertask_param.Return_Angle = 30.0f;
		RsID_Task(&rsid_param);	            //电阻辨析任务
	}
	else if(motor_flag.Mode_Select == 7)
	{
		motor_flag.Econder_Mode = 3;        //角度定点模式
		LdqID_Task(&rsid_param);	            //电感辨析任务
	}
	else if(motor_flag.Mode_Select == 8)  //扫频模式
	{
		motor_flag.Econder_Mode = 3;
		encodertask_param.Return_Angle = 0.0f;
		if(scanfre_param.start_flag==0 && scanfre_param.done_flag==0 && scanfre_param.scanfre_state == SCANFRE_IDLE)
			ScanFrequence_Start(&scanfre_param,scanfre_param.iq_bias,scanfre_param.iq_amp,scanfre_param.frequence_hz,encodertask_param.Return_Angle);
		ScanFrequence_Task(&scanfre_param);
		PID_I_Control(&pid_param);
		Set_Svpwm(pid_param.Uq,pid_param.Ud,encodertask_param.Return_Angle,&svpwm_param);
	}
	else if(motor_flag.Mode_Select == 9)  //编码器校准
	{
    motor_flag.Econder_Mode = 3;       // 使用命令角，不走闭环角度

    if(enc_nlcal_param.start_flag == 0 && enc_nlcal_param.done_flag == 0)
    {
        // 先做正转；做反转时把 +1 改成 -1
        EncoderNL_Start(&enc_nlcal_param, -1);
    }

    EncoderNL_Task(&enc_nlcal_param, &encodertask_param);

    // Ud强拖，Uq=0
    Set_Svpwm(0.0f, enc_nlcal_param.ud, encodertask_param.Return_Angle, &svpwm_param);
	
	}
}

//20kHz运行
void usermain()
{
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
	//1、进行ADC采样，用时1.68us
	ADC_Task(&adctask_param,ADC1InjectDate);  //先进行电流采样，并进行过压、欠压、过流检测
	
	//2、进行编码器任务，用时10.8us
	Encoder_Task(&encodertask_param);
	
	//3、进行dq实际电流计算
	Getdq(&pid_param.Iq_now,&pid_param.Id_now);              //用时1.4us
	
	//4、计算电机轴实时速度
//	Speed_Measure_Task(&pid_param,&encodertask_param);
	Speed_PLL_Task(&encodertask_param);
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET); 
	

	if(motor_flag.Adc_OffectOver_Flag == 1 && motor_flag.Zero_Flag == 1)  //电压电流没问题再进行后续操作
	{
		if(motor_flag.Error_Flag != 0)
			{  //如果发生错误，直接赋值初始值，停止运算使用
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2100);   //Duty=4200/8400
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2100);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2100);
			}
		Mode_Task();                                             //用时2.6us  
		if(motor_flag.pid_param_flag == 0)
		{
			motor_flag.pid_param_flag = 1;
			pid_param.Kp_I = 2.0f*PI_F*motor_param.I_Width*motor_param.motor_phaseL;            //2*PI*f_bw*L
			pid_param.Ki_I = 2.0f*PI_F*motor_param.I_Width*motor_param.motor_phaseR/20000.0f;            //2*PI*f_bw*R*dt
			pid_param.erro_iq_sum = 0;
			pid_param.erro_id_sum = 0;
		}
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);

//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
	}
	
}
