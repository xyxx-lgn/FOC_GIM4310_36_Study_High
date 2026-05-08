#include "usermain.h"
#include "MT6701.h"
#include "FOC.h"
#include "MT6701.h"

extern TIM_HandleTypeDef htim1;

extern Motor_Param motor_param;              //电机参数结构体
extern SPWM_Param spwm_param;                //SPWM生成的过程参数
extern ADCTask_Param adctask_param;          //ADC采样任务参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体
extern Motor_Flag motor_flag;                //电机标志位结构体
extern SVPWM_Param svpwm_param;              //SPWM生成的过程参数

extern uint16_t ADC1InjectDate[4];     //注入组采样数组

void Data_Init()
{
	//电机参数结构体
	motor_param.supply_Udc = 24.0f;       //供电电压
	motor_param.pole = 14;                //电机极对数
	motor_param.Tpwm = 8400;              //定时器计数最大值
	motor_param.Rs = 0.005;               //采样电阻值大小
	motor_param.Gain = 50;                //运算放大器增益
	
	//电机运行标志位结构体
	motor_flag.Zero_Flag = 1;             //默认零偏校准标志位为1，避免每次启动都校准
	motor_flag.Econder_Mode = 1;          //编码器模式，1为开环自增角度，2为闭环真实角度
	
	//SPWM生成的过程参数
	spwm_param.supply_Udc = motor_param.supply_Udc;      //SPWM电压抬升
	spwm_param.Tpwm = motor_param.Tpwm;                  //定时器计数最大值
	
	//SVPWM生成的过程参数
	svpwm_param.Ts = motor_param.Tpwm;                  //SVPWM里面的时间系数Ts
	svpwm_param.Udc = motor_param.supply_Udc;           //SVPWM母线电压
	
	//ADC采样任务的参数
	adctask_param.Ia_offect = 1.65f;
	adctask_param.Ib_offect = 1.65f;
	adctask_param.Ic_offect = 1.65f;
	adctask_param.IGain = motor_param.Rs*motor_param.Gain;
	
	//Encoder编码器任务参数
	encodertask_param.motordir = 0;
	encodertask_param.Zero_Angle = 0.0f;
	encodertask_param.virtual_step = 0.252f;                    //自增步长，14极对数，360/20000*14 = 0.252相当于1圈每秒
	encodertask_param.vf_v = 1.0f;                              //VF强托系数里面的V
	encodertask_param.vf_k = 0.252f;                            //VF强托系数，即Uq = k*F; 得到step = k*Uq
}

void ADC_Task(ADCTask_Param* adctask_param,uint16_t* adc_raw)
{
	//1、先得到母线电压  电压系数：0.0088623046875f = 1/4096*3.3*(20+2)/2
	adctask_param->Udc = (float)adc_raw[3]*0.0088623046875f;
	
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
		motor_flag.Adc_OffectOver_Flag = 0; //如果电压报警错误一次就重新进行电流校准，避免错误
	}
	
	//3、采集三相电流数据
	adctask_param->Ia_Sample = (float)adc_raw[0]*3.3f/4096.0f;
	adctask_param->Ib_Sample = (float)adc_raw[1]*3.3f/4096.0f;
	adctask_param->Ic_Sample = (float)adc_raw[2]*3.3f/4096.0f;
	
	//4、进行三相电流校准(母线电压正常时才进行电流计算，只要母线电压异常就重新校准)
	if(motor_flag.Adc_OffectOver_Flag == 0 && motor_flag.Error_Flag != 1 && motor_flag.Error_Flag != 2)
	{
		//进行10k次校准，用时0.5s
		if(adctask_param->Iadc_offect_counts<10000)
		{
			adctask_param->Ia_offect = adctask_param->Ia_offect*0.95f + adctask_param->Ia_Sample*0.05f;
			adctask_param->Ib_offect = adctask_param->Ib_offect*0.95f + adctask_param->Ib_Sample*0.05f;
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
		adctask_param->Ia = (adctask_param->Ia_Sample - adctask_param->Ia_offect)/adctask_param->IGain;
		adctask_param->Ib = (adctask_param->Ib_Sample - adctask_param->Ib_offect)/adctask_param->IGain;
		adctask_param->Ic = (adctask_param->Ic_Sample - adctask_param->Ic_offect)/adctask_param->IGain;
	}
	
	//6、进行是否过流判断
	if(adctask_param->Ia>6.0f &&adctask_param->Ib>6.0f &&adctask_param->Ic>6.0f)
	{
		motor_flag.Error_Flag = 3; 
	}
	
}

//编码器采样任务
void Encoder_Task(EncoderTask_Param* encodertask_param)
{
	//1、首先读读取MT6701原始值(用时6.8us)
	encodertask_param->Encoder_raw = MT6701_ReadRaw();    

	//2、转化角度值，将编码值转化为机械角度、电角度
		////0.02197265625 = 1/16384*360
	encodertask_param->Shaft_Angle = (float)encodertask_param->Encoder_raw * 0.02197265625f - encodertask_param->Zero_Angle; 
	encodertask_param->Elect_Angle = Angle_Limit(encodertask_param->Shaft_Angle * (float)motor_param.pole,180.0f);
	
	//3、零偏校准程序
	if(motor_flag.Zero_Flag == 0)
	{
		Set_Svpwm(0,0.5f,0,&svpwm_param);
		if(encodertask_param->Zero_counts>20000)
		{
			if(encodertask_param->Zero_counts<21000)
			{
				encodertask_param->Zero_Angle += encodertask_param->Shaft_Angle;
			}
			else
			{
				encodertask_param->Zero_Angle /= 1000;
				motor_flag.Zero_Flag = 1;
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
	}
	else if(motor_flag.Econder_Mode == 2)  //闭环角度处理
	{
		encodertask_param->Return_Angle = encodertask_param->Elect_Angle;
	}
	
	//5、计算正余弦值
	//注意arm_sin_cos_f32这个函数，角度传参是角度值  ，arm_sin_f32和arm_cos_f32传参是弧度值
	arm_sin_cos_f32(encodertask_param->Return_Angle,&encodertask_param->sin_dsp,&encodertask_param->cos_dsp);  //DSP库计算三角，在电流环帕克变换处计算一次即可
}

//20kHz运行
void usermain()
{
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
	//1、进行ADC采样，用时1.68us
	ADC_Task(&adctask_param,ADC1InjectDate);  //先进行电流采样，并进行过压、欠压、过流检测
	
	//2、进行编码器任务，用时10.8us
	Encoder_Task(&encodertask_param);
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
	
	if(motor_flag.Error_Flag != 0)
	{  //如果发生错误，直接赋值初始值，停止运算使用
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2100);   //Duty=4200/8400
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2100);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2100);
	}
	else if(motor_flag.Adc_OffectOver_Flag == 1)  //电压电流没问题再进行后续操作
	{
		//虚拟自增角度，使用SPWM强驱电机旋转
//		Set_SPWM(encodertask_param.vf_v,0,&spwm_param);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
		Set_Svpwm(encodertask_param.vf_v,0,encodertask_param.Return_Angle,&svpwm_param);   //用时3.3us
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
	}
	
}
