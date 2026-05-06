#include "usermain.h"
#include "FOC.h"

extern TIM_HandleTypeDef htim1;

extern Motor_Param motor_param;      //电机参数结构体
extern SPWM_Param spwm_param;        //SPWM生成的过程参数
extern ADCTask_Param adctask_param;   //ADC采样任务参数
extern Motor_Flag motor_flag;         //电机标志位结构体

extern uint16_t ADC1InjectDate[4];     //注入组采样数组

void Data_Init()
{
	//电机参数结构体
	motor_param.supply_Udc = 24.0f;       //供电电压
	motor_param.pole = 14;                //电机极对数
	motor_param.Tpwm = 4200;              //定时器计数最大值
	motor_param.Rs = 0.005;               //采样电阻值大小
	motor_param.Gain = 50;                //运算放大器增益
	
	//SPWM生成的过程参数
	spwm_param.Uq = 0.0f;
	spwm_param.Ud = 0.0f;
	spwm_param.virtual_step = 0.252f;                    //自增步长，14极对数，360/20000*14 = 0.252相当于1圈每秒
	spwm_param.vf_k = 0.252f;                            //VF强托系数，即Uq = k*F; 得到step = k*Uq
	spwm_param.supply_Udc = motor_param.supply_Udc;      //SPWM电压抬升
	spwm_param.Tpwm = motor_param.Tpwm;                  //定时器计数最大值
	
	//ADC采样任务的参数
	adctask_param.Ia_offect = 1.65f;
	adctask_param.Ib_offect = 1.65f;
	adctask_param.Ic_offect = 1.65f;
	adctask_param.IGain = motor_param.Rs*motor_param.Gain;
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


//20kHz运行
void usermain()
{
	ADC_Task(&adctask_param,ADC1InjectDate);  //先进行电流采样，并进行过压、欠压、过流检测
	
	
	if(motor_flag.Error_Flag != 0)
	{  //如果发生错误，直接赋值初始值，停止运算使用
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2100);   //Duty=4200/8400
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2100);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2100);
	}
	else   //电压电流没问题再进行后续操作
	{
		
	}
	//虚拟自增角度，使用SPWM强驱电机旋转
	Set_SPWM(&spwm_param);
	
}
