#include "Start_Init.h"
#include "tim.h"
#include "adc.h"
#include "usermain.h"
/*
函数功能：
	进行外设的一些初始化配置
具体描述:
	进行TIM1定时器对三相PWM通道配置
	对ADC1注入组采样中断进行配置	
*/

void Start_Init(void)
{
	//1.定时器1的PWM开启
	HAL_TIM_Base_Start(&htim1);   //打开定时器1
	
	//打开PWM通道
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);            //两者二选一
//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
	
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
	//给定初始值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2100);   //Duty=4200/8400
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2100);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2100);
	
	
	//2.ADC1初始化，开启注入组采样，用于电流采样和母线电压采样
	/*FOC_GIM4310_36_Study_High
		ADC_IT_JEOC   每个注入 rank 完成就可触发,我这里4个通道，不合适、
		ADC_IT_JEOS   整个注入序列（4个都完）才触发一次
	*/
	__HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOS);    //ADC注入通道中断    //还需要在it.c里面打开HAL_ADCEx_InjectedConvCpltCallback中断读取
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);   //ADC内部增益校准
	HAL_ADCEx_InjectedStart(&hadc1);            //开启ADC注入采样
	

	
/*
    //注入组ADC采样中断回调
	void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
	{
	//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
		//ADC采样 AB相电流和母线电压
		ADC1InjectDate[0] = hadc->Instance->JDR1;    //A相电流
		ADC1InjectDate[1] = hadc->Instance->JDR2;    //B相电流
		ADC1InjectDate[2] = hadc->Instance->JDR3;    //C相电压
		ADC1InjectDate[3] = hadc->Instance->JDR4;    //母线电压

	//	Adcpro(&adcvalue,&allflag,ADC1InjectDate);
	//	encoder_str.Encoder = MT6701_ReadRaw();
		Data_Treating();
	//	Encoderpro(&encoder_str,&allflag);
	//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
		
	}
*/

/*
Timer1的通道4中断选择，可以在这个中断里面拉高引脚电平，在ADC注入中断拉低来查看电流采样所需时间是否正确，需要先开启中断
	1.如果 CH4 继续用 PWM 方式（现在是 PWM2 no output），用
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
	对应回调一般是void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)。
	
	2.如果 CH4改成 OC 比较方式，用
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
	对应回调是 HAL_TIM_OC_DelayElapsedCallback()。
	
*/
	
	
	//参数初始化
	Data_Init();
}


