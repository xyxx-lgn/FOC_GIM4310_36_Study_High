#include "usermain.h"
#include "type.h"
#include "FOC.h"

extern Motor_Param motor_param;      //电机参数结构体
extern SPWM_Param spwm_param;        //SPWM生成的过程参数

void Data_Init()
{
	//电机参数结构体
	motor_param.supply_Udc = 24.0f;       //供电电压
	motor_param.pole = 14;                //电机极对数
	motor_param.Tpwm = 4200;                //定时器计数最大值
	
	//SPWM生成的过程参数
	spwm_param.Uq = 0.0f;
	spwm_param.Ud = 0.0f;
	spwm_param.virtual_step = 0.252f;                    //自增步长，14极对数，360/20000*14 = 0.252相当于1圈每秒
	spwm_param.vf_k = 0.252f;                            //VF强托系数，即Uq = k*F; 得到step = k*Uq
	spwm_param.supply_Udc = motor_param.supply_Udc;      //SPWM电压抬升
	spwm_param.Tpwm = motor_param.Tpwm;                  //定时器计数最大值
}

//20kHz运行
void usermain()
{
	//虚拟自增角度，使用SPWM强驱电机旋转
	Set_SPWM(&spwm_param);
	
}
