#include "PID.h"


extern PID_Param pid_param;                  //PID参数结构体
extern ADCTask_Param adctask_param;          //ADC采样任务参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体

void PID_I_Control(PID_Param* pid_i)
{
	//1、克拉克变化和反帕克变化
	const float _1_sqrt3 = 0.5773502691896258f;  //1/sqrt3
	const float _2_1_sqrt3 = 1.15470053838f;     //2/sqrt3
	
	//(1)克拉克变化（已做过等幅值处理，及乘2/3）Iabc->alhpa、beta
	float Ialpha = adctask_param.Ia;
	float Ibeta = _1_sqrt3 * adctask_param.Ia + _2_1_sqrt3 * adctask_param.Ib;
	
	//(2)帕克逆变化 alhpa、beta->Iq、Id
	arm_park_f32(Ialpha,Ibeta,&pid_i->Id_now,&pid_i->Iq_now,encodertask_param.sin_dsp,encodertask_param.cos_dsp);
	
	//2、进行电流环Iq、Id限幅
	pid_i->Iq_aim = Limit(pid_i->Iq_aim,-pid_i->Iqd_Max,pid_i->Iqd_Max); 
	pid_i->Id_aim = Limit(pid_i->Id_aim,-pid_i->Iqd_Max,pid_i->Iqd_Max);
	
	//3、进行PID误差运算
	float erro_iq = pid_i->Iq_aim - pid_i->Iq_now;
	float erro_id = pid_i->Id_aim - pid_i->Id_now;
	
	//4、进行PID积分运算，并限幅
	pid_i->erro_iq_sum += pid_i->Ki_I*erro_iq;
	pid_i->erro_id_sum += pid_i->Ki_I*erro_id;
	
	pid_i->erro_iq_sum = Limit(pid_i->erro_iq_sum,-pid_i->Iqd_Max,pid_i->Iqd_Max);
	pid_i->erro_id_sum = Limit(pid_i->erro_id_sum,-pid_i->Iqd_Max,pid_i->Iqd_Max);
	
	//5、电流环输出计算并限幅
	pid_i->Uq = pid_i->Kp_I*erro_iq + pid_i->erro_iq_sum;
	pid_i->Ud = pid_i->Kp_I*erro_id + pid_i->erro_id_sum;
	
	pid_i->Uq = Limit(pid_i->Uq,-pid_i->Iqd_Max,pid_i->Iqd_Max);
	pid_i->Ud = Limit(pid_i->Ud,-pid_i->Iqd_Max,pid_i->Iqd_Max);
}


