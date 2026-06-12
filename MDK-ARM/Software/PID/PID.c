#include "PID.h"

extern Motor_Param motor_param;              //电机参数结构体
extern PID_Param pid_param;                  //PID参数结构体
extern ADCTask_Param adctask_param;          //ADC采样任务参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体
extern Motor_Flag motor_flag;                //电机标志位结构体

//电流环PID运算
void PID_I_Control(PID_Param* pid_i)
{
	//2、进行电流环Iq、Id限幅
	pid_i->Iq_aim = Limit(pid_i->Iq_aim,-pid_i->Iqd_Max,pid_i->Iqd_Max); 
	pid_i->Id_aim = Limit(pid_i->Id_aim,-pid_i->Iqd_Max,pid_i->Iqd_Max);
	
	//3、进行PI误差运算
	float erro_iq = pid_i->Iq_aim - pid_i->Iq_now;
	float erro_id = pid_i->Id_aim - pid_i->Id_now;
	
	//4、进行PI积分运算，我使用的是离散PI，此处不乘dt，因此Ki值很小，因为乘了dt（相反，连续性的Ki很大，因为积分项乘了dt）	
	pid_i->erro_iq_sum += pid_i->Ki_I * erro_iq;
	pid_i->erro_id_sum += pid_i->Ki_I * erro_id;
	
	//5、电流环输出计算
	float Uq = pid_i->Kp_I*erro_iq + pid_i->erro_iq_sum;
	float Ud = pid_i->Kp_I*erro_id + pid_i->erro_id_sum;
	
	//6、抗积分饱和，如果输出饱和且误差与饱和方向相同，则回退本次积分
	if ((Uq >= pid_i->Uout_Max && erro_iq > 0.0f) || (Uq <= -pid_i->Uout_Max && erro_iq < 0.0f))     // 停止积分
  {
    pid_i->erro_iq_sum -= pid_i->Ki_I * erro_iq;  // 回退
	}
	if ((Ud >= pid_i->Uout_Max && erro_id > 0.0f) || (Ud <= -pid_i->Uout_Max && erro_id < 0.0f))      // 停止积分   
	{
		pid_i->erro_id_sum -= pid_i->Ki_I * erro_id; //回退
	}
	
	pid_i->erro_iq_sum = Limit(pid_i->erro_iq_sum,-pid_i->Ki_I_SumMax,pid_i->Ki_I_SumMax);
	pid_i->erro_id_sum = Limit(pid_i->erro_id_sum,-pid_i->Ki_I_SumMax,pid_i->Ki_I_SumMax);
	
	Uq = pid_i->Kp_I*erro_iq + pid_i->erro_iq_sum;
  Ud = pid_i->Kp_I*erro_id + pid_i->erro_id_sum;
	
	pid_i->Uq = Limit(Uq,-pid_i->Uout_Max,pid_i->Uout_Max);
	pid_i->Ud = Limit(Ud,-pid_i->Uout_Max,pid_i->Uout_Max);

	
//	//6、进行前馈补偿，包括dq轴解耦和反电动势补偿  !!!!!!!!!!!!!!!!!!(注意后期需要把这个速度滤波，不然可能会出现补偿进去有噪声)
//	if(motor_flag.dec_mode!=FOC_CC_DECOUPLING_DISABLED)
//	{
//		static float last_elect_angle = 0.0f;
//		float now_elect_angle = encodertask_param.Return_Rads;  
//		float elect_erro = (now_elect_angle - last_elect_angle);  
//		//粗略计算一下该电机，额定输出转速40rpm，减速比36，极对数14，执行频率20k
//		//弧度变化值最大为40*36*14*6.28/60/20000 = 0.105504,如果发现特别大则出现过零点
//		if(elect_erro>3.14f)        //反转过零点
//			elect_erro -= PI2_F;
//		else if(elect_erro<-3.14f)  //正转过零点
//			elect_erro += PI2_F;
//		float elect_speed = elect_erro*20000.0f;   //计算电角速度，该电流环执行频率20k，化为rad/s，乘系数20000
//		last_elect_angle = now_elect_angle;
//		
//		//计算前馈补偿项
//		// vd = Rs*id + Ld*did/dt - ωe*iq*Lq
//		// vq = Rs*iq + Lq*diq/dt + ωe*id*Ld + ωe*ψm
//		float dec_vd = elect_speed*pid_i->Iq_now*motor_param.motor_Lq;
//		float dec_vq = elect_speed*pid_i->Id_now*motor_param.motor_Ld;
//		float dec_bemf = elect_speed*motor_param.motor_filed_link;
//	
//		switch(motor_flag.dec_mode)
//		{
//			case FOC_CC_DECOUPLING_DISABLED:        //不解耦
//				break;
//			case FOC_CC_DECOUPLING_CROSS:           //只交叉耦合（dq轴电流解耦）
//				Ud = Ud-dec_vd;
//				Uq = Uq+dec_vq;
//				break;
//			case FOC_CC_DECOUPLING_BEMF:            //只反电势
//				Uq = Uq+dec_bemf;
//				break;
//			case FOC_CC_DECOUPLING_CROSS_BEMF:      //交叉+反电势
//				Ud = Ud-dec_vd;
//				Uq = Uq+dec_vq+dec_bemf;
//				break;
//		}
//	}
//	
//	//7、选择dq轴优先限幅策略：默认等比例限幅
//	/*
//		进行d轴优先限幅（目的：弱磁控制，效率优化）
//		进行q轴优先限幅（目的：确保最大转矩）
//	*/
//	float Umax = pid_i->Ki_I_SumMax;
//	switch(motor_flag.v_limit_mode)
//	{
//		case V_LIMIT_Q_PRIORITY:  //q轴优先
//		{
//			Uq = Limit(Uq,-Umax,Umax);
//			float Ud_m = sqrtf(Umax*Umax - Uq*Uq);
//			Ud = Limit(Ud,-Ud_m,Ud_m);
//			break;
//		}
//		case V_LIMIT_D_PRIORITY:  //d轴优先
//		{
//			Ud = Limit(Ud,-Umax,Umax);
//			float Uq_m = sqrtf(Umax*Umax - Ud*Ud);
//			Uq = Limit(Uq,-Uq_m,Uq_m);
//			break;
//		}
//		case V_LIMIT_VECTOR:      //等比例限幅
//		{
//			float Uqd2 = Uq*Uq + Ud*Ud;
//			if(Uqd2 > Umax*Umax)
//			{
//				float k = Umax/sqrtf(Uqd2);
//				Uq *= k;
//				Ud *= k;
//			}
//			break;
//		}
//	}

}

//速度环PID运算
void PID_Speed_Control(PID_Param* pid_sp)
{
	float erro_speed = pid_sp->Motor_Speed_aim - pid_sp->Motor_Speed_filt_now;   //rad/s

	//先积分
	pid_sp->Speed_erro_sum += pid_sp->Ki_S * erro_speed;
	
	//未限幅输出
	float Speed_out = pid_sp->Kp_S * erro_speed + pid_sp->Speed_erro_sum;
	
	//抗积分饱和：若输出已饱和且误差仍推动饱和，则回退本次积分
	if ((Speed_out >= pid_sp->Iqd_Max && erro_speed > 0.0f) ||
			(Speed_out <= -pid_sp->Iqd_Max && erro_speed < 0.0f))
	{
			pid_sp->Speed_erro_sum -= pid_sp->Ki_S * erro_speed;
	}
	
	pid_sp->Speed_erro_sum = Limit(pid_sp->Speed_erro_sum, -pid_sp->Speed_KISumMax, pid_sp->Speed_KISumMax);
	
	Speed_out = pid_sp->Kp_S * erro_speed + pid_sp->Speed_erro_sum;
	Speed_out = Limit(Speed_out, -pid_sp->Iqd_Max, pid_sp->Iqd_Max);
	
	pid_sp->Iq_aim = Speed_out;
	pid_sp->Id_aim = 0.0f;
}
