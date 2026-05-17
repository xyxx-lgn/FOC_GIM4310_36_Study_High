#include "MotorID.h"
#include "FOC.h"

extern ADCTask_Param adctask_param;          //ADC采样任务参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体
extern SVPWM_Param svpwm_param;              //SPWM生成的过程参数
extern RsID_Param rsid_param;                //电阻辨析结构体参数


//通过三相电流，求得实时的Id
static void RsID_Getdq(float* Iq,float* Id)
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

//电阻辨析初始化操作
static void RsID_Init(RsID_Param* rsparam)
{
	rsparam->Ud_Set = 1.2;                //参数辨析Ud设置值
	rsparam->wait_cnt = 1200;             //当电压设置后等待电流稳定所记次数，执行频率20k，所花时间60ms
	rsparam->collect_cnt = 2000;          //电流稳定后电流采集次数，所花时间100ms
	rsparam->cnt_sum = 0;                 //清空累计次数
	rsparam->lock_angle = encodertask_param.Return_Angle;   //确保辨析过程中保持该角度不变
	rsparam->RsID_Start = 1;              //电阻辨析初始化完成
	rsparam->Rs_done = 0;                 //电阻辨析任务未结束
}

/*
	电机电阻辨析任务：
	原理如下；
		u_d = R_s*i_d + u_err
		u_err 是一些误差，如死区
	而两次正负电压注入可得
		+i_d =( +u_d+u_err  ) / R_s
		-i_d =( -u_d+u_err  ) / R_s
	两式相减，消去u_err，得到R_s为：
		R_s = 2*u_d / ( +i_d-(-i_d) )
*/
void RsID_Task(RsID_Param* rsparam)
{
	float Iq,Id;
	switch(rsparam->Rsid_state)
	{
		case RsID_IDLE:               //电阻辨析任务状态空闲
		{
			if(rsparam->RsID_Start == 0)
			{
				RsID_Init(rsparam);
				rsparam->Rsid_state = RsID_Pos_Set;   //完成初始化设置，进入正电压设置稳定状态
			}
			break;
		}
		case RsID_Pos_Set:            //正电压设置并等待稳定状态
		{
			Set_Svpwm(0.0f,rsparam->Ud_Set,rsparam->lock_angle,&svpwm_param);  //设置正电压Ud
			rsparam->cnt_sum ++;
			if(rsparam->cnt_sum >= rsparam->wait_cnt)
			{
				rsparam->cnt_sum = 0;
				rsparam->Rsid_state = RsID_Pos_Average;   //稳定时间60ms已完成，进行正电流采集
			}
			break;
		}
		case RsID_Pos_Average:
		{
			Set_Svpwm(0.0f,rsparam->Ud_Set,rsparam->lock_angle,&svpwm_param);  //设置正电压Ud
			RsID_Getdq(&Iq,&Id);               //求得此刻的Id
			rsparam->Id_collect_sum += Id;     //累计Id
			rsparam->cnt_sum ++;
			if(rsparam->cnt_sum >= rsparam->collect_cnt)
			{
				rsparam->Id_pos_avg = rsparam->Id_collect_sum / (float)rsparam->cnt_sum; //计算正向电流平均值
				rsparam->cnt_sum = 0;
				rsparam->Id_collect_sum = 0.0f;
				rsparam->Rsid_state = RsID_Neg_Set;   //进入负向Ud设置稳定状态
			}
			break;
		}
		case RsID_Neg_Set:
		{
			Set_Svpwm(0.0f,-rsparam->Ud_Set,rsparam->lock_angle,&svpwm_param);  //设置负电压Ud
			rsparam->cnt_sum ++;
			if(rsparam->cnt_sum >= rsparam->wait_cnt)
			{
				rsparam->cnt_sum = 0;
				rsparam->Rsid_state = RsID_Neg_Average;   //稳定时间60ms已完成，进行负电流采集
			}
			break;
		}
		case RsID_Neg_Average:
		{
			Set_Svpwm(0.0f,-rsparam->Ud_Set,rsparam->lock_angle,&svpwm_param);  //设置负电压Ud
			RsID_Getdq(&Iq,&Id);               //求得此刻的Id
			rsparam->Id_collect_sum += Id;     //累计Id
			rsparam->cnt_sum ++;
			if(rsparam->cnt_sum >= rsparam->collect_cnt)
			{
				rsparam->Id_neg_avg = rsparam->Id_collect_sum / (float)rsparam->cnt_sum; //计算负向电流平均值
				rsparam->cnt_sum = 0;
				rsparam->Id_collect_sum = 0.0f;
				rsparam->Rsid_state = RsID_Neg_Set;   //正负向电流采集结束进行Rs计算模式
			}
			break;
		}
		case RsID_Done:
		{
			Set_Svpwm(0.0f,0.0f,rsparam->lock_angle,&svpwm_param);                                 //电压设置取消 
			rsparam->Rs_result = 2*rsparam->Ud_Set / (rsparam->Id_pos_avg - rsparam->Id_neg_avg);  //求解相电阻值
			rsparam->Rs_done = 1;   //电阻辨析任务结束
			break;
		}
		
	}
}

