#include "MotorID.h"
#include "FOC.h"

extern ADCTask_Param adctask_param;          //ADC采样任务参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体
extern SVPWM_Param svpwm_param;              //SPWM生成的过程参数
extern MotorID_Param rsid_param;                //电阻辨析结构体参数
extern PID_Param pid_param;                  //PID参数结构体

//电阻辨析初始化操作
static void RsID_Init(MotorID_Param* rsparam)
{
//	rsparam->Ud_Set = 1.0;                //参数辨析Ud设置值
	rsparam->wait_cnt = 4800;             //当电压设置后等待电流稳定所记次数，执行频率20k，所花时间240ms
	rsparam->collect_cnt = 4000;          //电流稳定后电流采集次数，所花时间200ms
	rsparam->cnt_sum = 0;                 //清空累计次数
	rsparam->lock_angle = encodertask_param.Return_Angle;   //确保辨析过程中保持该角度不变
	rsparam->Rs_result = 0.0f;            //初始化时把相电阻值清0
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
	注意：参数辨析时，应当把rsid_param.Ud_Set设置值在1.0-2.0之间合适
*/
void RsID_Task(MotorID_Param* rsparam)
{
	float Id;
	
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
				rsparam->Rsid_state = RsID_Pos_Average;   //稳定时间120ms已完成，进行正电流采集
			}
			break;
		}
		case RsID_Pos_Average:
		{
			Set_Svpwm(0.0f,rsparam->Ud_Set,rsparam->lock_angle,&svpwm_param);  //设置正电压Ud
			Id = pid_param.Id_now;             //求得此刻的Id
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
				rsparam->Rsid_state = RsID_Neg_Average;   //稳定时间120ms已完成，进行负电流采集
			}
			break;
		}
		case RsID_Neg_Average:
		{
			Set_Svpwm(0.0f,-rsparam->Ud_Set,rsparam->lock_angle,&svpwm_param);  //设置负电压Ud
			Id = pid_param.Id_now;             //求得此刻的Id
			rsparam->Id_collect_sum += Id;     //累计Id
			rsparam->cnt_sum ++;
			if(rsparam->cnt_sum >= rsparam->collect_cnt)
			{
				rsparam->Id_neg_avg = rsparam->Id_collect_sum / (float)rsparam->cnt_sum; //计算负向电流平均值
				rsparam->cnt_sum = 0;
				rsparam->Id_collect_sum = 0.0f;
				rsparam->Rsid_state = RsID_Done;   //正负向电流采集结束进行Rs计算模式
			}
			break;
		}
		case RsID_Done:
		{
			Set_Svpwm(0.0f,0.0f,rsparam->lock_angle,&svpwm_param);                                 //电压设置取消 
			rsparam->Rs_result = 2*rsparam->Ud_Set / (rsparam->Id_pos_avg - rsparam->Id_neg_avg);  //求解相电阻值
			rsparam->Id_pos_avg = 0;
			rsparam->Id_neg_avg = 0;
			rsparam->Rs_done = 1;   //电阻辨析任务结束
			rsparam->Rsid_state = RsID_IDLE;    //等待rsparam->RsID_Start = 1再次进行相电阻辨析
			break;
		}
		
	}
}


//电感Ldq辨析初始化操作
static void LdqID_Init(MotorID_Param* ldqparam)
{
//	ldqparam->Udq_inject = 1.0f;         //注入方波电压幅值
//	ldqparam->frequency_inject = 400;    //注入方波频率，要小于电流环执行频率
	ldqparam->lock_angle = encodertask_param.Return_Angle;   //确保辨析过程中保持该角度不变
	
	ldqparam->half_cnts = (uint16_t)(20000.0f*0.5f/ldqparam->frequency_inject);  //求得半个周期下对应注入频率的计次值
	ldqparam->calculate_cnt = 2*30;       //执行完整的30次方波周期计算
	
	ldqparam->Ldq_half_cnt = 0;        //计次比较先清0
	ldqparam->half_index = 0;          //半个周期已执行个数清零
	ldqparam->iavgs_half_sum = 0.0f;   //清0每半周期的平均值
	ldqparam->Ldq_cnt = 0;
	ldqparam->Ldq_sum = 0.0f;
	
	ldqparam->i_max = -1e9f;
	ldqparam->i_min =  1e9f;     
	ldqparam->Ldq_done = 0;
	
	ldqparam->Lq_result = 0.0f;   //dq轴电感值清0
	ldqparam->Ld_result = 0.0f;
	ldqparam->LdqID_Start = 1;   //启动开始
}

/*
	电机Ld、Lq电感辨析任务：
	原理如下：
	电机不旋转时，q、d轴电压值为：
	U = Rs*i_avgs + L*di/dt
	可以由该式子推出
	L = {(U-Rs*i_avgs)*dt} / di
	其中di = i_max - i_min （半个周期内）,不用担心正负半周期的极性问题，
	由于负半周期，U也会改成负值，所以给抵消了
	dt = T/2 = 0.5*（1/f_inject） 
	如果注入频率够高，可以忽略Rs*i_avgs，因为此时电感阻抗高，为大占比
*/
void LdqID_Task(MotorID_Param* ldqparam)
{
	if(ldqparam->LdqID_Start == 0)  //执行初始化操作
		LdqID_Init(ldqparam);    
	
	if(ldqparam->Ldq_done == 0)
	{
		//判断目前正、负半周期，求得方波符号:奇数：-U，偶数：+U
		float sign = (ldqparam->half_index&0x01)?-1.0f:1.0f;
		float Ucmd = sign*ldqparam->Udq_inject;
		
		if(ldqparam->Ldq_select == 0)   //进行q轴电感辨析
			Set_Svpwm(Ucmd,0.0f,ldqparam->lock_angle,&svpwm_param); 
		else                            //进行d轴电感辨析
			Set_Svpwm(0.0f,Ucmd,ldqparam->lock_angle,&svpwm_param); 
		
		//如果Ldq_select为0则辨析q轴电感，为0辨析d轴电感
		float i_now = (ldqparam->Ldq_select == 0) ? pid_param.Iq_now : pid_param.Id_now;
		
		if(i_now > ldqparam->i_max) ldqparam->i_max = i_now;
		if(i_now < ldqparam->i_min) ldqparam->i_min = i_now;
		ldqparam->iavgs_half_sum += i_now;
		ldqparam->Ldq_half_cnt++;
		
		if(ldqparam->Ldq_half_cnt>=ldqparam->half_cnts)   //已到达半个周期切换点
		{
			float i_half_avg = ldqparam->iavgs_half_sum / (float)ldqparam->Ldq_half_cnt;  //半周期内电流的平均值
			float di_half = ldqparam->i_max - ldqparam->i_min;                            //半周期内的电流变化值
			float dt = 0.5f / ldqparam->frequency_inject;   // 半周期时间
			if(di_half > 0.02f)         //如果发现电流变化差值过小则不计算
			{
				float Ueff;
				
				if(ldqparam->Rs_result>0.0f)
					Ueff = ldqparam->Udq_inject - ldqparam->Rs_result * fabsf(i_half_avg);
				else
					Ueff = ldqparam->Udq_inject;  //如果发现辨析出来的相电阻有问题就去除
				
				if(Ueff>1e-5f)
				{
					float L_half = Ueff * dt / di_half;
					ldqparam->Ldq_sum += L_half;
					ldqparam->Ldq_cnt++;
				}
			}
			
			ldqparam->half_index++;
			
			ldqparam->i_max = -1e5f;            //不直接赋值0，避免出现错误的di
			ldqparam->i_min = +1e5f;
			ldqparam->iavgs_half_sum = 0.0f;
			ldqparam->Ldq_half_cnt = 0;
		}
		if(ldqparam->half_index>=ldqparam->calculate_cnt)  //已满足执行次数
		{
			float L_result = 0.0f;
			if (ldqparam->Ldq_cnt>0)
				L_result = ldqparam->Ldq_sum / (float)ldqparam->Ldq_cnt;
			
			if(ldqparam->Ldq_select == 0)
				ldqparam->Lq_result = L_result;
			else
				ldqparam->Ld_result = L_result;
			
			Set_Svpwm(0.0f,0.0f,ldqparam->lock_angle,&svpwm_param);         //电压设置取消 
			
			ldqparam->Ldq_cnt = 0;
			ldqparam->Ldq_sum = 0.0f;
			ldqparam->half_index = 0;
			ldqparam->Ldq_done = 1;
		}
	}
}


/*
	扫频法测试电流环带宽：
	
*/


