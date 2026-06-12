#include "MotorID.h"
#include "FOC.h"


extern ADCTask_Param adctask_param;          //ADC采样任务参数
extern EncoderTask_Param encodertask_param;  //编码器任务结构体
extern SVPWM_Param svpwm_param;              //SPWM生成的过程参数
extern MotorID_Param rsid_param;                //电阻辨析结构体参数
extern PID_Param pid_param;                  //PID参数结构体

extern ScanFre_Sample scanfre_buff[1200];       //扫频法数据存储区
extern ScanFre_Param scanfre_param;             //扫频法测带宽结构体





/********************************************电阻辨析任务**************************************************************/

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
		U = R*i + L*di/dt
		u_d = R_s*i_d + u_err
		u_err 是一些误差，如死区
	而两次正负电压注入可得
		+i_d =( +u_d+u_err  ) / R_s
		-i_d =( -u_d+u_err  ) / R_s
	两式相减，消去u_err，得到R_s为：
		R_s = 2*u_d / ( +i_d-(-i_d) )
	注意：参数辨析时，应当把rsid_param.Ud_Set设置值在1.0-2.0之间合适

	当rsparam->Rsid_state为RsID_IDLE（每次辨析完会自动变成这个），且rsparam->RsID_Start给0就可实现再次电阻辨析！！！！！！！！！
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

/*********************************************************************************************************************/



/********************************************电感辨析任务**************************************************************/
//电感Ldq辨析初始化操作
static void LdqID_Init(MotorID_Param* ldqparam)
{
//	ldqparam->Udq_inject = 1.0f;         //注入方波电压幅值
//	ldqparam->frequency_inject = 400;    //注入方波频率，要小于电流环执行频率
	ldqparam->lock_angle = encodertask_param.Return_Angle;   //确保辨析过程中保持该角度不变
	
	//求得半个周期下对应注入频率的计次值
	ldqparam->half_cnts = (uint16_t)(Current_ISR_FRE*0.5f/ldqparam->frequency_inject);  
	ldqparam->calculate_cnt = 2*30;       //执行完整的30次方波周期计算
	
	ldqparam->Ldq_half_cnt = 0;        //计次比较先清0
	ldqparam->half_index = 0;          //半个周期已执行个数清零
	

	ldqparam->Ldq_cnt = 0;
	ldqparam->Ldq_sum = 0.0f;
	
    
	ldqparam->Ldq_done = 0;
	
	// 半周期积分法要用到的变量
	ldqparam->i_start_half = 0.0f;
	ldqparam->i_end_half   = 0.0f;
	ldqparam->i_sum_half   = 0.0f;
	
	
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
	其中di = i_end - i_start （半个周期内）,不用担心正负半周期的极性问题，
	由于负半周期，U也会改成负值，所以给抵消了
	dt = T/2 = 0.5*（1/f_inject） 
	如果注入频率够高，可以忽略Rs*i_avgs，因为此时电感阻抗高，为大占比

	可将LdqID_Start设置为0来再次进行辨析！！！！！！！！！！！！！！
*/
void LdqID_Task(MotorID_Param* ldqparam)
{
	//1、初始化参数设置
	if(ldqparam->LdqID_Start == 0)  //执行初始化操作
		LdqID_Init(ldqparam);    
	
	if(ldqparam->Ldq_done == 0)   //如果没有完成则进行电感辨析
	{
		//2、判断正负周期，并设定注入电压，求得半周期起始电流和实时电流
		//判断目前正、负半周期，求得方波符号:奇数：-U，偶数：+U
		float sign = (ldqparam->half_index&0x01)?-1.0f:1.0f;
		float Ucmd = sign*ldqparam->Udq_inject;
		
		if(ldqparam->Ldq_select == 0)   //进行q轴电感辨析
			Set_Svpwm(Ucmd,0.0f,ldqparam->lock_angle,&svpwm_param); 
		else                            //进行d轴电感辨析
			Set_Svpwm(0.0f,Ucmd,ldqparam->lock_angle,&svpwm_param); 
		
		//如果Ldq_select为0则辨析q轴电感，为0辨析d轴电感
		float i_now = (ldqparam->Ldq_select == 0) ? pid_param.Iq_now : pid_param.Id_now;
		
    if(ldqparam->Ldq_half_cnt == 0)  //记录起始电流值
    {
        ldqparam->i_start_half = i_now;
        ldqparam->i_sum_half = 0.0f;
    }
		ldqparam->i_sum_half += i_now;
    ldqparam->Ldq_half_cnt++;

		//3、到达半周期后记录半周期末电流值，并计算半周期电感L
		if(ldqparam->Ldq_half_cnt>=ldqparam->half_cnts)   //已到达半个周期切换点
		{
			ldqparam->i_end_half = i_now;
			float T_half = 0.5f / ldqparam->frequency_inject;
			float i_avg  = ldqparam->i_sum_half / (float)ldqparam->Ldq_half_cnt;
			float di     = ldqparam->i_end_half - ldqparam->i_start_half;
			
			// 半周期积分法：
			// L = (Ucmd - Rs*i_avg) * T_half / (i_end - i_start)
			float Ueff = Ucmd - ldqparam->Rs_result * i_avg;
			
			// 防止除零或极小摆幅带来异常结果
			if(fabsf(di) > 0.01f)
			{
					float L_half = Ueff * T_half / di;  //计算电感

					//给一个计算出来的估计值进行上下限判断，过于离谱的值舍去
					if(L_half > 0.00001f && L_half < 0.005f)  
					{
							// 丢掉前几个半周期，只保留稳定后的结果
							if(ldqparam->half_index >= 6)
							{
									ldqparam->Ldq_sum += L_half;
									ldqparam->Ldq_cnt++;
							}
					}
			}
			// 进入下一个半周期
			ldqparam->half_index++;
			ldqparam->Ldq_half_cnt = 0;
		}
		
		//4、到达指定执行周期数，将所有半周期电感进行平均，求出最终电感L
		if(ldqparam->half_index>=ldqparam->calculate_cnt)  //已满足执行次数
		{
			float L_result = 0.0f;
			if (ldqparam->Ldq_cnt>0)
				L_result = ldqparam->Ldq_sum / (float)ldqparam->Ldq_cnt;
			
			if(ldqparam->Ldq_select == 0)
				ldqparam->Lq_result = L_result;
			else
				ldqparam->Ld_result = L_result;
			
			Set_Svpwm(0.0f,0.0f,ldqparam->lock_angle,&svpwm_param);//电压设置取消 
			
			ldqparam->Ldq_cnt = 0;
			ldqparam->Ldq_sum = 0.0f;
			ldqparam->half_index = 0;
			ldqparam->Ldq_done = 1;
		}
	}
}

/*********************************************************************************************************************/



/********************************************扫频法测电流环带宽任务****************************************************/

/*
	扫频法电流带宽测量初始化函数
*/
void ScanFrequence_Init(ScanFre_Param* sf_p)
{
	sf_p->iq_bias = 0.0f;                                 //扫频电流直流偏置值
	sf_p->iq_amp = 0.3f;                                  //扫频电流幅值
	sf_p->iq_ref = 0.0f;
	sf_p->lock_angle = 0;                                 //设定固定角度设定
	
	sf_p->frequence_hz = 100;
	sf_p->phase = 0.0f;                                   //相位为0
	sf_p->phase_step = 0.0f;                              //相位步进为0
	
	sf_p->start_flag = 0;                                 //开始标志位清0
	sf_p->done_flag = 0;                                  //结束标志位清0
	
	sf_p->wait_cycle = 5;                                 //等待n个完整波形稳定
	sf_p->wait_cnt = 0;                                   //等待计次比较
	sf_p->sample_cycle = 5;                               //采集n个波形数据
	sf_p->sample_cnt = 0;                                 //采集计次比较

	sf_p->buf_len = 1200;                                 //16kb大小存储区，一个数组下标是4个数据 , 100Hz采集5周期是1000个点,注意100Hz以下可能会超限数组
	sf_p->buf_now = 0;                                    //当前采样数组下标
	
	sf_p->scanfre_state = SCANFRE_IDLE;
}

/*
	根据执行周期与频率计算进行的计次数
	参数：
		float frequence  扫频法当前电流注入频率
		float cycle      扫频法等待或采样的完整波形数
*/
static uint16_t ScanFrequence_CntCal(float frequence,float cycle)
{
	float cnt = (Current_ISR_FRE / frequence) * cycle;
	return cnt;
}




void ScanFrequence_Start(ScanFre_Param* sf_p,float iq_bias,float iq_amp,float fre_hz,float lock_angle)
{
	sf_p->done_flag = 0;
	sf_p->start_flag = 0;
	
	sf_p->iq_bias = iq_bias;
	sf_p->iq_amp = iq_amp;
	sf_p->frequence_hz = fre_hz;
	sf_p->lock_angle = lock_angle;
	
	//自主选择周期数
	uint16_t point_cycle = Current_ISR_FRE / sf_p->frequence_hz;
	point_cycle = (uint16_t)ceil(300.0f/point_cycle);    //向上取整采样/等待周期
	sf_p->sample_cycle = point_cycle;
	
	sf_p->scanfre_state = SCANFRE_IINIT;
}


/*
	扫频法测试电流环带宽：
	
*/
void ScanFrequence_Task(ScanFre_Param* sf_p)
{
	switch(sf_p->scanfre_state)
	{
		case SCANFRE_IDLE:
		{
				pid_param.Iq_aim = 0.0f;
				pid_param.Id_aim = 0.0f;
			break;
		}
		case SCANFRE_IINIT:
		{
			sf_p->start_flag = 1;
			sf_p->wait_cnt = ScanFrequence_CntCal(sf_p->frequence_hz,sf_p->wait_cycle);
			sf_p->sample_cnt = ScanFrequence_CntCal(sf_p->frequence_hz,sf_p->sample_cycle);
			
			sf_p->phase = 0.0f;
			sf_p->phase_step = PI2_F * sf_p->frequence_hz / Current_ISR_FRE;
			
			sf_p->cnt_now = 0;
			sf_p->buf_now = 0;
			
			sf_p->scanfre_state = SCANFRE_WAIT;
			break;
		}
		case SCANFRE_WAIT:
		{
			sf_p->phase += sf_p->phase_step;
			if(sf_p->phase > PI2_F) sf_p->phase -= PI2_F;
			sf_p->iq_ref =  sf_p->iq_bias + sf_p->iq_amp*arm_sin_f32(sf_p->phase);
			
			pid_param.Iq_aim = sf_p->iq_ref;
			pid_param.Id_aim = 0.0f;
			
			sf_p->cnt_now++;
			if(sf_p->cnt_now>=sf_p->wait_cnt)
			{
				sf_p->cnt_now = 0;
				sf_p->scanfre_state = SCANFRE_SAMPLE;
			}
			break;
		}
		case SCANFRE_SAMPLE:
		{
			sf_p->phase += sf_p->phase_step;
			if(sf_p->phase > PI2_F) sf_p->phase -= PI2_F;
			sf_p->iq_ref =  sf_p->iq_bias + sf_p->iq_amp*arm_sin_f32(sf_p->phase);
			
			pid_param.Iq_aim = sf_p->iq_ref;
			pid_param.Id_aim = 0.0f;
			
			if(sf_p->buf_now<sf_p->buf_len)
			{
				sf_p->scanfre_buff[sf_p->buf_now].frequence = sf_p->frequence_hz;
				sf_p->scanfre_buff[sf_p->buf_now].iq_now = pid_param.Iq_now;
				sf_p->scanfre_buff[sf_p->buf_now].iq_ref = sf_p->iq_ref;
				sf_p->scanfre_buff[sf_p->buf_now].sample_index = sf_p->cnt_now;
				sf_p->buf_now++;
			}
			
			sf_p->cnt_now++;
			if(sf_p->cnt_now>=sf_p->sample_cnt || sf_p->buf_now>=sf_p->buf_len)
			{
				sf_p->start_flag = 0;
				sf_p->done_flag = 1;
				
				sf_p->scanfre_state = SCANFRE_DONE;
			}
			
			break;
		}
		case SCANFRE_DONE:
		{
			pid_param.Iq_aim = 0.0f;
			pid_param.Id_aim = 0.0f;
			break;
		}
		
	}
}


/*
	扫频法的数组打印
*/
void ScanFrequence_PrintBuff(void)
{
    uint16_t i;

//    printf("sample_index,frequence,iq_ref,iq_now\r\n");
    for(i = 0; i < scanfre_param.buf_now; i++)
    {
        printf("%u,%.3f,%.6f,%.6f\r\n",
               scanfre_param.scanfre_buff[i].sample_index,
               scanfre_param.scanfre_buff[i].frequence,
               scanfre_param.scanfre_buff[i].iq_ref,
               scanfre_param.scanfre_buff[i].iq_now);
    }
}

/*
	扫频法对数坐标频率值：生成波特图（对数坐标）扫频所需的频率数组
	此函数生成一个频率序列，在对数坐标下均匀分布，这是绘制波特图时进行扫频测试的标准方法
	因此序列要满足：log10(freq[i]) 构成一个等差数列
	参数：
		start_freq_Hz 扫频起始频率 (Hz), 必须 > 0
		end_freq_Hz 扫频终止频率 (Hz), 必须 > start_freq_Hz
		num_points 需要生成的总频率点数，必须 >= 2
		freq_array_Hz 用于存储生成的频率数组，至少 num_points 个浮点数的空间
	成功返回0，失败返回-1（参数错误）
*/
int generate_bode_frequencies(float start_freq_Hz,float end_freq_Hz,int num_points,float *freq_array_Hz) 
{
    // 参数有效性检查
    if (start_freq_Hz <= 0.0f || end_freq_Hz <= start_freq_Hz || num_points < 2 || !freq_array_Hz) 
		{
        fprintf(stderr, "错误：无效的输入参数。\n");
        return -1;
    }

    // 计算起始和终止频率的对数值
    double log_start = log10(start_freq_Hz);
    double log_end = log10(end_freq_Hz);

    // 计算对数坐标下的步长
    double log_step = (log_end - log_start) / (num_points - 1);

    // 生成频率点
    for (int i = 0; i < num_points; ++i) 
		{
        double current_log_freq = log_start + i * log_step;
        freq_array_Hz[i] = (float)pow(10.0, current_log_freq);
    }

    // 强制保证起点和终点的精确性，避免浮点误差
    freq_array_Hz[0] = start_freq_Hz;
    freq_array_Hz[num_points - 1] = end_freq_Hz;

    return 0;
}



/**
 * @brief 打印生成的频率数组，并验证其对数间隔特性
 * 
 * @param freq_array_Hz 频率数组
 * @param num_points 数组长度
 */
void print_and_verify_frequencies(const float *freq_array_Hz, int num_points)
{
    if (num_points < 2) return;

    printf("序号\t频率 (Hz)\t\t相邻频率比 (f[i]/f[i-1])\n");
    printf("------------------------------------------------------------------------\n");
    for (int i = 0; i < num_points; ++i) 
		{
        if (i == 0) 
				{
            printf("%4d\t%12.4f\t\t%s\n", i + 1, freq_array_Hz[i], "N/A (起点)");
        } 
				else 
				{
            float ratio = freq_array_Hz[i] / freq_array_Hz[i - 1];
            printf("%4d\t%12.4f\t\t%12.6f\n", i + 1, freq_array_Hz[i], ratio);
        }
    }
    printf("\n验证：相邻频率比应为常数。\n");
}

/*********************************************************************************************************************/






/*********************************************Kw辨析任务**************************************************************/
/*
	Kω = Kt/J，辨析Kw用于整定速度环PI参数
	因为在速度环PI里面：
	Kp = 2ζωn / Kω
	Ki_cont = ωn^2 / Kω ，Ki离散化还要乘执行周期Ts_v,即速度环执行周期
	ωn = 2π*f_bw_speed
	阻尼比ζ一般取0.707
	速度环模型简化有：J * dω/dt = Kt * Iq  得出-> dω/dt = (Kt/J) * Iq = Kω * Iq
*/
void SpeedID_Kw_Task(PID_Param* p)
{
    // 1kHz任务里调用
    static uint16_t cnt = 0;
    cnt++;

    p->Id_aim = 0.0f;

    // 0~200ms: 0A
    if (cnt < 200) {
        p->Iq_aim = 0.0f;
    }
    // 200~500ms: +0.2A
    else if (cnt < 500) {
        p->Iq_aim = 0.2f;
    }
    // 500~700ms: 0A
    else if (cnt < 700) {
        p->Iq_aim = 0.0f;
    }
    // 700~1000ms: -0.2A
    else if (cnt < 1000) {
        p->Iq_aim = -0.2f;
    }
    // 1000~1200ms: 0A
    else if (cnt < 1200) {
        p->Iq_aim = 0.0f;
    }
    else {
        cnt = 0;
    }
}

