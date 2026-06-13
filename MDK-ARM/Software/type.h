#ifndef __TYPE_H
#define __TYPE_H

#include "main.h"
#include "math.h"
#include "arm_math.h"         //DSP库头文件
#include "usart.h"
#include <stdio.h>


#define PI_F 3.14159265f                
#define PI2_F 6.283185307f             
#define Current_ISR_FRE   20000.0f     //电流环执行频率
#define Speed_ISR_FRE  1000.0f         //速度环执行频率

#define Limit(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/*
函数功能:
	包含通用头文件，并且定义结构体和全局变量
*/

//电机参数结构体
typedef struct
{
	float supply_Udc;      //供电电压
	uint8_t pole;          //电机极对数
	float motor_gear;      //电机减速比
	float motor_phaseL;    //电机相电感
	float motor_phaseR;    //电机相电阻
	float motor_Lq;        //电机q轴电感
	float motor_Ld;        //电机d轴电感
	float motor_filed_link;//磁永磁体磁链
	uint16_t Tpwm;         //定时器计数最大值
	float Rs;              //采样电阻大小
	float Gain;            //运放增益大小
	float I_Width;         //电流环带宽大小
	float Kw;              //Kw = Kt/J,单位：rad/s2/A，用于速度环PI整定使用
	float Speed_Width;     //速度环带宽大小
}Motor_Param;  

//dq轴电压限幅策略选择
typedef enum
{
    V_LIMIT_VECTOR = 0,   //等比例矢量限幅,默认模式
    V_LIMIT_Q_PRIORITY,   //q轴优先
    V_LIMIT_D_PRIORITY    //d轴优先
}VLimitMode;

//前馈解耦策略选择
typedef enum
{
    FOC_CC_DECOUPLING_DISABLED   = 0,  //不解耦
    FOC_CC_DECOUPLING_CROSS      = 1,  //只交叉耦合（dq轴电流解耦）
    FOC_CC_DECOUPLING_BEMF       = 2,  //只反电势
    FOC_CC_DECOUPLING_CROSS_BEMF = 3   //交叉+反电势
}FOC_CC_DecouplingMode;    //磁场定向电流控制解耦模式

//电机运行标志位结构体
typedef struct  //所有标志位默认为0
{
	uint8_t Error_Flag;                //为1表示过压，2为欠压，3表示过流
	uint8_t Adc_OffectOver_Flag;       //1代表ADC校准完成
	uint8_t Zero_Flag;                 //1代表零偏校准完成，默认为1，需要校准时再零偏校准
	uint8_t Econder_Mode;              //编码器模式，1为开环自增角度，2为闭环真实角度
	uint8_t Mode_Select;               //电机运行模式选择，1为SPWM运行，2为SVPWM运行，3为电流环运行，4为速度-电流环运行，5为位置-速度-电流环运行
	VLimitMode v_limit_mode;           //dq轴电压限幅模式选择：1为q轴优先，2为d轴优先，3为等比例限幅
	FOC_CC_DecouplingMode dec_mode;    //电流环前馈控制模式选择，0不补偿，1dq轴解耦，2反电动势补偿，3都补偿
	uint8_t Death_Compensation_Enable; //死区补偿使能标志位，为1开启死区补偿，0不补偿
	uint8_t pid_param_flag;    
}Motor_Flag;


typedef struct
{
	float Ua,Ub,Uc;        //ABC三相电压值
	float Ualpha,Ubeta;     
	float supply_Udc;        //母线电压
	uint16_t Tpwm;         //定时器计数最大值
}SPWM_Param;

typedef struct
{
	float Udc;             //母线电压大小
	uint16_t Ts;         //定时器计数最大值
}SVPWM_Param;

typedef struct
{
	float Ia_Sample,Ib_Sample,Ic_Sample;    //ADC采样原始值，12位ADC：0-4095
	float Ia_offect,Ib_offect,Ic_offect;    //ADC三相电流偏置值，正常是3.3V/2 = 1.65V
	float Ia,Ib,Ic,Udc;
	uint16_t Iadc_offect_counts;            //三相电流偏置值计次
	float IGain;                            //运算放大器和采样电阻组合的电流增益大小：IGain = Rs(采样电阻大小)*运放倍数
}ADCTask_Param;

typedef struct
{
	uint16_t Encoder_Max;           //编码器最大值，14位磁编最大值16384
	uint16_t Encoder_raw;           //编码器原始数据，0-16383
	uint8_t motordir;               //编码器旋转方向
	float Shaft_Angle;              //机械角度
	float Elect_Angle;              //电角度，电角度=机械角度*极对数
	float Zero_Angle_Sum;           //零偏校准角度和
	float Zero_Angle;               //零偏角度，机械0与电角度0的差值
	uint16_t Zero_counts;           //零偏校准次数
	uint16_t Zero_n;                //实际零偏计次次数
	float Return_Angle;             //真实程序使用的角度值
	float Return_Rads;              //真实使用的弧度值
	float vf_v;                     //vf强拖的系数v
	float vf_k;                     //vf系数
	float virtual_step;             //每次自增的虚拟角度步长
	float sin_dsp,cos_dsp;          //用Return_Angle
	
	//PLL锁相环参数
	float pll_theta_hat;  //PLL内部状态：机械角(rad)
	float pll_omega_hat;  //PLL内部状态：机械角速度(rad/s)
	float pll_zeta;       //PLL阻尼比
	float pll_wn;         //wn：PLL 自然频率，单位 rad/s建议100~150
	float pll_kp;
	float pll_ki;
	uint8_t pll_init_flag; //第一次PLL初始化对齐标志位，1代表已初始化对齐
	
}EncoderTask_Param;

typedef struct
{
	//电流环参数
	float Kp_I,Ki_I;
	float Iq_aim,Id_aim;
	float Iq_now,Id_now;
	float Iqd_Max;
	float Ki_I_SumMax;
	float erro_iq_sum,erro_id_sum;
	float Uq,Ud;
	float Uout_Max;
	
	//速度环参数
	float Kp_S,Ki_S;                //速度环PI参数
	
	float Speed_now;      //输出轴实际速度，单位：rad/s
	float Speed_filt;               //输出轴滤波速度
	float Motor_Speed_aim,Motor_Speed_now;          //电机轴原始机械速度(过减速器前速度)，rad/s
	float Motor_Speed_filt_now;     //电机轴滤波机械速度，rad/s
	
	float Speed_lpf_k;              //速度一阶低通滤波系数
	float Speed_erro_sum;           //速度积分项
	float Speed_KISumMax;           //速度积分限幅
	float Speed_Max;                //速度环设置限幅(速度设置为输出轴速度限制，单位rad/s)
	
	uint16_t Speed_Div;             //分频系数，决定速度环执行频率,电流环执行频率/Speed_Div = 速度环执行频率
	uint16_t Speed_cnt;             //分频计数
	float _1_Ts;                       //速度环周期倒数（1/s），计算速度的时候进行乘法
	
	float Shaft_Angle_Last;         //上一次机械角度（deg）
	
}PID_Param;

//电阻参数辨析状态
typedef enum
{
	RsID_IDLE = 0,           //空闲模式，还未完成初始化,赋值为0，默认为RsID_IDLE，不赋值就会导致为0出错误
	RsID_Pos_Set,            //正向电压设置，设置完之后要等稳定在进下一个状态采集
	RsID_Pos_Average,        //正向电压设置后电流平均值采集
	RsID_Neg_Set,            //反向电压设置，设置完之后要等稳定在进下一个状态采集
	RsID_Neg_Average,        //反向电压设置后电流平均值采集
	RsID_Done                //参数辨析完成
}RsID_State;

//电机参数辨析结构体参数
typedef struct
{
	//Rs电阻辨析
	RsID_State Rsid_state;    //电阻辨析状态枚举
	
	float Ud_Set;             //参数辨析Ud设置值，一般取（1.0-1.5）
	uint16_t cnt_sum;         //执行状态的累计次数
	uint16_t wait_cnt;        //等待稳定所花的次数，20k的执行频率，等待1200次，为60ms
	uint16_t collect_cnt;     //电流稳定后采集的次数，20k执行频率，采集2000次，为100ms
	uint8_t RsID_Start;       //电阻辨析初始化完成标志位，置1表示初始化完成，下一步开始辨析
	float lock_angle;         //电机锁定角度，参数辨析时保持在该角度不变

	float Id_collect_sum;     //电流采集累计值
	float Id_pos_avg;         //正向电流平均值
	float Id_neg_avg;         //反向电流平均值
	
	float Rs_result;          //相电阻测量结果
	uint8_t Rs_done;          //相电阻辨析结束，1表示结束
	
	//Ld、Lq电感辨析
	float Udq_inject;         //相电感Udq注入电压大小
	float frequency_inject;   //注入方波频率值，可以进行更改，不要过低，过低时电机电阻压降影响大
	uint8_t Ldq_select;       //dq轴电感测量，默认为0辨析q轴电感，1辨析d轴电感
	
	uint16_t half_cnts;       //计算出半个周期需要计次值总是多少
	uint16_t Ldq_half_cnt;    //半个周期计次值，对比看是否该执行下半个周期
	uint16_t half_index;      //半个周期计次数，看完成了多少半个周期
	uint16_t calculate_cnt;   //计算周期总执行次数
	uint16_t Ldq_cnt;         //实际计算的电感L的次数
	
	uint16_t LdqID_Start;     //电感辨析初始化完成标志位
	uint16_t Ldq_done;        //电感辨析结束标志位
	
	float Ldq_sum;            //最终所有半周期计算累计的电感和
	
	float i_start_half;        // 半周期起点电流
	float i_end_half;          // 半周期终点电流
	float i_sum_half;          // 半周期电流累加
	
	float Lq_result;          //q轴电感求解值
	float Ld_result;          //d轴电感求解值

	
}MotorID_Param;


//扫频法测电流环带宽结构体
//扫频法数据采集结构体数组参数
typedef struct
{
	uint16_t sample_index;        // 第几个20k中断点
	float frequence;              // 当前测试频率
	float iq_ref;                 // Iq目标
	float iq_now;                 // Iq实际
}ScanFre_Sample;

//扫频法状态机
typedef enum
{
	SCANFRE_IDLE = 0,           //扫频法空闲状态
	SCANFRE_IINIT,              //扫频法初始化状态
	SCANFRE_WAIT,               //扫频法等待稳定状态
	SCANFRE_SAMPLE,             //扫频法采集状态
	SCANFRE_DONE                //扫频法完成状态
}ScanFre_State;

//扫频法结构体汇总
typedef struct
{
	ScanFre_State scanfre_state;   
	
	ScanFre_Sample* scanfre_buff;  //扫频法采集的结构体数组
	
	uint8_t start_flag;           //扫频法开启标志位，为1表示初始化完成
	uint8_t done_flag;            //扫频法结束标志位，为1表示此次频率扫频完成
	
	float frequence_hz;           //当前扫频频率设定
	float iq_bias;                //扫频直流分量大小，默认设置为0
	float iq_amp;                 //扫频幅值大小设定
	float iq_ref;                 //扫频法当前目标值设定，iq_ref = iq_bias + iq_amp*arm_sin_f32(phase);
	float lock_angle;             //当前扫频时的锁轴角度，无意义，可以对照参考扫频时电机是否有大抖动
	
	float phase;                  //当前相位值
	float phase_step;             //当前频率下的相位步进值
	
	uint16_t wait_cycle;          //等待稳定的周期数，不使用该周期时的数据
	uint16_t sample_cycle;        //采样周期数，采集sample_cycle个周期的数据
	uint16_t wait_cnt;            //等待稳定周期数所需的总计次值
	uint16_t sample_cnt;          //采样周期所需的的总计次值
	uint16_t cnt_now;             //当前计次值，用于比较确定当前处于什么状态
	
	uint16_t buf_len;             //采样存储数组长度
	uint16_t buf_now;             //采样存储数组当前的下标值
}ScanFre_Param;

//编码器非线性校准结构体
#define ENC_NLCAL_FIFO_LEN   64

typedef struct
{
    uint32_t idx;             // 第几个采样点
    int32_t  cmd_mech_tick;   // 理想机械角，对应0~16383一圈
    uint16_t encoder_raw;     // MT6701原始值
} EncoderNLCal_Frame;

typedef struct
{
    uint8_t start_flag;       // 1:开始扫描
    uint8_t done_flag;        // 1:扫描完成
    int8_t  dir;              // +1正转，-1反转

    uint16_t div_cnt;         // 分频计数
    uint16_t div_num;         // 分频系数，100表示20kHz/100=200Hz，即5ms采一点

    float ud;                 // 强拖Ud
    float step_deg;           // 每次推进的电角度，单位deg
    float cmd_deg;            // 当前命令电角度，单位deg
    float mech_tick_sum;      // 累计走过的机械角tick，用来判断是否扫满一圈

    uint32_t sample_idx;      // 当前采样序号

    uint16_t fifo_wr;         // FIFO写指针
    uint16_t fifo_rd;         // FIFO读指针
    EncoderNLCal_Frame fifo[ENC_NLCAL_FIFO_LEN];
} EncoderNLCal_Param;


#endif
