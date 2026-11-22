#include "zf_common_headfile.h"

Motor_IncPID_t LM_PID;
Motor_IncPID_t RM_PID;
Servo_DualPD_t Servo_PID;
LADRC LM_LADRC;
LADRC RM_LADRC;
OmegaPID_t Omega_PID;

void pid_init(void)
{

        LM_PID.Kp = 10.0f;
		LM_PID.Ki = 0.8f;
		LM_PID.Kd = 0.0f;
        LM_PID.Kv = 12.0f; 
        LM_PID.Ka = 0.0f;
        LM_PID.alpha_D = 0.4f;

        LM_PID.last_error = 0.0f;
        LM_PID.last_derivative = 0.0f;
        LM_PID.last_setspeed = 0.0f;
        LM_PID.out_pid = 0.0f;     
        LM_PID.out = 0.0f;


        RM_PID.Kp = 10.0f;
		RM_PID.Ki = 0.8f;
		RM_PID.Kd = 0.0f;
        RM_PID.Kv = 12.0f; 
        RM_PID.Ka = 0.0f;
        RM_PID.alpha_D = 0.4f;
        
        RM_PID.last_error = 0.0f;
        RM_PID.last_derivative = 0.0f;
        RM_PID.last_setspeed = 0.0f;
        RM_PID.out_pid = 0.0f;     
        RM_PID.out = 0.0f;

		Servo_PID.Kp=0.0f;
        Servo_PID.Kp2 =0.0f;  
        Servo_PID.Kd=0.0f; 
        Servo_PID.Gyro_Kd = 0.005f;

        Omega_PID.Kp = 0.0f;
        Omega_PID.Kd = 0.0f;
        Omega_PID.error = 0.0f;
        Omega_PID.last_error = 0.0f;
        Omega_PID.out_p = 0.0f;
        Omega_PID.out_d = 0.0f;
        Omega_PID.out = 0.0f;
        Omega_PID.limit = 0.5f;      


}


void ladrc_init(void)
{
    LM_LADRC.wc = 10.0f;       // 控制器带宽 
    LM_LADRC.w0 = 18.0f;       // 观测器带宽 
    LM_LADRC.b  = 0.5f;        // 系统增益补偿
    LM_LADRC.kd = 1.0f;        // 扰动补偿系数 
    
    LM_LADRC.Ts = 0.005f;      // 采样周期 
    LM_LADRC.limit = 3000.0f;  // PWM 输出限幅 

    LM_LADRC.z1 = 0.0f;
    LM_LADRC.z2 = 0.0f;
    LM_LADRC.e = 0.0f;
    LM_LADRC.u0 = 0.0f;
    LM_LADRC.out = 0.0f;
    LM_LADRC.last_u = 0.0f;

    RM_LADRC.wc = 10.0f;       
    RM_LADRC.w0 = 18.0f;       
    RM_LADRC.b  = 0.4f;        
    RM_LADRC.kd = 1.0f;        
    
    RM_LADRC.Ts = 0.005f;      
    RM_LADRC.limit = 3000.0f;  

    RM_LADRC.z1 = 0.0f;
    RM_LADRC.z2 = 0.0f;
    RM_LADRC.e = 0.0f;
    RM_LADRC.u0 = 0.0f;
    RM_LADRC.out = 0.0f;
    RM_LADRC.last_u = 0.0f;
}

void Ladrc_Reset_Single(LADRC *ladrc)
{
    if (ladrc == NULL) return;
    
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->last_u = 0.0f;
    ladrc->out = 0.0f;
    ladrc->e = 0.0f;
    ladrc->u0 = 0.0f;
}

void Motor_IncPID_Reset(Motor_IncPID_t *pid)
{
    if (pid == NULL) return;
    pid->last_setspeed = 0.0f;
    pid->last_actspeed = 0.0f;
    pid->last_filtered_delta_error = 0.0f;
    pid->out_pid = 0.0f;
    pid->error = 0.0f;
    pid->filtered_delta_error = 0.0f;  
    pid->out_ff = 0.0f;
    pid->out_p = 0.0f;
    pid->out_i = 0.0f;
    pid->out = 0.0f;
}

#define OUT_MAX   ( 5000.0f)
#define OUT_MIN   (-5000.0f)
//前馈+微分先行增量式pid
float PidIncCtrl(Motor_IncPID_t * pid, float setspeed, float actspeed)
{
            float delta_error;
     
            pid->out_ff = pid->Kv * setspeed + 
                          pid->Ka * (setspeed - pid->last_setspeed) / DT;
            pid->last_setspeed = setspeed;

            pid->error = setspeed - actspeed;
            delta_error = pid->last_actspeed - actspeed;
            pid->last_actspeed = actspeed;

            pid->filtered_delta_error = pid->alpha_D * delta_error + 
                                (1.0f - pid->alpha_D) * pid->last_filtered_delta_error;
            pid->last_filtered_delta_error = pid->filtered_delta_error;
            
			pid->out_p = pid->Kp * pid->filtered_delta_error;
            pid->out_i = pid->Ki * pid->error;

			pid->out_pid += pid->out_p + pid->out_i; 

            pid->out = pid->out_ff + pid->out_pid;

            if (pid->out > OUT_MAX) {
                pid->out = OUT_MAX;
                pid->out_pid = OUT_MAX - pid->out_ff;  
            } else if (pid->out < OUT_MIN) {
                pid->out = OUT_MIN;
                pid->out_pid = OUT_MIN - pid->out_ff;
            }
            
			return pid->out;
}

//舵机双PD控制
float Servo_PidLocCtrl(Servo_DualPD_t* pid, float error, float gyro_z)
{
    pid->out_p = pid->Kp * error;
    pid->out_p2 = pid->Kp2 * error * abs_float(error);

    pid->out_d = pid->Kd * (error - pid->last_error);
    pid->out_gyro = pid->Gyro_Kd * gyro_z;

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_p2 + pid->out_d + pid->out_gyro;


    return pid->out;
}

float Omega_PidCtrl(OmegaPID_t* pid, float omega_target, float omega_actual)
{
    pid->error = omega_target - omega_actual;
    
    pid->out_p = pid->Kp * pid->error;
    pid->out_d = pid->Kd * (pid->error - pid->last_error);
    
    pid->last_error = pid->error;
    
    pid->out = pid->out_p + pid->out_d;
    
    //双向限幅
    if (pid->out > pid->limit)  pid->out = pid->limit;
    if (pid->out < -pid->limit) pid->out = -pid->limit;
    
    return pid->out;
}

float LadrcCtrl(LADRC *ladrc, float setspeed, float actspeed)
{
    float z1_new;
    float z2_new;
    
    //ESO 观测器误差
    ladrc->e = actspeed - ladrc->z1;
    
    //ESO 离散化更新
    z1_new = ladrc->z1 + ladrc->Ts * (ladrc->z2 + 2.0f * ladrc->w0 * ladrc->e + ladrc->b * ladrc->last_u);
    z2_new = ladrc->z2 + ladrc->Ts * (ladrc->w0 * ladrc->w0 * ladrc->e);
    ladrc->z1 = z1_new;
    ladrc->z2 = z2_new;
    

    ladrc->u0 = ladrc->wc * (setspeed - ladrc->z1);
    

    ladrc->out = (ladrc->u0 - ladrc->kd * ladrc->z2) / ladrc->b;
    
    //输出限幅
    if (ladrc->out > ladrc->limit)
        ladrc->out = ladrc->limit;
    else if (ladrc->out < -ladrc->limit)
        ladrc->out = -ladrc->limit;
   
    ladrc->last_u = ladrc->out;
    
    return ladrc->out;
}


int16 limit_float(int16 x, int16 y)
{
    if(x > y)        return y;
    else if(x < -y)  return -y;
    else             return x;
}

float ERROR_MAX = 27.0;                         //误差最大值
float DEDT_MAX = 6.0;                           //误差变化率最大值
float KP_Fuzzy = 2.3;                           //kp模糊值
float KD_Fuzzy = 0.0;                          //kd模糊值

float KP_Base = 0.2;                              //kp基础值
float KD_Base = 0.0;                              //kd基础值
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWW Error论域 WWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

#define ER_ZO0                              (0.0f)
#define ER_PSS                              (3.5f)
#define ER_PSB                              (4.0f)
#define ER_PMS                              (4.5f)
#define ER_PMB                              (5.0f)
#define ER_PBS                              (5.5f)
#define ER_PBB                              (6.0f)

const float Er_lunYu[7] = {ER_ZO0, ER_PSS, ER_PSB, ER_PMS, ER_PMB, ER_PBS, ER_PBB};
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

/* WWWWWWWWWWWWWWWWWWWWWWWWWWWW dedt论域 WWWWWWWWWWWWWWWWWWWWWWWWWWWWW */
#define DT_NB                               (-3.0f)
#define DT_NM                               (-2.0f)
#define DT_NS                               (-1.0f)
#define DT_Z0                               (0.0f)
#define DT_PS                               (1.0f)
#define DT_PM                               (2.0f)
#define DT_PB                               (3.0f)

const float Dedt_lunYu[7] = {DT_NB, DT_NM, DT_NS, DT_Z0, DT_PS, DT_PM, DT_PB};
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

static float er_liShu[2];                      //er隶属
static float dedt_liShu[2];                    //dedt隶属
static float er_liShuDu[2];                    //er隶属度
static float dedt_liShuDu[2];                  //dedt隶属度
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWW KP规则 WWWWWWWWWWWWWWWWWWWWWWWWWWWWW */
#define KP_ZO0                              (0.0f)
#define KP_PSS                              (1.0f)
#define KP_PSB                              (2.0f)
#define KP_PMS                              (3.0f)
#define KP_PMB                              (4.0f)
#define KP_PBS                              (5.0f)
#define KP_PBB                              (6.0f)
const float KP_Rule[7][7] = {
/*  er/dedt          DT_NB   DT_NM   DT_NS   DT_Z0   DT_PS   DT_PM   DT_PB */
    /*ER_ZO0*/      {KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0, KP_ZO0},
    /*ER_PSS*/      {KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS, KP_PSS},
    /*ER_PSB*/      {KP_PSS, KP_PSS, KP_PSS, KP_PSB, KP_PMS, KP_PMS, KP_PMS},
    /*ER_PMS*/      {KP_PSB, KP_PSB, KP_PMS, KP_PMS, KP_PMS, KP_PMB, KP_PMB},
    /*ER_PMB*/      {KP_PMS, KP_PMS, KP_PMB, KP_PMB, KP_PMB, KP_PBS, KP_PBS},
    /*ER_PBS*/      {KP_PMB, KP_PBS, KP_PBS, KP_PBS, KP_PBS, KP_PBS, KP_PBB},
    /*ER_PBB*/      {KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB, KP_PBB},
};
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

/* WWWWWWWWWWWWWWWWWWWWWWWWWWWW KD规则 WWWWWWWWWWWWWWWWWWWWWWWWWWWWW */
#define KD_ZO0                              (0.0f)
#define KD_PSS                              (1.0f)
#define KD_PSB                              (2.0f)
#define KD_PMS                              (3.0f)
#define KD_PMB                              (4.0f)
#define KD_PBS                              (5.0f)
#define KD_PBB                              (6.0f)
const float KD_Rule[7][7] = {
/*  er/dedt          DT_NB   DT_NM   DT_NS   DT_Z0   DT_PS   DT_PM   DT_PB */
    /*ER_ZO0*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMB, KD_PMB, KD_PMS, KD_PSB},
    /*ER_PSS*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMB, KD_PMB, KD_PMS, KD_PSB},
    /*ER_PSB*/      {KD_PBB, KD_PBS, KD_PMB, KD_PMS, KD_PMS, KD_PSB, KD_PSB},
    /*ER_PMS*/      {KD_PBS, KD_PMB, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS},
    /*ER_PMB*/      {KD_PMB, KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSS, KD_ZO0},
    /*ER_PBS*/      {KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS, KD_ZO0},
    /*ER_PBB*/      {KD_PMS, KD_PMS, KD_PMS, KD_PSB, KD_PSB, KD_PSS, KD_ZO0},
};
/* WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW */

/* 函数用处：   隶属度获取
 *
 * 传入参数：   float value         映射到论域上的值
 * 传入参数：   float* liShu        映射值的隶属
 * 传入参数：   float* liShuDu      映射值的隶属度
 * 传入参数：   const int* lunYu    论域
 * 函数介绍：   统一隶属度获取接口，由于er和dedt的隶属逻辑相同，故封装统一接口便于代码阅读
 */
void calculateMembership (float value, float* liShu, float* liShuDu, const float* lunYu)
{
	int i;
    if (value > lunYu[0] && value < lunYu[6]) {
        for (i = 0; i < 6; i++) {
            if (lunYu[i] < value && value <= lunYu[i + 1]) {             /* 判断在哪个区间内 */
                /* 三角隶属度 */
                liShu[0] = i;                                                                  /* 记录前隶属 */
                liShu[1] = i + 1;                                                              /* 记录后隶属 */
                liShuDu[0] = (lunYu[i + 1] - value) / (lunYu[i + 1] - lunYu[i]);               /* 前隶属度:靠近lunYu[i]时大 */
                liShuDu[1] = (value - lunYu[i])     / (lunYu[i + 1] - lunYu[i]);               /* 后隶属度:靠近lunYu[i+1]时大 */
                break;
            }
        }
    } else if (value <= lunYu[0]) {
        /* 三角隶属度 */
        liShu[0] = 0;                   /* 记录前隶属 */
        liShu[1] = 1;                   /* 记录后隶属 */
        liShuDu[0] = 1.0;               /* 记录前隶属度(100%) */
        liShuDu[1] = 0;                 /* 记录后隶属度 */
    } else if (value >= lunYu[6]) {
        /* 三角隶属度 */
        liShu[0] = 5;                   /* 记录前隶属 */
        liShu[1] = 6;                   /* 记录后隶属 */
        liShuDu[0] = 0;                 /* 记录前隶属度 */
        liShuDu[1] = 1.0;               /* 记录后隶属度(100%) */
    }
}

/* 函数用处：   获取er与de/dt的隶属度
 * 
 * 传入参数：   float er        当前差值
 * 函数介绍：   传入输出与输入目标值的差值作为er, 内部自动更新上一次误差, 对误差和变化率做归一化映射
 */
void get_degree_of_membership (float er)
{
    static float laster = 0;
    float dedt = er - laster;

    float erRemap = er / ERROR_MAX * ER_PBB;        /* 对er映射 */
    float dedtRemap = dedt / DEDT_MAX * DT_PB;      /* 对dedt映射 */

    //误差为负时 取反
    if(erRemap < 0) {
        erRemap = -erRemap;
        dedtRemap = -dedtRemap;
    }
    //误差在正负震荡时让KD最大
    if(laster * er < 0) {
        dedtRemap = DT_NB;
    }

    calculateMembership(erRemap, er_liShu, er_liShuDu, Er_lunYu);
    calculateMembership(dedtRemap, dedt_liShu, dedt_liShuDu, Dedt_lunYu);

    laster = er;
}

void upPIDdata_pd(float* kp, float* kd)
{
    float kptmp = er_liShuDu[0] * dedt_liShuDu[0] * KP_Rule[(int)er_liShu[0]][(int)dedt_liShu[0]]
                + er_liShuDu[0] * dedt_liShuDu[1] * KP_Rule[(int)er_liShu[0]][(int)dedt_liShu[1]]
                + er_liShuDu[1] * dedt_liShuDu[0] * KP_Rule[(int)er_liShu[1]][(int)dedt_liShu[0]]
                + er_liShuDu[1] * dedt_liShuDu[1] * KP_Rule[(int)er_liShu[1]][(int)dedt_liShu[1]];

    float kdtmp = er_liShuDu[0] * dedt_liShuDu[0] * KD_Rule[(int)er_liShu[0]][(int)dedt_liShu[0]]
                + er_liShuDu[0] * dedt_liShuDu[1] * KD_Rule[(int)er_liShu[0]][(int)dedt_liShu[1]]
                + er_liShuDu[1] * dedt_liShuDu[0] * KD_Rule[(int)er_liShu[1]][(int)dedt_liShu[0]]
                + er_liShuDu[1] * dedt_liShuDu[1] * KD_Rule[(int)er_liShu[1]][(int)dedt_liShu[1]];

    kptmp = (kptmp / KP_PBB) * KP_Fuzzy;
    kdtmp = (kdtmp / KD_PBB) * KD_Fuzzy;

    *kp = KP_Base + kptmp;
    *kd = KD_Base + kdtmp;
}


