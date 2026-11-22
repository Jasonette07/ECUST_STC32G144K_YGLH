
#define DT 0.005f

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
    float Kv;              // 速度前馈系数
    float Ka;              // 加速度前馈系数
	float error;
	float out_p;
	float out_i;
    float out_ff;
	float out_pid;
    float out;
    float alpha_D;                   // PD滤波系数
    float filtered_delta_error;      // 本次滤波后的误差增量
    float last_filtered_delta_error; // 上次滤波后的误差增量 
    float last_actspeed;
	float last_error; //上次误差
    float last_derivative;//上次误差与上上次误差之差
    float last_setspeed; //上次设置速度
}Motor_IncPID_t;
typedef struct
{
    float Kp;        // 比例系数
    float Kd;        // 微分系数
    float Kp2;       // 非线性比例系数 
    float Gyro_Kd;   // 陀螺仪角速度前馈系数    
    float out_p;
    float out_p2;    // 非线性项单独输出
    float out_d;
    float out_gyro;  // 陀螺仪阻尼项单独输出
    float out;       // 最终输出    
    float last_error; // 上次偏差
} Servo_DualPD_t;
typedef struct {
    float wc;           
    float w0;           
    float b;            
    float kd;           
    float Ts;           
    float limit;        
    float Kv;           
    float Ka;           
    float last_setspeed; 
    float e;
    float u0;
    float out_ff;       
    float out_fb;       
    float out;
    float z1;
    float z2;
    float last_u;
} LADRC;

typedef struct {
    float Kp;
    float Kd; 
    float error;
    float last_error;  
    float out_p;
    float out_d;
    float out;   
    float limit;          
} OmegaPID_t;


extern  Motor_IncPID_t LM_PID;
extern  Motor_IncPID_t RM_PID;
extern Servo_DualPD_t Servo_PID;
extern LADRC LM_LADRC;
extern LADRC RM_LADRC;
extern OmegaPID_t Omega_PID;

void pid_init(void);
void ladrc_init(void);
void Ladrc_Reset_Single(LADRC *ladrc);


#define abs_float(x) ((x) > 0.0f ? (x) : -(x))
#define abs_int(x)   ((x) > 0 ? (x) : -(x))

float PidIncCtrl(Motor_IncPID_t * pid, float setspeed, float actspeed);
void Motor_IncPID_Reset(Motor_IncPID_t *pid);
float Servo_PidLocCtrl(Servo_DualPD_t* pid, float error, float gyro_z);
float Omega_PidCtrl(OmegaPID_t* pid, float omega_target, float omega_actual);

int16 limit_float(int16 x, int16 y);

float LadrcCtrl(LADRC *ladrc, float setspeed, float actspeed);

void upPIDdata_pd(float* kp, float* kd);
void get_degree_of_membership (float er);


