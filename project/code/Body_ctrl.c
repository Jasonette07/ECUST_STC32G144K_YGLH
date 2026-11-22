#include "zf_common_headfile.h"

float final_angle = SERVO_MOTOR_MID;
void Servo_Control_Process(float Image_err)
{
    float pid_out;
    
    pid_out = Servo_PidLocCtrl(&Servo_PID, Image_err, imu660ra_gyro_z);
    final_angle = SERVO_MOTOR_MID + pid_out;
    Steer_set(final_angle);


}

void PID_Scheduler(void)
{
    float kp_fuzzy, kd_fuzzy;
    //模糊pid
    get_updataPD_pid((float)posi, &kp_fuzzy, &kd_fuzzy);
    Servo_PID.Kp = kp_fuzzy;
    Servo_PID.Kd = kd_fuzzy;

}

void get_updataPD_pid (float er, float *kp, float *kd)
{
    get_degree_of_membership(er);
    upPIDdata_pd(kp, kd);
}

float Target_Speed_L = 0.0f;
float Target_Speed_R = 0.0f;
float ackCoeff = ACK_COEFF_DEFAULT; //默认差速系数
#define DIFF_K1  0.0322f
#define DIFF_K2  0.000088f
#define INNER_RATIO_MIN  0.5f      

void realDiff(float base_speed, float current_angle)
{
    float dev = current_angle - SERVO_MOTOR_MID;
    float abs_dev = (dev < 0) ? -dev : dev;
    float inner_ratio;
    float outer_speed, inner_speed;
    float omega_corr;    
    //1. 基础差速比
    inner_ratio = 1.0f - DIFF_K1 * abs_dev + DIFF_K2 * abs_dev * abs_dev;
    inner_ratio = 1.0f - (1.0f - inner_ratio) * ackCoeff;    
    //2. 角速度闭环双向修正
    omega_corr = omega_assist(current_angle);
    inner_ratio -= omega_corr;	
    //3. 下限保护 
    if (inner_ratio < 0.0f) inner_ratio = 0.0f;
    //上限保护 
    if (inner_ratio > 1.0f) inner_ratio = 1.0f;   
    //4. 平均速度基准分配
    outer_speed = 2.0f * base_speed / (1.0f + inner_ratio);
    inner_speed = outer_speed * inner_ratio;
    
    if (current_angle > SERVO_MOTOR_MID + 0.5f)
    {
        Target_Speed_L = inner_speed;
        Target_Speed_R = outer_speed;
    }
    else if (current_angle < SERVO_MOTOR_MID - 0.5f)
    {
        Target_Speed_R = inner_speed;
        Target_Speed_L = outer_speed;
    }
    else
    {
        Target_Speed_L = base_speed;
        Target_Speed_R = base_speed;
    }
}
float calc_target_omega_dps(float v, float servo_angle)
{
    float dev = servo_angle - SERVO_MOTOR_MID;
    float wheel_angle_rad = dev * STEERING_RATIO * 0.01745f;
    float omega_rad = v * tan(wheel_angle_rad) / LDISTANCE;   //rad/s    
    //rad/s → dps (度/秒): 乘 180/π = 57.2958
    return omega_rad * 57.2958f;
}

float omega_assist(float servo_angle)
{
    float v_now;
    float omega_target;
    float omega_actual;
    float abs_target, abs_actual;
    float force_error;       //转向力度误差
    float correction;
    
    v_now = ((float)LA_SPEED + (float)RA_SPEED) / 2.0f * SPEED_TO_MMPS;
    omega_target = calc_target_omega_dps(v_now, servo_angle);
    omega_actual = current_gyro_z_dps;
    
    //转向力度 = 角速度绝对值
    abs_target = (omega_target < 0) ? -omega_target : omega_target;
    abs_actual = (omega_actual < 0) ? -omega_actual : omega_actual;
    
    //力度误差: >0 转不够(加大差速), <0 转过头(减小差速)
    force_error = abs_target - abs_actual;
    correction = Omega_PidCtrl(&Omega_PID, abs_target, abs_actual);
    return correction;
}

//电机控制策略
void motorSpeedPidControl(float set_speed)
{
    float pwm_L, pwm_R;
    encoder_pulse_get(); 	
	//真实差速
    realDiff(set_speed, final_angle);
    pwm_L = PidIncCtrl(&LM_PID, Target_Speed_L, (float)LA_SPEED);
    pwm_R = PidIncCtrl(&RM_PID, Target_Speed_R, (float)RA_SPEED);
    //无差速
    // pwm_L = PidIncCtrl(&LM_PID, set_speed, (float)LA_SPEED);
    // pwm_R = PidIncCtrl(&RM_PID, set_speed, (float)RA_SPEED);
    DRV8701_ctrl((int16)pwm_L, (int16)pwm_R);   
}

void motorSpeedLadrcControl(float set_speed)
{
    float pwm_L, pwm_R;
    encoder_pulse_get();     
    //真实差速
    realDiff(set_speed, final_angle);
    pwm_L = LadrcCtrl(&LM_LADRC, Target_Speed_L, (float)LA_SPEED);
    pwm_R = LadrcCtrl(&RM_LADRC, Target_Speed_R, (float)RA_SPEED);
    //无差速
    // pwm_L  = LadrcCtrl(&LM_LADRC, set_speed, (float)LA_SPEED);
    // pwm_R = LadrcCtrl(&RM_LADRC, set_speed, (float)RA_SPEED);  
    DRV8701_ctrl((int16)pwm_L, (int16)pwm_R);   
}



static uint8 laser_state = 0;       //0=关, 1=开

void laser_toggle_by_key(void)
{
    if (key3_flag == 1)
    {
        KEY3_clear();
        
        if (laser_state == 0)
        {
            laser_mid_on();
            laser_state = 1;
            //Buzzer_check1(50);
        }
        else
        {
            laser_off();
            laser_state = 0;
        }
    }
}
//发车标志位与按键控制
volatile uint8 car_run_flag = 0;
extern volatile uint16 start_delay_ms;

void Car_State_Update(void)
{
    //按键处理: 切换发车/停车
    if(key1_flag == 1)
    {
        KEY1_clear();
        
        if(car_run_flag == 0)
        {
            start_delay_ms = 500;//500ms倒计时发车
            Buzzer_check1(50);
        }
        else
        {
            car_run_flag = 0;
            start_delay_ms = 0;
            Buzzer_check1(50);
        }
    }

    //车控逻辑
    if(car_run_flag == 1)
    {
        PID_Scheduler();
        Servo_Control_Process(posi);
        //motorSpeedPidControl(speed);
         motorSpeedLadrcControl(speed);
		// motorSpeedLadrcControl(zebra_detect ? ZEBRA_SPEED : speed);
		 Esc_set( 650);
    }
    else
    {
		//stop_box = 0;
        Steer_set(SERVO_MOTOR_MID);
//        motorSpeedPidControl(0.0f);
//        Motor_IncPID_Reset(&LM_PID);
//        Motor_IncPID_Reset(&RM_PID);
        motorSpeedLadrcControl(0.0f);
       
		Esc_set(0);
    }
}

void pit_handler(void)//定时器isr
{

    Key_scan();
    //encoder_pulse_get();        // 先更新 LA_SPEED / RA_SPEED
    //Encoder_Accumulate();           //再累加到标定变量
    //laser_toggle_by_key();
    Gyro_Update();
    Car_State_Update();
    Timer();
    
}

