
extern volatile uint8 car_run_flag;


void Motor_Speed_Control(float set_speed);
float calc_target_omega_dps(float v, float servo_angle);
float omega_assist(float servo_angle);
void realDiff(float base_speed, float current_angle);
void get_updataPD_pid (float er, float *kp, float *kd);
typedef struct {
    float Kp;
    float Kp2;
    float Kd;
    float Gyro_Kd;
} PID_Set_t;
#define LDISTANCE 200.0f 
#define BORDWIDTH 150.0f 
#define STEERING_RATIO   2.626f  
#define speed 145.0f  
#define SPEED_TO_MMPS   40.0f       
#define RAD_TO_DEG      57.2958f  
#define DEG_TO_RAD      0.01745f    
#define ACK_COEFF_DEFAULT       1.0f    
#define ACK_COEFF_ROUNDABOUT    1.5f     
#define ACK_COEFF_CROSSROAD     1.5f     
// void Servo_Control_Process(float Image_err);
extern float ackCoeff; 
extern volatile uint16 start_delay_count;
void Car_State_Update(void);
 void motorSpeedLadrcControl(float set_speed); 

void pit_handler(void);