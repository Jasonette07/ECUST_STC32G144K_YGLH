#define Buzzer_pin 			     IO_P65

#define LED_pin 		         IO_P52

#define KEY1 	    	 		 IO_PB2
#define KEY2 			 		 IO_PB3
#define KEY3			 		 IO_PB4
#define KEY4 			 		 IO_P32

#define SWITCH1 IO_PB0
#define SWITCH2 IO_PB1

#define SERVO_MOTOR_PWM 	     PWME_CH1P_PA0
#define SERVO_MOTOR_FREQ 		 (300)
#define SERVO_MOTOR_MID  		 (102)
extern float SERVO_MOTOR_LMAX; 		 //(120)
extern float SERVO_MOTOR_RMAX; 		 //(85)
#define SERVO_MOTOR_DUTY(x)      ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
	
#define R_DIR 					 IO_P50
#define R_PWM					 PWMD_CH2_P51
#define L_DIR					 IO_P52
#define L_PWM					 PWMD_CH4_P53
#define M_MAX                    8000
#define M_MIN                    -8000

#define ENCODER_QUAD_1                 	(PWMA_ENCODER)              
#define ENCODER_QUAD_1_CHA            	(PWMA_ENCODER_CH1P_P60)     
#define ENCODER_QUAD_1_CHB              (PWMA_ENCODER_CH2P_P62)     
#define ENCODER_QUAD_2                 	(PWMC_ENCODER)              
#define ENCODER_QUAD_2_CHA   		    (PWMC_ENCODER_CH1P_P40)    
#define ENCODER_QUAD_2_CHB       	    (PWMC_ENCODER_CH2P_P42)     

#define Laser_mid IO_P11
#define Laser_left IO_P63
#define Laser_right IO_P67


#define FREQ               (50)                                                // ����Ƶ��Ϊ50HZ�����֧��300HZ
#define Esc_PWM_1          (PWMF_CH3_PA5)
#define Esc_PWM_2          (PWMF_CH4_PA7)


extern uint8 key1_flag;																																		
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;

extern int16 LA_SPEED;
extern int16 RA_SPEED;

void Buzzer_init();
void Buzzer_check(int ms);
void Buzzer_check1(int ms);
void Buzzer_on();
void Buzzer_off();

void LED_init(void);

void Key_init(void);
void Key_scan(void);
void KEY1_clear(void);
void KEY2_clear(void);
void KEY3_clear(void);
void KEY4_clear(void);

void Steer_init(void);
void Steer_set(float angle);
void Steer_text(void);

void DRV8701_init(void);
void DRV8701_ctrl(int16 L_SPEED,int16 R_SPEED);

void encoder_init();
void encoder_pulse_get();
void Encoder_text(void);

#define WIFI_SSID_TEST          "iQOO Neo9"
#define WIFI_PASSWORD_TEST      "jiang051104"

void WifiSpiInit(void);

void laser_init(void);
void laser_mid_on(void);
void laser_right_on(void);
void laser_left_on(void);
void laser_mid_off(void);
void laser_left_off(void);
void laser_right_off(void);
void laser_off(void);


typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;


typedef struct {
    float pitch; //������
    float roll; //�����
    float yaw; //ƫ����
    float last_yaw;
    int8 Dirchange;
} euler_param_t;

void gyroOffset_init(void);

extern float yaw_total;
extern float current_gyro_z_dps;
void Gyro_Update(void);

void Timer(void);
extern volatile uint16 start_delay_ms;
extern volatile uint16 buzzer_remain_ms;
extern volatile uint16 print_interval_ms;
extern volatile uint8 print_flag;
extern char print_buf[32];
extern char wireless_buf[64];


extern float Target_Speed_test;

void Esc_init();
void Esc_set(int duty);


