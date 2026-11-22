#include "zf_common_headfile.h"
//---------------------------------蜂鸣器
void Buzzer_init(void)
{
	gpio_init(Buzzer_pin,GPO,0,GPO_PUSH_PULL);
}
void Buzzer_check(int ms)
{
	gpio_set_level(Buzzer_pin,1);
	system_delay_ms(ms);
	gpio_set_level(Buzzer_pin,0);                                                                                                                                                                                                                             
} 
void Buzzer_check1(int ms)//非阻塞式          
{
    gpio_set_level(Buzzer_pin, 1);
    buzzer_remain_ms = ms;            
}
void Buzzer_on(void)
{
    gpio_set_level(Buzzer_pin, 1);
}
void Buzzer_off(void)
{
    gpio_set_level(Buzzer_pin, 0);
}
//---------------------------------LED
void LED_init(void)
{
	gpio_init(LED_pin,GPO,0,GPO_PUSH_PULL);
}
//---------------------------------按键
uint8 key1_state = 1;                                                              
uint8 key2_state = 1;                                                               
uint8 key3_state = 1;                                                               
uint8 key4_state = 1;                                                            
uint8 key1_state_last = 1;                                                          
uint8 key2_state_last = 1;                                                        
uint8 key3_state_last = 1;                                                      
uint8 key4_state_last = 1;                                                         
uint8 key1_flag;																																		
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;
void Key_init(void)
{
	gpio_init(KEY1,GPI,1,GPI_PULL_UP);
	gpio_init(KEY2,GPI,1,GPI_PULL_UP);
	gpio_init(KEY3,GPI,1,GPI_PULL_UP);
	gpio_init(KEY4,GPI,1,GPI_PULL_UP);
	
	gpio_init(SWITCH1,GPI,1, GPI_PULL_UP);
	gpio_init(SWITCH2,GPI,1, GPI_PULL_UP);

}
void Key_scan(void)
{
	key1_state_last = key1_state;
	key2_state_last = key2_state;
	key3_state_last = key3_state;
	key4_state_last = key4_state;
	key1_state = gpio_get_level(KEY1);
	key2_state = gpio_get_level(KEY2);
	key3_state = gpio_get_level(KEY3);
	key4_state = gpio_get_level(KEY4);	
	if(key1_state && !key1_state_last)   {key1_flag = 1;}
	if(key2_state && !key2_state_last)   {key2_flag = 1;}
	if(key3_state && !key3_state_last)   {key3_flag = 1;}
	if(key4_state && !key4_state_last)   {key4_flag = 1;}

}
//清零标志位
void KEY1_clear(void)
{
	
	key1_flag = 0;	
}

void KEY2_clear(void)
{
	
	key2_flag = 0;
	
}

void KEY3_clear(void)
{
	
	key3_flag = 0;
	
}

void KEY4_clear(void)
{
	
	key4_flag = 0;
	
}
//---------------------------------电调
uint16 duty = 0;

void Esc_init()
{
	pwm_init(Esc_PWM_1, FREQ, 0);                   
    pwm_init(Esc_PWM_2, FREQ, 0);                   
}
void Esc_set(int duty)
{
	if(duty>850)
		duty = 850;
	pwm_set_duty(Esc_PWM_1,duty);
    pwm_set_duty(Esc_PWM_2,duty);
    
}
//---------------------------------舵机
float SERVO_MOTOR_LMAX = 120 ;		 //(120)
float SERVO_MOTOR_RMAX = 85 ;		 //(85)
void Steer_init(void)
{
	pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_MID));
}
void Steer_set(float angle)
{
	  if(angle>SERVO_MOTOR_LMAX){angle=SERVO_MOTOR_LMAX;}
      if(angle<SERVO_MOTOR_RMAX){angle=SERVO_MOTOR_RMAX;}
	  pwm_set_duty(SERVO_MOTOR_PWM,(uint32)SERVO_MOTOR_DUTY(angle));
}

//---------------------------------	电机
void DRV8701_init()
{
	gpio_init(L_DIR,GPO,1,GPO_PUSH_PULL);
	pwm_init(L_PWM, 17000, 0);
	gpio_init(R_DIR,GPO,1,GPO_PUSH_PULL);
	pwm_init(R_PWM, 17000, 0);
}
void DRV8701_ctrl(int16 L_SPEED,int16 R_SPEED)
{
	L_SPEED=L_SPEED>M_MAX?M_MAX:(L_SPEED<M_MIN)?M_MIN:L_SPEED;
	R_SPEED=R_SPEED>M_MAX?M_MAX:(R_SPEED<M_MIN)?M_MIN:R_SPEED;
	if(L_SPEED>0)//左正转
	{
		gpio_set_level (L_DIR, 0);
		pwm_set_duty  (L_PWM, L_SPEED);
	}
	else//左反转
	{
		gpio_set_level (L_DIR, 1);
		pwm_set_duty  (L_PWM, -L_SPEED);
	}
	if(R_SPEED>0)//右正转
	{
		gpio_set_level (R_DIR, 0);
		pwm_set_duty  (R_PWM,R_SPEED);
	}
	else//右反转
	{
		gpio_set_level (R_DIR, 1);
		pwm_set_duty  (R_PWM, -R_SPEED);
	}
	
}
//--------------------------------- 正交编码器
int16 LA_SPEED=0;//左实际速度
int16 RA_SPEED=0;//右实际速度
void encoder_init()
{
	encoder_quad_init(ENCODER_QUAD_1, ENCODER_QUAD_1_CHA, ENCODER_QUAD_1_CHB);   
    encoder_quad_init(ENCODER_QUAD_2, ENCODER_QUAD_2_CHA, ENCODER_QUAD_2_CHB);   
}
void encoder_pulse_get()
{                 
	LA_SPEED  = encoder_get_count(ENCODER_QUAD_2);                 
    RA_SPEED  = -encoder_get_count(ENCODER_QUAD_1);                  
    encoder_clear_count(ENCODER_QUAD_1);                                		
    encoder_clear_count(ENCODER_QUAD_2);  
}

//----------------------------------------------------WiFiSPI
void WifiSpiInit(void)
{
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST)){system_delay_ms(100); }
    while(wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT)) {system_delay_ms(100); }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X, 
        (uint8 *)image_small[0], 
        IMAGE_W_SMALL, 
        IMAGE_H_SMALL
    );

    seekfree_assistant_camera_boundary_config(
        X_BOUNDARY, 
        IMAGE_H_SMALL, 
        l_border, mid_line, r_border, 
        NULL, NULL, NULL
    );
}

char wireless_buf[64];
void Wireless_Cmd_Process(void)
{
    uint8 cmd_dat = 0;
    if (wireless_uart_read_buffer(&cmd_dat, 1) == 1) 
    {
        switch (cmd_dat)
        {
            case 'g':       //'g' = go 发车
            case 'G':
                car_run_flag = 1;
                Buzzer_check1(50);
                wireless_uart_send_string("Car GO\r\n");
                break;
            
            case 's':       //'s' = stop 停车
            case 'S':
                car_run_flag = 0;
                Buzzer_check1(50);
                wireless_uart_send_string("Car STOP\r\n");
                break;
            
            case 'b':       //'b' = buzzer 蜂鸣
            case 'B':
                Buzzer_check1(200);
                wireless_uart_send_string("Beep\r\n");
                break;
            
            case '+':       //加速
                Target_Speed_test += 10.0f;
                sprintf(wireless_buf, "Speed=%.1f\r\n", Target_Speed_test);
                wireless_uart_send_string(wireless_buf);
                break;
            
            case '-':       //减速
                Target_Speed_test -= 10.0f;
                sprintf(wireless_buf, "Speed=%.1f\r\n", Target_Speed_test);
                wireless_uart_send_string(wireless_buf);
                break;
            
            default:
                break;
        }
    }
}
//-------------------------------激光
void laser_init(void)
{
	gpio_init(Laser_mid, GPO, 0, GPO_PUSH_PULL);
	gpio_init(Laser_left, GPO, 0, GPO_PUSH_PULL);
	gpio_init(Laser_right, GPO, 0, GPO_PUSH_PULL);
}

void laser_mid_on(void)
{
	gpio_set_level(Laser_mid, 1);
}
void laser_left_on()
{
	gpio_set_level(Laser_left, 1);
}
void laser_right_on()
{
	gpio_set_level(Laser_right, 1);
}


void laser_mid_off(void)
{
	gpio_set_level(Laser_mid, 0);
	
}
void laser_left_off(void)
{
	gpio_set_level(Laser_left, 0);
	
}
void laser_right_off(void)
{
	gpio_set_level(Laser_right, 0);
}
void laser_off()
{
	laser_right_off();
	laser_left_off();
	laser_mid_off();						
}

//--------------------------------- 陀螺仪
gyro_param_t GyroOffset;
euler_param_t eulerAngle;
void gyroOffset_init(void)//零漂初始化
{
    uint16 i;
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    //Buzzer_check(50);
    //system_delay_ms(500);
    for (i = 0; i < 1000; ++i) 
    {
        imu660ra_get_gyro();

        GyroOffset.Xdata += imu660ra_gyro_x;
        GyroOffset.Ydata += imu660ra_gyro_y;
        GyroOffset.Zdata += imu660ra_gyro_z;

        system_delay_ms(2);
    }
    GyroOffset.Xdata /= 1000;
    GyroOffset.Ydata /= 1000;
    GyroOffset.Zdata /= 1000;
    
    eulerAngle.Dirchange = 0;
    Buzzer_check(50);
	system_delay_ms(500);

}
//计算偏航角
float yaw_total = 0;
float current_gyro_z_dps = 0;
void Gyro_Update(void)
{
    static float gyro_filt = 0;
    float raw_dps;
    
    imu660ra_get_gyro();
    raw_dps = ((float)imu660ra_gyro_z - GyroOffset.Zdata) / 16.4f;
    yaw_total += (raw_dps * 0.005f);
    gyro_filt = 0.5f * raw_dps + 0.5f * gyro_filt;
    current_gyro_z_dps = gyro_filt;
}

volatile uint16 start_delay_ms = 0;
volatile uint16 buzzer_remain_ms = 0;
volatile uint16 print_interval_ms = 0;
volatile uint8 print_flag = 0;
char print_buf[32];
void Timer(void)
{
    //蜂鸣器倒计时
    if (buzzer_remain_ms > 0)
    {
        if (buzzer_remain_ms <= 5)
        {
            buzzer_remain_ms = 0;
            gpio_set_level(Buzzer_pin, 0);
        }
        else
        {
            buzzer_remain_ms -= 5;
        }
    }
    
    //发车倒计时
    if (start_delay_ms > 0)
    {
        if (start_delay_ms <= 5)
        {
            start_delay_ms = 0;
            car_run_flag = 1;       //时间到自动发车
        }
        else
        {
            start_delay_ms -= 5;
        }
    }

    //打印倒计时 (200ms 一次)
    if (print_interval_ms > 0)
    {
        print_interval_ms -= 5;
    }
    else
    {
        print_interval_ms = 200;     //重置 200ms
        print_flag = 1;              //置位,通知主循环可以打印
    }
}

//----------------------------------------------------------------------------------测试函数
void Steer_text(void)//舵机测试
{

       static int32 angle=SERVO_MOTOR_MID;

          Key_scan();
          if(key1_flag)
             {
                 key1_flag=0;
                 angle+=10;
             }
          if(key2_flag)
             {
                 key2_flag=0;
                 angle-=10;
             }
          if(key3_flag)
             {
                 key3_flag=0;
                 angle+=1;
    //             angle=55;//左打死
             }
          if(key4_flag)
             {
                 key4_flag=0;
                angle-=1;
    //             angle=85;//右打死
             }


          ips200_show_string(0,100, "Steer_text");
          ips200_show_int32(100,  16*3,angle, 5);
          Steer_set(angle);
}
void Encoder_text(void)
{
	 ips200_show_string(8*0,16*0, "L_SPEED");   		ips200_show_int32(8*9,16*0,LA_SPEED,5);
     ips200_show_string(8*0,16*1, "R_SPEED");   		ips200_show_int32(8*9,16*1,RA_SPEED,5);
    
}

float Target_Speed_test = 100.0f; 
