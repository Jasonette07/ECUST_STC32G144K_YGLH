#include "zf_common_headfile.h"


void main(void)
{
	uint16 cnt = 0;
	char buf[32];
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
    //此处编写用户代码 例如外设初始化代码等
    //gpio_init(IO_P61, GPO, GPIO_LOW, GPO_PUSH_PULL);  //输出 默认低电平 推挽输出模式
    Buzzer_init();
    Key_init();	
    Steer_init();
    DRV8701_init();
    encoder_init();
    laser_init();
    pid_init();
    ladrc_init();
    mt9v03x_init();
    wireless_uart_init();
    //ips200_init();
   //WifiSpiInit();
	//ips200_init();
    DRV8701_ctrl(0,0); 
    Steer_set(SERVO_MOTOR_MID);
    imu660ra_init();
    gyroOffset_init();
  	pit_ms_init(TIM0_PIT, 5, pit_handler);
	interrupt_set_priority(TIMER0_IRQn, 0);
	Esc_init();
    Buzzer_check(100);//上电鸣叫1秒
	
    

    while(1)
    {
            
            if(mt9v03x_finish_flag)
                {
                    //gpio_high(IO_P61);
                    imageProcess();
                    //gpio_low(IO_P61);  
                   // seekfree_assistant_camera_send();
                    //print_left_edge();
                    //print_right_edge();
                    mt9v03x_finish_flag = 0;
                }

        
       // printf("%d,%d,%d\r\n",imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z);
        //偏航角测试
        // printf("Yaw: %.2f\r\n", yaw_total);
        // system_delay_ms(500);
		//Esc_set(600);
//		cnt++;	
//  if (cnt >= 50)  // 250ms 一次，不拖主循环
//  {
//      sprintf(buf, "L=%d R=%d\r\n", LA_SPEED, RA_SPEED);
//          wireless_uart_send_string(buf);
//          cnt = 0;
//  }
    }                 
}