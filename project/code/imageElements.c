#include "zf_common_headfile.h"

//十字路口识别
uint8 cross_left_near = 0;     //左近端丢线点
uint8 cross_left_far = 0;      //左远端丢线点
uint8 cross_right_near = 0;    //右近端丢线点
uint8 cross_right_far = 0;     //右远端丢线点
uint8 crossroad0(void)
{
    uint8 left_lost_row_near, left_lost_row_far;
    uint8 right_lost_row_near, right_lost_row_far;
    uint8 left_gap, right_gap;
    uint8 diff_near, diff_far;
    
    //左边线判断
    left_lost_row_near = scanFirstLostPoint(l_effect_flag, l_start, first_end, 5, 2);
    if (left_lost_row_near == 0) return 0;
    left_lost_row_far = scanFirstLostPoint(l_effect_flag, first_end, left_lost_row_near, 2, 2);
    if (left_lost_row_far == 0) return 0;
    if (left_lost_row_near <= left_lost_row_far) return 0;
    left_gap = left_lost_row_near - left_lost_row_far;
    if (left_gap < 5) return 0;
    if (Check_Straight_Line(l_border, l_start - 1, left_lost_row_near + 2, 4) == 0)
        return 0;
    if (checkLostLine(l_effect_flag, left_lost_row_near, left_lost_row_far, 1))
        return 0;
	
    //右边线判断  
    right_lost_row_near = scanFirstLostPoint(r_effect_flag, r_start, first_end, 5, 2);
    if (right_lost_row_near == 0) return 0;
    
    right_lost_row_far = scanFirstLostPoint(r_effect_flag, first_end, right_lost_row_near, 2, 2);
    if (right_lost_row_far == 0) return 0;
    
    if (right_lost_row_near <= right_lost_row_far) return 0;
    right_gap = right_lost_row_near - right_lost_row_far;
    if (right_gap < 5) return 0;
    
    if (Check_Straight_Line(r_border, r_start - 1, right_lost_row_near + 2, 4) == 0)
        return 0;
    if (checkLostLine(r_effect_flag, right_lost_row_near, right_lost_row_far, 1))
        return 0;
    
    //左右对称性     
    diff_near = (left_lost_row_near > right_lost_row_near) ?
                (left_lost_row_near - right_lost_row_near) :
                (right_lost_row_near - left_lost_row_near);
    if (diff_near > 11) return 0;
    
    diff_far = (left_lost_row_far > right_lost_row_far) ?
               (left_lost_row_far - right_lost_row_far) :
               (right_lost_row_far - left_lost_row_far);
    if (diff_far > 5) return 0;

    cross_left_near  = left_lost_row_near;
    cross_left_far   = left_lost_row_far;
    cross_right_near = right_lost_row_near;
    cross_right_far  = right_lost_row_far;
    
    return 1;
}

uint8 crossroad1(void)
{
    uint8 l_first_lost;
    uint8 r_first_lost;
    uint8 l_seg_len;
    uint8 r_seg_len;   
    //左边线
    l_first_lost = scanFirstLostPoint(l_effect_flag, l_start, first_end, 3, 2);
    if (l_first_lost == 0) return 0;  
    if (l_first_lost - 10 < first_end) return 0;     
    if (checkLostLine(l_effect_flag, l_first_lost - 1, l_first_lost - 10, 0)) return 0;
	
    //右边线
    r_first_lost = scanFirstLostPoint(r_effect_flag, r_start, first_end, 3, 2);
    if (r_first_lost == 0) return 0;
    
    if (r_first_lost - 10 < first_end) return 0;
    if (checkLostLine(r_effect_flag, r_first_lost - 1, r_first_lost - 10, 0)) return 0;
    
    //判断
    if (l_start <= l_first_lost) return 0;            //方向异常
    if (r_start <= r_first_lost) return 0;
    
    l_seg_len = l_start - l_first_lost;
    r_seg_len = r_start - r_first_lost;
    
if (l_seg_len >= 20 && r_seg_len >= 20) 
    {
        return 0; 
    }
    
    return 1;
}

uint8 crossroad2(void)
{
    uint8 l_lost, l_valid;
    uint8 r_lost, r_valid;
    
    //左边线 
    l_lost = scanFirstLostPoint(l_effect_flag, IMAGE_H_SMALL - 1, first_end, 0, 3);
    if (l_lost == 0) return 0;
    l_valid = scanFirstValidPoint(l_effect_flag, l_lost, first_end, 3);
    if (l_valid == 0) return 0;
    if ((IMAGE_H_SMALL - 1 - l_valid) >= 30) return 0;   
    //右边线
    r_lost = scanFirstLostPoint(r_effect_flag, IMAGE_H_SMALL - 1, first_end, 0, 3);
    if (r_lost == 0) return 0;
    
    r_valid = scanFirstValidPoint(r_effect_flag, r_lost, first_end, 3);
    if (r_valid == 0) return 0;
    
    if ((IMAGE_H_SMALL - 1 - r_valid) >= 30) return 0;
    
    return 1;
}

//十字补线
void crossFillLine(void)
{
    uint8 left_lost_row_near;
    uint8 left_lost_row_far;
    uint8 right_lost_row_near;
    uint8 right_lost_row_far;
    
    // 左边线 
    left_lost_row_near = scanFirstLostPoint(l_effect_flag, l_start, first_end, 3, 2);
    if (left_lost_row_near == 0) goto right_fill;
    left_lost_row_far = scanFirstLostPoint(l_effect_flag, first_end, left_lost_row_near, 2, 2);
    if (left_lost_row_far == 0) goto right_fill;
    connect_point(l_border, left_lost_row_near + 3, left_lost_row_far - 3);   
right_fill:
    //右边线 
    
    right_lost_row_near = scanFirstLostPoint(r_effect_flag, r_start, first_end, 3, 2);
    if (right_lost_row_near == 0) return; 
    right_lost_row_far = scanFirstLostPoint(r_effect_flag, first_end, right_lost_row_near, 2, 2);
    if (right_lost_row_far == 0) return;    
    connect_point(r_border, right_lost_row_near + 3, right_lost_row_far - 3);
}

//垂直补线
void crossFillLine_vertical(void)
{
    uint8 l_top, r_top;
	
//    static uint8 print_count = 0;
	
    l_top = scanFirstLostPoint(l_effect_flag, first_end, IMAGE_H_SMALL - 1, 2, 3);
    r_top = scanFirstLostPoint(r_effect_flag, first_end, IMAGE_H_SMALL - 1, 2, 3);
     fill_vertical_down(l_border, l_top-3);
     fill_vertical_down(r_border, r_top-3);
}

void crossFillLine_vertical1(void)
{
    uint8 l_first_lost, l_valid_top;
    uint8 r_first_lost, r_valid_top;
    //左边线
    l_first_lost = scanFirstLostPoint(l_effect_flag, IMAGE_H_SMALL - 1, first_end, 0, 2);   
        l_valid_top = scanFirstValidPoint(l_effect_flag, l_first_lost, first_end, 1);        
            fill_vertical_down(l_border, l_valid_top-3);    
    //右边线 
    r_first_lost = scanFirstLostPoint(r_effect_flag, IMAGE_H_SMALL - 1, first_end, 0, 2);  
        r_valid_top = scanFirstValidPoint(r_effect_flag, r_first_lost, first_end, 1);      
            fill_vertical_down(r_border, r_valid_top-3);
}

uint8 crossState = 0; //十字路口状态机的状态变量
uint8 Cross_State = 0;
float Cross_Start_Yaw = 0;
 uint8 Crossroad_Passed_Count = 0;
uint8 Total_Crossroad_Num = 10; 

void crossroadState(void)
{
    //十字识别期间不能和环岛冲突
    if (Left_Roundabout_State != 0 || Right_Roundabout_State != 0) 
		return;
	if (Crossroad_Passed_Count >= Total_Crossroad_Num)
          return; 
    switch (Cross_State)
    {
        case 0:
            if (crossroad0() == 1)
            {
                Cross_State = 1;
                crossState = 1; 
                Cross_Start_Yaw = yaw_total;
                //Buzzer_check1(50);
                //Buzzer_on();
            }
            break;
        
        case 1:
            if (crossroad1() == 1)
            {
                Cross_State = 2;
                //Buzzer_check1(50);
            }
            crossFillLine();
            break;
        
        case 2:
            if ((Both_Lost_Time <= 8))
            {
                Cross_State = 3;
                //Buzzer_check1(50);
            }
//			SERVO_MOTOR_LMAX = 107;
//			SERVO_MOTOR_RMAX = 97;
            crossFillLine_vertical(); 
            break;
            
        case 3: 
            if (Both_Lost_Time > 20)
            {
                Cross_State = 4;
                //crossState = 0;
                //Buzzer_check1(50);
                //Buzzer_off();
            }
//			SERVO_MOTOR_LMAX = 120;
//			SERVO_MOTOR_RMAX = 85;
            break;
            
        case 4:
            if (crossroad2() == 1)
            {
                Cross_State = 0;
                crossState = 0;
                Buzzer_check1(50);
				 Crossroad_Passed_Count++;
                //Buzzer_off();
            }
            crossFillLine_vertical1();
            break;
            
        default:
            Cross_State = 0;
            break;
    }
}

//左环岛识别
uint8 leftRoundabout0(void)
{
    uint8 left_lost_row;
    uint8 right_lost_row;
    
    //左右共同丢线数 >= 3直接排除 
    if (Both_Lost_Time >= 5)
    {
        //printf("[Fail 0] Both_Lost=%d\r\n", Both_Lost_Time);
        return 0;
    }
    
    //左边线判断逻辑
    left_lost_row = scanFirstLostPoint(l_effect_flag, l_start, first_end + 1, 5, 1);
    if(left_lost_row == 0 || left_lost_row <= 28)
    {
        //printf("[Fail 1] L_start=%d, L_lost=%d\r\n", l_start, left_lost_row);
        return 0;
    }

    if (Check_Straight_Line(l_border, l_start - 2, left_lost_row + 1, 1) == 0)
    {
        //printf("[Fail 2] L_start=%d, L_end=%d\r\n", l_start, left_lost_row);
        return 0;
    }

    if (checkLostLine(l_effect_flag, left_lost_row, left_lost_row - 3, 0))
    {
        //printf("[Fail 3] L_lost=%d\r\n", left_lost_row);
        return 0;
    }
    
    //右边线判断逻辑
    right_lost_row = scanFirstLostPoint(r_effect_flag, r_start, first_end + 1, 5, 1);
    if (right_lost_row == 0)
    {
        right_lost_row = first_end;
    }

    if (Check_Straight_Line(r_border, r_start - 2, right_lost_row + 2, 1) == 0)
    {
        //printf("[Fail 4] R_start=%d, R_lost=%d\r\n", r_start, right_lost_row);
        return 0;
    }
	if(r_start<28)
		return 0;
	if (Check_Straight_Line(r_border, r_start , r_start-28, 1) == 0)
    {
        //printf("[Fail 4] R_start=%d, R_lost=%d\r\n", r_start, right_lost_row);
        return 0;
    }

    return 1;
}

//右环岛识别
uint8 rightRoundabout0(void)
{
    uint8 right_lost_row;
    uint8 left_lost_row;
    
    //左右共同丢线数 >= 直接排除
    if (Both_Lost_Time >= 5)
    {
        //printf("[Fail 0] Both_Lost=%d\r\n", Both_Lost_Time);
        return 0;
    }
 
    //右边线判断逻辑
    right_lost_row = scanFirstLostPoint(r_effect_flag, r_start, first_end, 5, 1);
    if (right_lost_row == 0|| right_lost_row <= 28)
    {
        //printf("[Fail 1] R_start=%d, R_lost=%d\r\n", r_start, right_lost_row);
        return 0;
    }
 
    if (Check_Straight_Line(r_border, r_start - 2, right_lost_row + 1, 1) == 0)
    {
        //printf("[Fail 2] R_start=%d, R_end=%d\r\n", r_start, right_lost_row);
        return 0;
    }
 
    if (checkLostLine(r_effect_flag, right_lost_row, right_lost_row - 3, 0))
    {
        //printf("[Fail 3] R_lost=%d\r\n", right_lost_row);
        return 0;
    }
 
    //左边线判断逻辑
    left_lost_row = scanFirstLostPoint(l_effect_flag, l_start, first_end, 5, 1);
    if (left_lost_row == 0)
    {
        left_lost_row = first_end;
    }
 
    if (Check_Straight_Line(l_border, l_start - 2, left_lost_row + 2, 1) == 0)
    {
        //printf("[Fail 4] L_start=%d, L_lost=%d\r\n", l_start, left_lost_row);
        return 0;
    }
	if(l_start<28)
		return 0;
	if (Check_Straight_Line(l_border, l_start , l_start-28, 1) == 0)
    {
        //printf("[Fail 4] R_start=%d, R_lost=%d\r\n", r_start, right_lost_row);
        return 0;
    }
 
    return 1;
}

uint8 leftRoundabout1(void)
{
    uint8 valid_start;
    uint8 lost_row;
    uint8 scan_top;
    if (checkLostLine(l_effect_flag, IMAGE_H_SMALL - 1, 25, 3) != 0)
    {
        return 0;
    }
//    scan_top = first_end;
//    if (scan_top >= 35) return 0;
//    valid_start = scanFirstValidPoint(l_effect_flag, 35, scan_top, 5);
//    if (valid_start == 0)
//    {
//        return 0;
//    }
//    lost_row = scanFirstLostPoint(l_effect_flag, valid_start, first_end, 1, 1);
//    if (lost_row == 0)
//    {
//        lost_row = first_end;
//    }
//    if (valid_start <= lost_row + 1) return 0;
//    if (Check_Monotonic_Increase(l_border, valid_start, lost_row + 1, 0) == 0)
//    {
//        return 0;
//    }
    
    return 1;
}

//右环岛入环判断
uint8 rightRoundabout1(void)
{
    uint8 valid_start;
    uint8 lost_row;
    uint8 scan_top;
    if (checkLostLine(r_effect_flag, IMAGE_H_SMALL - 1, 25, 3) != 0)
    {
        return 0;
    }
//    scan_top = first_end;
//    if (scan_top >= 35) return 0;
//    valid_start = scanFirstValidPoint(r_effect_flag, 35, scan_top, 5);
//    if (valid_start == 0)
//    {
//        return 0;
//    }
//    lost_row = scanFirstLostPoint(r_effect_flag, valid_start, first_end, 1, 1);
//    if (lost_row == 0)
//    {
//        lost_row = first_end;
//    }
//    if (valid_start <= lost_row + 1) return 0;
//    if (Check_Monotonic_Decrease(r_border, valid_start, lost_row + 1, 0) == 0)
//    {
//        return 0;
//    }

    return 1;
}
//左环岛出口判断
uint8 checkLeftRoundaboutExit(void)
{
    if (l_start == 0) return 0;
    if (r_start == 0) return 0;
    if ((IMAGE_H_SMALL - 1 - l_start) < 12 &&
        (IMAGE_H_SMALL - 1 - r_start) < 12)
    {
        return 1;
    }    
    return 0;
}

//右环岛出口判断
uint8 checkRightRoundaboutExit(void)
{
    if (l_start == 0) return 0;
    if (r_start == 0) return 0;
    if ((IMAGE_H_SMALL - 1 - l_start) < 13 &&
        (IMAGE_H_SMALL - 1 - r_start) < 13)
    {
        return 1;
    }
    
    return 0;
}

//环岛数量限制
Roundabout_Dir_e Roundabout_Seq[] = {RightRoundabout};

//自动计算需要跑的总环岛数 
uint8 Total_Roundabout_Num = sizeof(Roundabout_Seq) / sizeof(Roundabout_Seq[0]);

//环岛主控状态机
uint8 Left_Roundabout_State = 0; 
float Left_Round_Start_Yaw = 0; 
uint8 Right_Roundabout_State = 0; 
float Right_Round_Start_Yaw = 0;
uint8 Current_Roundabout_Index = 0;
void roundaboutState(void)
{
    Roundabout_Dir_e expected_dir;
    // 记录当前已经成功出环的个数
	if (Cross_State != 0) 
		return;
    if (Current_Roundabout_Index >= Total_Roundabout_Num)
    {
        return; 
    }
    expected_dir = Roundabout_Seq[Current_Roundabout_Index];
    if (expected_dir == LeftRoundabout)
    {
        switch (Left_Roundabout_State)
        {
            case 0:
                if (leftRoundabout0() == 1) 
                {
                    Left_Roundabout_State = 1;
                    Current_Track_Mode = TRACK_RIGHT_SINGLE;
                    //Buzzer_on();
                    //Buzzer_check1(100);
                }
                break;
            case 1:
                if (leftRoundabout1() == 1)
                {
                    Left_Roundabout_State = 2; 
                    Left_Round_Start_Yaw = yaw_total;
                    Current_Track_Mode = TRACK_LEFT_SINGLE;
					ackCoeff = ACK_COEFF_ROUNDABOUT; 
					SERVO_MOTOR_RMAX = 112;
                    //Buzzer_check1(200);
                }
                break;

            case 2:
                if (abs_float(yaw_total - Left_Round_Start_Yaw) > 45.0f)
                {
                    Left_Roundabout_State = 3; 
                    Current_Track_Mode = TRACK_DOUBLE;
					SERVO_MOTOR_RMAX = 107;
                    //Buzzer_check1(200);
                }
                break;
            case 3:
                if (abs_float(yaw_total - Left_Round_Start_Yaw) > 245.0f)
                {
                    Left_Roundabout_State = 4; 
                    Current_Track_Mode = TRACK_LEFT_SINGLE;
                   // Buzzer_check1(200);
                }
                break;
            case 4:
                if (abs_float(yaw_total - Left_Round_Start_Yaw) > 315.0f)
                {
                    Left_Roundabout_State = 5; 
                    Current_Track_Mode = TRACK_RIGHT_SINGLE;
				    SERVO_MOTOR_RMAX = 85;
                    //Buzzer_check1(200);
                }
                break;

            case 5:
                if (checkLeftRoundaboutExit() == 1)
                {
                    Left_Roundabout_State = 0; 
                    ackCoeff = ACK_COEFF_DEFAULT; 
                    Current_Track_Mode = TRACK_DOUBLE; 
                    Current_Roundabout_Index++;
                    //Buzzer_check1(200);
                    //Buzzer_off();
                }
                break;
            default:
                Left_Roundabout_State = 0;
                break;
        }
    }
    else if (expected_dir == RightRoundabout)
    {
        switch (Right_Roundabout_State)
        {
            case 0:
                if (rightRoundabout0() == 1) 
                {
                    Right_Roundabout_State = 1;
                    Current_Track_Mode = TRACK_LEFT_SINGLE;  
                    //Buzzer_on();
                    // Buzzer_check1(100);
                }
                break;

            case 1:
                if (rightRoundabout1() == 1)
                {
                    Right_Roundabout_State = 2; 
                    Right_Round_Start_Yaw = yaw_total;
                    Current_Track_Mode = TRACK_RIGHT_SINGLE;
					SERVO_MOTOR_LMAX = 92;
					//Buzzer_check1(500);
                }
                break;

            case 2:
                if (abs_float(yaw_total - Right_Round_Start_Yaw) > 45.0f)
                {
                    Right_Roundabout_State = 3; 
                    ackCoeff = ACK_COEFF_ROUNDABOUT; 
                    Current_Track_Mode = TRACK_DOUBLE;
					SERVO_MOTOR_LMAX = 97;
                }
                break;

            case 3:
                if (abs_float(yaw_total - Right_Round_Start_Yaw) > 245.0f)
                {
                    Right_Roundabout_State = 4; 
                    Current_Track_Mode = TRACK_RIGHT_SINGLE;
                }
                break;

            case 4:
                if (abs_float(yaw_total - Right_Round_Start_Yaw) > 315.0f)
                {
                    Right_Roundabout_State = 5; 
                    Current_Track_Mode = TRACK_LEFT_SINGLE;
					SERVO_MOTOR_LMAX = 120;
					
                }
                break;

            case 5:
                if (checkRightRoundaboutExit() == 1)
                {
                    Right_Roundabout_State = 0; 
                    ackCoeff = ACK_COEFF_DEFAULT; 
                    Current_Track_Mode = TRACK_DOUBLE; 
                    Current_Roundabout_Index++;
					//Buzzer_check1(200);
                    //Buzzer_off();
                }
                break;
            default:
                Right_Roundabout_State = 0;
                break;
        }
    }
}

//----------------------------------------------------------打靶
 uint8 target_pass_count = 0;         // 已通过的靶子数
 uint8 stop_box = 0;
 static uint16 target_cooldown = 0;
Target_State_e Target_State = TARGET_SEARCHING;
uint8  Target_Found = 0;   //本帧是否检测到靶环
int16  Target_X     = 0;   //靶心列(大图)
int16  Target_Y     = 0;   //靶心行(大图)

uint8 targetCoarseScan(void)
{
    int16 y, col;
    int16 lb, rb;                  
    int16 left_hit, right_hit;
    int16 scan_top;
    uint8 consec        = 0;       //连续命中行计数
    int16 sum_cx        = 0;       //命中行靶心列累加
    int16 first_hit_row = -1;      //连续段近端行
    int16 last_hit_row  = -1;      //连续段远端行
	
    if (scan_top < (first_end << 1) + 2) scan_top = (first_end << 1) + 2;
    for (y = TGT_SCAN_BOTTOM; y >= scan_top; y--)
    {
        if (y < 0 || y >= MT9V03X_H) 
			continue;
        lb = get_filled_l_border(y >> 1) << 1;     //小图边界映射到大图
        rb = get_filled_r_border(y >> 1) << 1;
        if (lb < TGT_JUMP)  
			lb = TGT_JUMP;
        if (rb > MT9V03X_W - 1 - TGT_JUMP) 
			rb = MT9V03X_W - 1 - TGT_JUMP;
        if (rb - lb < TGT_MIN_GAP) 
			{ 
				consec = 0;
				sum_cx = 0;
				first_hit_row = -1; 
				continue; 
			}
        left_hit = -1;
        for (col = lb + TGT_JUMP; col <= rb; col += 2)
            if (IS_EDGE(mt9v03x_image[y][col - TGT_JUMP], mt9v03x_image[y][col], TGT_EDGE_TH))
            { 
				left_hit = col;
				break; 
			}
        right_hit = -1;
        for (col = rb - TGT_JUMP; col >= lb; col -= 2)
            if (IS_EDGE(mt9v03x_image[y][col + TGT_JUMP], mt9v03x_image[y][col], TGT_EDGE_TH))
            {
				right_hit = col; 
				break; 
			}
        if (left_hit >= 0 && right_hit >= 0 &&
            (right_hit - left_hit) >= TGT_MIN_GAP &&
            (right_hit - left_hit) <= TGT_MAX_GAP)
        {
            consec++;
            sum_cx += (left_hit + right_hit) >> 1;
            if (first_hit_row < 0) 
				first_hit_row = y;
            last_hit_row = y;
            if (consec >= TGT_MIN_ROWS)
            {
                Target_X = sum_cx / consec;                  //平均列
                Target_Y = (first_hit_row + last_hit_row) >> 1;
                return 1;
            }
        }
        else
        {
            consec = 0; sum_cx = 0; first_hit_row = -1;
        }
    }
    return 0;
}
//细筛
uint8 targetTrackScan(void)
{
    int16 seed_x = Target_X;
    int16 seed_y = Target_Y;
    int16 lb, rb;
    int16 left_limit, right_limit, up_limit, down_limit;
    int16 c, r;
    int16 left_edge = -1, right_edge = -1;
    int16 up_edge   = -1, down_edge  = -1;
    int16 new_x, new_y;
    int16 w_gap, h_gap;
    //种子点必须是白
    if (mt9v03x_image[seed_y][seed_x] <= current_white-50) return 0;
    lb = get_filled_l_border(seed_y >> 1) << 1;
    rb = get_filled_r_border(seed_y >> 1) << 1;
    left_limit  = lb + TGT_BORDER_MARGIN;
    right_limit = rb - TGT_BORDER_MARGIN;
    up_limit    = first_end << 1;       
    down_limit  = MT9V03X_H - 1;        
    if (left_limit >= right_limit) 
		return 0;  
    for (c = seed_x; c >= left_limit; c--)     
        if (IS_EDGE(mt9v03x_image[seed_y][c + TGT_JUMP], mt9v03x_image[seed_y][c], TGT_EDGE_TH))
        { 
			left_edge = c; break; 
		}
    for (c = seed_x; c <= right_limit; c++)   
        if (IS_EDGE(mt9v03x_image[seed_y][c - TGT_JUMP], mt9v03x_image[seed_y][c], TGT_EDGE_TH))
        { 
			right_edge = c; break; 
		}

    for (r = seed_y; r >= up_limit; r--)       //向上
    {
        if (r + TGT_JUMP >= MT9V03X_H) 
			continue;   //下方参考点越界保护
        if (IS_EDGE(mt9v03x_image[r + TGT_JUMP][seed_x], mt9v03x_image[r][seed_x], TGT_EDGE_TH))
        { 
			up_edge = r; break; 
		}
    }

    for (r = seed_y; r <= down_limit; r++)     //向下
    {
        if (r - TGT_JUMP < 0) continue;            //上方参考点越界保护
        if (IS_EDGE(mt9v03x_image[r - TGT_JUMP][seed_x], mt9v03x_image[r][seed_x], TGT_EDGE_TH))
        {
			down_edge = r; break; 
		}
    }

    if (left_edge < 0 || right_edge < 0 ) 
		return 0;
	
    w_gap = right_edge - left_edge;
    h_gap = down_edge  - up_edge;
    if (w_gap < TGT_HOLE_MIN || w_gap > TGT_HOLE_MAX) 
		return 0;
    if (h_gap < TGT_HOLE_MIN || h_gap > TGT_HOLE_MAX) 
		return 0;
    //新靶心
    new_x = (left_edge + right_edge) >> 1;
    new_y = (up_edge   + down_edge ) >> 1;
    //靶心跳变上限
    if (abs_int(new_x - seed_x) >= TGT_JUMP_MAX) 
		return 0;
    if (abs_int(new_y - seed_y) >= TGT_JUMP_MAX) 
		return 0;

    Target_X = new_x;
    Target_Y = new_y;
    return 1;
}
//靶环状态机
void targetStateMachine(void)
{
    static uint8 print_count = 0;
    int16 x_small, y_small;
//	if (zebra_detect) 
//		return;
    if (target_cooldown > 0)
	 target_cooldown--;
    switch (Target_State)
    {
        case TARGET_SEARCHING:
            Target_Found = 0;
            laser_off();                         
            if (targetCoarseScan())             
            {
                Target_State = TARGET_TRACKING;
				//Buzzer_check1(50);
            }
            break;
        case TARGET_TRACKING:
            if (targetTrackScan())               
            {
				Buzzer_check1(200);
                Target_Found = 1;
				if (target_cooldown == 0)
                  {
                      target_pass_count++;
                      target_cooldown = 50;
//					  if (STOP_TARGET_NUM > 1 && target_pass_count == STOP_TARGET_NUM - 1)
//						  {
//                            stop_box = 1;
//							Servo_PID.Kp2 = 0;
//							LM_LADRC.limit = 8500.0f;
//							RM_LADRC.limit = 8500.0f;
//						  }

                      if (STOP_TARGET_NUM > 0 && target_pass_count >= STOP_TARGET_NUM)
					  {
                          car_run_flag = 0;
						   Ladrc_Reset_Single(&LM_LADRC);//内部变量清零
                           Ladrc_Reset_Single(&RM_LADRC);
					  }
                  }
				if(Target_Y>=60)
				{
                if (Target_X>=85&&Target_X<=115)
				{
					if(Target_Y >= TGT_FIRE_MID_MIN && Target_Y <= TGT_FIRE_MID_MAX)
					{
						laser_mid_on();
						laser_right_off();
						laser_left_off();
						//Buzzer_check1(50);	
					}
					
				}
				else if (Target_X>=40&&Target_X<=85)
				{
					
					if(Target_Y >= TGT_FIRE_LEFT_MIN && Target_Y <= TGT_FIRE_LEFT_MAX)//
					{
						laser_left_on();
						laser_mid_off();
						laser_right_off();
						//Buzzer_check1(50);
					}
					
				}
				else if (Target_X>=115&&Target_X<=150)
				{
					if(Target_Y >= TGT_FIRE_RIGHT_MIN && Target_Y <= TGT_FIRE_RIGHT_MAX)//
					{
						laser_right_on();
						laser_left_off();
						laser_mid_off();
						//Buzzer_check1(50);
					}	
				}
			   }
                else
                    laser_off();
            }
            else                                 
            {
                Target_State = TARGET_SEARCHING;
                Target_Found = 0;
                laser_off();                     
            }
            break;

        default:
            Target_State = TARGET_SEARCHING;
            laser_off();
            break;
    }

//    //==================== 调试打印 ====================
//    print_count++;
//    if (print_count >= 5)
//    {
//        print_count = 0;

//       if (Target_Found)
//  {
//      sprintf(wireless_buf, "State:%s Found:YES location:(%d,%d)\r\n",
//              (Target_State == TARGET_TRACKING ? "TRACKING" : "SEARCHING"),
//              (int)Target_X, (int)Target_Y);
//      wireless_uart_send_string(wireless_buf);
//  }
//  else
//  {
//      wireless_uart_send_string("State:SEARCHING Found:NO\r\n");
//  }
//    }
}







//// ==================== 斑马线检测 ====================
// uint8 zebra_detect = 0;
//  static uint8 zebra_confirm_cnt = 0;
// static uint8 zebra_lost_cnt = 0;
// 
//  #define ZEBRA_CONFIRM_THRESHOLD  3
//  #define ZEBRA_EDGE_MIN           4    // 单行最少跳变次数
//  #define ZEBRA_ROWS_MIN           2    // 最少满足行数
//uint8 check_zebra(void)
//  {
//      uint8 i, j;
//      uint8 edge_count;
//      uint8 zebra_rows = 0;

//       uint8 row_start = (IMAGE_H_SMALL >= 4) ? (IMAGE_H_SMALL - 4) : 0;
//  uint8 col_start = IMAGE_W_SMALL / 3;
//  uint8 col_end   = IMAGE_W_SMALL * 2 / 3;

//      for (i = row_start; i < IMAGE_H_SMALL; i++)
//      {
//          edge_count = 0;

//          for (j = col_start; j < col_end; j++)
//          {
//              // IS_EDGE(左, 右, 阈值): 左右像素对比度 > 阈值即判为边
//              if (IS_EDGE(image_small[i][j], image_small[i][j + 1], THRESHOLD_ZEBRA))
//              {
//                  edge_count++;
//                  if (edge_count >= ZEBRA_EDGE_MIN)
//                  {
//                      zebra_rows++;
//                      break;
//                  }
//              }
//          }
//      }

//      return (zebra_rows >= ZEBRA_ROWS_MIN) ? 1 : 0;
//  }
//  void zebraState(void)
//  {
//      if (crossState != 0 || Left_Roundabout_State != 0 || Right_Roundabout_State != 0)
//          return;

//      if (check_zebra())
//      {
//          zebra_confirm_cnt++;
//          if (zebra_confirm_cnt >= ZEBRA_CONFIRM_THRESHOLD||check_out_of_track(4) == 1)
//          {
//              zebra_detect = 1;
//              zebra_lost_cnt = 0;          // 检测到了清零丢失计数
//          }
//      }
//      else
//      {
//          if (zebra_detect)
//          {
//              zebra_lost_cnt++;
//              if (zebra_lost_cnt >= 20)    // 连续20帧(~100ms)无斑马线 → 退出
//              {
//                  zebra_detect = 0;
//                  zebra_confirm_cnt = 0;
//                  zebra_lost_cnt = 0;
//              }
//          }
//          else
//          {
//              zebra_confirm_cnt = 0;
//          }
//      }
//  }