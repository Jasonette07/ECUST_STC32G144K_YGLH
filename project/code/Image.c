#include "zf_common_headfile.h"


//赛道半宽数组
uint8 half_width_small[IMAGE_H_SMALL] = {
     4,  4,  5,  5,  6,  7,  7,  8,  9,  9,  // 0~9 行
    10, 10, 11, 12, 12, 13, 14, 14, 15, 15,  // 10~19 行
    16, 17, 17, 18, 19, 19, 20, 20, 21, 22,  // 20~29 行
    22, 23, 23, 24, 25, 25, 26, 27, 27, 28,  // 30~39 行
    28, 29, 30, 30, 31, 32, 32, 33, 33, 34,  // 40~49 行
    35, 35, 36, 37, 37, 38, 38, 39, 40, 40   // 50~59 行
};

//降采样图像
uint8 image_small[60][94];

void downsample_image(void)
{
    int i, j;
    for(i = 0; i < 60; i++)
    {
        for(j = 0; j < 94; j++)
        {
            image_small[i][j] = mt9v03x_image[i * 2][j * 2];
        }
    }
}

//找基础白点灰度值 
int16  current_white = 0;                                  
uint8  histogram[256];                                      
void search_white(uint8 *photo, uint16 point_num)
{
    uint16 i;
    int16 white_lost;
    int16 white_start, white_end;
    int16 temp_value, temp_value2;
    for(i = 0; i < 256; i++) histogram[i] = 0;
    for(i = 0; i < point_num; i++)
    {
        histogram[*photo]++;
        photo++;
    }
    white_start = 0;
    white_end   = 0;
    white_lost  = 0;

    for(i = 255; i >= 0; i--)
    {
        if(!white_start)
        {
            if(histogram[i] > 5) white_start = i;
        }
        else
        {
            if(histogram[i] < 5)
            {
                if(!white_end) white_end = i + white_lost;
                white_lost++;
            }
            else
            {
                white_lost = 0;
            }
        }
        if(white_lost >= 5) break;
    }

    // 计算参考白点
    temp_value = white_end + (white_start - white_end) / 5;
    // 更新当前白点
    if(!current_white)
    {
        current_white = temp_value;
    }
    else
    {
        temp_value2 = calc_diff(temp_value, current_white);
        if(temp_value2 > -THRESHOLD / 2)
        {
            current_white = current_white + func_limit_ab((temp_value - current_white), -10, 10);
        }
    }


}
//最长白列搜索
int16  white_length[2][WH_COL_NUM];                         // [0][i]=远端行号 [1][i]=列坐标
int16  white_length_max[2][1];                              // [0][0]=最远行号 [1][0]=最远列坐标
int16  Search_Stop_Line = 0;                                // 搜索截止行
void search_longest_white_col_small(void)
{
    int16 i, j, k;
    int16 col;
    int16 col_index;
    int16 next_start;
    int16 start_row;
    int16 win_top, win_bot;
    int16 jump_row;
    int16 best_row, best_col;
    uint8 bottom_dark;
    uint8 first_valid_found;
    int16 prev_jump_row;
    int16 prev_col;
    uint8 anchor_found;
    int16 anchor_col;
    int16 anchor_row;
    uint8 var_count;
    int16 diff_abs;
    int16 diff_sign;
    int16 col_diff;
    int16 rescan_jump;

    next_start        = WH_START_ROW;                     
    first_valid_found = 0;
    best_row          = IMAGE_H_SMALL - 1;
    best_col          = IMAGE_W_SMALL / 2;

    prev_jump_row = -1;
    prev_col      = -1;
    anchor_found  = 0;
    anchor_col    = -1;
    anchor_row    = IMAGE_H_SMALL - 1;
    var_count     = 0;

    for(i = 0; i < WH_COL_NUM; i++)
    {
        white_length[0][i] = IMAGE_H_SMALL - 1;
        white_length[1][i] = WH_COL_MIN + i * WH_COL_STEP;
    }

    col_index = 0;
    for(col = WH_COL_MIN; col <= WH_COL_MAX; col += WH_COL_STEP)
    {
        if(col_index >= WH_COL_NUM) break;
        white_length[1][col_index] = col;

        // ---------- 底行黑点检测 ----------
        bottom_dark = 0;
        for(j = IMAGE_H_SMALL - 1; j > IMAGE_H_SMALL - 1 - PHASE1_ROWS; j--)
        {
            if(IS_EDGE(current_white, image_small[j][col], THRESHOLD))
            { 
                bottom_dark = 1; 
                break; 
            }
        }
        if(!bottom_dark
           && image_small[IMAGE_H_SMALL - 1][col] < BOTTOM_DARK_GATE        
           && image_small[IMAGE_H_SMALL - 2][col] < BOTTOM_DARK_GATE)
        {
            bottom_dark = 1;
        }
        if(bottom_dark)
        {
            col_index++;
            continue;
        }

        if(!first_valid_found)
        {
            next_start        = WH_START_ROW;
            first_valid_found = 1;
        }

        // ----------  行窗口主搜索 ----------
        start_row = next_start;
        if(start_row > IMAGE_H_SMALL - 1 - JUMP_NUM) start_row = IMAGE_H_SMALL - 1 - JUMP_NUM; // ← 改
        if(start_row < WH_ROW_MIN)                   start_row = WH_ROW_MIN;

        win_bot = start_row;                               
        win_top = start_row - (WH_WINDOW_SIZE - 1);         
        if(win_top < WH_ROW_MIN) win_top = WH_ROW_MIN;

        jump_row = -1;
        for(j = win_bot; j >= win_top; j -= WH_ROW_STEP)
        {
            if(j + JUMP_NUM > IMAGE_H_SMALL - 1) continue;
            if(IS_EDGE(image_small[j + JUMP_NUM][col],
                       image_small[j][col], THRESHOLD))
            { 
                jump_row = j; 
                break; 
            }
        }

        // ---------- 窗口未中 + 未到顶 → 向上全列回扫 ----------
        if(jump_row < 0 && win_top > WH_ROW_MIN)
        {
            for(j = WH_START_ROW; j >= WH_ROW_MIN; j -= WH_ROW_STEP)
            {
                if(j + JUMP_NUM > IMAGE_H_SMALL - 1) continue;              
                if(IS_EDGE(image_small[j + JUMP_NUM][col],                  
                           image_small[j][col], THRESHOLD))
                { 
                    jump_row = j; 
                    break; 
                }
            }
        }

        if(jump_row < 0)
        {
            jump_row = WH_ROW_MIN;
        }

        white_length[0][col_index] = jump_row;

        next_start = jump_row + WH_NEXT_OFFSET;
        if(next_start > IMAGE_H_SMALL - 1 - JUMP_NUM) next_start = IMAGE_H_SMALL - 1 - JUMP_NUM;
        if(next_start < WH_ROW_MIN)                   next_start = WH_ROW_MIN;

        if(jump_row < best_row)
        {
            if(best_row >= IMAGE_H_SMALL - 1)
            {
                best_row = jump_row;
                best_col = col;
            }
            else
            {
                col_diff = (col > best_col) ? (col - best_col) : (best_col - col);
                if(col_diff < 30)
                {
                    best_row = jump_row;
                    best_col = col;
                }
            }
        }

        if(!anchor_found && prev_jump_row >= 0)
        {
            diff_abs = jump_row - prev_jump_row;
            if(diff_abs < 0) diff_abs = -diff_abs;
            if(diff_abs <= 2)
            {
                anchor_found = 1;
                anchor_col   = prev_col;                    
                anchor_row   = prev_jump_row;
            }
        }

        // ---------- 锚点容差检查 + 突变列重扫 ----------
        if(anchor_found)
        {
            diff_abs = jump_row - anchor_row;
            if(diff_abs < 0) diff_abs = -diff_abs;
            if(diff_abs > 3)
            {
                rescan_jump = -1;
                for(k = IMAGE_H_SMALL - 1; k >= WH_ROW_MIN; k -= WH_ROW_STEP) 
                {
                    if(k + JUMP_NUM > IMAGE_H_SMALL - 1) continue;
                    if(IS_EDGE(image_small[k + JUMP_NUM][col],
                               image_small[k][col], THRESHOLD))
                    { 
                        rescan_jump = k; 
                        break; 
                    }
                }
                if(rescan_jump < 0) rescan_jump = WH_ROW_MIN;
                if(rescan_jump < anchor_row)
                {
                    anchor_col = col;
                    anchor_row = rescan_jump;
                }
            }
        }

        if(prev_jump_row >= 0)
        {
            diff_sign = jump_row - prev_jump_row;
            if(diff_sign > 0)      var_count++;            
            else if(diff_sign < 0) var_count = 0;           
            if(var_count >= 4)
            {
                col_index++;
                break;                                    
            }
        }

        prev_jump_row = jump_row;
        prev_col      = col;

        col_index++;
    }

    if(anchor_found)
    {
        best_col = anchor_col;
        best_row = anchor_row;
    }
    else if(best_row >= IMAGE_H_SMALL - 1)
    {
        best_col = IMAGE_W_SMALL / 2;
        best_row = WH_START_ROW;
    }

    white_length_max[0][0] = best_row;
    white_length_max[1][0] = best_col;
    Search_Stop_Line       = IMAGE_H_SMALL - 1 - best_row;
}


//向左右两边搜线 
uint8  l_border[IMAGE_H_SMALL], r_border[IMAGE_H_SMALL];               
int16  image_position[IMAGE_H_SMALL];                             
uint8  l_effect_flag[IMAGE_H_SMALL], r_effect_flag[IMAGE_H_SMALL];
int16  first_end;                                           
int16  l_start;                                             
int16  r_start;                                             
int16  l_effect_num;                                        
int16  r_effect_num;                                        
int16  Both_Lost_Time = 0;                                

void search_border_small(void)
{
    int16 i, j;
    int16 ref_col;
    int16 left_startcol, left_endcol;
    int16 right_startcol, right_endcol;
    uint8 search_pass;
    uint8 *row_ptr;

    ref_col = white_length_max[1][0];
    if(ref_col < SEARCH_MIN + JUMP_NUM) ref_col = SEARCH_MIN + JUMP_NUM;
    if(ref_col > SEARCH_MAX - JUMP_NUM) ref_col = SEARCH_MAX - JUMP_NUM;

    first_end = white_length_max[0][0] + 1; 
    if(first_end > IMAGE_H_SMALL - 1) first_end = IMAGE_H_SMALL - 1; 

    l_start = 0;
    r_start = 0;
    l_effect_num = 0;
    r_effect_num = 0;
    Both_Lost_Time = 0;

    for(i = 0; i < IMAGE_H_SMALL; i++) 
    {
        l_border[i] = 0;
        r_border[i] = IMAGE_W_SMALL;
        l_effect_flag[i] = 0;
        r_effect_flag[i] = 0;
    }

    left_startcol = ref_col;
    left_endcol   = SEARCH_MIN;
    right_startcol = ref_col;
    right_endcol   = SEARCH_MAX;

    for(i = IMAGE_H_SMALL - 1; i > first_end; i--) 
    {
        row_ptr = image_small[i];
        
        //左边界搜索 
        search_pass = 2;
        while(search_pass > 0)
        {
            if(search_pass == 1)
            {
                left_startcol = ref_col;
                left_endcol   = SEARCH_MIN;
            }
            search_pass--;

            for(j = left_startcol; j >= left_endcol; j--)
            {
                if(j < SEARCH_MIN || j + JUMP_NUM >= IMAGE_W_SMALL) continue; 

                if(IS_EDGE(row_ptr[j + JUMP_NUM], row_ptr[j], THRESHOLD))
                {
                    l_border[i] = j;
                    l_effect_flag[i] = 1;
                    if(!l_start) l_start = i;
                    l_effect_num++;
                    
                    left_startcol = j + SEARCH_RANGE;
                    if(left_startcol > SEARCH_MAX - JUMP_NUM) left_startcol = SEARCH_MAX - JUMP_NUM;
                    left_endcol = j - SEARCH_RANGE;
                    if(left_endcol < SEARCH_MIN) left_endcol = SEARCH_MIN;
                    
                    search_pass = 0;        
                    break;
                }
            }
        }

        //右边界搜索 
        search_pass = 2;
        while(search_pass > 0)
        {
            if(search_pass == 1)
            {
                right_startcol = ref_col;
                right_endcol   = SEARCH_MAX;
            }
            search_pass--;

            for(j = right_startcol; j <= right_endcol; j++)
            {
                if(j > SEARCH_MAX || j - JUMP_NUM < 0) continue;

                if(IS_EDGE(row_ptr[j - JUMP_NUM], row_ptr[j], THRESHOLD))
                {
                    r_border[i] = j;
                    r_effect_flag[i] = 1;
                    if(!r_start) r_start = i;
                    r_effect_num++;
                    
                    right_startcol = j - SEARCH_RANGE;
                    if(right_startcol < SEARCH_MIN + JUMP_NUM) right_startcol = SEARCH_MIN + JUMP_NUM;
                    right_endcol = j + SEARCH_RANGE;
                    if(right_endcol > SEARCH_MAX) right_endcol = SEARCH_MAX;
                    
                    search_pass = 0;
                    break;
                }
            }
        }

        if(!l_effect_flag[i] && !r_effect_flag[i])
        {
            Both_Lost_Time++;
        }
    }
}


uint8 mid_line[IMAGE_H_SMALL] = {0};
Track_Mode_e Current_Track_Mode = TRACK_DOUBLE;

void middleLine(void)
{
    uint8 y;
    uint8 expected_mid;

    for(y = 0; y < IMAGE_H_SMALL; y++)       
    {
        switch (Current_Track_Mode)
        {
            case TRACK_RIGHT_SINGLE: // 右单边巡线
                expected_mid = r_border[y] - half_width_small[y]; 
                mid_line[y] = (uint8)func_limit_ab(expected_mid, 0, IMAGE_W_SMALL - 1); 
                break;

            case TRACK_LEFT_SINGLE:  // 左单边巡线
                expected_mid = l_border[y] + half_width_small[y] - 1;
                mid_line[y] = (uint8)func_limit_ab(expected_mid, 0, IMAGE_W_SMALL - 1);
                break;

            case TRACK_DOUBLE:       // 正常双边巡线
            default:
                mid_line[y] = (l_border[y] + r_border[y]) / 2;
                break;
        }
    }
}

 // 前瞻点加权求中线误差
  static const float Weighting[5] = {0.85f, 0.75f, 0.6f, 0.3f, 0.20f};
  int16 Image_Error = 0;
  volatile int16 posi = 0;
  int16 TowPoint_True = 0;
  int16 off_line;
  void GetDet(void)
  {
      float DetTemp = 0.0f;
      float UnitAll = 0.0f;
      int16 TowPoint = 0;
      int16 i;
      int16 mirror_end;
      uint8 in_roundabout;

      // 路径选择前瞻点
      if(crossState != 0)
          TowPoint = TOWPOINT_CROSS;
      else if(Left_Roundabout_State != 0 || Right_Roundabout_State != 0)
          TowPoint = TOWPOINT_ROUND;
      else
          TowPoint = TOWPOINT_NORMAL;

      // 环岛模式：不跳过双丢行
      in_roundabout = (Left_Roundabout_State != 0 || Right_Roundabout_State != 0);

      // 图像有效上界
      off_line = white_length_max[0][0] + 1;
      if(off_line < 2) off_line = 2;

      // 前瞻点限幅
      if(TowPoint < off_line + 3)
          TowPoint = off_line + 3;
      if(TowPoint >= IMAGE_H_SMALL - TOWPOINT_WINDOW - 1)
          TowPoint = IMAGE_H_SMALL - TOWPOINT_WINDOW - 2;
      if (crossState == 0 && Left_Roundabout_State == 0 && Right_Roundabout_State == 0)
      {
          while (l_effect_flag[TowPoint] == 0 && r_effect_flag[TowPoint] == 0
                 && TowPoint < IMAGE_H_SMALL - 2)
          {
              TowPoint++;
          }
          if(TowPoint >= IMAGE_H_SMALL - TOWPOINT_WINDOW - 1)
              TowPoint = IMAGE_H_SMALL - TOWPOINT_WINDOW - 2;
      }

      //--- 情况 A: 正常窗口 ---
      if((TowPoint - TOWPOINT_WINDOW) >= off_line)
      {
          for(i = TowPoint - TOWPOINT_WINDOW; i < TowPoint; i++)
          {
              if (in_roundabout || (l_effect_flag[i] || r_effect_flag[i]))
              {
                  DetTemp += Weighting[TowPoint - i - 1] * (float)mid_line[i];
                  UnitAll += Weighting[TowPoint - i - 1];
              }
          }
          for(i = TowPoint + TOWPOINT_WINDOW; i > TowPoint; i--)
          {
              if (in_roundabout || (l_effect_flag[i] || r_effect_flag[i]))
              {
                  DetTemp += Weighting[i - TowPoint - 1] * (float)mid_line[i];
                  UnitAll += Weighting[i - TowPoint - 1];
              }
          }
          DetTemp = ((float)mid_line[TowPoint] + DetTemp) / (UnitAll + 1.0f);
      }
      //--- 情况 B: 上方不够,对称扩展 ---
      else if(TowPoint > off_line)
      {
          for(i = off_line; i < TowPoint; i++)
          {
              if (in_roundabout || (l_effect_flag[i] || r_effect_flag[i]))
              {
                  DetTemp += Weighting[TowPoint - i - 1] * (float)mid_line[i];
                  UnitAll += Weighting[TowPoint - i - 1];
              }
          }
          mirror_end = TowPoint + (TowPoint - off_line);
          if(mirror_end >= IMAGE_H_SMALL) mirror_end = IMAGE_H_SMALL - 1;
          for(i = mirror_end; i > TowPoint; i--)
          {
              if (in_roundabout || (l_effect_flag[i] || r_effect_flag[i]))
              {
                  DetTemp += Weighting[i - TowPoint - 1] * (float)mid_line[i];
                  UnitAll += Weighting[i - TowPoint - 1];
              }
          }
          DetTemp = ((float)mid_line[TowPoint] + DetTemp) / (UnitAll + 1.0f);
      }
      //--- 情况 C: 极短,只取 off_line 附近 ---
      else if(off_line < IMAGE_H_SMALL - 2)
      {
          for(i = off_line + 2; i > off_line; i--)
          {
              if(i < IMAGE_H_SMALL)
              {
                  if (in_roundabout || (l_effect_flag[i] || r_effect_flag[i]))
                  {
                      DetTemp += Weighting[i - off_line - 1] * (float)mid_line[i];
                      UnitAll += Weighting[i - off_line - 1];
                  }
              }
          }
          if (UnitAll > 0.0f)
              DetTemp = ((float)mid_line[off_line] + DetTemp) / (UnitAll + 1.0f);
          else
              DetTemp = (float)mid_line[off_line];
      }
      //--- 情况 D: 完全无效,沿用上一帧 ---
      else
      {
          DetTemp = (float)Image_Error + (float)(IMAGE_W_SMALL / 2);
      }

      Image_Error = (int16)(DetTemp - (float)(IMAGE_W_SMALL / 2));

      posi = -Image_Error;
      TowPoint_True = TowPoint;
  }

//uint8 check_out_of_track(uint8 rows)
//{
//    int16 i, j;
//    uint32 gray_sum = 0;
//    uint32 pixel_count = 0;
//    uint16 avg_gray;
//    
//    //固定检测中间 1/3 区域
//    int16 col_start = IMAGE_W_SMALL / 3;
//    int16 col_end   = IMAGE_W_SMALL * 2 / 3;

//    for (i = IMAGE_H_SMALL - 1; i > IMAGE_H_SMALL - 1 - rows; i--)
//    {
//        if (i < 0) break;
//        for (j = col_start; j <= col_end; j++)
//        {
//            gray_sum += image_small[i][j];
//            pixel_count++;
//        }
//    }
//    
//    if (pixel_count == 0) return 0;
//    avg_gray = (uint16)(gray_sum / pixel_count);
//    // sprintf(print_buf, "Avg Gray: %d\r\n", (int)avg_gray);
//    // wireless_uart_send_string(print_buf);   
//    return (avg_gray < OUT_GRAY_THRESHOLD) ? 1 : 0;
//}

//图像总处理
void imageProcess(void)
{   
    downsample_image();
    search_white(&mt9v03x_image[IMAGE_H - 1][1], MT9V03X_W - 2);
    search_longest_white_col_small();                                              
    search_border_small(); 
    crossroadState();	
    roundaboutState();
	//zebraState();
    targetStateMachine();
    middleLine();                                               
    GetDet();                                                   
//    if (check_out_of_track(4) == 1)
//   {
//        car_run_flag = 0;
//       start_delay_ms = 0;
//   }
}