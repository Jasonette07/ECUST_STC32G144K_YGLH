#include "zf_common_headfile.h"

//检测跳变点
uint8 find_jump_point(uint8 *array_value , uint8 num0 , uint8 num1 , uint8 jump_num , uint8 model)
{

	uint8 temp_data;
    uint8 temp_jump_point;
	int i=0;
	
	if(model)
	{
		temp_jump_point = num0;
		for(i;i<(num0 - num1); i++)
		{
			temp_data = func_abs( array_value[num0 - i] - array_value[num0 - i - 1]);
				if(temp_data > jump_num)
				{
					temp_jump_point = (uint8)(num0 - i);
						break;
				}
		}
	}
	else
	{
		temp_jump_point = num1;
		for(i;i<(num0 - num1); i++)
		{
			temp_data = func_abs( array_value[num1 + i] - array_value[num1 + i + 1]);
				if(temp_data > jump_num)
				{
					temp_jump_point = (uint8)(num1 + i);
						break;
				}
		}
	}
	return temp_jump_point;
}

//两点间线性补线
void connect_point(uint8 *array_value, uint8 num3, uint8 num4)
{
	int i = 0;
	float point_1;
	float point_2;
	float temp_slope;
	point_1 = (float)array_value[num3];
	point_2 = point_1;
	temp_slope = (point_2 - point_1) / (num3 - num4);
	
	for(i; i < (num3 - num4);i++)
	{
		array_value[num3 - i] = (int8)(temp_slope * i) + array_value[num3];
	}
}



//找第一个不丢线的点
uint8 scanFirstValidPoint(uint8 *lost_flag_array, uint8 start_row, uint8 end_row, uint8 gap_threshold)
{
    int8 i;
    uint8 gap_count = 0;  

    if (start_row >= end_row)  
    {
        for (i = start_row; i >= end_row; i--)
        {
            if (lost_flag_array[i] == 0)
            {
                gap_count++;
            }
            else
            {
                if (gap_count >= gap_threshold)
                    return (uint8)i;
                else
                    gap_count = 0;
            }
        }
    }
    else  
    {
        for (i = start_row; i <= end_row; i++)
        {
            if (lost_flag_array[i] == 0)
            {
                gap_count++;
            }
            else
            {
                if (gap_count >= gap_threshold)
                    return (uint8)i;
                else
                    gap_count = 0;
            }
        }
    }
    
    return 0;  
}



//找第一个丢线的点
uint8 scanFirstLostPoint(uint8 *lost_flag_array, uint8 start_row, uint8 end_row, 
                         uint8 valid_threshold, uint8 lost_threshold)
{
    int8 i, k;
    uint8 valid_count = 0;
    uint8 lost_count;
    int8 step;
    step = (start_row >= end_row) ? -1 : 1;
    
    for (i = start_row; ; i += step)
    {
        if (step == -1 && i < end_row) break;
        if (step == 1  && i > end_row) break;
        
        if (lost_flag_array[i] == 1)
        {
            valid_count++;
        }
        else
        {
            if (valid_count >= valid_threshold)
            {
                lost_count = 1;       
                for (k = i + step; ; k += step)
                {
                    if (step == -1 && k < end_row) break;
                    if (step == 1  && k > end_row) break;
                    if (lost_count >= lost_threshold)
                        return (uint8)i;           
                    if (lost_flag_array[k] == 0)
                        lost_count++;
                    else
                        break;        
                }
                if (lost_count >= lost_threshold)
                    return (uint8)i;
                valid_count = 0;
            }
            else
            {
                valid_count = 0;
            }
        }
    }
    
    return 0;   
}


uint8 Check_Monotonic_Increase(uint8 *edge_line, uint8 start_row, uint8 end_row, uint8 tolerance)
{
    int i;
    uint8 max_col;             //历史最大列坐标
    uint8 last_col;            //上一行的列坐标
    uint8 rebound_count;       //连续反向计数器
    
    max_col = edge_line[start_row];
    last_col = edge_line[start_row];
    rebound_count = 0;
    for (i = start_row - 1; i >= end_row; i--)
    {
        uint8 cur_col = edge_line[i];
        
        if (cur_col >= max_col)
        {
            max_col = cur_col;
            rebound_count = 0;          
        }
        else if (cur_col + tolerance < max_col && cur_col < last_col)
        {
            rebound_count++;
            if (rebound_count >= 2)
            {
                return 0;
            }
        }
        else
        {
            rebound_count = 0;         
        }
        
        last_col = cur_col;             
    }
    
    return 1;
}



uint8 Check_Monotonic_Decrease(uint8 *edge_line, uint8 start_row, uint8 end_row, uint8 tolerance)
{
    int i;
    uint8 min_col;             //历史最小列坐标
    uint8 last_col;            //上一行的列坐标
    uint8 rebound_count;       //连续反向计数器
    
    min_col = edge_line[start_row];
    last_col = edge_line[start_row];
    rebound_count = 0;
    for (i = start_row - 1; i >= end_row; i--)
    {
        uint8 cur_col = edge_line[i];
        
        if (cur_col <= min_col)
        {
            min_col = cur_col;
            rebound_count = 0;          
        }
        else if (cur_col > min_col + tolerance && cur_col > last_col)
        {
            rebound_count++;
            if (rebound_count >= 2)
            {
                return 0;
            }
        }
        else
        {
            rebound_count = 0;          
        }        
        last_col = cur_col;           
    }
    
    return 1;
}



uint8 Check_Straight_Line(uint8 *edge_line, uint8 start_row, uint8 end_row, uint8 max_deviation)
{
    int i;
    int y1 = start_row;
    int x1 = edge_line[start_row];
    int y2 = end_row;
    int x2 = edge_line[end_row];
    int error_count = 0; 
    int max_offset = 0;  //

    if(y1 == y2)
    {
        return 1;
    }

    // 从下往上遍历
    for(i = y1 - 1; i > y2; i--)
    {
        //弦高法公式,中间运算强转 int32 防止 C251 下 16 位 int 溢出
        int expected_x = x1 + (int32)(x2 - x1) * (y1 - i) / (y1 - y2);
        int actual_x = edge_line[i];
        int offset = actual_x - expected_x;
        offset = abs_int(offset);
        if(offset > max_offset) max_offset = offset;
        if(offset > max_deviation)
        {
            error_count++;
        }
    }

    // 无论结果如何都打印一次完整统计
    // printf("[Line] rows=%d->%d, dev=%d, err=%d, max_off=%d\r\n",
    //        start_row, end_row, max_deviation, error_count, max_offset);
    // 允许有 3 行的噪点
    if(error_count > 3)
    {
        return 0;
    }

    return 1;
}
//检测大面积丢线
uint8 checkLostLine(uint8 *lost_flag, uint8 start_row, uint8 end_row, uint8 threshold)
{
    uint8 i;
    uint8 valid_count = 0; // 记录没丢线的数量

    for (i = start_row; i >= end_row; i--)
    {
        if (lost_flag[i] != 0) 
        {
            valid_count++;
            
            if (valid_count > threshold) 
            {
                return 1; 
            }
        }
    }

    return 0; 
}
//延长补线
void extend_line(uint8 *array, uint8 start_row, uint8 end_row)
{
    int16 slope_x16;
    int16 cur_val_x16;
    int16 row_diff;
    int16 val_diff;
    int16 i;
    int16 result;
    int8 step;
    int16 fill_end;
    
    row_diff = (int16)start_row - (int16)end_row;
    if (row_diff == 0) return;
    
    val_diff = (int16)array[end_row] - (int16)array[start_row];
    slope_x16 = (val_diff << 4) / row_diff;
    
    cur_val_x16 = ((int16)array[end_row]) << 4;
    
    if (start_row > end_row)
    {
        //向上延伸
        step = -1;
        fill_end = first_end;          
    }
    else
    {
        step = 1;
        fill_end = IMAGE_H_SMALL - 1;
    }
    
    for (i = (int16)end_row + step; ; i += step)
    {
        if (step == -1 && i < fill_end) break;
        if (step == 1  && i > fill_end) break;
        
        cur_val_x16 += slope_x16 * step;
        
        result = cur_val_x16 >> 4;
        
        if (result < 0)            array[i] = 0;
        else if (result > 255)     array[i] = 255;
        else                       array[i] = (uint8)result;
    }
}
//寻找左边线拐点
uint8 find_turn_point_left(uint8 *array, uint8 start_row, uint8 end_row)
{
    int8 i;
    uint8 max_val = 0;
    uint8 max_row = 0;
    uint8 dec_count = 0;
    
    for (i = (int8)start_row; i >= (int8)end_row; i--)
    {
        if (array[i] > max_val)
        {
            max_val = array[i];
            max_row = (uint8)i;
            dec_count = 0;
        }
        else if (array[i] <= array[i + 1])
        {
            dec_count++;
            
            if (dec_count >= 3)
            {
                return max_row;
            }
        }
        else
        {
            dec_count = 0;
        }
    }
    
    return 0;
}

uint8 find_turn_point_right(uint8 *array, uint8 start_row, uint8 end_row)
{
    int8 i;
    uint8 min_val;
    uint8 min_row;
    uint8 inc_count = 0;
    min_val = array[start_row];
    min_row = start_row;

    for (i = (int8)start_row - 1; i >= (int8)end_row; i--)
    {
        if (array[i] < min_val)
        {
            min_val = array[i];
            min_row = (uint8)i;
            inc_count = 0;
        }
        else if (array[i] >= array[i + 1])
        {
            inc_count++;
            if (inc_count >= 3) return min_row;
        }
        else
        {
            inc_count = 0;
        }
    }
    
    return 0;
}

void fill_vertical_down(uint8 *array, uint8 start_row)
{
   
   uint8 val;
    int16 i;
    val = array[start_row];
    
    for (i = (int16)start_row + 1; i < IMAGE_H_SMALL; i++)
    {
        array[i] = val;
    }
}


//打靶

int16 get_filled_l_border(int16 small_row)
{
    int16 k;
    if (l_effect_flag[small_row]) 
		return l_border[small_row];
    for (k = small_row + 1; k < IMAGE_H_SMALL; k++) 
		if (l_effect_flag[k]) 
			return l_border[k];
    for (k = small_row - 1; k >= 0; k--)
		if (l_effect_flag[k]) return l_border[k];
			return 0;                        
}

int16 get_filled_r_border(int16 small_row)
{
    int16 k;
    if (r_effect_flag[small_row]) 
		return r_border[small_row];
    for (k = small_row + 1; k < IMAGE_H_SMALL; k++) 
		if (r_effect_flag[k])
			return r_border[k];
    for (k = small_row - 1; k >= 0;k--) 
		if (r_effect_flag[k]) return r_border[k];
			return IMAGE_W_SMALL - 1;          
}