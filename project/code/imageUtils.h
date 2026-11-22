
uint8 find_jump_point(uint8 *array_value, uint8 num0, uint8 num1, uint8 jump_num, uint8 model);
void connect_point(uint8 *array_value, uint8 num3, uint8 num4);
uint8 scanFirstValidPoint(uint8 *lost_flag_array, uint8 start_row, uint8 end_row, uint8 gap_threshold);
uint8 scanFirstLostPoint(uint8 *lost_flag_array, uint8 start_row, uint8 end_row, 
                         uint8 valid_threshold, uint8 lost_threshold);
uint8 Check_Monotonic_Increase(uint8 *edge_line, uint8 start_row, uint8 end_row, uint8 tolerance);
uint8 Check_Monotonic_Decrease(uint8 *edge_line, uint8 start_row, uint8 end_row, uint8 tolerance);
uint8 Check_Straight_Line(uint8 *edge_line, uint8 start_row, uint8 end_row, uint8 max_deviation);
uint8 checkLostLine(uint8 *lost_flag, uint8 start_row, uint8 end_row, uint8 threshold);
void extend_line(uint8 *array, uint8 start_row, uint8 end_row);
uint8 find_turn_point_left(uint8 *array, uint8 start_row, uint8 end_row);
uint8 find_turn_point_right(uint8 *array, uint8 start_row, uint8 end_row);
void fill_vertical_down(uint8 *array, uint8 start_row);

int16 get_filled_l_border(int16 small_row);
int16 get_filled_r_border(int16 small_row);

