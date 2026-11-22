#ifndef __IMAGE_H__
#define __IMAGE_H__

// **************************** 宏定义 ****************************
#define IMAGE_H                 MT9V03X_H                   // 图像高度 120
#define IMAGE_W                 MT9V03X_W                   // 图像宽度 188
#define IMAGE_H_SMALL    60
#define IMAGE_W_SMALL    94


#define THRESHOLD               25                          // 对比度阈值(边界提取用)
#define THRESHOLD_ZEBRA         20                          // 斑马线检测阈值
#define BLACK_POINT             40                          // 黑点阈值
#define BOTTOM_DARK_GATE        80                          // 底部像素过暗兜底阈值
#define CONT_RASTOFFSET         3                           // 搜索对比偏移(如果是灰度差值则保持不变)

// ==================== 减半适配的距离/跨度宏 ====================
// 核心跳点与搜索边界
#define JUMP_NUM                3                           // 差分跳跃数: 3
#define SEARCH_MAX              (IMAGE_W_SMALL - 2)         // 搜索最大值: 自动适配为 92
#define SEARCH_MIN              1                           // 搜索最小值保持 1
#define CROSS_JUMP              2                           // 十字拐点跳跃数: 
#define SEARCH_RANGE            5                           // 跟踪搜索半径: 

// 行列控制与边缘参数
#define REFRENCE_ROW            2                           // 参考行统计行数: 
#define STOP_ROW                1                           // 停止行保持 1

// ==================== 最长白列 (滑动窗口) 专属参数 ====================
#define SEARCH_WH_DEN           2                           // 搜索白色密度: 
#define WH_COL_MIN              SEARCH_MIN                  // 列搜索起点 = 1
#define WH_COL_MAX              SEARCH_MAX                  // 列搜索终点 = 92
#define WH_COL_NUM              ((WH_COL_MAX - WH_COL_MIN) / SEARCH_WH_DEN + 1) // 自动计算采样列数

#define PHASE1_ROWS             3                           // 底部校验行数: 3
#define WH_WINDOW_SIZE          10                          // 搜索窗口大小(行数): 10
#define WH_ROW_STEP             1                           // 行跳点步长: 
#define WH_COL_STEP             2                           // 列跳点步长: 
#define WH_ROW_MIN              0                           // 行搜索最上方 (顶端)
#define WH_START_ROW            57                          // 行搜索起点 
#define WH_NEXT_OFFSET          5                           // 下一列起点偏移: 
// 前瞻行配置
#define TOWPOINT_NORMAL         28                          // 正常前瞻行
#define TOWPOINT_CROSS          38                          // 十字路口前瞻行
#define TOWPOINT_ROUND          35                         // 圆环前瞻行
#define TOWPOINT_WINDOW         4                           // 前瞻窗口半径

//#define OUT_GRAY_THRESHOLD   80  //出界灰度阈值

#define calc_diff(x, y)  ((((int16)(x) - (int16)(y)) << 7) / ((int16)(x) + (int16)(y)))
#define IS_EDGE(x, y, thresh)  ((((int16)(x) - (int16)(y)) << 7) > ((int16)(thresh) * ((int16)(x) + (int16)(y))))
void GetDet(void);
extern int16 Image_Error;
extern volatile int16   posi;                                            // 方向偏差值
extern int16 off_line;
extern int16  first_end;   
extern int16  l_effect_num;                                        
extern int16  r_effect_num;  
extern int16  l_start;                                             
extern int16  r_start;    
extern int16  Both_Lost_Time;  

//循迹模式结构体
typedef enum {
    TRACK_DOUBLE = 0,   // 正常双边循迹
    TRACK_LEFT_SINGLE,  // 左单边循迹 
    TRACK_RIGHT_SINGLE  // 右单边循迹 
} Track_Mode_e;

extern Track_Mode_e Current_Track_Mode;
extern uint8  l_border[IMAGE_H_SMALL], r_border[IMAGE_H_SMALL];               // 左/右边界数组
extern uint8  l_effect_flag[IMAGE_H_SMALL], r_effect_flag[IMAGE_H_SMALL];     // 有效边界标志(1=有效)
extern uint8  mid_line[IMAGE_H_SMALL];    
extern uint8 image_small[60][94];                                    // 中线数组
extern int16 current_white;


void imageProcess(void);
uint8 check_out_of_track(uint8 rows);


#endif