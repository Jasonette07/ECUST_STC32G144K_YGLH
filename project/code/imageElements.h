#define TGT_EDGE_TH      25     //跳变差比阈值
#define TGT_JUMP         3      //大图差分跨度
#define TGT_SCAN_TOP     35     //检测行范围-上界
#define TGT_SCAN_BOTTOM  90    //检测行范围-下界
#define TGT_MIN_GAP      5      //左右命中点最小列间距
#define TGT_MAX_GAP      80     //左右命中点最大列间距
#define TGT_MIN_ROWS     4      //连续命中行数阈值

#define TGT_BORDER_MARGIN  10    //左右搜索从赛道边界往内缩的像素
#define TGT_HOLE_MIN        5    //内孔间距下限
#define TGT_HOLE_MAX       50    //内孔间距上限
#define TGT_JUMP_MAX       20    //相邻两帧靶心跳变上限
#define TGT_FIRE_MID_MIN   85    //开火区间上界(中)
#define TGT_FIRE_MID_MAX   110   //开火区间下界(中)
#define TGT_FIRE_LEFT_MIN   85    //开火区间上界(左)
#define TGT_FIRE_LEFT_MAX   110   //开火区间下界(左)
#define TGT_FIRE_RIGHT_MIN   85    //开火区间上界(右)
#define TGT_FIRE_RIGHT_MAX   110    //开火区间下界(右)

 #define STOP_TARGET_NUM  5        // 在第几个靶子前停车
 extern uint8 target_pass_count; 
//#define stop_speed 100.0f 
// extern uint8 stop_box;
 
// #define ZEBRA_SPEED  100.0f        // 斑马线减速目标速度
// extern uint8 zebra_detect;
//uint8 check_zebra(void);
//void zebraState(void);
// 
 

//打靶状态机
typedef enum {
    TARGET_SEARCHING = 0,   //搜索态:每帧跑初筛
    TARGET_TRACKING  = 1    //跟踪态:每帧从上一帧靶心四向搜边
} Target_State_e;

//左右环岛状态机
typedef enum {
    LeftRoundabout = 1,
    RightRoundabout = 2
} Roundabout_Dir_e;

extern uint8 Left_Roundabout_State;
extern uint8 Right_Roundabout_State;  
extern uint8 crossState;
extern uint8 Target_Shoot_State;

//功能函数声明
uint8 leftRoundabout0(void);
uint8 rightRoundabout0(void);
uint8 leftRoundabout1(void);
uint8 rightRoundabout1(void);
uint8 checkRoundaboutExit(void);
uint8 checkLeftRoundaboutExit(void);
uint8 checkRightRoundaboutExit(void);

uint8 crossroad0(void);
void crossFillLine(void);
void crossFillLine_vertical(void);

uint8 targetCoarseScan(void);//初筛
uint8 targetTrackScan(void);//细筛
void Target_Gimbal_Aim(int16 target_col);

//调用函数
void crossroadState(void);
void roundaboutState(void);
void targetStateMachine(void);