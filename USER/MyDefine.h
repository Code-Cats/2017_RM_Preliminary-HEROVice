#ifndef __MYDEF_H
#define __MYDEF_H

#include "sys.h"



#define DEBUG_MODE 0							//是否开启外部辅助调试模式
																	//不用时关闭，节省系统资源
//示例：
//#if DEBUG_MODE
//************
//#endif



#define ABS(x)	( (x>0) ? (x) : (-x) )


/***********************************----pid.c define----*****************************************/

#define ROBUST_P 120
#define ROBUST_D 0.5
#define ROBUST_MER 150
#define ROBUST_DEAD 0


////////////////////////////////////////////////////////////////////////////////////////


/********************************---led.c define----****************************************/
#define LIGHT 0
#define LIGHT_OUT 1


extern u8 LED_PointStatu;

//
// 
/**************************-----pwm.c define-----******************************/
#define SERVO_CYCLE 20000		//84M/84=1Mhz的计数频率,重装载值20000，所以PWM频率为 1M/20000=50hz.  即20ms周期
#define ESC_CYCLE 2500			//2.5ms周期


/******************************************************************************/



//
//
/**************************-----control.c define----***************************/

/**********步进电机************/
#define STEP1 PFout(10)	//步进电机1的脉冲输出端口
#define DIR1 PIout(9)	//步进电机1方向输出口
#define STEP2 PAout(5)	//步进电机2的脉冲输出端口
#define DIR2 PAout(4)	//步进电机2方向输出口

#define DIR_PLUS 1	//伸出方向电平定义
#define DIR_MINUS 0	//缩回方向电平定义


#define STEPMOTOR_STARTSPEED 5	//步进电机启动速度

#define STEP1_LIMIT_DISTANCE 24000	//待测 X轴步进电机极限行程
#define STEP2_LIMIT_DISTANCE 6000	//待测 Z轴步进电机极限行程
#define STEP1_CONTACT_DISTANCE 22000//待测 接触行程
#define STEP2_CONTACT_DISTANCE 3500//待测 接触行程

/**********升降电机***********/
#define FALL 15
#define ISLAND 670
#define BULLET 770	//?待测

/************舵机***********/
#define VIDEO_UP 1500	//500
#define VIDEO_BACK 600	//2500

#define OPEN_LEFT 500
#define OPEN_RIGHT 1450
#define OPEN_BACK 2500	//待测	//测得2500&1500？
#define CLOSE_LEFT 1500
#define CLOSE_RIGHT 600
#define CLOSE_BACK 1800	//待测

/***********摩擦轮************/
#define FRICTION_OPEN 1750	//待试最优参数
#define FRICTION_CLOSE 1000   

/************保护************/
#define FOREROKE 380	//前后倾斜阈值
#define HYPSOKINESIS -580

#define CYCLE_LIFTRAMP 10	//计算方法CYCLE=CYCLE_LIFTRAMP*100*time定时器频率

typedef struct
{
	u32 cycle;
	u8 count;	//累加计数
	float calc;	//爬坡系数0-1
	s32 last_value;	//为升降电机增加的纪录值
} RampTypeDef;		//爬坡结构体


typedef struct
{
	s32 position_val;	//记录当前步进电机位置值
	u8 dir_set;
	u32 step_count;	//步进电机脉冲计数（记录当前脉冲数）
	u32 step_set;
	u8 cycle_set;
	u8 step_complete;//标志一次脉冲输出完成
} StepMotorTypeDef;		//步进电机结构体

extern StepMotorTypeDef StepMotor1;
extern StepMotorTypeDef StepMotor2;


extern RampTypeDef Step1Ramp;
extern RampTypeDef Step2Ramp;
extern RampTypeDef CM12_Ramp;
extern RampTypeDef CM34_Ramp;

//extern u8 dir_set;
//extern u32 step_count;
//extern u32 step_set;
//extern u8 cycle_set;
//extern u8 step_complete;


extern s32 stepmotor1_position_send;
extern u8 stepmotor1_cycle_send;
extern s32 stepmotor2_position_send;
extern u8 stepmotor2_cycle_send;
/******************************************************************************/


/********************************--imu.c define---**********************************/
#define FILTERING_RANGE 8

typedef struct
{
	s32 ax_sum[FILTERING_RANGE];
	s32 ay_sum[FILTERING_RANGE];
	s32 az_sum[FILTERING_RANGE];
	u8 i;
	s32 ax_calc;
	s32 ay_calc;
	s32 az_calc;
	u8 count;	//计数
} ImuTypeDef;		//陀螺仪数据处理结构体

extern ImuTypeDef Imu_Deal;

////////////////////////////////////////////////////////////////////////////////////



/**************************-----timer.c define------***************************/

extern u32 TimeCount_Common;	//全场公用时间计数

////////////////////////////////////////////////////////////////////////////////


/******************************-----keyboard.c define-----************************************************/

typedef struct
{
	u16 count;
	u8 value;
	u8 last;
	u8 statu;
}KeyBoardTypeDef;


#define Y_RANGE 3

typedef struct
{
	u32 time_record;
	u8 sendlast;
	u8 statu;
	u8 send[Y_RANGE];
	u8 circulation_i;
	u8 send_sign;
	u8 depth;
}TakeBulletTypeDef;


#define STEPMOTOR_DELAY 0
#define STEPMOTOR1_EXTEND 1
#define STEPMOTOR1_BACK 2
#define STEPMOTOR2_EXTEND 3
#define STEPMOTOR_BACK 4
#define DELAY_TIME 1000


extern u8 MainBoard_SendDate;
extern u8 MainBoard_SendBuffer_L;
extern u8 MainBoard_SendBuffer_H;

extern TakeBulletTypeDef Bullet;

extern KeyBoardTypeDef Key1;
extern KeyBoardTypeDef Key2;
extern KeyBoardTypeDef Key3;
extern KeyBoardTypeDef Key4;
extern KeyBoardTypeDef Key5;
extern KeyBoardTypeDef Key6;
extern KeyBoardTypeDef Key7;
extern KeyBoardTypeDef KeyQ;

extern u8 KeyValue[8];
extern u8 TakBult_STPCM;
////////////////////////////////////////////////////////////////////////////////////////////////////////


/***********************--工作状态--**********************/
typedef enum
{
		
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
 //   STANDBY_STATE,			//云台停止不转状态
    NORMAL_STATE,			//正常输入状态
 //   STOP_STATE,        	//停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;

extern WorkState_e workState;
void SetWorkState(WorkState_e state);
WorkState_e GetWorkState(void);
/*
WorkState_e workState = PREPARE_STATE;

static void SetWorkState(WorkState_e state)
{
    workState = state;
}


WorkState_e GetWorkState()
{
	return workState;
}

//使用示例
if(GetWorkState()==PREPARE_STATE) //启动阶段，底盘不旋转
	{
		ChassisSpeedRef.rotate_ref = 0;	 
	}
*/	
	
	
	
#endif

