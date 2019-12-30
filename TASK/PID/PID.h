#ifndef __PID_H
#define __PID_H	

#include "sys.h"

#define NOW 0
#define LAST 1

typedef struct
{
    //PID 三参数
    double Kp;
//    double Ki;
    double Kd;
		//最大输出 死区
//		double max_out;  //最大输出
		double dead_band;//PID偏差死区
//		double intergral_dead_band;//积分死区
		double max_error;//最大输入
    //PID输出值
    double output;
//		double output_compensation;
    //误差
    double Position_Error[2];//0最新 1上一次
	  double Speed[2];//0最新 1上一次
//		double intergral;
} PidTypeDef;

//void PID_Init(PidTypeDef * pid,double kp,double kd,double dead_band,double max_error);
void PID_Init(PidTypeDef * pid,double kp,double kd,double dead_band,double max_error);
void PID_Calc(PidTypeDef * pid,s16 Position_Expectation,s16 Speed_Feedback,s16 Position_Feedback);
float MyAbs(float num);

#endif


extern PidTypeDef CM1_Robust_Pid;
extern PidTypeDef CM2_Robust_Pid;
extern PidTypeDef CM3_Robust_Pid;
extern PidTypeDef CM4_Robust_Pid;
