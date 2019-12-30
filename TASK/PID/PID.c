#include "PID.h"


PidTypeDef CM1_Robust_Pid={0};
PidTypeDef CM2_Robust_Pid={0};
PidTypeDef CM3_Robust_Pid={0};
PidTypeDef CM4_Robust_Pid={0};



void PID_Init(PidTypeDef * pid,double kp,double kd,double dead_band,double max_error)
{
	pid->Kp=kp;
//	pid->Ki=ki;
	pid->Kd=kd;
//	pid->max_out=max_out;
	pid->dead_band=dead_band;
//	pid->intergral_dead_band=i_deadband;
	pid->max_error=max_error;
	//PID输出值
	pid->output=0;
	//误差
	pid->Position_Error[LAST]=0;
	pid->Position_Error[NOW]=0;
	pid->Speed[LAST]=0;
	pid->Speed[NOW]=0;
}



void PID_Calc(PidTypeDef * pid,s16 Position_Expectation,s16 Speed_Feedback,s16 Position_Feedback)	//鲁棒式PID
{
  pid->Position_Error[NOW]=Position_Expectation-Position_Feedback;
	
	if(pid->Position_Error[NOW]>pid->max_error)	//限制最大输入，实现爬坡效果
	{
		pid->Position_Error[NOW]=pid->max_error;
	}
	else if(pid->Position_Error[NOW]<-(pid->max_error))
	{
		pid->Position_Error[NOW]=-(pid->max_error);
	}
	
	pid->Speed[NOW]=Speed_Feedback;
	pid->output=pid->output+pid->Kp*(pid->Position_Error[NOW]-pid->Position_Error[LAST])-pid->Kd*(pid->Speed[NOW]-pid->Speed[LAST]);	//	增量式PID计算
	pid->Position_Error[LAST]=pid->Position_Error[NOW];
	pid->Speed[LAST]=pid->Speed[NOW];
}


float MyAbs(float num)
{
	if(num>=0)
		return num;
	else 
		return -num;	
}

