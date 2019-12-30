#ifndef __PID_H
#define __PID_H	

#include "sys.h"

#define NOW 0
#define LAST 1

typedef struct
{
    //PID ������
    double Kp;
//    double Ki;
    double Kd;
		//������ ����
//		double max_out;  //������
		double dead_band;//PIDƫ������
//		double intergral_dead_band;//��������
		double max_error;//�������
    //PID���ֵ
    double output;
//		double output_compensation;
    //���
    double Position_Error[2];//0���� 1��һ��
	  double Speed[2];//0���� 1��һ��
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
