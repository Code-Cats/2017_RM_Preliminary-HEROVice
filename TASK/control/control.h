#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"	



void Lift_Calibration(void);
void Ascend_Island(void);//////////////////
void Descend_Island(void);///////////////////
void TakeBullets(void);////////////////////
void Lift_Anterior(u8 statu1);
void Lift_Posterior(u8 statu2);
void Lift_Bullet(u8 statu3);
void Servo_ALL(u8 statu3);
void Servo_Video(u8 statu1);

void Hardware_Init(void);
void Date_Init(void);

void TopplingPreventing(void);
void LiftRmap_Detaction(void);

void RampBegin_ST1(u16 cycle);
void RampCalc_ST1(void);

void RampBegin_ST2(u16 cycle);
void RampCalc_ST2(void);

void RampBegin_CM12(u16 cycle);
void RampCalc_CM12(void);

void RampBegin_CM34(u16 cycle);
void RampCalc_CM34(void);

void SteppingMotor1_Run(u8 cycle_min,u32 step);
void SteppingMotor1_Set(s32 position,u8 cycle_min);
void SteppingMotor2_Run(u8 cycle_min,u32 step);
void SteppingMotor2_Set(s32 position,u8 cycle_min);
void SteppMotor_Init(void);

#endif

extern s16 CM1_Position_Send;
extern s16 CM2_Position_Send;
extern s16 CM3_Position_Send;
extern s16 CM4_Position_Send;
extern u8 waiting;

extern u8 Calibration_Start;
extern u8 Calibration_Time_Count;
