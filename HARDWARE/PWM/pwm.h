#ifndef __PWM_H
#define __PWM_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//ALIENTEK STM32F407开发板
//3路舵机 驱动代码	   
//机械系RM控制组
//创建日期:2017/4/1
//版本：V1.0							  
////////////////////////////////////////////////////////////////////////////////// 	



void TIM4_PWM_Init(u32 arr,u32 psc);
void TIM5_PWM_Init(u32 arr,u32 psc);
void ALL_PWM_Init(void);
void Servo_Video_Set(u32 set5);
void Servo_Left_Set(u32 set1);
void Servo_Right_Set(u32 set2);
void Servo_Back_Set(u32 set3);
void Friction_ALL_Set(u32 set4);
#endif
