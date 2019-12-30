#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
typedef struct
{
  s16 real[8];
//	u8  count;	//曾经的取平均值计数
	s16 sum;
	s16 calc;	//	两次一平均的角度数值
	
	s16 calc_diff;
	s16 calc_last;	//上一次计算的角度值
	s16 turns;	//监视两次角度值之差得出的圈数相对值
} ReceiveTypeDef;
					    								 							 				    
void CAN1_Init(void);//CAN初始化
void Set_CM_Speed(s16 cm1_iq,s16 cm2_iq,s16 cm3_iq,s16 cm4_iq);		//发送数据
void Motor_Data_Receive(CanRxMsg *msg);	//反馈数据处理合总函数
void Speed_Data_deal(ReceiveTypeDef *Receive,CanRxMsg * msg);
void Position_Data_deal(ReceiveTypeDef *Receive,CanRxMsg * msg);
void Position_To_Turns(ReceiveTypeDef *Receive);	//由角度值得到圈数值
#endif



extern ReceiveTypeDef CM1_Speed_Feedback;
extern ReceiveTypeDef CM2_Speed_Feedback;
extern ReceiveTypeDef CM3_Speed_Feedback;
extern ReceiveTypeDef CM4_Speed_Feedback;

extern ReceiveTypeDef CM1_Position_Feedback;
extern ReceiveTypeDef CM2_Position_Feedback;
extern ReceiveTypeDef CM3_Position_Feedback;
extern ReceiveTypeDef CM4_Position_Feedback;
