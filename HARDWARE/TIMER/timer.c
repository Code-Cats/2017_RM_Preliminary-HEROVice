#include "timer.h"
#include "led.h"
#include "PID.h"
#include "control.h"
#include "can.h"
#include "Mydefine.h"
#include "imu.h"

#include "keyboard.h"

u8 PID_Cycle=0;
//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	PID_Cycle=10/(arr+1);	//使PID计算频率始终为1ms	//分频数不更改的情况下
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

u32 TimeCount_Common=0;	//全场公用时间计数
int tem_send=0;

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	
	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		if(TimeCount_Common%PID_Cycle==0)	//1ms周期部分
		{
		
			LED_Pointing(GetWorkState());
			
			RampCalc_CM12();	//爬坡函数运行部分
			RampCalc_CM34();
			
			PID_Calc(&CM1_Robust_Pid,CM1_Position_Send,CM1_Speed_Feedback.calc,CM1_Position_Feedback.turns);
			PID_Calc(&CM2_Robust_Pid,CM2_Position_Send,CM2_Speed_Feedback.calc,CM2_Position_Feedback.turns);
			PID_Calc(&CM3_Robust_Pid,CM3_Position_Send,CM3_Speed_Feedback.calc,CM3_Position_Feedback.turns);
			PID_Calc(&CM4_Robust_Pid,CM4_Position_Send,CM4_Speed_Feedback.calc,CM4_Position_Feedback.turns);
			
//			TopplingPreventing();	//防倾倒	//本函数仅提供思路，请勿使用
			LiftRmap_Detaction();	//升降爬坡
			
			if(GetWorkState()==NORMAL_STATE||GetWorkState()==CALI_STATE)
			{
				
				if(Imu_Deal.ay_calc>FOREROKE)	//前倾
				{
					if((CM3_Robust_Pid.output+CM4_Robust_Pid.output-CM1_Robust_Pid.output-CM2_Robust_Pid.output)>300)
					{
						Set_CM_Speed(0,0,0,0);
					}
					else
					{
						Set_CM_Speed((int)(CM1_Robust_Pid.output*CM12_Ramp.calc),(int)(CM2_Robust_Pid.output*CM12_Ramp.calc),(int)(CM3_Robust_Pid.output*CM34_Ramp.calc),(int)(CM4_Robust_Pid.output*CM34_Ramp.calc));	//	电流值发送给电调
					}
				}
				else if(Imu_Deal.ay_calc<HYPSOKINESIS)	//后倾
				{
					if((CM3_Robust_Pid.output+CM4_Robust_Pid.output-CM1_Robust_Pid.output-CM2_Robust_Pid.output)<-300)
					{
						Set_CM_Speed(0,0,0,0);
					}
					else
					{
						Set_CM_Speed((int)(CM1_Robust_Pid.output*CM12_Ramp.calc),(int)(CM2_Robust_Pid.output*CM12_Ramp.calc),(int)(CM3_Robust_Pid.output*CM34_Ramp.calc),(int)(CM4_Robust_Pid.output*CM34_Ramp.calc));	//	电流值发送给电调
					}
				}
				else
				{
					Set_CM_Speed((int)(CM1_Robust_Pid.output*CM12_Ramp.calc),(int)(CM2_Robust_Pid.output*CM12_Ramp.calc),(int)(CM3_Robust_Pid.output*CM34_Ramp.calc),(int)(CM4_Robust_Pid.output*CM34_Ramp.calc));	//	电流值发送给电调
				}
				
			}
			else if(GetWorkState()==PREPARE_STATE) //准备阶段，电机不旋转
			{
				Set_CM_Speed(0,0,0,0);
			}
			
			
			
			tem_send=(int)CM1_Robust_Pid.output*CM12_Ramp.calc;/////////////////////////////////////
			
			
			
			
			if(Calibration_Start==1)//给初始化标定用的计数值
			{
				Calibration_Time_Count++;
			}
		}			//1ms周期部分
		
		
		if(TimeCount_Common%(PID_Cycle*10)==0)	//10ms周期部分 即100HZ
		{
			MainBoard_SendDate=MainBoard_SendBuffer_H|MainBoard_SendBuffer_L;
			USART_SendData(USART6,MainBoard_SendDate);
		}
		
		
		if(GetWorkState()==NORMAL_STATE)
		{
			
			TakeBullet(Key6.statu);	//取弹执行
			
			SteppingMotor1_Run(StepMotor1.cycle_set,StepMotor1.step_set);	//传输值与不断循环
			RampCalc_ST1();
			SteppingMotor2_Run(StepMotor2.cycle_set,StepMotor2.step_set);	//传输值与不断循环
			RampCalc_ST2();
		}
		
		
		TimeCount_Common++;	//不清零 2000hz足够跑596小时
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}

