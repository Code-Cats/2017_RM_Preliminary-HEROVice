#include "timer.h"
#include "led.h"
#include "PID.h"
#include "control.h"
#include "can.h"
#include "Mydefine.h"
#include "imu.h"

#include "keyboard.h"

u8 PID_Cycle=0;
//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	PID_Cycle=10/(arr+1);	//ʹPID����Ƶ��ʼ��Ϊ1ms	//��Ƶ�������ĵ������
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

u32 TimeCount_Common=0;	//ȫ������ʱ�����
int tem_send=0;

//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
	
	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
		if(TimeCount_Common%PID_Cycle==0)	//1ms���ڲ���
		{
		
			LED_Pointing(GetWorkState());
			
			RampCalc_CM12();	//���º������в���
			RampCalc_CM34();
			
			PID_Calc(&CM1_Robust_Pid,CM1_Position_Send,CM1_Speed_Feedback.calc,CM1_Position_Feedback.turns);
			PID_Calc(&CM2_Robust_Pid,CM2_Position_Send,CM2_Speed_Feedback.calc,CM2_Position_Feedback.turns);
			PID_Calc(&CM3_Robust_Pid,CM3_Position_Send,CM3_Speed_Feedback.calc,CM3_Position_Feedback.turns);
			PID_Calc(&CM4_Robust_Pid,CM4_Position_Send,CM4_Speed_Feedback.calc,CM4_Position_Feedback.turns);
			
//			TopplingPreventing();	//���㵹	//���������ṩ˼·������ʹ��
			LiftRmap_Detaction();	//��������
			
			if(GetWorkState()==NORMAL_STATE||GetWorkState()==CALI_STATE)
			{
				
				if(Imu_Deal.ay_calc>FOREROKE)	//ǰ��
				{
					if((CM3_Robust_Pid.output+CM4_Robust_Pid.output-CM1_Robust_Pid.output-CM2_Robust_Pid.output)>300)
					{
						Set_CM_Speed(0,0,0,0);
					}
					else
					{
						Set_CM_Speed((int)(CM1_Robust_Pid.output*CM12_Ramp.calc),(int)(CM2_Robust_Pid.output*CM12_Ramp.calc),(int)(CM3_Robust_Pid.output*CM34_Ramp.calc),(int)(CM4_Robust_Pid.output*CM34_Ramp.calc));	//	����ֵ���͸����
					}
				}
				else if(Imu_Deal.ay_calc<HYPSOKINESIS)	//����
				{
					if((CM3_Robust_Pid.output+CM4_Robust_Pid.output-CM1_Robust_Pid.output-CM2_Robust_Pid.output)<-300)
					{
						Set_CM_Speed(0,0,0,0);
					}
					else
					{
						Set_CM_Speed((int)(CM1_Robust_Pid.output*CM12_Ramp.calc),(int)(CM2_Robust_Pid.output*CM12_Ramp.calc),(int)(CM3_Robust_Pid.output*CM34_Ramp.calc),(int)(CM4_Robust_Pid.output*CM34_Ramp.calc));	//	����ֵ���͸����
					}
				}
				else
				{
					Set_CM_Speed((int)(CM1_Robust_Pid.output*CM12_Ramp.calc),(int)(CM2_Robust_Pid.output*CM12_Ramp.calc),(int)(CM3_Robust_Pid.output*CM34_Ramp.calc),(int)(CM4_Robust_Pid.output*CM34_Ramp.calc));	//	����ֵ���͸����
				}
				
			}
			else if(GetWorkState()==PREPARE_STATE) //׼���׶Σ��������ת
			{
				Set_CM_Speed(0,0,0,0);
			}
			
			
			
			tem_send=(int)CM1_Robust_Pid.output*CM12_Ramp.calc;/////////////////////////////////////
			
			
			
			
			if(Calibration_Start==1)//����ʼ���궨�õļ���ֵ
			{
				Calibration_Time_Count++;
			}
		}			//1ms���ڲ���
		
		
		if(TimeCount_Common%(PID_Cycle*10)==0)	//10ms���ڲ��� ��100HZ
		{
			MainBoard_SendDate=MainBoard_SendBuffer_H|MainBoard_SendBuffer_L;
			USART_SendData(USART6,MainBoard_SendDate);
		}
		
		
		if(GetWorkState()==NORMAL_STATE)
		{
			
			TakeBullet(Key6.statu);	//ȡ��ִ��
			
			SteppingMotor1_Run(StepMotor1.cycle_set,StepMotor1.step_set);	//����ֵ�벻��ѭ��
			RampCalc_ST1();
			SteppingMotor2_Run(StepMotor2.cycle_set,StepMotor2.step_set);	//����ֵ�벻��ѭ��
			RampCalc_ST2();
		}
		
		
		TimeCount_Common++;	//������ 2000hz�㹻��596Сʱ
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}

