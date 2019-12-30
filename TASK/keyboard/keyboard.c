#include "keyboard.h"
#include "control.h"
#include "pwm.h"
#include "can.h"

KeyBoardTypeDef Key1={0};
KeyBoardTypeDef Key2={0};
KeyBoardTypeDef Key3={0};
KeyBoardTypeDef Key4={0};
KeyBoardTypeDef Key5={0};
KeyBoardTypeDef Key6={0};
KeyBoardTypeDef Key7={0};
KeyBoardTypeDef KeyQ={0};

TakeBulletTypeDef Bullet={0};


/***********���������ֵ*************/
u8 MainBoard_SendBuffer_L=0;
u8 MainBoard_SendBuffer_H=0;
u8 MainBoard_SendDate=0;
/*************************/





void VideoRelay_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOFʱ��

  //GPIOA8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	//////////////////�̵���IO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//GPIOF14,��0���ӵ�����·

}








int tem_i=0;

u8 KeyValue[8]={0,0,0,0,0,0,0,0};	//�����ֵ

void Res_Deal(u8 key[8],u8 res)
{
	for(tem_i=0;tem_i<8;tem_i++)
	{
	key[tem_i]=(res>>tem_i)&1;
	}
}




//��λ������
//���ֵ��1:�̰� 2:���� 0:δ��
//���ֶ̰��Ĵ�����ʽ ����ע��ѡ��
//2017.4.29
void ButtonStatu_Verdict(KeyBoardTypeDef * Key,u8 key_value)	//�����ּ�ⷽ����һ������ʱ��Ϊ�ֽ��ͺ����߼�Ϊ����ֵ��ԭ����һ���޸��˳���״̬ǰʼ�ջ������һ״̬��ȱ��
{																			//�ú�������10ms��ʱ���У�
	if(Key->last==1)
	{
		Key->count++;
	}
	else
	{
		Key->count=0;
	}
	
	if(Key->count>1)	//���������� 10ms
	{
		if(Key->count<100)
		{
			if(Key->last==1&&key_value==0)
			{
				Key->value=1;
			}
		}
		else
		{
			Key->value=2;
		}
	}
	else
	{
		Key->value=0;
	}
	Key->last=key_value;
}



void KeyQ_Deal(u8 key_value)	//�ǵ�ģʽ����
{
	if(KeyQ.last==0&&key_value==1)	//��
	{
		Lift_Anterior(1);
		Lift_Posterior(1);
		Servo_ALL(1);
		Servo_Back_Set(OPEN_BACK);
		
		if(Key7.statu==0)	//ͼ���Զ�����
		{
			Key7.statu=!Key7.statu;
			Servo_Video(Key7.statu);
			MainBoard_SendBuffer_H=Key7.statu<<7;
		}
	}
	
	if(KeyQ.last==1&&key_value==0)	//��
	{
		Lift_Anterior(0);
		Lift_Posterior(0);
		Servo_ALL(0);
		Servo_Back_Set(CLOSE_BACK);
		
		if(Key7.statu==1)	//ͼ���Զ�����
		{
			Key7.statu=!Key7.statu;
			Servo_Video(Key7.statu);
			MainBoard_SendBuffer_H=Key7.statu<<7;
		}
	}
	KeyQ.last=key_value;
	
}


void KeyG_Deal(u8 key_value)	//ǰ����
{
	ButtonStatu_Verdict(&Key1,key_value);
	
	switch(Key1.value)
	{
		case 1:
		{
			if(StepMotor1.position_val==0)
			{
				Lift_Anterior(1);
			}
			break;
		}
		case 2:
		{
			Lift_Bullet(1);
			Bullet.circulation_i=0;
			break;
		}
	}
	
}



void KeyF_Deal(u8 key_value)	//ǰ����
{
	ButtonStatu_Verdict(&Key2,key_value);
	
	switch(Key2.value)
	{
		case 1:
		{
			if(StepMotor1.position_val==0)
			{
				Lift_Anterior(0);
			}
			break;
		}
		case 2:
		{
			
			break;
		}
	}
	
}



void KeyB_Deal(u8 key_value)	//����
{
	ButtonStatu_Verdict(&Key3,key_value);
	
	switch(Key3.value)
	{
		case 1:
		{
			if(StepMotor1.position_val==0)
			{
				Lift_Posterior(1);
			}
			break;
		}
		case 2:
		{
			Lift_Bullet(1);
			Bullet.circulation_i=0;
			break;
		}
	}
	
}



void KeyV_Deal(u8 key_value)	//�󲿽�
{
	ButtonStatu_Verdict(&Key4,key_value);
	
	switch(Key4.value)
	{
		case 1:
		{
			if(StepMotor1.position_val==0)
			{
				Lift_Posterior(0);
			}
			break;
		}
		case 2:
		{
			
			break;
		}
	}
	
}



void KeyE_Deal(u8 key_value)	//���ֶ������	�̰��� ������
{
	ButtonStatu_Verdict(&Key5,key_value);
	
	switch(Key5.value)
	{
		case 1:
		{
			//Servo_ALL(1);
			Servo_Back_Set(OPEN_BACK);
			break;
		}
		case 2:
		{
			//Servo_ALL(0);
			Servo_Back_Set(CLOSE_BACK);
			break;
		}
	}
	
}



void KeyR_Deal(u8 key_value)	//ȡ���߼���־  ִ���п���д
{
	ButtonStatu_Verdict(&Key6,key_value);
	
	switch(Key6.value)
	{
		case 1:
		{
			if(KeyQ.last==1)	//���ڵǵ�ģʽ����Ч
			{
				VIDEO=1;	//ͼ��ת������
			}
			
			Key6.statu=0;	//�ջ�
			break;
		}
		case 2:
		{
			if(CM1_Position_Feedback.turns+CM2_Position_Feedback.turns+CM3_Position_Feedback.turns+CM4_Position_Feedback.turns-4*BULLET>(-40))
			{
				VIDEO=0;	//ͼ��ת������
				Key6.statu=1;	//ȡ��
			}
			break;
		}
	}
	
}



void KeyC_Deal(u8 key_value)	//ͼ�����	//�̰����� ������Ч	//ֵΪ0ʱ��ǰ��ֵΪ1ʱ���
{
	ButtonStatu_Verdict(&Key7,key_value);
	 
	if(KeyValue[7]==1)	//�ǵ�ģʽ��
	{
		switch(Key7.value)
		{
			case 1:
			{
				Key7.statu=!Key7.statu;
				Servo_Video(Key7.statu);
				MainBoard_SendBuffer_H=Key7.statu<<7;
				break;
			}
			case 2:
			{
			
				break;
			}
		}
	}
	else	//�ǵǵ�ģʽ
	{
		switch(Key7.value)
		{
			case 1:
			{
				Key7.statu=!Key7.statu;
				Servo_Video(Key7.statu);
				//MainBoard_SendBuffer_H=Key7.statu<<7;
				VIDEO=1;	//ͼ��ת������
				break;
			}
			case 2:
			{
			VIDEO=0;	//ͼ��ת������
				break;
			}
		}
	}
}



void MainBoard_Deal(void)
{
	if(Bullet.sendlast==0&&Bullet.send_sign==1)
	{
		MainBoard_SendBuffer_L=Bullet.send[Bullet.circulation_i];
		Bullet.circulation_i++;
		if(Bullet.circulation_i>(Y_RANGE-1))
		{
			Bullet.circulation_i=0;
		}
	}
	else
	{
		if(Key6.statu==0)
		{
			MainBoard_SendBuffer_L=0;
		}
	}
	Bullet.sendlast=Bullet.send_sign;
}




u8 TakBult_STPCM=0;
u8 TakBult_statu=0;
//�°�ȡ��
void TakeBullet(u8 input)	//0Ϊ��  1Ϊ��
{
	switch(input)
	{
		case 0:	//ȡ�����
		{
			Bullet.depth=0;
			Bullet.statu=STEPMOTOR2_EXTEND;
			SteppingMotor2_Set(0,1);	//�ջ�Z����
			
			
			if(StepMotor2.position_val==0)
			{
				Friction_ALL_Set(FRICTION_CLOSE);
				SteppingMotor1_Set(0,1);	//�ջ�X��
			}
			
			break;
		}
		case 1:	//��ʼȡ��
		{
			if(CM1_Position_Feedback.turns+CM2_Position_Feedback.turns+CM3_Position_Feedback.turns+CM4_Position_Feedback.turns-4*BULLET>(-40))
			{
				if(StepMotor1.position_val>STEP1_CONTACT_DISTANCE/2)
				{
					Friction_ALL_Set(FRICTION_OPEN);
				}
				
				switch(Bullet.statu)
				{
					case STEPMOTOR2_EXTEND:
					{
						SteppingMotor1_Set(STEP1_CONTACT_DISTANCE,2);	//���X��
				
						if(StepMotor1.position_val==STEP1_CONTACT_DISTANCE)
						{
							SteppingMotor2_Set(STEP2_LIMIT_DISTANCE+1500*Bullet.depth,2);	//���Z�Ἣ���г�
							if(StepMotor2.position_val==STEP2_LIMIT_DISTANCE)
							{
								Bullet.statu=STEPMOTOR1_EXTEND;	//�������1��ʼ��ת
							}
						}
						break;
					}
					case STEPMOTOR1_EXTEND:
					{
						SteppingMotor1_Set(STEP1_LIMIT_DISTANCE,2);
						if(StepMotor1.position_val==STEP1_LIMIT_DISTANCE)
						{
							Bullet.statu=STEPMOTOR_BACK;
						}
						break;
					}
					case STEPMOTOR_BACK:
					{
						SteppingMotor1_Set(STEP1_CONTACT_DISTANCE,2);
						if(StepMotor1.position_val==STEP1_CONTACT_DISTANCE)
						{
							SteppingMotor2_Set(STEP2_CONTACT_DISTANCE+Bullet.depth*400,2);
							if(StepMotor2.position_val==STEP2_CONTACT_DISTANCE+Bullet.depth*400) //??
							{
							Bullet.statu=STEPMOTOR_DELAY;
							Bullet.time_record=TimeCount_Common;	//��ȡ��ǰϵͳʱ��
							}
						}
						break;
					}
					case STEPMOTOR_DELAY:
					{
						Bullet.send_sign=1;
						if(TimeCount_Common>Bullet.time_record+DELAY_TIME)
						{
							Bullet.send_sign=0;
							Bullet.statu=STEPMOTOR2_EXTEND;
							Bullet.depth++;
							if(Bullet.depth>3)
							{
								Bullet.depth=3;
							}
						}
						break;
					}
				}
				
			}
			
			break;
		}
	}
}




//�ϰ�ȡ����2������
//void TakeBullet(u8 input)	//0Ϊ��  1Ϊ��
//{
//	switch(input)
//	{
//		case 0:	//ȡ�����
//		{
//			Friction_ALL_Set(FRICTION_CLOSE);
//			SteppingMotor2_Set(0,1);	//�ջ�Z����
//			if(StepMotor2.position_val==0)
//			{
//				SteppingMotor1_Set(0,1);	//�ջ�X��
//			}
//			
//			break;
//		}
//		case 1:	//��ʼȡ��
//		{
//			if(CM1_Position_Feedback.turns+CM2_Position_Feedback.turns+CM3_Position_Feedback.turns+CM4_Position_Feedback.turns-4*BULLET>(-40))
//			{
//				SteppingMotor1_Set((STEP1_DISTANCE-TakBult_STPCM*STEP1_DISTANCE/8),1);	//���X��
//				
//				if(StepMotor1.position_val>STEP1_DISTANCE/2)
//				{
//					Friction_ALL_Set(FRICTION_OPEN);
//				}
//				
//				
//				if(StepMotor1.position_val==STEP1_DISTANCE/2)
//				{
//					Friction_ALL_Set(FRICTION_OPEN);
//				
//					SteppingMotor2_Set((STEP2_DISTANCE-TakBult_STPCM*STEP2_DISTANCE/2),stepmotor2_cycle_send);	//���Z�� ����2s���������ƶ�
//					
//					if(TimeCount_Common%4000==0)
//					{
//						TakBult_STPCM=!TakBult_STPCM;
//					}
//					
//				}
//			}
//			break;
//		}
//	}
//}
