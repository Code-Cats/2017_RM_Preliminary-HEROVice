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


/***********主板回送数值*************/
u8 MainBoard_SendBuffer_L=0;
u8 MainBoard_SendBuffer_H=0;
u8 MainBoard_SendDate=0;
/*************************/





void VideoRelay_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOF时钟

  //GPIOA8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	//////////////////继电器IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	GPIO_SetBits(GPIOA,GPIO_Pin_8);//GPIOF14,置0，接底盘线路

}








int tem_i=0;

u8 KeyValue[8]={0,0,0,0,0,0,0,0};	//储存键值

void Res_Deal(u8 key[8],u8 res)
{
	for(tem_i=0;tem_i<8;tem_i++)
	{
	key[tem_i]=(res>>tem_i)&1;
	}
}




//键位处理函数
//结果值：1:短按 2:长按 0:未按
//两种短按的触发方式 自行注释选择
//2017.4.29
void ButtonStatu_Verdict(KeyBoardTypeDef * Key,u8 key_value)	//有两种检测方法，一种是以时间为分界点和后来者即为最终值的原理。另一种修复了长按状态前始终会存在另一状态的缺点
{																			//该函数放在10ms定时器中！
	if(Key->last==1)
	{
		Key->count++;
	}
	else
	{
		Key->count=0;
	}
	
	if(Key->count>1)	//防抖动部分 10ms
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



void KeyQ_Deal(u8 key_value)	//登岛模式开启
{
	if(KeyQ.last==0&&key_value==1)	//入
	{
		Lift_Anterior(1);
		Lift_Posterior(1);
		Servo_ALL(1);
		Servo_Back_Set(OPEN_BACK);
		
		if(Key7.statu==0)	//图传自动换向
		{
			Key7.statu=!Key7.statu;
			Servo_Video(Key7.statu);
			MainBoard_SendBuffer_H=Key7.statu<<7;
		}
	}
	
	if(KeyQ.last==1&&key_value==0)	//出
	{
		Lift_Anterior(0);
		Lift_Posterior(0);
		Servo_ALL(0);
		Servo_Back_Set(CLOSE_BACK);
		
		if(Key7.statu==1)	//图传自动换向
		{
			Key7.statu=!Key7.statu;
			Servo_Video(Key7.statu);
			MainBoard_SendBuffer_H=Key7.statu<<7;
		}
	}
	KeyQ.last=key_value;
	
}


void KeyG_Deal(u8 key_value)	//前部升
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



void KeyF_Deal(u8 key_value)	//前部降
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



void KeyB_Deal(u8 key_value)	//后部升
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



void KeyV_Deal(u8 key_value)	//后部降
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



void KeyE_Deal(u8 key_value)	//导轮舵机开关	短按开 长按关
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



void KeyR_Deal(u8 key_value)	//取弹逻辑标志  执行中控另写
{
	ButtonStatu_Verdict(&Key6,key_value);
	
	switch(Key6.value)
	{
		case 1:
		{
			if(KeyQ.last==1)	//仅在登岛模式下有效
			{
				VIDEO=1;	//图传转到底盘
			}
			
			Key6.statu=0;	//收回
			break;
		}
		case 2:
		{
			if(CM1_Position_Feedback.turns+CM2_Position_Feedback.turns+CM3_Position_Feedback.turns+CM4_Position_Feedback.turns-4*BULLET>(-40))
			{
				VIDEO=0;	//图传转到弹舱
				Key6.statu=1;	//取弹
			}
			break;
		}
	}
	
}



void KeyC_Deal(u8 key_value)	//图传舵机	//短按开关 长按无效	//值为0时向前，值为1时向后
{
	ButtonStatu_Verdict(&Key7,key_value);
	 
	if(KeyValue[7]==1)	//登岛模式下
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
	else	//非登岛模式
	{
		switch(Key7.value)
		{
			case 1:
			{
				Key7.statu=!Key7.statu;
				Servo_Video(Key7.statu);
				//MainBoard_SendBuffer_H=Key7.statu<<7;
				VIDEO=1;	//图传转到弹舱
				break;
			}
			case 2:
			{
			VIDEO=0;	//图传转到底盘
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
//新版取弹
void TakeBullet(u8 input)	//0为收  1为出
{
	switch(input)
	{
		case 0:	//取弹完成
		{
			Bullet.depth=0;
			Bullet.statu=STEPMOTOR2_EXTEND;
			SteppingMotor2_Set(0,1);	//收回Z轴电机
			
			
			if(StepMotor2.position_val==0)
			{
				Friction_ALL_Set(FRICTION_CLOSE);
				SteppingMotor1_Set(0,1);	//收回X轴
			}
			
			break;
		}
		case 1:	//开始取弹
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
						SteppingMotor1_Set(STEP1_CONTACT_DISTANCE,2);	//伸出X轴
				
						if(StepMotor1.position_val==STEP1_CONTACT_DISTANCE)
						{
							SteppingMotor2_Set(STEP2_LIMIT_DISTANCE+1500*Bullet.depth,2);	//伸出Z轴极限行程
							if(StepMotor2.position_val==STEP2_LIMIT_DISTANCE)
							{
								Bullet.statu=STEPMOTOR1_EXTEND;	//步进电机1开始运转
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
							Bullet.time_record=TimeCount_Common;	//获取当前系统时间
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




//老版取弹，2轴联动
//void TakeBullet(u8 input)	//0为收  1为出
//{
//	switch(input)
//	{
//		case 0:	//取弹完成
//		{
//			Friction_ALL_Set(FRICTION_CLOSE);
//			SteppingMotor2_Set(0,1);	//收回Z轴电机
//			if(StepMotor2.position_val==0)
//			{
//				SteppingMotor1_Set(0,1);	//收回X轴
//			}
//			
//			break;
//		}
//		case 1:	//开始取弹
//		{
//			if(CM1_Position_Feedback.turns+CM2_Position_Feedback.turns+CM3_Position_Feedback.turns+CM4_Position_Feedback.turns-4*BULLET>(-40))
//			{
//				SteppingMotor1_Set((STEP1_DISTANCE-TakBult_STPCM*STEP1_DISTANCE/8),1);	//伸出X轴
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
//					SteppingMotor2_Set((STEP2_DISTANCE-TakBult_STPCM*STEP2_DISTANCE/2),stepmotor2_cycle_send);	//伸出Z轴 后半段2s周期上下移动
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
