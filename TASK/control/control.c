#include "control.h"
#include "pwm.h"
#include "can.h"
#include "PID.h"
#include "Mydefine.h"
#include "imu.h"

s16 CM1_Position_Send=0;
s16 CM2_Position_Send=0;
s16 CM3_Position_Send=0;
s16 CM4_Position_Send=0;
u8 waiting=1;//���ڵǵ����µ��߼����ĵ���һ��ָ��

u8 Calibration_Start=0;
u8 Calibration_Time_Count=0;


RampTypeDef Step1Ramp;	//����
RampTypeDef Step2Ramp;
RampTypeDef CM12_Ramp;	//�������������
RampTypeDef CM34_Ramp;

StepMotorTypeDef StepMotor1;
StepMotorTypeDef StepMotor2;

s32 stepmotor1_position_send=0;	/////////
u8 stepmotor1_cycle_send=1;	//////////
s32 stepmotor2_position_send=0;	///////
u8 stepmotor2_cycle_send=1;	///////

/***************************************************************************/
WorkState_e workState = PREPARE_STATE;

void SetWorkState(WorkState_e state)
{
    workState = state;
}

WorkState_e GetWorkState(void)
{
	return workState;
}
/***************************************************************************/

void Hardware_Init(void)	//��ʼ��Ĭ��״̬
{
	CM1_Position_Send=FALL;
	CM2_Position_Send=FALL;
	CM3_Position_Send=FALL;
	CM4_Position_Send=FALL;
	
	Servo_Left_Set(CLOSE_LEFT);
  Servo_Right_Set(CLOSE_RIGHT);
  Servo_Back_Set(CLOSE_BACK);
	Friction_ALL_Set(FRICTION_CLOSE);
}

void Date_Init(void)	//����������ݳ�ʼ������ֹ���ֵ����
{
	CM12_Ramp.cycle=10;
	CM34_Ramp.cycle=10;
	
	Key1.statu=0;
	Key2.statu=0;
	Key3.statu=0;
	Key4.statu=0;
	Key5.statu=0;
	Key6.statu=0;
	Key7.statu=0;
	KeyQ.statu=0;
	Key1.last=0;
	Key2.last=0;
	Key3.last=0;
	Key4.last=0;
	Key5.last=0;
	Key6.last=0;
	Key7.last=0;
	KeyQ.last=0;
	
	Bullet.circulation_i=0;
	Bullet.sendlast=0;
	Bullet.send[0]=1;
	Bullet.send[1]=2;
	Bullet.send[2]=3;
	Bullet.send_sign=0;
	Bullet.statu=STEPMOTOR2_EXTEND;
	Bullet.depth=0;
	
}


void Lift_Calibration(void)	//�ϵ�궨
{
	u32 Record_Last=0;	//�궨��������һ�νǶȼ�¼ֵ
	
	SetWorkState(CALI_STATE);
	
	CM1_Robust_Pid.max_error=10;
	CM2_Robust_Pid.max_error=11;	//����������ͬ
	CM3_Robust_Pid.max_error=11;
	CM4_Robust_Pid.max_error=13;	//�����Ž�
	
	CM1_Position_Send=30;
	CM2_Position_Send=30;
	CM3_Position_Send=30;
	CM4_Position_Send=30;
	while(CM1_Position_Feedback.turns<20||CM2_Position_Feedback.turns<20||CM3_Position_Feedback.turns<20||CM4_Position_Feedback.turns<20)
	{;}
	
	CM1_Robust_Pid.max_error=5;
	CM2_Robust_Pid.max_error=6;	//����������ͬ
	CM3_Robust_Pid.max_error=6;
	CM4_Robust_Pid.max_error=9;
	
	CM1_Position_Send=-1000;
	CM2_Position_Send=-1000;
	CM3_Position_Send=-1000;
	CM4_Position_Send=-1000;
	
	Calibration_Start=1;	//	��ʼTimer�еı궨��ʱ
	while(CM1_Position_Feedback.turns!=0||CM2_Position_Feedback.turns!=0||CM3_Position_Feedback.turns!=0||CM4_Position_Feedback.turns!=0)
	{
		if(Calibration_Time_Count>200)
		{
			if(MyAbs(Record_Last-CM1_Position_Feedback.calc-CM2_Position_Feedback.calc-CM3_Position_Feedback.calc-CM4_Position_Feedback.calc)<3)	//��һ����һ��λ��ֵ֮��
			{
				CM1_Position_Feedback.turns=0;
				CM2_Position_Feedback.turns=0;
				CM3_Position_Feedback.turns=0;
				CM4_Position_Feedback.turns=0;
				
				CM1_Position_Send=0;
				CM2_Position_Send=0;
				CM3_Position_Send=0;
				CM4_Position_Send=0;
				
				Calibration_Start=0;	//	����Timer�еı궨��ʱ
				Calibration_Time_Count=0;
				
				CM1_Robust_Pid.max_error=150;
				CM2_Robust_Pid.max_error=151;	//����������ͬ
				CM3_Robust_Pid.max_error=150;
				CM4_Robust_Pid.max_error=150;
			}
			
			Record_Last=CM1_Position_Feedback.calc+CM2_Position_Feedback.calc+CM3_Position_Feedback.calc+CM4_Position_Feedback.calc;
			Calibration_Time_Count=0;
		}
	}
}

void TopplingPreventing(void)	//���㵹
{
	if(Imu_Deal.ay_calc>FOREROKE)	//ǰ��
	{
		if((CM3_Robust_Pid.output+CM4_Robust_Pid.output-CM1_Robust_Pid.output-CM2_Robust_Pid.output)>100)
		{
			CM1_Robust_Pid.output=0;
			CM2_Robust_Pid.output=0;
			CM3_Robust_Pid.output=0;
			CM4_Robust_Pid.output=0;
		}
	}
	else if(Imu_Deal.ay_calc<HYPSOKINESIS)	//����
	{
		if((CM3_Robust_Pid.output+CM4_Robust_Pid.output-CM1_Robust_Pid.output-CM2_Robust_Pid.output)<-100)
		{
			CM1_Robust_Pid.output=0;
			CM2_Robust_Pid.output=0;
			CM3_Robust_Pid.output=0;
			CM4_Robust_Pid.output=0;
		}
	}
}

	
void LiftRmap_Detaction(void)	//������������߼��ж�   RampBegin_CM12(u16 cycle) CYCLE_LIFTRAMP
{
	if(MyAbs(CM12_Ramp.last_value-CM1_Robust_Pid.output)>400)		//ǰ���������Ч���ƣ�������ĸ����
	{
		RampBegin_CM12(CYCLE_LIFTRAMP);
	}
	
	if(MyAbs(CM34_Ramp.last_value-CM3_Robust_Pid.output)>400)		//�����������Ч���ƣ�������ĸ����
	{
		RampBegin_CM34(CYCLE_LIFTRAMP);
	}
	CM12_Ramp.last_value=CM1_Robust_Pid.output;	//����
	CM34_Ramp.last_value=CM3_Robust_Pid.output;
}


/*************************************************************************�������************************************************************************/

/**************--�洢��ֵ��ʼ��--******************/
void SteppMotor_Init(void)
{
	
	
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOI|RCC_AHB1Periph_GPIOA, ENABLE);//ʹ�ܲ��������������ʱ��

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;									//�������1��STEP�� 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;									//�������1��DIR�� 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//��©���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOI, &GPIO_InitStructure);//��ʼ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;									//�������2��STEP�� 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;									//�������2��DIR�� 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//��©���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

	
	
	StepMotor1.position_val=0;
	StepMotor1.cycle_set=1;
	StepMotor1.dir_set=0;
	StepMotor1.step_complete=0;
	StepMotor1.step_count=0;
	StepMotor1.step_set=0;
	
	StepMotor2.position_val=0;
	StepMotor2.cycle_set=1;
	StepMotor2.dir_set=0;
	StepMotor2.step_complete=0;
	StepMotor2.step_count=0;
	StepMotor2.step_set=0;
}



/********************--�����������״̬���ú���--**********************/
/*����ʱֻ��ʹ��һ��*/
void SteppingMotor1_Set(s32 position,u8 cycle_min)
{
	StepMotor1.cycle_set=cycle_min;	//�趨�ٶ����ڣ�Խ��Խ������СΪ1
	if(StepMotor1.step_complete==1)	//���������񣬲����������
	{
		if(position>StepMotor1.position_val)
		{ 
			StepMotor1.dir_set=DIR_PLUS;	//�͵�ƽΪ�������
			StepMotor1.step_set=position-StepMotor1.position_val;
			StepMotor1.step_count=0;
			RampBegin_ST1(10);	//��ʼб�� 
		}
		else if(position<StepMotor1.position_val)
		{
			StepMotor1.dir_set=DIR_MINUS;	//�ߵ�ƽΪ���ط���
			StepMotor1.step_set=StepMotor1.position_val-position;
			StepMotor1.step_count=0;
			RampBegin_ST1(10);	//��ʼб�� 
		}
	}
	else if(StepMotor1.step_complete==0)	//�����������ռ��
	{
		if(position>StepMotor1.position_val)	//�����趨λ����ʵ��λ��֮�� ��������� 1.���ϴη�����ͬ 2.���ϴη����෴
		{
			if(StepMotor1.dir_set==DIR_PLUS)	//���ϴη�����ͬ	��������� 1.�����趨λ�����ϴ��趨λ��֮ǰ 2.�����������ϴ�����֮��
			{
			StepMotor1.dir_set=DIR_PLUS;	//�͵�ƽΪ�������
			StepMotor1.step_set=position-StepMotor1.position_val+StepMotor1.step_count;	//�����Ѿ���ȥ��ֵ��������count 
			}	//��Ϊ�����˶����ʲ���б��
			else if(StepMotor1.dir_set==DIR_MINUS)	//���ϴη���ͬ
			{
			StepMotor1.dir_set=DIR_PLUS;	//�͵�ƽΪ�������
			StepMotor1.step_set=position-StepMotor1.position_val;
			StepMotor1.step_count=0;
			RampBegin_ST1(20);	//��ʼб�� ������������Ҫ���� ��Ϊ������ˣ�
			}
		}
		else if(position<StepMotor1.position_val)
		{
			if(StepMotor1.dir_set==DIR_MINUS)	//���ϴη�����ͬ	��������� 1.�����趨λ�����ϴ��趨λ��֮ǰ 2.�����������ϴ�����֮��
			{
			StepMotor1.dir_set=DIR_MINUS;	//�ߵ�ƽΪ���ط���
			StepMotor1.step_set=StepMotor1.position_val-position+StepMotor1.step_count;	//�����Ѿ���ȥ��ֵ��������count 
			}	//��Ϊ�����˶����ʲ���б��
			else if(StepMotor1.dir_set==DIR_PLUS)	//���ϴη���ͬ
			{
			StepMotor1.dir_set=DIR_MINUS;	//�ߵ�ƽΪ���ط���
			StepMotor1.step_set=StepMotor1.position_val-position;
			StepMotor1.step_count=0;
			RampBegin_ST1(20);	//��ʼб�� ������������Ҫ���� ��Ϊ������ˣ�
			}
		}
		else
		{}
	}
	else	//�������ᷢ�������Ĵ���
	{}
}


void SteppingMotor2_Set(s32 position,u8 cycle_min)	//�������2�����ú���
{
	StepMotor2.cycle_set=cycle_min;	//�趨�ٶ����ڣ�Խ��Խ������СΪ1
	if(StepMotor2.step_complete==1)	//���������񣬲����������
	{
		if(position>StepMotor2.position_val)
		{
			StepMotor2.dir_set=DIR_PLUS;	//�͵�ƽΪ�������
			StepMotor2.step_set=position-StepMotor2.position_val;
			StepMotor2.step_count=0;
			RampBegin_ST2(10);	//��ʼб�� 
		}
		else if(position<StepMotor2.position_val)
		{
			StepMotor2.dir_set=DIR_MINUS;	//�ߵ�ƽΪ���ط���
			StepMotor2.step_set=StepMotor2.position_val-position;
			StepMotor2.step_count=0;
			RampBegin_ST2(10);	//��ʼб�� 
		}
	}
	else if(StepMotor2.step_complete==0)	//�����������ռ��
	{
		if(position>StepMotor2.position_val)	//�����趨λ����ʵ��λ��֮�� ��������� 1.���ϴη�����ͬ 2.���ϴη����෴
		{
			if(StepMotor2.dir_set==DIR_PLUS)	//���ϴη�����ͬ	��������� 1.�����趨λ�����ϴ��趨λ��֮ǰ 2.�����������ϴ�����֮��
			{
			StepMotor2.dir_set=DIR_PLUS;	//�͵�ƽΪ�������
			StepMotor2.step_set=position-StepMotor2.position_val+StepMotor2.step_count;	//�����Ѿ���ȥ��ֵ��������count 
			}	//��Ϊ�����˶����ʲ���б��
			else if(StepMotor2.dir_set==DIR_MINUS)	//���ϴη���ͬ
			{
			StepMotor2.dir_set=DIR_PLUS;	//�͵�ƽΪ�������
			StepMotor2.step_set=position-StepMotor2.position_val;
			StepMotor2.step_count=0;
			RampBegin_ST2(20);	//��ʼб�� ������������Ҫ���� ��Ϊ������ˣ�
			}
		}
		else if(position<StepMotor2.position_val)
		{
			if(StepMotor2.dir_set==DIR_MINUS)	//���ϴη�����ͬ	��������� 1.�����趨λ�����ϴ��趨λ��֮ǰ 2.�����������ϴ�����֮��
			{
			StepMotor2.dir_set=DIR_MINUS;	//�ߵ�ƽΪ���ط���
			StepMotor2.step_set=StepMotor2.position_val-position+StepMotor2.step_count;	//�����Ѿ���ȥ��ֵ��������count 
			}	//��Ϊ�����˶����ʲ���б��
			else if(StepMotor2.dir_set==DIR_PLUS)	//���ϴη���ͬ
			{
			StepMotor2.dir_set=DIR_MINUS;	//�ߵ�ƽΪ���ط���
			StepMotor2.step_set=StepMotor2.position_val-position;
			StepMotor2.step_count=0;
			RampBegin_ST2(20);	//��ʼб�� ������������Ҫ���� ��Ϊ������ˣ�
			}
		}
		else
		{}
	}
	else	//�������ᷢ�������Ĵ���
	{}
}

/**********************************************************************/





/********************���º���********************/

void RampBegin_ST1(u16 cycle)	//���¿�ʼ������״̬���ú�����
{
	Step1Ramp.count=0;
	Step1Ramp.cycle=cycle;	//���㹫ʽ��cycle*100*��ʱ������
}

void RampCalc_ST1()	//�������к���
{
	if((TimeCount_Common%Step1Ramp.cycle==0)&&(Step1Ramp.count<100))
	{
		Step1Ramp.count++;
		Step1Ramp.calc=Step1Ramp.count/100.0;
	}
}


void RampBegin_ST2(u16 cycle)
{
	Step2Ramp.count=0;
	Step2Ramp.cycle=cycle;
}

void RampCalc_ST2()
{
	if((TimeCount_Common%Step2Ramp.cycle==0)&&(Step2Ramp.count<100))
	{
		Step2Ramp.count++;
		Step2Ramp.calc=Step2Ramp.count/100.0;
	}
}


void RampBegin_CM12(u16 cycle)
{
	CM12_Ramp.count=0;
	CM12_Ramp.cycle=cycle;
}

void RampCalc_CM12()
{ 
	if((TimeCount_Common%CM12_Ramp.cycle==0)&&(CM12_Ramp.count<100))
	{
		CM12_Ramp.count++;
		CM12_Ramp.calc=CM12_Ramp.count/100.0;
	}
}


void RampBegin_CM34(u16 cycle)
{
	CM34_Ramp.count=0;
	CM34_Ramp.cycle=cycle;
}

void RampCalc_CM34()
{
	if((TimeCount_Common%CM34_Ramp.cycle==0)&&(CM34_Ramp.count<100))
	{
		CM34_Ramp.count++;
		CM34_Ramp.calc=CM34_Ramp.count/100.0;
	}
}
//////////////////////////////////////////////////


//u32 step_count=0;	//������������������¼��ǰ��������
//u8 dir_set=0;	//��������
//u32 step_set=0;
//u8 cycle_set=1;
//u8 step_complete=0;//��־һ������������




/****************---�ɱ��ٶȱ����λ�ò�����������������߼����֣�---*********************/
void SteppingMotor1_Run(u8 cycle_min,u32 step)	//����ֵ�벻��ѭ��
{
	if((TimeCount_Common%(int)(cycle_min+STEPMOTOR_STARTSPEED*(1-Step1Ramp.calc)))==0)
	{
		if(StepMotor1.step_count<step)
		{
			StepMotor1.step_complete=0;
			STEP1=!STEP1;	//PF10
			StepMotor1.step_count++;
			StepMotor1.position_val+=1-2*(StepMotor1.dir_set==DIR_MINUS);	//��ʵֵ�ۼ�
		}
		else
		{
			StepMotor1.step_complete=1;
		}
	}
	DIR1=StepMotor1.dir_set;	//PI9 ���ƻ���
	
}


void SteppingMotor2_Run(u8 cycle_min,u32 step)	//����ֵ�벻��ѭ��
{
	if((TimeCount_Common%(int)(cycle_min+STEPMOTOR_STARTSPEED*(1-Step2Ramp.calc)))==0)
	{
		if(StepMotor2.step_count<step)
		{
			StepMotor2.step_complete=0;
			STEP2=!STEP2;	//PF10
			StepMotor2.step_count++;
			StepMotor2.position_val+=1-2*(StepMotor2.dir_set==DIR_MINUS);	//��ʵֵ�ۼ�
		}
		else
		{
			StepMotor2.step_complete=1;
		}
	}
	DIR2=StepMotor2.dir_set;	//PF10 ���ƻ���
	
}
/********************************************************************************************************************************************************/







void Servo_ALL(u8 statu3)	//���	ֵΪ1ʱ����0ʱ��			//ֻ�������ҵ��ֶ��
{
	Servo_Left_Set(CLOSE_LEFT-statu3*(CLOSE_LEFT-OPEN_LEFT));
  Servo_Right_Set(CLOSE_RIGHT-statu3*(CLOSE_RIGHT-OPEN_RIGHT));
//  Servo_Back_Set(CLOSE_BACK-statu3*(CLOSE_BACK-OPEN_BACK));	//����ͬ���������µ�ì��
}


void Servo_Video(u8 statu1)	//ͼ�����  ֵΪ1ʱ������󣩣�0ʱ��
{
	Servo_Video_Set(VIDEO_UP-statu1*(VIDEO_UP-VIDEO_BACK));
}




void Lift_Anterior(u8 statu1)	//ǰ�������ֿ���	//ֵΪ1ʱ����ֵΪ0ʱ��
{
	CM1_Position_Send=FALL-statu1*(FALL-ISLAND); 
	CM2_Position_Send=FALL-statu1*(FALL-ISLAND);
}


void Lift_Posterior(u8 statu2)	//�������ֿ���
{
	CM3_Position_Send=FALL-statu2*(FALL-ISLAND);
	CM4_Position_Send=FALL-statu2*(FALL-ISLAND);
}


void Lift_Bullet(u8 statu3)	//
{
	CM1_Position_Send=FALL-statu3*(FALL-BULLET); 
	CM2_Position_Send=FALL-statu3*(FALL-BULLET);
	CM3_Position_Send=FALL-statu3*(FALL-BULLET); 
	CM4_Position_Send=FALL-statu3*(FALL-BULLET);
}



void Ascend_Island(void)	//�ǵ��߼�����
{
	CM1_Position_Send=ISLAND;
	CM2_Position_Send=ISLAND;
	CM3_Position_Send=ISLAND;
	CM4_Position_Send=ISLAND;

	while(CM1_Position_Feedback.turns<690||CM2_Position_Feedback.turns<690||CM3_Position_Feedback.turns<690||CM4_Position_Feedback.turns<690)
	{;
	}	//�ȵ��������������

	Servo_Left_Set(OPEN_LEFT);
  Servo_Right_Set(OPEN_RIGHT);
  Servo_Back_Set(OPEN_BACK);
	
	waiting=1;
	while(waiting!=0);//�˴�����һ�ȴ�ǰ��ָ���ѽ����ı�־������һ��ָ��ͺ�ִ����������
	
	CM3_Position_Send=FALL;
	CM4_Position_Send=FALL;
	while(CM3_Position_Feedback.turns>5||CM4_Position_Feedback.turns>5);	//�ȴ�ǰ���������
	waiting=1;
	while(waiting!=0);//�˴�����һ�ȴ�ǰ��ָ���ѽ����ı�־������һ��ָ��ͺ�ִ����������
	waiting=1;
	
	CM1_Position_Send=FALL;
	CM2_Position_Send=FALL;
	while(CM1_Position_Feedback.turns>5||CM2_Position_Feedback.turns>5);	//�ȴ������������
	
	Servo_Back_Set(CLOSE_BACK);	//����ǰ����
}


void Descend_Island(void)		//�µ��߼�����
{
	CM1_Position_Send=FALL; 
	CM2_Position_Send=FALL;
	CM3_Position_Send=FALL;
	CM4_Position_Send=FALL;
	
	Servo_Back_Set(OPEN_BACK);	
	
	while(CM1_Position_Feedback.turns>10||CM2_Position_Feedback.turns>10||CM3_Position_Feedback.turns>10||CM4_Position_Feedback.turns>10)
	{;}
	waiting=1;
	while(waiting!=0);//�˴�����һ�ȴ�ǰ��ָ���ѽ����ı�־������һ��ָ��ͺ�ִ����������

	CM3_Position_Send=ISLAND;
	CM4_Position_Send=ISLAND;
	while(CM3_Position_Feedback.turns<690||CM4_Position_Feedback.turns<690);
	waiting=1;
	while(waiting!=0);//�˴�����һ�ȴ�ǰ��ָ���ѽ����ı�־������һ��ָ��ͺ�ִ����������
	
	CM1_Position_Send=ISLAND;
	CM2_Position_Send=ISLAND;
	while(CM1_Position_Feedback.turns<690||CM2_Position_Feedback.turns<690);
	waiting=1;
	while(waiting!=0);//�˴�����һ�ȴ�ǰ��ָ���ѽ����ı�־������һ��ָ��ͺ�ִ����������
	
	CM1_Position_Send=FALL;
	CM2_Position_Send=FALL;
	CM3_Position_Send=FALL;
	CM4_Position_Send=FALL;

	Servo_Left_Set(CLOSE_LEFT);
  Servo_Right_Set(CLOSE_RIGHT);
  Servo_Back_Set(CLOSE_BACK);
}


void TakeBullets(void)	//ȡ���߼����ģ���ʱ��д
{
	CM1_Position_Send=ISLAND;
	CM2_Position_Send=ISLAND;
	CM3_Position_Send=ISLAND;
	CM4_Position_Send=ISLAND;
	while(CM1_Position_Feedback.turns<695||CM2_Position_Feedback.turns<695||CM3_Position_Feedback.turns<695||CM4_Position_Feedback.turns<695);
	//�˴�����һ�ȴ�ǰ��ָ���ѽ����ı�־������һ��ָ��ͺ�ִ����������
	//�˴����ò���������ȡ������
}
