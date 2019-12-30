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
u8 waiting=1;//用于登岛、下岛逻辑中心的下一步指令

u8 Calibration_Start=0;
u8 Calibration_Time_Count=0;


RampTypeDef Step1Ramp;	//爬坡
RampTypeDef Step2Ramp;
RampTypeDef CM12_Ramp;	//升降电机缓启动
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

void Hardware_Init(void)	//初始化默认状态
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

void Date_Init(void)	//所有相关数据初始化，防止随机值作乱
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


void Lift_Calibration(void)	//上电标定
{
	u32 Record_Last=0;	//标定程序中上一次角度纪录值
	
	SetWorkState(CALI_STATE);
	
	CM1_Robust_Pid.max_error=10;
	CM2_Robust_Pid.max_error=11;	//该轮阻力不同
	CM3_Robust_Pid.max_error=11;
	CM4_Robust_Pid.max_error=13;	//该轮张紧
	
	CM1_Position_Send=30;
	CM2_Position_Send=30;
	CM3_Position_Send=30;
	CM4_Position_Send=30;
	while(CM1_Position_Feedback.turns<20||CM2_Position_Feedback.turns<20||CM3_Position_Feedback.turns<20||CM4_Position_Feedback.turns<20)
	{;}
	
	CM1_Robust_Pid.max_error=5;
	CM2_Robust_Pid.max_error=6;	//该轮阻力不同
	CM3_Robust_Pid.max_error=6;
	CM4_Robust_Pid.max_error=9;
	
	CM1_Position_Send=-1000;
	CM2_Position_Send=-1000;
	CM3_Position_Send=-1000;
	CM4_Position_Send=-1000;
	
	Calibration_Start=1;	//	开始Timer中的标定计时
	while(CM1_Position_Feedback.turns!=0||CM2_Position_Feedback.turns!=0||CM3_Position_Feedback.turns!=0||CM4_Position_Feedback.turns!=0)
	{
		if(Calibration_Time_Count>200)
		{
			if(MyAbs(Record_Last-CM1_Position_Feedback.calc-CM2_Position_Feedback.calc-CM3_Position_Feedback.calc-CM4_Position_Feedback.calc)<3)	//上一次这一次位置值之差
			{
				CM1_Position_Feedback.turns=0;
				CM2_Position_Feedback.turns=0;
				CM3_Position_Feedback.turns=0;
				CM4_Position_Feedback.turns=0;
				
				CM1_Position_Send=0;
				CM2_Position_Send=0;
				CM3_Position_Send=0;
				CM4_Position_Send=0;
				
				Calibration_Start=0;	//	结束Timer中的标定计时
				Calibration_Time_Count=0;
				
				CM1_Robust_Pid.max_error=150;
				CM2_Robust_Pid.max_error=151;	//该轮阻力不同
				CM3_Robust_Pid.max_error=150;
				CM4_Robust_Pid.max_error=150;
			}
			
			Record_Last=CM1_Position_Feedback.calc+CM2_Position_Feedback.calc+CM3_Position_Feedback.calc+CM4_Position_Feedback.calc;
			Calibration_Time_Count=0;
		}
	}
}

void TopplingPreventing(void)	//防倾倒
{
	if(Imu_Deal.ay_calc>FOREROKE)	//前倾
	{
		if((CM3_Robust_Pid.output+CM4_Robust_Pid.output-CM1_Robust_Pid.output-CM2_Robust_Pid.output)>100)
		{
			CM1_Robust_Pid.output=0;
			CM2_Robust_Pid.output=0;
			CM3_Robust_Pid.output=0;
			CM4_Robust_Pid.output=0;
		}
	}
	else if(Imu_Deal.ay_calc<HYPSOKINESIS)	//后倾
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

	
void LiftRmap_Detaction(void)	//升降电机爬坡逻辑判断   RampBegin_CM12(u16 cycle) CYCLE_LIFTRAMP
{
	if(MyAbs(CM12_Ramp.last_value-CM1_Robust_Pid.output)>400)		//前两个电机等效控制，随便用哪个检测
	{
		RampBegin_CM12(CYCLE_LIFTRAMP);
	}
	
	if(MyAbs(CM34_Ramp.last_value-CM3_Robust_Pid.output)>400)		//后两个电机等效控制，随便用哪个检测
	{
		RampBegin_CM34(CYCLE_LIFTRAMP);
	}
	CM12_Ramp.last_value=CM1_Robust_Pid.output;	//迭代
	CM34_Ramp.last_value=CM3_Robust_Pid.output;
}


/*************************************************************************步进电机************************************************************************/

/**************--存储数值初始化--******************/
void SteppMotor_Init(void)
{
	
	
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOI|RCC_AHB1Periph_GPIOA, ENABLE);//使能步进电机驱动引脚时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;									//步进电机1的STEP端 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;									//步进电机1的DIR端 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//开漏输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOI, &GPIO_InitStructure);//初始化
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;									//步进电机2的STEP端 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;									//步进电机2的DIR端 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//开漏输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	
	
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



/********************--步进电机驱动状态设置函数--**********************/
/*设置时只需使用一次*/
void SteppingMotor1_Set(s32 position,u8 cycle_min)
{
	StepMotor1.cycle_set=cycle_min;	//设定速度周期，越大越慢，最小为1
	if(StepMotor1.step_complete==1)	//现在无任务，步进电机空闲
	{
		if(position>StepMotor1.position_val)
		{ 
			StepMotor1.dir_set=DIR_PLUS;	//低电平为伸出方向
			StepMotor1.step_set=position-StepMotor1.position_val;
			StepMotor1.step_count=0;
			RampBegin_ST1(10);	//开始斜坡 
		}
		else if(position<StepMotor1.position_val)
		{
			StepMotor1.dir_set=DIR_MINUS;	//高电平为缩回方向
			StepMotor1.step_set=StepMotor1.position_val-position;
			StepMotor1.step_count=0;
			RampBegin_ST1(10);	//开始斜坡 
		}
	}
	else if(StepMotor1.step_complete==0)	//步进电机正在占用
	{
		if(position>StepMotor1.position_val)	//本次设定位置在实际位置之后 有两种情况 1.与上次方向相同 2.与上次方向相反
		{
			if(StepMotor1.dir_set==DIR_PLUS)	//与上次方向相同	有两种情况 1.本次设定位置在上次设定位置之前 2.本次设置在上次设置之后
			{
			StepMotor1.dir_set=DIR_PLUS;	//低电平为伸出方向
			StepMotor1.step_set=position-StepMotor1.position_val+StepMotor1.step_count;	//加上已经记去的值，不清零count 
			}	//因为还在运动，故不加斜坡
			else if(StepMotor1.dir_set==DIR_MINUS)	//与上次方向不同
			{
			StepMotor1.dir_set=DIR_PLUS;	//低电平为伸出方向
			StepMotor1.step_set=position-StepMotor1.position_val;
			StepMotor1.step_count=0;
			RampBegin_ST1(20);	//开始斜坡 （比正常启动要更慢 因为方向变了）
			}
		}
		else if(position<StepMotor1.position_val)
		{
			if(StepMotor1.dir_set==DIR_MINUS)	//与上次方向相同	有两种情况 1.本次设定位置在上次设定位置之前 2.本次设置在上次设置之后
			{
			StepMotor1.dir_set=DIR_MINUS;	//高电平为缩回方向
			StepMotor1.step_set=StepMotor1.position_val-position+StepMotor1.step_count;	//加上已经记去的值，不清零count 
			}	//因为还在运动，故不加斜坡
			else if(StepMotor1.dir_set==DIR_PLUS)	//与上次方向不同
			{
			StepMotor1.dir_set=DIR_MINUS;	//高电平为缩回方向
			StepMotor1.step_set=StepMotor1.position_val-position;
			StepMotor1.step_count=0;
			RampBegin_ST1(20);	//开始斜坡 （比正常启动要更慢 因为方向变了）
			}
		}
		else
		{}
	}
	else	//讲道理不会发生这样的错误
	{}
}


void SteppingMotor2_Set(s32 position,u8 cycle_min)	//步进电机2号设置函数
{
	StepMotor2.cycle_set=cycle_min;	//设定速度周期，越大越慢，最小为1
	if(StepMotor2.step_complete==1)	//现在无任务，步进电机空闲
	{
		if(position>StepMotor2.position_val)
		{
			StepMotor2.dir_set=DIR_PLUS;	//低电平为伸出方向
			StepMotor2.step_set=position-StepMotor2.position_val;
			StepMotor2.step_count=0;
			RampBegin_ST2(10);	//开始斜坡 
		}
		else if(position<StepMotor2.position_val)
		{
			StepMotor2.dir_set=DIR_MINUS;	//高电平为缩回方向
			StepMotor2.step_set=StepMotor2.position_val-position;
			StepMotor2.step_count=0;
			RampBegin_ST2(10);	//开始斜坡 
		}
	}
	else if(StepMotor2.step_complete==0)	//步进电机正在占用
	{
		if(position>StepMotor2.position_val)	//本次设定位置在实际位置之后 有两种情况 1.与上次方向相同 2.与上次方向相反
		{
			if(StepMotor2.dir_set==DIR_PLUS)	//与上次方向相同	有两种情况 1.本次设定位置在上次设定位置之前 2.本次设置在上次设置之后
			{
			StepMotor2.dir_set=DIR_PLUS;	//低电平为伸出方向
			StepMotor2.step_set=position-StepMotor2.position_val+StepMotor2.step_count;	//加上已经记去的值，不清零count 
			}	//因为还在运动，故不加斜坡
			else if(StepMotor2.dir_set==DIR_MINUS)	//与上次方向不同
			{
			StepMotor2.dir_set=DIR_PLUS;	//低电平为伸出方向
			StepMotor2.step_set=position-StepMotor2.position_val;
			StepMotor2.step_count=0;
			RampBegin_ST2(20);	//开始斜坡 （比正常启动要更慢 因为方向变了）
			}
		}
		else if(position<StepMotor2.position_val)
		{
			if(StepMotor2.dir_set==DIR_MINUS)	//与上次方向相同	有两种情况 1.本次设定位置在上次设定位置之前 2.本次设置在上次设置之后
			{
			StepMotor2.dir_set=DIR_MINUS;	//高电平为缩回方向
			StepMotor2.step_set=StepMotor2.position_val-position+StepMotor2.step_count;	//加上已经记去的值，不清零count 
			}	//因为还在运动，故不加斜坡
			else if(StepMotor2.dir_set==DIR_PLUS)	//与上次方向不同
			{
			StepMotor2.dir_set=DIR_MINUS;	//高电平为缩回方向
			StepMotor2.step_set=StepMotor2.position_val-position;
			StepMotor2.step_count=0;
			RampBegin_ST2(20);	//开始斜坡 （比正常启动要更慢 因为方向变了）
			}
		}
		else
		{}
	}
	else	//讲道理不会发生这样的错误
	{}
}

/**********************************************************************/





/********************爬坡函数********************/

void RampBegin_ST1(u16 cycle)	//爬坡开始函数（状态设置函数）
{
	Step1Ramp.count=0;
	Step1Ramp.cycle=cycle;	//计算公式：cycle*100*定时器周期
}

void RampCalc_ST1()	//爬坡运行函数
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


//u32 step_count=0;	//步进电机脉冲计数（记录当前脉冲数）
//u8 dir_set=0;	//方向设置
//u32 step_set=0;
//u8 cycle_set=1;
//u8 step_complete=0;//标志一次脉冲输出完成




/****************---可变速度变相对位置步进电机驱动（不含逻辑部分）---*********************/
void SteppingMotor1_Run(u8 cycle_min,u32 step)	//传输值与不断循环
{
	if((TimeCount_Common%(int)(cycle_min+STEPMOTOR_STARTSPEED*(1-Step1Ramp.calc)))==0)
	{
		if(StepMotor1.step_count<step)
		{
			StepMotor1.step_complete=0;
			STEP1=!STEP1;	//PF10
			StepMotor1.step_count++;
			StepMotor1.position_val+=1-2*(StepMotor1.dir_set==DIR_MINUS);	//真实值累加
		}
		else
		{
			StepMotor1.step_complete=1;
		}
	}
	DIR1=StepMotor1.dir_set;	//PI9 控制换向
	
}


void SteppingMotor2_Run(u8 cycle_min,u32 step)	//传输值与不断循环
{
	if((TimeCount_Common%(int)(cycle_min+STEPMOTOR_STARTSPEED*(1-Step2Ramp.calc)))==0)
	{
		if(StepMotor2.step_count<step)
		{
			StepMotor2.step_complete=0;
			STEP2=!STEP2;	//PF10
			StepMotor2.step_count++;
			StepMotor2.position_val+=1-2*(StepMotor2.dir_set==DIR_MINUS);	//真实值累加
		}
		else
		{
			StepMotor2.step_complete=1;
		}
	}
	DIR2=StepMotor2.dir_set;	//PF10 控制换向
	
}
/********************************************************************************************************************************************************/







void Servo_ALL(u8 statu3)	//舵机	值为1时开，0时关			//只控制左右导轮舵机
{
	Servo_Left_Set(CLOSE_LEFT-statu3*(CLOSE_LEFT-OPEN_LEFT));
  Servo_Right_Set(CLOSE_RIGHT-statu3*(CLOSE_RIGHT-OPEN_RIGHT));
//  Servo_Back_Set(CLOSE_BACK-statu3*(CLOSE_BACK-OPEN_BACK));	//不能同步操作，下岛矛盾
}


void Servo_Video(u8 statu1)	//图传舵机  值为1时开（向后），0时关
{
	Servo_Video_Set(VIDEO_UP-statu1*(VIDEO_UP-VIDEO_BACK));
}




void Lift_Anterior(u8 statu1)	//前部升降轮控制	//值为1时升，值为0时降
{
	CM1_Position_Send=FALL-statu1*(FALL-ISLAND); 
	CM2_Position_Send=FALL-statu1*(FALL-ISLAND);
}


void Lift_Posterior(u8 statu2)	//后部升降轮控制
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



void Ascend_Island(void)	//登岛逻辑中心
{
	CM1_Position_Send=ISLAND;
	CM2_Position_Send=ISLAND;
	CM3_Position_Send=ISLAND;
	CM4_Position_Send=ISLAND;

	while(CM1_Position_Feedback.turns<690||CM2_Position_Feedback.turns<690||CM3_Position_Feedback.turns<690||CM4_Position_Feedback.turns<690)
	{;
	}	//等到所有轮上升完成

	Servo_Left_Set(OPEN_LEFT);
  Servo_Right_Set(OPEN_RIGHT);
  Servo_Back_Set(OPEN_BACK);
	
	waiting=1;
	while(waiting!=0);//此处设置一等待前进指令已结束的标志，当下一步指令发送后执行下面的语句
	
	CM3_Position_Send=FALL;
	CM4_Position_Send=FALL;
	while(CM3_Position_Feedback.turns>5||CM4_Position_Feedback.turns>5);	//等待前轮下落完成
	waiting=1;
	while(waiting!=0);//此处设置一等待前进指令已结束的标志，当下一步指令发送后执行下面的语句
	waiting=1;
	
	CM1_Position_Send=FALL;
	CM2_Position_Send=FALL;
	while(CM1_Position_Feedback.turns>5||CM2_Position_Feedback.turns>5);	//等待后轮下落完成
	
	Servo_Back_Set(CLOSE_BACK);	//收起前导轮
}


void Descend_Island(void)		//下岛逻辑中心
{
	CM1_Position_Send=FALL; 
	CM2_Position_Send=FALL;
	CM3_Position_Send=FALL;
	CM4_Position_Send=FALL;
	
	Servo_Back_Set(OPEN_BACK);	
	
	while(CM1_Position_Feedback.turns>10||CM2_Position_Feedback.turns>10||CM3_Position_Feedback.turns>10||CM4_Position_Feedback.turns>10)
	{;}
	waiting=1;
	while(waiting!=0);//此处设置一等待前进指令已结束的标志，当下一步指令发送后执行下面的语句

	CM3_Position_Send=ISLAND;
	CM4_Position_Send=ISLAND;
	while(CM3_Position_Feedback.turns<690||CM4_Position_Feedback.turns<690);
	waiting=1;
	while(waiting!=0);//此处设置一等待前进指令已结束的标志，当下一步指令发送后执行下面的语句
	
	CM1_Position_Send=ISLAND;
	CM2_Position_Send=ISLAND;
	while(CM1_Position_Feedback.turns<690||CM2_Position_Feedback.turns<690);
	waiting=1;
	while(waiting!=0);//此处设置一等待前进指令已结束的标志，当下一步指令发送后执行下面的语句
	
	CM1_Position_Send=FALL;
	CM2_Position_Send=FALL;
	CM3_Position_Send=FALL;
	CM4_Position_Send=FALL;

	Servo_Left_Set(CLOSE_LEFT);
  Servo_Right_Set(CLOSE_RIGHT);
  Servo_Back_Set(CLOSE_BACK);
}


void TakeBullets(void)	//取弹逻辑中心，暂时不写
{
	CM1_Position_Send=ISLAND;
	CM2_Position_Send=ISLAND;
	CM3_Position_Send=ISLAND;
	CM4_Position_Send=ISLAND;
	while(CM1_Position_Feedback.turns<695||CM2_Position_Feedback.turns<695||CM3_Position_Feedback.turns<695||CM4_Position_Feedback.turns<695);
	//此处设置一等待前进指令已结束的标志，当下一步指令发送后执行下面的语句
	//此处设置步进电机伸出取弹函数
}
