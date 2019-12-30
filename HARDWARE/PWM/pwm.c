#include "pwm.h"
#include "led.h"
#include "usart.h"
#include "Mydefine.h"
 
//////////////////////////////////////////////////////////////////////////////////	 
//舵机PWM驱动代码
//摩擦轮PWM 驱动代码	   
//创建日期:2017/4/22  
////////////////////////////////////////////////////////////////////////////////// 	 

void ALL_PWM_Init(void)	//对七个PWM进行初始化
{
	TIM4_PWM_Init(SERVO_CYCLE-1,90-1);	//84M/84=1Mhz的计数频率,重装载值20000，所以PWM频率为 1M/20000=50hz.  即20ms周期
	TIM5_PWM_Init(ESC_CYCLE-1,90-1);	//84M/84=1Mhz的计数频率,重装载值14000，即14ms周期
}

void Servo_Left_Set(u32 set1)
{
	if(GetWorkState()==NORMAL_STATE)
	{
	TIM_SetCompare1(TIM4,SERVO_CYCLE-set1);
	}
	else
	{TIM_SetCompare1(TIM4,SERVO_CYCLE-CLOSE_LEFT);}
}

void Servo_Right_Set(u32 set2)
{
	if(GetWorkState()==NORMAL_STATE)
	{
	TIM_SetCompare2(TIM4,SERVO_CYCLE-set2);
	}
	else
	{TIM_SetCompare2(TIM4,SERVO_CYCLE-CLOSE_RIGHT);}
}

void Servo_Back_Set(u32 set3)
{
//	if(GetWorkState()==NORMAL_STATE)
//	{
	TIM_SetCompare3(TIM4,SERVO_CYCLE-set3);
//	}
//	else
//	{TIM_SetCompare3(TIM4,SERVO_CYCLE-CLOSE_BACK);}
}
  
void Servo_Video_Set(u32 set5)
{
	if(GetWorkState()==NORMAL_STATE)
	{
	TIM_SetCompare4(TIM4,SERVO_CYCLE-set5);
	}
	else
	{TIM_SetCompare4(TIM4,SERVO_CYCLE-VIDEO_UP);}
}

void Friction_ALL_Set(u32 set4)
{
	if(GetWorkState()==NORMAL_STATE)
	{
	TIM_SetCompare1(TIM5,ESC_CYCLE-set4);
	TIM_SetCompare2(TIM5,ESC_CYCLE-set4);
	TIM_SetCompare3(TIM5,ESC_CYCLE-set4);
	}
	else
	{TIM_SetCompare1(TIM5,ESC_CYCLE-FRICTION_CLOSE);
	TIM_SetCompare2(TIM5,ESC_CYCLE-FRICTION_CLOSE);
	TIM_SetCompare3(TIM5,ESC_CYCLE-FRICTION_CLOSE);}
}







//TIM4 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM4_PWM_Init(u32 arr,u32 psc)	//舵机信号
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;           //GPIOD
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化定时器4PWM：PD12-15
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM4 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC2
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC3
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
	
	Servo_Left_Set(CLOSE_LEFT);
  Servo_Right_Set(CLOSE_RIGHT);
  Servo_Back_Set(CLOSE_BACK);
	Servo_Video_Set(VIDEO_UP); 
}  


void TIM5_PWM_Init(u32 arr,u32 psc)	//GPIOH10-12
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOH,&GPIO_InitStructure);              //初始化定时器4PWM：PD12-15
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM5 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR1上的预装载寄存器
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM5, ENABLE);  //使能TIM5
	
	TIM_SetCompare1(TIM5,ESC_CYCLE-FRICTION_CLOSE);
	TIM_SetCompare2(TIM5,ESC_CYCLE-FRICTION_CLOSE);
	TIM_SetCompare3(TIM5,ESC_CYCLE-FRICTION_CLOSE);
 
}
