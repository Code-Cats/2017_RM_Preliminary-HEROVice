//#include "main.h"

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "can.h"
#include "PID.h"
#include "pwm.h"
#include "control.h"
#include "timer.h"
#include "exti.h"
#include "imu.h"
#include "spi.h"

#include "keyboard.h"

#include "Mydefine.h"

//Robomasters信仰板
//英雄副板
//作者：Derst Rangers控制组

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(180);    //初始化延时函数
	uart_init(115200);	//初始化串口波特率为115200   //标定完成后再打开与主板的通信
	
	LED_Init();					//初始化LED
	
	TIM3_Int_Init(5 -1,9000-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5次为0.5ms
	
	Date_Init();
	
	SPI5_Init();        //板载陀螺仪通信协议初始化
	MPU6500_Init();     //陀螺仪初始化
	IST8310_Init();     //磁力计初始化
	
	SteppMotor_Init();	//步进电机初始化
	
	ALL_PWM_Init();	//	所有PWM通道初始化
	
	CAN1_Init(); //电机CAN初始化
	KEY_Init(); 				//按键初始化 
//	EXTIX_Init();	//外部中断初始化（调试用）
	
	VideoRelay_Init();
	
	PID_Init(&CM1_Robust_Pid,ROBUST_P,ROBUST_D,ROBUST_DEAD,ROBUST_MER);	//暂未用死区参数
	PID_Init(&CM2_Robust_Pid,ROBUST_P,ROBUST_D,ROBUST_DEAD,ROBUST_MER);
	PID_Init(&CM3_Robust_Pid,ROBUST_P,ROBUST_D,ROBUST_DEAD,ROBUST_MER);
	PID_Init(&CM4_Robust_Pid,ROBUST_P,ROBUST_D,ROBUST_DEAD,ROBUST_MER);

	
	delay_ms(30);	//很重要！
	RC_Calibration();	//主板发送数值校准
	Lift_Calibration();	//升降电机标定程序
	SetWorkState(NORMAL_STATE);
	Hardware_Init();	//设置为默认状态
		
while(1)
	{
		key=KEY_Scan(0);
		switch(key)
		{
			case KEY0_PRES://KEY0按下
			{
				key=0;
				break;
			}
		}
		
		IMU_Get_Data(); //板载陀螺仪数据读取
		
	} 
	
}
