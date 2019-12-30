#include "sys.h"
#include "usart.h"	

#include "control.h"
#include "led.h"
#include "Mydefine.h"
#include "keyboard.h"

#include "pwm.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//////////////////////////////////////////////////////////////////////////////////	 
//串口6初始化	（主板通信模块）	   
//修改日期:2017/4/23
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

//串口6中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
	

//初始化IO 串口6
//bound:波特率
void uart_init(u32 bound)
{
//		GPIO_InitTypeDef GPIO_InitStruct;
//		NVIC_InitTypeDef NVIC_InitStructure;
//		USART_InitTypeDef USART6_InitStruct;
///* -------------- Enable Module Clock Source ----------------------------*/
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
//	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9|GPIO_PinSource14, GPIO_AF_USART6);
//	
///* -------------- Configure GPIO ---------------------------------------*/

//		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_14;
//		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOG, &GPIO_InitStruct);
//		
//		USART_DeInit(USART6);
//		USART6_InitStruct.USART_BaudRate = 115200;
//		USART6_InitStruct.USART_WordLength = USART_WordLength_8b;
//		USART6_InitStruct.USART_StopBits = USART_StopBits_1;
//		USART6_InitStruct.USART_Parity = USART_Parity_No; //偶校验
//		USART6_InitStruct.USART_Mode = USART_Mode_Rx;
//		USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//		USART_Init(USART6,&USART6_InitStruct);
//		USART_Cmd(USART6,ENABLE);
//	
//	//USART_ClearFlag(USART1, USART_FLAG_TC);
//		


//	//Usart1 NVIC 配置
//  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
//	
//	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断
	
	
	
	
			GPIO_InitTypeDef GPIO_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_InitTypeDef USART6_InitStruct;
/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14, GPIO_AF_USART6);
	
/* -------------- Configure GPIO ---------------------------------------*/

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_14;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOG, &GPIO_InitStruct);
		
//		USART_DeInit(USART6);
		USART6_InitStruct.USART_BaudRate = 115200;
		USART6_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART6_InitStruct.USART_StopBits = USART_StopBits_1;
		USART6_InitStruct.USART_Parity = USART_Parity_No; //偶校验
		USART6_InitStruct.USART_Mode =  USART_Mode_Tx|USART_Mode_Rx;
		USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART6,&USART6_InitStruct);
		USART_Cmd(USART6,ENABLE);
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
		
	

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断
}

u8 Res=0;

void USART6_IRQHandler(void)                	//串口6中断服务程序
{

	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{ 
		Res =USART_ReceiveData(USART6);//(USART6->DR);	//读取接收到的数据   //副板数  "X"BVCGFRE
		
		if(GetWorkState()==NORMAL_STATE)
		{
			Res_Deal(KeyValue,Res);
		
			KeyQ_Deal(KeyValue[7]);	//登岛模式开启
		
			if(KeyValue[7]==1)	//登岛模式和平常模式控制任务切换
			{
				if(Key7.statu==0)	//保持视觉前方始终和键盘前方对应
				{
					KeyG_Deal(KeyValue[3]);
		
					KeyF_Deal(KeyValue[2]);
		
					KeyB_Deal(KeyValue[6]);
		
					KeyV_Deal(KeyValue[5]);
				}
				else	//升降轮
				{
					KeyG_Deal(KeyValue[6]);
		
					KeyF_Deal(KeyValue[5]);
			
					KeyB_Deal(KeyValue[3]);
		
					KeyV_Deal(KeyValue[2]);
				}
		
				KeyE_Deal(KeyValue[0]); //导轮舵机
		
				KeyR_Deal(KeyValue[1]);	//取弹
			}
			else
			{
					KeyG_Deal(KeyValue[6]|KeyValue[3]);
		
					KeyF_Deal(KeyValue[5]|KeyValue[2]);
			
					KeyB_Deal(KeyValue[3]|KeyValue[6]);
		
					KeyV_Deal(KeyValue[2]|KeyValue[5]);
				
				KeyE_Deal(KeyValue[0]); //导轮舵机
				
	//			KeyR_Deal(KeyValue[1]);	//取弹////////////////////////////////////////////////////////////////
			}
			
		
		
			KeyC_Deal(KeyValue[4]);	//图传舵机
		
			MainBoard_Deal();
			
		}
		
//		switch(Res)
//		{
//			case 10:
//			{
//				Lift_Anterior(1);
////				Res=0;
//				break;
//			}
//			
//			case 11:
//			{
//				Lift_Anterior(0);
////				Res=0;
//				break;
//			}
//			
//			case 12:
//			{
//				Lift_Posterior(1);
//				Friction_ALL_Set(1600);
////				Res=0;
//				break;
//			}
//			
//			case 13:
//			{
//				Lift_Posterior(0);
////				Res=0;
//				break;
//			}
//			
//			case 14:
//			{
//				Lift_Anterior(0);
//				Lift_Posterior(0);
//				Servo_Video(0);
//				Servo_ALL(0);
//				LED_GREEN=LIGHT;
//				Friction_ALL_Set(2000);
//				SteppingMotor1_Set(0,1);
////				Res=0;
//				break;
//			}
//			
//			case 15:
//			{
//				Lift_Anterior(1);
//				Lift_Posterior(1);
//				Servo_Video(1);
//				Servo_ALL(1);
//				Friction_ALL_Set(1000);
//				SteppingMotor1_Set(4000,1);
//		//		RampBegin_ST1(10);
//		//		RampBegin_CM12(10);
//				LED_GREEN=LIGHT_OUT;
////				Res=0;
//				break;
//			}
//			
//		}
  } 
} 

 
void RC_Calibration(void)	//上电检测遥控器接收值并与默认参数比较，判断是否正常，否则软复位
{													//注：必须放在遥控器接收初始化后（此处为判断串口数值）
	if(Res!=0)
	{
		NVIC_SystemReset();
	}
}


