#include "sys.h"
#include "usart.h"	

#include "control.h"
#include "led.h"
#include "Mydefine.h"
#include "keyboard.h"

#include "pwm.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//////////////////////////////////////////////////////////////////////////////////	 
//����6��ʼ��	������ͨ��ģ�飩	   
//�޸�����:2017/4/23
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

//����6�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
	

//��ʼ��IO ����6
//bound:������
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
//		USART6_InitStruct.USART_Parity = USART_Parity_No; //żУ��
//		USART6_InitStruct.USART_Mode = USART_Mode_Rx;
//		USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//		USART_Init(USART6,&USART6_InitStruct);
//		USART_Cmd(USART6,ENABLE);
//	
//	//USART_ClearFlag(USART1, USART_FLAG_TC);
//		


//	//Usart1 NVIC ����
//  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����1�ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//	
//	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�
	
	
	
	
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
		USART6_InitStruct.USART_Parity = USART_Parity_No; //żУ��
		USART6_InitStruct.USART_Mode =  USART_Mode_Tx|USART_Mode_Rx;
		USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART6,&USART6_InitStruct);
		USART_Cmd(USART6,ENABLE);
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
		
	

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�
}

u8 Res=0;

void USART6_IRQHandler(void)                	//����6�жϷ������
{

	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{ 
		Res =USART_ReceiveData(USART6);//(USART6->DR);	//��ȡ���յ�������   //������  "X"BVCGFRE
		
		if(GetWorkState()==NORMAL_STATE)
		{
			Res_Deal(KeyValue,Res);
		
			KeyQ_Deal(KeyValue[7]);	//�ǵ�ģʽ����
		
			if(KeyValue[7]==1)	//�ǵ�ģʽ��ƽ��ģʽ���������л�
			{
				if(Key7.statu==0)	//�����Ӿ�ǰ��ʼ�պͼ���ǰ����Ӧ
				{
					KeyG_Deal(KeyValue[3]);
		
					KeyF_Deal(KeyValue[2]);
		
					KeyB_Deal(KeyValue[6]);
		
					KeyV_Deal(KeyValue[5]);
				}
				else	//������
				{
					KeyG_Deal(KeyValue[6]);
		
					KeyF_Deal(KeyValue[5]);
			
					KeyB_Deal(KeyValue[3]);
		
					KeyV_Deal(KeyValue[2]);
				}
		
				KeyE_Deal(KeyValue[0]); //���ֶ��
		
				KeyR_Deal(KeyValue[1]);	//ȡ��
			}
			else
			{
					KeyG_Deal(KeyValue[6]|KeyValue[3]);
		
					KeyF_Deal(KeyValue[5]|KeyValue[2]);
			
					KeyB_Deal(KeyValue[3]|KeyValue[6]);
		
					KeyV_Deal(KeyValue[2]|KeyValue[5]);
				
				KeyE_Deal(KeyValue[0]); //���ֶ��
				
	//			KeyR_Deal(KeyValue[1]);	//ȡ��////////////////////////////////////////////////////////////////
			}
			
		
		
			KeyC_Deal(KeyValue[4]);	//ͼ�����
		
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

 
void RC_Calibration(void)	//�ϵ���ң��������ֵ����Ĭ�ϲ����Ƚϣ��ж��Ƿ�������������λ
{													//ע���������ң�������ճ�ʼ���󣨴˴�Ϊ�жϴ�����ֵ��
	if(Res!=0)
	{
		NVIC_SystemReset();
	}
}


