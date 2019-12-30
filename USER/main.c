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

//Robomasters������
//Ӣ�۸���
//���ߣ�Derst Rangers������

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(180);    //��ʼ����ʱ����
	uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200   //�궨��ɺ��ٴ��������ͨ��
	
	LED_Init();					//��ʼ��LED
	
	TIM3_Int_Init(5 -1,9000-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5��Ϊ0.5ms
	
	Date_Init();
	
	SPI5_Init();        //����������ͨ��Э���ʼ��
	MPU6500_Init();     //�����ǳ�ʼ��
	IST8310_Init();     //�����Ƴ�ʼ��
	
	SteppMotor_Init();	//���������ʼ��
	
	ALL_PWM_Init();	//	����PWMͨ����ʼ��
	
	CAN1_Init(); //���CAN��ʼ��
	KEY_Init(); 				//������ʼ�� 
//	EXTIX_Init();	//�ⲿ�жϳ�ʼ���������ã�
	
	VideoRelay_Init();
	
	PID_Init(&CM1_Robust_Pid,ROBUST_P,ROBUST_D,ROBUST_DEAD,ROBUST_MER);	//��δ����������
	PID_Init(&CM2_Robust_Pid,ROBUST_P,ROBUST_D,ROBUST_DEAD,ROBUST_MER);
	PID_Init(&CM3_Robust_Pid,ROBUST_P,ROBUST_D,ROBUST_DEAD,ROBUST_MER);
	PID_Init(&CM4_Robust_Pid,ROBUST_P,ROBUST_D,ROBUST_DEAD,ROBUST_MER);

	
	delay_ms(30);	//����Ҫ��
	RC_Calibration();	//���巢����ֵУ׼
	Lift_Calibration();	//��������궨����
	SetWorkState(NORMAL_STATE);
	Hardware_Init();	//����ΪĬ��״̬
		
while(1)
	{
		key=KEY_Scan(0);
		switch(key)
		{
			case KEY0_PRES://KEY0����
			{
				key=0;
				break;
			}
		}
		
		IMU_Get_Data(); //�������������ݶ�ȡ
		
	} 
	
}
