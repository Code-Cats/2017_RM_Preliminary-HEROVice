#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//Robomaster ������
//@Derst
//��������:2014/5/3
////////////////////////////////////////////////////////////////////////////////// 	 

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
#define KEY0 		GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_10) //PD10

/*���淽ʽ��ͨ��λ��������ʽ��ȡIO*/
/*
#define KEY0 		PDin(10)   	//PD10
*/


#define KEY0_PRES 1

void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8);  		//����ɨ�躯��	

#endif
