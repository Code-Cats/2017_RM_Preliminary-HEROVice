#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "Mydefine.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED�˿ڶ���
#define LED_GREEN PFout(14)	// DS0
#define LED_RED PEout(7)	// DS1	 

void LED_Init(void);//��ʼ��	

void LED_Pointing(WorkState_e work);
#endif
