#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//Robomaster 信仰板
//@Derst
//创建日期:2014/5/3
////////////////////////////////////////////////////////////////////////////////// 	 

/*下面的方式是通过直接操作库函数方式读取IO*/
#define KEY0 		GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_10) //PD10

/*下面方式是通过位带操作方式读取IO*/
/*
#define KEY0 		PDin(10)   	//PD10
*/


#define KEY0_PRES 1

void KEY_Init(void);	//IO初始化
u8 KEY_Scan(u8);  		//按键扫描函数	

#endif
