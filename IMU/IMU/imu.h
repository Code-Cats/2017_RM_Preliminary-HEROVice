/**
  *@file test_imu.h
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__IMU_H
#define _TEST__IMU_H

#include "stm32f4xx.h"

#define MPU6500     PFout(6)
/*

#define MPU6500_NSS_High()     GPIO_ResetBits(GPIOF,GPIO_Pin_6)
#define MPU6500_NSS_Low()    GPIO_SetBits(GPIOF,GPIO_Pin_6)
*/
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;

extern uint8_t MPU_id;
extern IMUDataTypedef imu_data;
uint8_t MPU6500_Init(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
void IMU_Get_Data(void);
uint8_t IST8310_Init(void);

#endif

