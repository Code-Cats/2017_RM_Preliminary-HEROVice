#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "sys.h"

typedef struct
	{
		uint32_t check;//陀螺仪的校验，确保数据是正确的。值为5AA50700
		struct
		{
		int16_t pitch;//俯仰角
		int16_t roll;//横滚角
		int16_t yaw;//航向角
		}angle;
		struct
		{
		int16_t pitch;//俯仰角
		int16_t roll;//横滚角
		int16_t yaw;//航向角
		}v;
	}MPU6050_t;

extern volatile unsigned char gyro_rx_buffer[25];
extern MPU6050_t MPUEX;
void IMU_EX_Init(void);

/* ----------------------- Internal Data ----------------------------------- */
#endif

