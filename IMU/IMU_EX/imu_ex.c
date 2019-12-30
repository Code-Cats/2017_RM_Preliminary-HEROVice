#include "imu.h"
#include "imu_deal.h"
#include "imu_ex.h"

volatile unsigned char gyro_rx_buffer[25];
volatile unsigned char gyro_rx_buffer_Last[25];//记录上一帧数据
volatile unsigned char gyro_rx_buffer_real[25];//记录正常数据帧值
MPU6050_t MPUEX;
/* ----------------------- Function Implements ---------------------------- */
/******************************************************************************
* @fn MPU6050_Init
*
* @brief configure stm32 usart2 port
* - USART Parameters
* - 100Kbps
* - 8-N-1
* - DMA Mode
*
* @return None.
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port it
* to the other platform.
*/


void IMU_EX_Init(void)
{
/* -------------- Enable Module Clock Source ----------------------------*/
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
GPIO_PinAFConfig(GPIOD,GPIO_PinSource9, GPIO_AF_USART3);
/* -------------- Configure GPIO ---------------------------------------*/
{
GPIO_InitTypeDef gpio;
USART_InitTypeDef usart3;
gpio.GPIO_Pin = GPIO_Pin_9 ;
gpio.GPIO_Mode = GPIO_Mode_AF;
gpio.GPIO_OType = GPIO_OType_PP;
gpio.GPIO_Speed = GPIO_Speed_100MHz;
gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_Init(GPIOD, &gpio);
USART_DeInit(USART3);
usart3.USART_BaudRate = 115200;
usart3.USART_WordLength = USART_WordLength_8b;
usart3.USART_StopBits = USART_StopBits_1;
usart3.USART_Parity = USART_Parity_No;
usart3.USART_Mode = USART_Mode_Rx| USART_Mode_Tx;
usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_Init(USART3,&usart3);
USART_Cmd(USART3,ENABLE);
USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
}
/* -------------- Configure NVIC ---------------------------------------*/
{
NVIC_InitTypeDef nvic;
nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;
nvic.NVIC_IRQChannelPreemptionPriority = 11;
nvic.NVIC_IRQChannelSubPriority = 0;
nvic.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&nvic);
}
/* -------------- Configure DMA -----------------------------------------*/
{
DMA_InitTypeDef dma;
DMA_DeInit(DMA1_Stream1);
dma.DMA_Channel = DMA_Channel_4;
dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
dma.DMA_Memory0BaseAddr = (uint32_t)gyro_rx_buffer;
dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
dma.DMA_BufferSize = 20;
dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
dma.DMA_Mode = DMA_Mode_Circular;
dma.DMA_Priority = DMA_Priority_VeryHigh;
dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
dma.DMA_MemoryBurst = DMA_Mode_Normal;
dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
DMA_Init(DMA1_Stream1,&dma);
DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
DMA_Cmd(DMA1_Stream1,ENABLE);
}
}
/******************************************************************************
* @fn DMA1_Stream1_IRQHandler
*
* @brief USART1 DMA ISR
*
* @return None.
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port it
* to the other platform.
*/



int n=0;	//定义记录次数变量（输出第一帧用）
int b=0;


int iii,j=0;
int real=0;




void DMA1_Stream1_IRQHandler(void)
{
if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
{
	
	real=0;
	//以下函数从上一接收帧与当前接收帧中提取有效数据帧（存在一个接收周期的物理延迟）
	for(iii=0;iii<20;iii++)
	{
		if(gyro_rx_buffer_Last[iii]==90)
		{
			if(iii==19)
			{
				if(gyro_rx_buffer[0]==165)
				{
					j=iii;	//得到有效起始位位置
					real=1;//标志已得到
				}
			}
			else
			{  
				if(gyro_rx_buffer_Last[iii+1]==165)
				{
					j=iii;	//得到有效起始位位置
					real=1;//标志已得到
				}
			}
		}
	}
	if(real==1)
	{
		for(iii=0;iii<20;iii++)	//传递有效帧给gyro_rx_buffer_real
		{
			if((iii+j)<20)
				gyro_rx_buffer_real[iii]=gyro_rx_buffer_Last[j+iii];
			else
				gyro_rx_buffer_real[iii]=gyro_rx_buffer[iii+j-20];
		}
	}
	
	for(iii=0;iii<20;iii++)	
	{
		gyro_rx_buffer_Last[iii]=gyro_rx_buffer[iii];
	}
	n++;
	
DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
MPUEX.check = ((gyro_rx_buffer_real[0]<< 24) | (gyro_rx_buffer_real[1]) << 16| (gyro_rx_buffer_real[2])<< 8 | (gyro_rx_buffer_real[3]));
if(MPUEX.check!=0x5AA50E00)
{
	//Dog.IMU=0;
}
else
{
	MPUEX.v.roll     = ((gyro_rx_buffer_real[7])  | (gyro_rx_buffer_real[8] <<8));
	MPUEX.v.pitch      = ((gyro_rx_buffer_real[9])  | (gyro_rx_buffer_real[10]<<8));
	MPUEX.v.yaw       = ((gyro_rx_buffer_real[11]) | (gyro_rx_buffer_real[12]<<8)); 
	MPUEX.angle.roll = ((gyro_rx_buffer_real[14]) | (gyro_rx_buffer_real[15]<<8));
	MPUEX.angle.pitch  = ((gyro_rx_buffer_real[16]) | (gyro_rx_buffer_real[17]<<8));
	MPUEX.angle.yaw   = ((gyro_rx_buffer_real[18]) | (gyro_rx_buffer_real[19]<<8)); 
}
}
}



