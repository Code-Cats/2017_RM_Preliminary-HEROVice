#include "imu.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "spi.h"
#include "delay.h"
#include "Mydefine.h"

ImuTypeDef Imu_Deal={0};

uint8_t MPU_id = 0;

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};

IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Tx;
  
  MPU6500=0;
  
  MPU_Tx = reg&0x7f;
  SPI5_ReadWriteByte(MPU_Tx);
  MPU_Tx = data;
  SPI5_ReadWriteByte(MPU_Tx);
  
  MPU6500=1;
  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500=0;
  
  MPU_Tx = reg|0x80;
  SPI5_ReadWriteByte(MPU_Tx);
  MPU_Rx=SPI5_ReadWriteByte(reg|0x80);
  
  MPU6500=1;
  return  MPU_Rx;
}
uint8_t MPU6500_Read_Regs(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  MPU_Tx = reg|0x80;
  MPU_Rx=SPI5_ReadWriteByte(MPU_Tx);
  return  MPU_Rx;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  delay_ms(10);
}

//Write IST8310 register through MPU6500
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  delay_ms(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  return data;
}

//Initialize the MPU6500 I2C Slave0 for I2C reading
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  delay_ms(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  delay_ms(7);
}

//Initialize the IST8310
uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  delay_ms(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  delay_ms(100);
  return 0;
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}

//Get 6 axis data from MPU6500
void IMU_Get_Data()
{
  uint8_t mpu_buff[20];
	MPU6500=0;
	SPI5_ReadWriteByte(MPU6500_ACCEL_XOUT_H|0x80);
  mpu_buff[0]=MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H); 
  mpu_buff[1]=MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_L); 
  imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];
	mpu_buff[2]=MPU6500_Read_Regs(MPU6500_ACCEL_YOUT_H); 
  mpu_buff[3]=MPU6500_Read_Regs(MPU6500_ACCEL_YOUT_L); 
  imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
	mpu_buff[4]=MPU6500_Read_Regs(MPU6500_ACCEL_ZOUT_H); 
  mpu_buff[5]=MPU6500_Read_Regs(MPU6500_ACCEL_ZOUT_L); 	
  imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];
	
	mpu_buff[6]=MPU6500_Read_Regs(MPU6500_TEMP_OUT_H); 
  mpu_buff[7]=MPU6500_Read_Regs(MPU6500_TEMP_OUT_L); 	
  imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
  
	mpu_buff[8]=MPU6500_Read_Regs(MPU6500_GYRO_XOUT_H);
  mpu_buff[9]=MPU6500_Read_Regs(MPU6500_GYRO_XOUT_L); 	
  imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9] - imu_data_offest.gx;
	mpu_buff[10]=MPU6500_Read_Regs(MPU6500_GYRO_YOUT_H); 
  mpu_buff[11]=MPU6500_Read_Regs(MPU6500_GYRO_YOUT_L); 	
  imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
	mpu_buff[12]=MPU6500_Read_Regs(MPU6500_GYRO_ZOUT_H); 
  mpu_buff[13]=MPU6500_Read_Regs(MPU6500_GYRO_ZOUT_L); 	
  imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;
	
	mpu_buff[14]=MPU6500_Read_Regs(IST8310_R_XL);
  mpu_buff[15]=MPU6500_Read_Regs(IST8310_R_XM ); 	
  imu_data.mx = mpu_buff[14]<<8 |mpu_buff[15];
	mpu_buff[16]=MPU6500_Read_Regs(IST8310_R_YL);
  mpu_buff[17]=MPU6500_Read_Regs(IST8310_R_YM ); 	
  imu_data.my = mpu_buff[16]<<8 |mpu_buff[17];
	mpu_buff[18]=MPU6500_Read_Regs(IST8310_R_ZL);
  mpu_buff[19]=MPU6500_Read_Regs(IST8310_R_ZM ); 	
  imu_data.mz = mpu_buff[18]<<8 |mpu_buff[19];
	
	//mpu接受数值处理 移动加权滤波
	for(Imu_Deal.i=0;Imu_Deal.i<(FILTERING_RANGE-1);Imu_Deal.i++)
	{
		Imu_Deal.ax_sum[FILTERING_RANGE-1-Imu_Deal.i]=Imu_Deal.ax_sum[FILTERING_RANGE-2-Imu_Deal.i];
		Imu_Deal.ay_sum[FILTERING_RANGE-1-Imu_Deal.i]=Imu_Deal.ay_sum[FILTERING_RANGE-2-Imu_Deal.i];
		Imu_Deal.az_sum[FILTERING_RANGE-1-Imu_Deal.i]=Imu_Deal.az_sum[FILTERING_RANGE-2-Imu_Deal.i];
	}
	Imu_Deal.ax_sum[0]=imu_data.ax;
	Imu_Deal.ay_sum[0]=imu_data.ay;
	Imu_Deal.az_sum[0]=imu_data.az;
	if(Imu_Deal.count>1)
	{
		Imu_Deal.ax_calc=(Imu_Deal.ax_sum[0]+Imu_Deal.ax_sum[1]+Imu_Deal.ax_sum[2]+Imu_Deal.ax_sum[3]+Imu_Deal.ax_sum[4]+Imu_Deal.ax_sum[5]+Imu_Deal.ax_sum[6]+Imu_Deal.ax_sum[7])/FILTERING_RANGE;
		Imu_Deal.ay_calc=(Imu_Deal.ay_sum[0]+Imu_Deal.ay_sum[1]+Imu_Deal.ay_sum[2]+Imu_Deal.ay_sum[3]+Imu_Deal.ay_sum[4]+Imu_Deal.ay_sum[5]+Imu_Deal.ay_sum[6]+Imu_Deal.ay_sum[7])/FILTERING_RANGE;
		Imu_Deal.az_calc=(Imu_Deal.az_sum[0]+Imu_Deal.az_sum[1]+Imu_Deal.az_sum[2]+Imu_Deal.az_sum[3]+Imu_Deal.az_sum[4]+Imu_Deal.az_sum[5]+Imu_Deal.az_sum[6]+Imu_Deal.az_sum[7])/FILTERING_RANGE;
		Imu_Deal.count=0;
	}
	Imu_Deal.count++;
	
	MPU6500=1;
  
}

//Initialize the MPU6500
uint8_t MPU6500_Init(void)
{
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
  };
  
  delay_ms(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  
  for(index = 0; index < 10; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    delay_ms(1);
  }

  return 0;
}


