#include "imu_deal.h"
#include "math.h"
#include "imu.h"


double Gyro_y;        //Y????????
double Accel_x;
double Angle_ax;      //???????????
double Angle;         //????????


double  Q_angle=0.001; 
double  Q_gyro=0.003;
double  R_angle=0.5;
double  dt=0.01;                  //dt?kalman???????;
char   C_0 = 1;
double  Q_bias, Angle_err;
double  PCt_0, PCt_1, E;
double  K_0, K_1, t_0, t_1;
double  Pdot[4] ={0,0,0,0};
double  PP[2][2] = { { 1, 0 },{ 0, 1 } }; 

double Kalman_Filter(double Accel,double Gyro)  
{
Angle+=(Gyro - Q_bias) * dt; 
Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; 
Pdot[1]= -PP[1][1];
Pdot[2]= -PP[1][1];
Pdot[3]=Q_gyro;
PP[0][0] += Pdot[0] * dt;   
PP[0][1] += Pdot[1] * dt;   
PP[1][0] += Pdot[2] * dt;
PP[1][1] += Pdot[3] * dt;
Angle_err = Accel - Angle;
PCt_0 = C_0 * PP[0][0];
PCt_1 = C_0 * PP[1][0];
E = R_angle + C_0 * PCt_0;
K_0 = PCt_0 / E;
K_1 = PCt_1 / E;
t_0 = PCt_0;
t_1 = C_0 * PP[0][1];
PP[0][0] -= K_0 * t_0;
PP[0][1] -= K_0 * t_1;
PP[1][0] -= K_1 * t_0;
PP[1][1] -= K_1 * t_1;
Angle += K_0 * Angle_err;
Q_bias  += K_1 * Angle_err; 
Gyro_y   = Gyro - Q_bias;
return  Gyro_y;
}

void Angle_Calc(void)  
{
Accel_x  = imu_data.ax; 
Angle_ax = (Accel_x +90) /2048;   
Angle_ax = Angle_ax*1.4*180/3.14;   
Gyro_y = imu_data.gy;  
Gyro_y = -(Gyro_y -12)/16.4; 
Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);
}

