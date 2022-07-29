#include "imu.h"
#include "mpu6050.h"
#include "usart.h"
#include <math.h>

/*************************************************
快速计算 1/Sqrt(x)
*************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/***************************************************
获取IMU原始数据
***************************************************/
void Get_IMU_Date(Float_XYZ* gyro, Float_XYZ* acc)
{
	short gyrox,gyroy,gyroz;//陀螺仪原始数据
	short aacx,aacy,aacz;		//加速度传感器原始数据
	
	//获取IMU相关数据
	MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);//陀螺仪原始数据
	MPU_Get_Accelerometer(&aacx, &aacy, &aacz);//加速度计原始数据
	
	//陀螺仪原始数据
	gyro->x = gyrox / 16.4 * PI / 180;	//陀螺仪量程为±2000°/s，所以灵敏度为16.4LSB/°/s
	gyro->y = gyroy / 16.4 * PI / 180;	//陀螺仪数据除以16.4得到的是角度，在进行四元数解算时需要将其转化为弧度
	gyro->z = gyroz / 16.4 * PI / 180;
//	gyro->x = gyrox;
//	gyro->y = gyroy;
//	gyro->z = gyroz;
	
	//加速度计原始数据
	acc->x = aacx;
	acc->y = aacy;
	acc->z = aacz;
}

/*************************************************
Myhony姿态解算
*************************************************/
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;		//四元数
float exInt = 0, eyInt = 0, ezInt = 0;	//比例积分误差

//void IMU_Update(Float_XYZ* Gyro, Float_XYZ* Acc, Float_Angle* Angle)//传入IMU的原始数据
void IMU_Update(Float_XYZ* Gyro, Float_XYZ* Acc, Float_Angle* Angle)//传入IMU的原始数据,获得角度
{
	float gx = Gyro->x, gy = Gyro->y, gz = Gyro->z;	//陀螺仪原始数据赋值，简化计算符号
	float ax = Acc->x, ay = Acc->y, az = Acc->z;	//加速度计原始数据赋值，简化计算符号
	float vx, vy, vz;		//理论的加速度
	float ex, ey, ez;		//理论加速度与实际加速度之间的误差
	float norm;		//用于归一化
	
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;		//便于计算
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	
	//加速度计测量的重力向量（机体坐标系下）
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	//陀螺仪积分估算的重力加速度向量（机体坐标系下）  为什么v不需要单位化
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	//测量的重力向量与估算的重力向量叉积求出向量间的误差
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	//对上面的误差进行积分
	exInt = exInt + ex*Ki*halfT;
	eyInt = eyInt + ey*Ki*halfT;
	ezInt = ezInt + ez*Ki*halfT;
	
	//将误差PI调参后补偿到陀螺仪
	gx =gx + Kp*ex + exInt;
	gy =gy + Kp*ey + eyInt;
	gz =gz + Kp*ez + ezInt;//gz由于没有观测者进行校正会产生漂移，表现出积分的自增或者自减（不太懂，再议）
	
	//四元数微分方程
	q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;
	
	//单位化四元数（这里为啥单位化）
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 +q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	//四元数转换成欧拉角
	Angle->pitch = asin(2.0f * (q1q3 - q0q2)) * RAD2DEG;
	Angle->roll = atan2(2.0f*q2q3 + 2.0f*q0q1, q0q0 - q1q1- q2q2 + q3q3) * RAD2DEG;
	Angle->yaw = atan2(2.0f*(q1q2 + q0q3), q0q0 + q1q1- q2q2 - q3q3) * RAD2DEG;
	
}






















