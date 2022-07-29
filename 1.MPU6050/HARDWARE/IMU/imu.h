#ifndef __IMU_H
#define __IMU_H	 
#include "sys.h"


#define	Kp			15.0f		//比例增益
#define	Ki			0.005f	//积分增益
#define	halfT		0.001f	//采样周期的一半

#define DEG2RAD  	0.017453293f //度转弧度π/180 
#define RAD2DEG  	57.29578f    //弧度转度180/π 
#define PI				3.141592654f

//存放三轴陀螺仪和三轴加速度计的数据
typedef struct 
{
	float x;
	float y;
	float z;
}Float_XYZ;


//飞行器的姿态角
typedef struct 
{
	float pitch;	//俯仰角
	float roll;		//横滚角
	float yaw;		//航向角
}Float_Angle;



void Get_IMU_Date(Float_XYZ* gyro, Float_XYZ* acc);//获取IMU原始数据
void IMU_Update(Float_XYZ* Gyro, Float_XYZ* Acc, Float_Angle* Angle);//姿态解算

		 				    
#endif


