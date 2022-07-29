#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "mpu6050.h"
#include "timer.h"
#include "imu.h"
/*************************************全局变量定义*************************************************/
//MPU6050
Float_XYZ gyro;
Float_XYZ acc;
Float_Angle angle;


int main(void)
{
/*************************************变量定义*************************************************/
	//测试
	
	//VOFA上位机
	u8 vofa_i = 0;//VOFA上位机循环变量
	u8 vofa_byte[4] = {0};//VOFA上位机数组
	u8 vofa_date[4] = {0};
	 
/*************************************初始化配置*************************************************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	delay_init();	    	//延时函数初始化	  
	uart_init(115200);	//串口初始化为115200
	LED_Init();					//LED初始化
	MPU_Init(); 	//初始化MPU6050
	TIM3_Int_Init(19, 7199);	//2ms中断
	
  while(1)
	{
//		//测试发送
//		Float_to_vofa_byte(1000.0, vofa_byte);  
//		for(vofa_i=0; vofa_i<4; vofa_i++)
//		{
//			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}	
		
//		//发送MPU的温度
//		Float_to_vofa_byte(tempt*0.01, vofa_byte);  
//		for(vofa_i=0; vofa_i<4; vofa_i++)
//		{
//			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}	

//		//发送陀螺仪x轴原始数据
//		Float_to_vofa_byte(gyro.x, vofa_byte);  
//		for(vofa_i=0; vofa_i<4; vofa_i++)
//		{
//			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}			
//		
//		//发送陀螺仪y轴原始数据
//		Float_to_vofa_byte(gyro.y, vofa_byte);  
//		for(vofa_i=0; vofa_i<4; vofa_i++)
//		{
//			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}	

//		//发送陀螺仪z轴原始数据
//		Float_to_vofa_byte(gyro.z, vofa_byte);  
//		for(vofa_i=0; vofa_i<4; vofa_i++)
//		{
//			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}	

//		//发送加速度计x轴原始数据
//		Float_to_vofa_byte(acc.x, vofa_byte);  
//		for(vofa_i=0; vofa_i<4; vofa_i++)
//		{
//			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}

//		//发送加速度计y轴原始数据
//		Float_to_vofa_byte(aacy*1.0, vofa_byte);  
//		for(vofa_i=0; vofa_i<4; vofa_i++)
//		{
//			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}		

//		//发送加速度计z轴原始数据
//		Float_to_vofa_byte(aacz*1.0, vofa_byte);  
//		for(vofa_i=0; vofa_i<4; vofa_i++)
//		{
//			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}

		//发送俯仰角数据
		Float_to_vofa_byte(angle.pitch, vofa_byte);  
		for(vofa_i=0; vofa_i<4; vofa_i++)
		{
			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
		
		//发送横滚角数据
		Float_to_vofa_byte(angle.roll, vofa_byte);  
		for(vofa_i=0; vofa_i<4; vofa_i++)
		{
			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
		
		//发送航向角数据
		Float_to_vofa_byte(angle.yaw, vofa_byte);  
		for(vofa_i=0; vofa_i<4; vofa_i++)
		{
			USART_SendData(USART1, vofa_byte[vofa_i]);         //向串口1发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}
		
		//发送帧尾
		vofa_date[0]=0X00;vofa_date[1]=0X00;
		vofa_date[2]=0X80;vofa_date[3]=0X7f;
		for(vofa_i=0;vofa_i<4;vofa_i++)
		{
			USART_SendData(USART1, vofa_date[vofa_i]);         //向串口3发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}

	}	 
} 

//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 
		
		Get_IMU_Date(&gyro, &acc);//获取IMU原始数据
		IMU_Update(&gyro, &acc, &angle);
	}
}










