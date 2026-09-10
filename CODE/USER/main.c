#include "stdio.h"
#include "string.h"
#include "sys.h"
#include "delay.h"
#include "stm32f4xx.h"
#include "control.h"
//#include "lamp.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "oled.h"
#include "usart1_dma.h"
#include "usart2_dma.h"
#include "usart3_dma.h"
#include "uart4_dma.h"
#include "timer7.h"
#include "servo.h"
#include "ultrasonic.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "io.h"

float start_yaw=0;

int main(void)
{
	//系统初始化
	u8 key;
	float yaw_t;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	//BEEP_Init();
	Key_Init();
	IO_init();
	LED_Init();
	TIM4_Servo_PWM_Init(20000,84);
	Ultrasonic_Init();
	OLED_Init();
	OLED_Display_On();
	OLED_Clear();
	Motor_Init();
	TIM7_Init(1000,84);
	TIM3_EncoderB_Init(0xFFFF,1);
	TIM5_EncoderA_Init(0xFFFF,1);
	PID_Init();
	Motor_Stop();
	Motor_Set_Speed();
	USART1_Init(115200);
	USART2_Init(9600);

	Servo1_Increase = 0;
	Servo2_Increase = 0;
	Servo3_Increase = 0;
	Servo4_Increase = 0;
	SpeedL1_Increase = 0;
	SpeedR1_Increase = 0;
	SpeedL2_Increase = 0;
	SpeedR2_Increase = 0;
	SpeedL1_Increase_9250 = 0;
	SpeedR1_Increase_9250 = 0;
	SpeedL2_Increase_9250 = 0;
	SpeedR2_Increase_9250 = 0;
//	LED1_ON;
	
	Servo_Forw();
//	Servo_Turn();
	Servo_Set_Angle();
	while(1)
	{
		key = Key_Scan();
		if(key == 1)
		{
			start_YAW=yaw;					//记录下当前指南针原始中心角度
			if(start_YAW>-180 && start_YAW<-135)mode=2;
			else if(start_YAW<180 && start_YAW>135)mode=3;

			delay_ms(100);			//等待yaw稳定
			
			start_yaw=yaw;
			PID_9250_Forward_motor.desired = start_yaw;
			PID_9250_Forward.desired = start_yaw;
			PID_9250_Right.desired = start_yaw;
			PID_9250_Left.desired = start_yaw;
			PID_9250_Right_slow.desired = start_yaw;
			PID_9250_Left_slow.desired = start_yaw;
			PID_9250_Back.desired = start_yaw;
			PID_9250_Back_last.desired = start_yaw;
			PID_9250_Back_last_motor.desired = start_yaw;
			flag_mission_start = 1;
			flag_delay_1500ms = 1;	//step0时开始2s后以60%速度继续行驶
		}
		
		if(key == 2)
		{
			flag_step += 2;
//			PID_F1_Left_Servo.kp+=0.1;
//			PID_F1_Left_Servo.kp+=0.1;
		}
		
		if(key == 3)
		{
//			start_YAW=yaw;					//记录下当前指南针原始中心角度
//			if(start_YAW>-180 && start_YAW<-135)mode=2;
//			else if(start_YAW<180 && start_YAW>135)mode=3;
//			PID_9250_Right.kd+=2;
		}
		
		if(key == 4)
		{
			delay_ms(100);
//			Motor_Forw();
//			Motor_Right();
//			Motor_Left();
//			Motor_Back();
//			Motor_Set_Speed();
			Servo_Forw();
			Servo_Set_Angle();
		}
		
		
		delay_us(20);
		OLED_Show3FNum(10,0,yaw ,4,2,12);
		OLED_ShowIntNum(64,0,flag_step,2,12);
		//OLED_ShowIntNum(84,0,flag_huanlong,1,12);
		//OLED_ShowIntNum(96,0,flag_chulong,1,12);
		
//		OLED_Show3FNum(64,0 ,	PID_9250_Back.kp,3,1,12);
//		OLED_Show3FNum(10,12 ,	PID_9250_Back.kd,3,1,12);
		
		
//		OLED_Show3FNum(0,36,PID_9250_Right.kp,5,2,12);
//		OLED_Show3FNum(48,36,PID_9250_Right.kd,5,2,12);
/*********************************************************/
		OLED_Show3FNum(0,12,ultra_F1.DIS ,5,2,12);
		OLED_Show3FNum(48,12,ultra_F2.DIS ,5,2,12);

		OLED_Show3FNum(0,24,ultra_L1.DIS ,5,2,12);
		OLED_Show3FNum(48,24,ultra_L2.DIS,5,2,12);
		
		OLED_Show3FNum(0,36,ultra_R1.DIS ,5,2,12);
		OLED_Show3FNum(48,36,ultra_R2.DIS ,5,2,12);
    
//		OLED_Show3FNum(10,48,RightLightLast ,1,0,12);
//		OLED_Show3FNum(24,48,LeftLightLast ,1,0,12);
//		OLED_Show3FNum(36,48,RightLight ,1,0,12);
//		OLED_Show3FNum(48,48,LeftLight ,1,0,12);
//		OLED_Show3FNum(10,48,PID_F1_Left_Servo.kp ,1,3,12);
//		OLED_Show3FNum(48,48,PID_F1_Left_Servo.kd ,1,3,12);
	  OLED_Refresh_Gram();
	}
}
