/* 23年东方红控制代码 
   编写人：周段文诚（原杨少文改）
	 时间：2023.4.25
	             头
			   R1----------L1
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 R2---------- L2
				       人
*/
/*
2024 沈子杰
		         F1头F2
			   L1----------R1
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 |            |
				 L2---------- R2
				       尾
							 人
*/
#include "stdio.h"
#include "string.h"
#include "control.h"
#include "sys.h"
#include "delay.h"
//#include "lamp.h"
#include "led.h"
#include "usart1_dma.h"
#include "usart2_dma.h"
#include "usart3_dma.h"
#include "uart4_dma.h"
#include "ultrasonic.h"
#include "oled.h"
#include "encoder.h"
#include "param.h"
#include "motor.h"
#include "pid.h"
#include "servo.h"
#include "io.h"

#define N 0.98
#define Move 185          //直行停车预留量420
#define move 320          //右行停车预留量 190
#define movez 320         //左行停车预留量 190
#define movez_last 350		//出垄前左行停车预留量

u8 parity						 = 0;								//判断奇数偶
float pitch, roll;
extern float yaw;
short aacx, aacy, aacz;
short gyrox, gyroy, gyroz;
u8 flag_step = 0;					//判断现在运行到了第几步的标志
u8 flag_yaw_new = 0;

char PIDOUT[20];
//int a=0;
/***********************************周段文诚 2022.9.14********************************************
0：入长垄          
1：→1
2：↓1
3：←1
4：↓2
5：→2
6：↓3
7：←2
8：↓4
9：→3
10：←3
11：出垄
12：停车
***********************************************************************************************/
u8 flag_mission_start 		= 0;				//按键按下后，进入控制函数的标志
u8 flag_huanlong 				  = 0;				//换垄标志，此flag=1时正在换垄
u8 flag_chulong 				  = 0;				//出笼标志，此flag=1时已经出笼
u8 flag_slowdown 				  = 0;				//减速标志，此flag=1时表示需要减速
u8 flag_zuowuslowdown 		= 0;				//减速标志，此flag=1时表示需要减速
u8 flag_found_zuowu				= 0;				//检测作物的标志，此flag=1时表示检测到作物，开始减速
u8 flag_delay					    = 0;				//延时标志，此flag=1时表示需要延时
u8 flag_delay_200ms				= 0;				//延时标志，此flag=1时表示需要延时200ms
u8 flag_delay_500ms				= 0;				//延时标志，此flag=1时表示需要延时500ms
u8 flag_delay_1000ms			= 0;				//延时标志，此flag=1时表示需要延时1000ms
u8 flag_delay_1500ms			= 0;				//延时标志，此flag=1时表示需要延时1500ms
u8 flag_delay_zuowu				= 0;				//延时标志，此flag=1时作物表示需要延时
u8 Color_SLOW							=0;
u8 flag_step0SLOW					=0;
uint8_t RightLightLast = 2;
uint8_t LeftLightLast = 2;

//float N_F_R = 1.0;
//根据不同步数选择舵机、电机的基本角度、速度
float Servo1_Base = 90, Servo2_Base = 90, Servo3_Base = 90, Servo4_Base = 90;
float Servo5_Base = 0, Servo6_Base = 0, Servo7_Base = 0, Servo8_Base = 0;
float SpeedL1_Base = 0, SpeedR1_Base = 0, SpeedL2_Base = 0, SpeedR2_Base = 0;
float SpeedL1_Base_c = 0, SpeedR1_Base_c = 0, SpeedL2_Base_c = 0, SpeedR2_Base_c = 0;
//根据PID的结果得到的舵机、电机的增量
float Servo1_Increase = 0, Servo2_Increase = 0, Servo3_Increase = 0, Servo4_Increase = 0;
float Servo5_Increase = 0, Servo6_Increase = 0, Servo7_Increase = 0, Servo8_Increase = 0;
float SpeedL1_Increase = 0, SpeedR1_Increase = 0, SpeedL2_Increase = 0, SpeedR2_Increase = 0;
float SpeedL1_Increase_9250 = 0, SpeedR1_Increase_9250 = 0, SpeedL2_Increase_9250 = 0, SpeedR2_Increase_9250 = 0;
//调试基础电机速度
//float Motor_Forw_test_L1 = 0, Motor_Forw_test_L2 = 0, Motor_Forw_test_R1 = 0, Motor_Forw_test_R2 = 0;
void TIM7_IRQHandler()
{

    static u16 time_1ms;
    static u16 time_5ms;
    static u16 time_10ms;
    static u16 time_25ms;
    static u16 time_50ms;
    static u16 time_200ms;
    static u16 time_500ms;
    static u16 time_1000ms;
    static u16 time_1500ms;
    static u16 time_zuowu;
    static u16 lamp_1000ms;

    if(TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) //溢出中断
    {
        if(flag_mission_start == 1)		//按键按下后1、5、10、50ms的任务才能进行
        {
            time_1ms++;
            time_5ms++;
            time_10ms++;
            time_50ms++;
        }

        //下面的则不受按键的影响
        time_25ms++;


        if(flag_delay_200ms)								//延时专用
        {
            time_200ms++;
        }

        if(flag_delay_500ms)
        {
            time_500ms++;
        }

        if(flag_delay_1000ms)
        {
            time_1000ms++;
        }

        if(flag_delay_1500ms)
        {
            time_1500ms++;
        }

        if(flag_delay_zuowu)
        {
            time_zuowu++;
        }

//----------------------------------------------------------------------
        if(time_1ms == 1)
        {
            time_1ms = 0;

        }

//----------------------------------------------------------------------
        if(time_5ms == 5)
        {
					time_5ms = 0;
					Detect_Flag_Step();				//判断运行到第几步了

					Detect_Flag_Mv();
					flag_delay = (flag_delay_200ms || flag_delay_500ms || flag_delay_1000ms);

					if(!flag_delay)						//在不需要延时的时候才运行下面的步骤
					{
							Detect_Flag_Slowdown();		//判断是否需要减速
							Select_Servo_Angle();			//根据步数选择舵机基本角度
							Select_Motor_Speed();			//根据步数选择电机基本速度
							Select_PID();							//根据当前运行的步数来选择用那个PID

							Servo_Set_Angle();				//设置舵机角度
							Motor_Set_Speed();				//设置电机转速

							if(!flag_huanlong)
							{
									TIM5->CNT = 0;
									TIM3->CNT = 0;
									Encoders.disA = 0;
									Encoders.disB = 0;
							}

							if(flag_huanlong)
							{
									Read_Encoder_Cnt();
							}
					}
					if(!flag_delay)
					{
						if(flag_huanlong&&!flag_chulong)
						{
							HuanLong();								//换垄，当换完垄后，flag_step会+1
						}

						if(flag_chulong)
						{
							ChuLong();
						}
					 }

        }
//----------------------------------------------------------------------
        if(time_10ms == 10)
        {
          time_10ms = 0;
//					sprintf(PIDOUT,"%f,%f\n",ultra_F1.DIS,yaw);
//					sprintf(PIDOUT,"%f,%f\n",ultra_F1.DIS,ultra_F2.DIS);
//					USART1_DMATransfer((uint32_t *)PIDOUT,strlen(PIDOUT));
        }

//----------------------------------------------------------------------
        if(time_25ms == 25)
        {
            time_25ms = 0;

            Ultra_Trig();
						//Yaw_new();
        }

//----------------------------------------------------------------------
        if(time_50ms == 50)
        {
            time_50ms = 0;
        }

//----------------------------------------------------------------------
        if(time_200ms == 200)
        {
            time_200ms = 0;

            flag_delay_200ms = 0;
        }

//----------------------------------------------------------------------
        if(time_500ms == 450)
        {
            time_500ms = 0;

            flag_delay_500ms = 0;
        }

//----------------------------------------------------------------------
        if(time_1000ms == 680)	//700
        {
            time_1000ms = 0;

            flag_delay_1000ms = 0;
        }
//----------------------------------------------------------------------
        if(time_1500ms == 2400)			//第0步运行time_1500ms秒后开始以60%速度行驶，2s+后开始减速
        {
            time_1500ms = 0;

            flag_delay_1500ms = 0;
						if(flag_step==0 && flag_step0SLOW==0)
						{
							flag_step0SLOW=1;
							SpeedL1_Base *= 0.75;
							SpeedR1_Base *= 0.75;
							SpeedL2_Base *= 0.75;
							SpeedR2_Base *= 0.75;
							
							Motor_Set_Speed();
						}
        }

//----------------------------------------------------------------------
        if(time_zuowu == 400)
        {
					time_zuowu = 0;
        }

    }

    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);			//清除中断标志位
}

//void Detect_Flag_yaw_new(void)
//{
//	if((flag_step == 1)||(flag_step == 3)||(flag_step == 5)||(flag_step == 7)||(flag_step == 9)||(flag_step == 10))
//		flag_yaw_new = 0;
//	else 
//		flag_yaw_new = 0;
//}

//void Yaw_new(void)
//{
//	if((flag_yaw_new == 1) && (ultra_F1.DIS<150) && (ultra_F1.DIS>130) && (ultra_F2.DIS>134) && (ultra_F2.DIS<154) && ((ultra_F1.DIS - ultra_F2.DIS < 4)||(ultra_F2.DIS - ultra_F1.DIS < 4)))
//	{
//			PID_9250_Forward.desired = yaw;
//			PID_9250_Right.desired = yaw;
//			PID_9250_Left.desired = yaw;
//			PID_9250_Right_slow.desired = yaw;
//			PID_9250_Left_slow.desired = yaw;
//			PID_9250_Back.desired = yaw;
//			PID_9250_Back_last.desired = yaw;
//			PID_9250_Back_last_motor.desired = yaw;
//	}
//}

/*
flag_step == 0时未入垄
flag_step == 1入了垄
flag_step == 2时在第一条垄上
*/
void Detect_Flag_Step(void)
{
//第0步-----------------------------------------------------------------
    if((flag_step == 0) && flag_delay_1500ms == 0 && ((ultra_F1.DIS < F1 + Move) && (ultra_F2.DIS < F2 + Move)))
    {
        flag_step = 1;
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
        Servo_Set_Angle();
				flag_delay_1000ms = 1;
				return;
    }

//----------------------------------------------------------------------

//第1步-----------------------------------------------------------------
//    if((flag_step == 1) && ((ultra_R1.DIS < R1 + move) && (ultra_R2.DIS < R2 + move)) && (ultra_L1.DIS > 200))
//    if((flag_step == 1) && RightLight)
		if((flag_step == 1) && (LightStop()))
    {
        flag_step = 2;
        //清除STEP==1时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        Motor_Stop();
        Motor_Set_Speed();							//设置电机转速
				Select_Servo_Angle();			//根据步数选择舵机基本角度
        Servo_Set_Angle();
        flag_delay_1000ms = 1;
        flag_huanlong = 1;
				return;
    }

//----------------------------------------------------------------------

//第2步-----------------------------------------------------------------
//    if((flag_step == 3) && ((ultra_L1.DIS < L1 + movez)&&(ultra_L2.DIS < L2 + movez)) && (ultra_R1.DIS > 200))
//		if((flag_step == 3) && LeftLight)
		if((flag_step == 3) && (LightStop()))
    {
				flag_step = 4;
        //清除STEP==2时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
        Servo_Set_Angle();
        flag_delay_1000ms = 1;
				flag_huanlong = 1;
				return;
    }

//----------------------------------------------------------------------

//第3步-----------------------------------------------------------------
//    if((flag_step == 5) && ((ultra_R1.DIS < R1 + move) && (ultra_R2.DIS < R2 + move)) && (ultra_L1.DIS > 200))
		if((flag_step == 5) && (LightStop()))
    {
				flag_step = 6;
        //清除STEP==3时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
        Servo_Set_Angle();
        flag_delay_1000ms = 1;
        flag_huanlong = 1;
				return;
    }

//----------------------------------------------------------------------

//第4步-----------------------------------------------------------------
//    if((flag_step == 7) && ((ultra_L1.DIS < L1 + movez)&&(ultra_L2.DIS < L2 + movez)) && (ultra_R1.DIS > 200))
		if((flag_step == 7) && (LightStop()))
    {
        //清除STEP==7时的PID输出
        flag_step = 8;
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
				Servo_Set_Angle();
        flag_delay_1000ms = 1;
        flag_huanlong = 1;
				return;
    }

//----------------------------------------------------------------------

//第5步-----------------------------------------------------------------
//    if((flag_step == 9) && ((ultra_R1.DIS < R1 + (move + 50)) && (ultra_R2.DIS < R2 + (move + 50))) && (ultra_L1.DIS > 200))
		if((flag_step == 9) && (LightStop()))
    {
				flag_step = 10;
        //清除STEP==5时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
				SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				Select_Servo_Angle();			//根据步数选择舵机基本角度
				Servo_Set_Angle();
        flag_delay_500ms = 1;
				return;
    }

//----------------------------------------------------------------------

//----------------------------------------------------------------------

//第7步-----------------------------------------------------------------
//    if((flag_step == 10) && ((ultra_L1.DIS < L1 + movez_last)||(ultra_L2.DIS < L2 + movez_last)) && (ultra_R1.DIS > 200))
		if((flag_step == 10) && (LightStop()))
    {
        //清除STEP==10时的PID输出
				flag_step = 11;
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
        SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				delay_ms(160);			//车会飘移
				Select_Servo_Angle();			//根据步数选择舵机基本角度
				Servo_Set_Angle();
				flag_delay_1000ms = 1;
        flag_huanlong = 1;
        flag_chulong = 1;
				return;
    }

		 if(flag_step == 12)
    {
        //清除STEP==12时的PID输出
        Servo1_Increase = 0;
        Servo2_Increase = 0;
        Servo3_Increase = 0;
        Servo4_Increase = 0;
				Servo5_Increase = 0;
        Servo6_Increase = 0;
        Servo7_Increase = 0;
        Servo8_Increase = 0;
        SpeedL1_Increase = 0;
        SpeedR1_Increase = 0;
        SpeedL2_Increase = 0;
        SpeedR2_Increase = 0;
        SpeedL1_Increase_9250 = 0;
        SpeedR1_Increase_9250 = 0;
        SpeedL2_Increase_9250 = 0;
        SpeedR2_Increase_9250 = 0;
        //----------------------
        Motor_Stop();
        Motor_Set_Speed();
				return;
    }
}

void Select_Servo_Angle(void)		 							//根据当前运行的步数来选择舵机基本角度
{
    if((flag_step == 0) || (flag_step == 2) || (flag_step == 4)|| (flag_step == 6) || (flag_step == 8) || (flag_step == 11))
    {
        Servo_Forw();															//轮子向前
    }

    if((flag_step == 1) || (flag_step == 3) || (flag_step == 5)|| (flag_step == 7) || (flag_step == 9) || (flag_step == 10))
    {
        Servo_Turn();															//轮子转向
    }
}

void Select_Motor_Speed(void)									//根据当前运行的步数来选择电机的基本速度
{
    if(flag_step == 0  && flag_delay_1500ms==1)
    {
        Motor_Forw();
    }

    if((flag_step == 1) || (flag_step == 5) || (flag_step == 9))
    {
        Motor_Right();
    }

    if((flag_step == 3) || (flag_step == 7))
    {
        Motor_Left();
    }

		if(flag_step == 10)
		{
			Motor_Left_last();
		}
    if(flag_slowdown)
    {
        Motor_SlowDown();
    }

    if(flag_step == 12)
    {
        Motor_Stop();
    }
}

void Select_PID(void)
{
    if(flag_step == 0)
    {
			if((ultra_L1.DIS<200)&&(ultra_L2.DIS<200))
			{
				Contrl_9250_Forward(yaw);
				Control_L1_For_Servo(ultra_L1.DIS);
			  Control_L2_For_Servo(ultra_L2.DIS);
			}
      else Contrl_9250_Forward(yaw);//进之前不用超声波
			return;
    }
		
		if(flag_step == 1)
		{
			SpeedL1_Increase = 0;
			SpeedR1_Increase = 0;
			SpeedL2_Increase = 0;
			SpeedR2_Increase = 0;
			
//			EdgeFilter(ultra_F1.DIS,1);
//			EdgeFilter(ultra_F2.DIS,2);
			
			Control_F1_Right_Servo(ultra_F1.DIS);
			Control_F2_Right_Servo(ultra_F2.DIS);
			Contrl_9250_Right(yaw);
		}
		
    if(((flag_step == 5) || (flag_step == 9)) && (!flag_huanlong))
		{
//			  if(((ultra_F1.DIS<280)&&(ultra_F2.DIS<280)) && ((ultra_R1.DIS>450) || (ultra_R2.DIS>450)) && ((ultra_L2.DIS>450) || (ultra_L1.DIS>450)))
//        {
//				SpeedL1_Increase = 0;
//        SpeedR1_Increase = 0;
//        SpeedL2_Increase = 0;
//        SpeedR2_Increase = 0;
//					if(flag_slowdown == 1)
//						Contrl_9250_Right_slow(yaw);
//					else
//					Contrl_9250_Right(yaw);	
//					Control_F2_Right_Servo(ultra_F2.DIS);
//					Control_F1_Right_Servo(ultra_F1.DIS);
//				}
//			  else        
//				{
//					SpeedL1_Increase_9250 = 0;
//					SpeedR1_Increase_9250 = 0;
//					SpeedL2_Increase_9250 = 0;
//					SpeedR2_Increase_9250 = 0;
//					SpeedL1_Increase = 0;
//          SpeedR1_Increase = 0;
//          SpeedL2_Increase = 0;
//          SpeedR2_Increase = 0;
//					Contrl_9250_Right_slow(yaw);
//					if(ultra_L1.DIS>450||ultra_L2.DIS>450)
//						Control_R1_Right_Servo_new((ultra_R1.DIS-ultra_R2.DIS),flag_step);
//					if(ultra_R1.DIS>450||ultra_R2.DIS>450)
//						Control_L1_Left_Servo_new((ultra_L1.DIS-ultra_L2.DIS),flag_step);
//				}
//			if(((ultra_R1.DIS>240) || (ultra_R2.DIS>240)) && ((ultra_L2.DIS>240) || (ultra_L1.DIS>240)))
				if(!LeftLight)
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
				
				Control_F1_Right_Servo(ultra_F1.DIS);
				Control_F2_Right_Servo(ultra_F2.DIS);
				Contrl_9250_Right(yaw);
//				if(ultra_F1.DIS>100 && ultra_F1.DIS<190)ultra_F1DISLast=ultra_F1.DIS;
//				if(ultra_F2.DIS>100 && ultra_F2.DIS<190)ultra_F2DISLast=ultra_F2.DIS;
			}
			else
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
				
				Contrl_9250_Right(yaw);
			}
    }
		
    if( ((flag_step == 3) || (flag_step == 7) || (flag_step == 10)) && (!flag_huanlong))
    {
//			if( (ultra_R1.DIS>240 || (ultra_R2.DIS>240)) && ((ultra_L2.DIS>240) || (ultra_L1.DIS>240)))
			if(!RightLight)
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
				
//				EdgeFilter(ultra_F1.DIS,1);
//				EdgeFilter(ultra_F2.DIS,2);
				
				Control_F1_Left_Servo(ultra_F1.DIS);
				Control_F2_Left_Servo(ultra_F2.DIS);
				Contrl_9250_Left(yaw);
			}
			else
			{
				SpeedL1_Increase = 0;
				SpeedR1_Increase = 0;
				SpeedL2_Increase = 0;
				SpeedR2_Increase = 0;
				
				Contrl_9250_Left(yaw);
			}
    }
		
    if(((flag_step == 2) || (flag_step == 6)) && flag_huanlong)	//靠右侧换垄
    {
		Contrl_9250_Back(yaw);
		Control_R2_Back_Servo(ultra_R2.DIS);
		Control_R1_Back_Servo(ultra_R1.DIS);
    }
		
    if(((flag_step == 4) || (flag_step == 8)) && flag_huanlong)
		{
			Contrl_9250_Back(yaw);
			Control_L2_Back_Servo(ultra_L2.DIS);
		  Control_L1_Back_Servo(ultra_L1.DIS);
		}

		if(flag_step == 11)
		{
				if((ultra_L1.DIS<200)&&(ultra_L2.DIS<200))
				{
					Control_L2_Backlast_Servo(ultra_L2.DIS);
					Control_L1_Backlast_Servo(ultra_L1.DIS);
				}
				else
				{
					Servo1_Increase = 0;
					Servo2_Increase = 0;
					Servo3_Increase = 0;
					Servo4_Increase = 0;
					SpeedL1_Increase = 0;
					SpeedR1_Increase = 0;
					SpeedL2_Increase = 0;
					SpeedR2_Increase = 0;
					Contrl_9250_Back_last(yaw);
				}
		}
}

void Detect_Flag_Slowdown(void)		//检测车是否需要减速
{
    flag_slowdown = 0;
		flag_found_zuowu = 0;
		Color_SLOW=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15);
//    if(flag_step == 0)
//    {
//        if(((ultra_F1.DIS < 560) && (ultra_F2.DIS < 560))&&(!flag_huanlong))
//        {
//          flag_slowdown = 1;
//					return;
//        }
//    }
		
    if(((flag_step == 1) || (flag_step == 5) || (flag_step == 9)) && (!flag_huanlong) && Color_SLOW!=0)
    {
      flag_slowdown = 1;
			flag_found_zuowu = 1;
			return;
    }

    if(((flag_step == 3) || (flag_step == 7) || (flag_step == 10)) && (!flag_huanlong) && Color_SLOW!=0)
    {
      flag_slowdown = 1;
			flag_found_zuowu = 1;
			return;
    }
		
//		if(flag_step==10 && ultra_L1.DIS<550 && ultra_L2.DIS<550)
//		{
//			SpeedL1_Base *= 0.7;
//      SpeedR1_Base *= 0.7;
//      SpeedL2_Base *= 0.7;
//      SpeedR2_Base *= 0.7;
//		}
}

void HuanLong(void)								//换垄，利用编码器
{
    static float	disA_desired, disB_desired;//调距离，编码器
    Servo_Forw();
    Motor_Back();

    if(flag_step == 2) {disA_desired = 17.4 ; disB_desired = -41;}
    if(flag_step == 4) {disA_desired = 22.4 ; disB_desired = -16;}

    if(flag_step == 6) {disA_desired = 17.5 ; disB_desired = -19;}

    if(flag_step == 8) {disA_desired = 22.8 ; disB_desired = -16.3;}

    if(Encoders.disA > disA_desired)
    {
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
        Motor_Stop();
        Motor_Set_Speed();							//设置电机转速
        Servo_Turn();
        Servo_Set_Angle();							//设置舵机角度
        flag_delay_1000ms = 1;
        flag_step++;
        flag_huanlong = 0;
        Encoders.disA = 0;
        Encoders.disB = 0;
    }
}

void ChuLong(void)									//出笼，利用编码器
{
    Servo_Forw();
    Motor_Back_last();

    if((Encoders.disB < -21.3))
    {
        Motor_Stop();
        Motor_Set_Speed();							//设置电机转速
        flag_delay_1000ms = 1;
        flag_step++;
        flag_huanlong = 0;
        flag_chulong = 0;
        Encoders.disA = 0;
        Encoders.disB = 0;
				flag_mission_start = 0;
    }
}
/*****************************************四轮舵机控制********************************************/
//这里的对应车子上标的
//Servo1_Base是L1
//Servo2_Base是R1
//Servo3_Base是L2
//Servo4_Base是R2
void Servo_Forw_pre(void)							//车轮向前的准备
{
    Servo1_Base = 62;
    Servo2_Base = 165;
    Servo3_Base = 142;
    Servo4_Base = 66;
	
	  Servo5_Base = 200;
    Servo6_Base = 200;
    Servo7_Base = 200;
    Servo8_Base = 200;
}

void Servo_Forw(void)							//车轮向前
{
    Servo1_Base = 42;	//改小向车外转
    Servo2_Base = 138;
    Servo3_Base = 138;
    Servo4_Base = 43;
}

void Servo_Turn(void)							//车轮转向
{
    Servo1_Base = 131;
    Servo2_Base = 48;
    Servo3_Base = 50;
    Servo4_Base = 133;
}
/*****************************************四轮电机控制********************************************/
//现实---代码中
//R1			R2
//L1			L2
//L2			R1
//R2			L1

void Motor_Forw(void)							//车轮向前转
{
    SpeedL1_Base = 5019;
    SpeedR1_Base = 5100;
    SpeedL2_Base = 5089;
    SpeedR2_Base = 5100;//5060
}

void Motor_Back(void)							//车轮向后转,换垄
{
    SpeedL1_Base = -3810;
    SpeedR1_Base = -3780;//4400
    SpeedL2_Base = -3780;
    SpeedR2_Base = -3810;//4060
}

void Motor_Back_last(void)							//车轮向后转，出垄
{
    SpeedL1_Base = -4710;
    SpeedR1_Base = -4680;
    SpeedL2_Base = -4900;
    SpeedR2_Base = -4710;
}

void Motor_Right(void)							//车轮向右转
{
    SpeedL1_Base = 3660*N;
    SpeedR1_Base = -3660*N;
    SpeedL2_Base = 3700*N;
    SpeedR2_Base = -3700*N;
}
void Motor_Left(void)							//车轮向左转4000
{
    SpeedL1_Base = -3700*N;
    SpeedR1_Base = 3700*N;
    SpeedL2_Base = -3690*N;
    SpeedR2_Base = 3690*N;
}

void Motor_Left_last(void)							//车轮向左转,回长垄，4000
{
    SpeedL1_Base = -4200;
    SpeedR1_Base = 4220;
    SpeedL2_Base = -3900;
    SpeedR2_Base = 3900;
}

void Motor_Stop(void)							//车停止
{
    SpeedL1_Base = 0;
    SpeedR1_Base = 0;
    SpeedL2_Base = 0;
    SpeedR2_Base = 0;
}

void Motor_SlowDown(void)					//车子减速
{
    if(flag_found_zuowu)
    {
        SpeedL1_Base *= 0.4;
        SpeedR1_Base *= 0.4;
        SpeedL2_Base *= 0.4;
        SpeedR2_Base *= 0.4;
    }	
}

void IQR_Delay(unsigned int n)		//中断中用的延时函数
{
    unsigned int i, j;

    for(j = 0; j < 50 * n; j++)
    {
        for(i = 0; i < 400; i++);
    }
}

void Detect_Flag_Mv()
{
//	if((flag_step == 1||flag_step == 5||flag_step==9) && ultra_L1.DIS>140 && ultra_L2.DIS>140)	//L12超声波	145
	if((flag_step == 1||flag_step == 5||flag_step==9) && !RightLight)
//	if((flag_step == 1||flag_step == 5||flag_step==9))
	{
//		Mv1_1 = 0;
//		Mv1_2 = 1;
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		GPIO_SetBits(GPIOE,GPIO_Pin_8);
	}
	
//	else if((flag_step == 3||flag_step == 7) && ultra_R1.DIS>140 && ultra_R2.DIS>140)		//R12超声波
	else if((flag_step == 3||flag_step == 7) && !LeftLight)		//R12超声波
//	else if((flag_step == 3||flag_step == 7))		//R12超声波

	{
//		Mv1_1 = 1;
//		Mv1_2 = 0;
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
	}
	else
	{
//		Mv1_1 = 0;
//		Mv1_2 = 0;
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
	}
}

uint8_t LightStop()		//垄中光电管停车，当对应光电管从1变到0时停车
{
	uint8_t a,b;
	a=RightLight;
	b=LeftLight;
	if((flag_step==1 || flag_step==5 ) && a==1 && b==1 && LeftLightLast==0) //向右跑垄，右侧光电管已经1，左侧光电管0->1，
	{
		RightLightLast=a;
		LeftLightLast=b;
		return 1;
	}
	else if((flag_step==9) && a==1 && RightLightLast==0 && LeftLightLast==0) //向右跑垄，右侧（同侧）光电管0->1	step 9到10不需要完全走到头
	{
		RightLightLast=a;
		LeftLightLast=b;
		return 1;
	}
	else if((flag_step==3 || flag_step==7) && b==1 && a==1 && RightLightLast==0)	//向左跑垄，左边光电管已经1，右侧光电管0->1
	{
		RightLightLast=a;
		LeftLightLast=b;
		return 1;
	}
	else if(flag_step==10 &&  b==1 && LeftLightLast==0 && RightLightLast==0)	//10->11是左侧（同侧）光电管
	{
		RightLightLast=a;
		LeftLightLast=b;
		delay_ms(83);
		return 1;
	}
	RightLightLast=a;
	LeftLightLast=b;
	return 0;
}
