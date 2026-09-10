#include "io.h"
//输出的引脚对应左右传给F1;输入的引脚一个是F1传回的需要播种，第二个是光电管接收电平D 9
//E7左对应F1	B13；E8右对应F1	B14
//F4	E14		F1	B3
void IO_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE,ENABLE);//开启按键GPIO口的时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;							//选择按键的引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;					//设置引脚为输入模式	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 				//设置引脚速率为2MHz 
	GPIO_Init(GPIOE, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;							//选择按键的引脚
	GPIO_Init(GPIOE, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
	GPIO_ResetBits(GPIOE,GPIO_Pin_8);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;							//F1传给F4的减速信号
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;					//设置引脚为输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 				//设置引脚速率为2MHz 
	GPIO_Init(GPIOE, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;							//车身左侧光电管
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;					//设置引脚为输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//浮空
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 				//设置引脚速率为2MHz 
	GPIO_Init(GPIOD, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;							//车身右侧光电管
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;					//设置引脚为输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//浮空
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 				//设置引脚速率为2MHz 
	GPIO_Init(GPIOC, &GPIO_InitStructure);   				//使用上面的结构体初始化按键
	
}
