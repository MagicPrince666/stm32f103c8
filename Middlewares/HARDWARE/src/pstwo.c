#include "pstwo.h"
#include "usart.h"
#include "spi.h"
#include <stdint.h>
/*********************************************************
Copyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File：PS2驱动程序
Author：pinggai    Version:1.0     Data:2015/05/16
Description: PS2驱动程序
**********************************************************/	 

#define _USE_SPI 0

u16 Handkey;
u8 Comd[2]={0x01,0x42};	//开始命令。请求数据
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组

u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//按键值与按键明

//手柄接口初始化    输入  DI->PC0 
//                  输出  DO->PC1    CS->PC3  CLK->PC2
void PS2_Init(void)
{
#if _USE_SPI
	RCC->APB2ENR |= 1<<4; //先使能外设PORTC时钟
	GPIOC->CRL &= 0XFFFFF0FF;
	GPIOC->CRL |= 0X00000300;
	GPIOC->ODR |= 0x1<<2;
	SPI1_Init();
	SPI2_SetSpeed(SPI_SPEED_128); // 72/8 = 9
#else
	RCC->APB2ENR |= 1<<4 | 1 << 2;//先使能外设PORTC时钟
	GPIOC->CRL &= 0XFFFFF0FF;
	GPIOC->CRL |= 0X00000300;
	GPIOC->ODR |= 0x1<<2;
	GPIOA->CRL &= 0X000FFFFF;
	GPIOA->CRL |= 0X38300000;
	GPIOA->ODR |= 0x7<<5;
#endif
}

//向手柄发送命令
void PS2_Cmd(u8 *CMD, int len)
{
#if _USE_SPI
	int i = 0;
	for(i = 0; i < len; i++) {
		Data[i] = SPI1_ReadWriteByte(CMD[i]);
	}
#else
	volatile u16 ref=0x01;
	int i = 0;
	for(i = 0; i < len; i++) {
		for(ref=0x01;ref<0x0100;ref<<=1)
		{
			if(ref&CMD[i])
			{
				PS2_JOYPAD_CMND = 1;                   //输出以为控制位
			}
			else PS2_JOYPAD_CMND = 0;

			delay_us(5);
			PS2_JOYPAD_CLOCK = 0;
			delay_us(5);
			PS2_JOYPAD_CLOCK = 1;

			if(PS2_JOYPAD_DATA)
				Data[i] = ref|Data[i];
		}
	}
#endif
}
//判断是否为红灯模式
//返回值；0，红灯模式
//		  其他，其他模式
u8 PS2_RedLight(void)
{
	PS2_JOYPAD_ATT = 0;
	PS2_Cmd(Comd, sizeof(Comd));
	PS2_JOYPAD_ATT = 1;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//读取手柄数据
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	u8 comd[9]={0x01,0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	PS2_JOYPAD_ATT = 0;

#if _USE_SPI
	for(byte = 0; byte < 9; byte++)          //开始接受数据
	{
		Data[byte] = SPI1_ReadWriteByte(comd[byte]);
	}
#else

	for(byte=0; byte<9; byte++)          //开始接受数据
	{
		for(ref = 0x01; ref < 0x100; ref <<= 1)
		{
			if(ref&comd[byte]) {
				PS2_JOYPAD_CMND = 1;                   //输出以为控制位
			}
			else PS2_JOYPAD_CMND = 0;

			delay_us(5);
			PS2_JOYPAD_CLOCK = 0;
			delay_us(5);
			PS2_JOYPAD_CLOCK = 1;

			if(PS2_JOYPAD_DATA)
		      Data[byte] = ref|Data[byte];
		}
	}

#endif
	PS2_JOYPAD_ATT = 1;	
}

//对读出来的PS2的数据进行处理      只处理了按键部分         默认数据是红灯模式  只有一个按键按下时
//按下为0， 未按下为1
u8 PS2_DataKey()
{
	u8 index;
	int i = 0;

	PS2_ClearData();
	PS2_ReadData();

	printf("Data[ ");
	for( i = 0; i < 9; i++) {
		printf("%02X ", Data[i]);
	}
	printf("]\r\n");

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //没有任何按键按下
}

//得到一个摇杆的模拟量	 范围0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//清除数据缓冲区
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}


/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	uint8_t comd[] = {0x01, 0x42, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	comd[3] = motor1;
	comd[4] = motor2;
	PS2_JOYPAD_ATT = 0;
	PS2_Cmd(comd, sizeof(comd));
	PS2_JOYPAD_ATT = 1;
}
//short poll
void PS2_ShortPoll(void)
{
	uint8_t comd[] = {0x01, 0x42, 0X00, 0x00, 0x00};
	PS2_JOYPAD_ATT = 0;
	PS2_Cmd(comd, sizeof(comd));
	PS2_JOYPAD_ATT = 1;
}
//进入配置
void PS2_EnterConfing(void)
{
	uint8_t comd[] = {0x01, 0x43, 0X00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	PS2_JOYPAD_ATT = 0;
	PS2_Cmd(comd, sizeof(comd));
	PS2_JOYPAD_ATT = 1;
}
//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
	uint8_t comd[] = {0x01, 0x44, 0X00, 0x01, 0xee, 0x00, 0x00, 0x00, 0x00};
	PS2_JOYPAD_ATT = 0;
	PS2_Cmd(comd, sizeof(comd));
	PS2_JOYPAD_ATT = 1;
}
//振动设置
void PS2_VibrationMode(void)
{
	uint8_t comd[] = {0x01, 0x4d, 0X00, 0x00, 0x01};
	PS2_JOYPAD_ATT = 0;
	PS2_Cmd(comd, sizeof(comd));
	PS2_JOYPAD_ATT = 1;
}
//完成并保存配置
void PS2_ExitConfing(void)
{
	uint8_t comd[] = {0x01, 0x43, 0X00, 0x00, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a};
	PS2_JOYPAD_ATT = 0;
	PS2_Cmd(comd, sizeof(comd));
	PS2_JOYPAD_ATT =1;
}
//手柄配置初始化
void PS2_SetInit(void)
{
	PS2_Init();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
	PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		//完成并保存配置
}
