#ifndef __JOYPAD_H
#define __JOYPAD_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//游戏手柄驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//手柄连接引脚
#define JOYPAD_CLK PCout(2)  	//时钟		PC9
#define JOYPAD_LAT PCout(1)  	//锁存     	PC8
#define JOYPAD_DAT PCin(0)	 	//数据     	PC12   
#define PS2_JOYPAD_ATT PCout(3)       //CS拉高

void JOYPAD_Init(void);			//初始化	
u8 JOYPAD_Read(void);			//读取键值	 			

#include "sys.h"
#include "delay.h"
#include "usart.h"

#define PS2_JOYPAD_DATA  PCin(0)
#define PS2_JOYPAD_CMND PCout(1)
#define PS2_JOYPAD_CLOCK PCout(2)
#define PS2_JOYPAD_ATT PCout(3)
#define PS2_JOYPAD_ACK PCin(4)

#define PS2_JOYPAD_CMND_START  0X01  //起始命令
#define PS2_JOYPAD_CMND_DEMAND 0X42 //数据请求
#define PS2_JOYPAD_CMND_NOP   0X00   //idle

void PS2_Wireless_JOYPAD_Init(void);
u16 PS2_Wireless_JOYPAD_DATA(void);

u16 key_scan(void);
void psout(void);
void psinout(void);

#endif
