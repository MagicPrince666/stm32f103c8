#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//按键输入 驱动代码		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/18
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved			
//********************************************************************************
//V1.1修改说明 20140318
//新增KEY0_Scan函数
//////////////////////////////////////////////////////////////////////////////////	 

#define KEY0_PRES	1		//KEY0按下
#define KEY1_PRES	2		//KEY1按下
#define WKUP_PRES	3		//WK_UP按下

#define KEY0 PEin(5)   	//PE4
#define KEY1 PEin(2)	//PE3 
#define KEY2 PEin(5)	//PD12
#define WK_UP PAin(0)	//PA0  WK_UP
	 
void KEY_Init(void);		//IO初始化
u8 KEY_Scan(u8 mode);		//按键扫描函数
u8 KEY0_Scan(void);			//单独扫描KEY0按键
#endif
