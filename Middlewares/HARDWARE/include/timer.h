#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//通用定时器 驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/07
//版本：V1.2
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved	
//********************************************************************************
//V1.1 20140306 
//增加TIM1_CH1，PWM输出设置相关内容 
//V1.2 20140307
//增加TIM2输入捕获初始化函数TIM2_Cap_Init及其中断处理
////////////////////////////////////////////////////////////////////////////////// 	  
 
//通过改变TIM1->CCR1的值来改变占空比，从而控制LED0的亮度
// #define PWM1_VAL TIM2->CCR1  
// #define PWM2_VAL TIM2->CCR2 
// #define PWM3_VAL TIM1->CCR1 
// #define PWM4_VAL TIM1->CCR4  
#define PWM1_VAL TIM3->CCR1  
#define PWM2_VAL TIM3->CCR2 
#define PWM3_VAL TIM3->CCR3 
#define PWM4_VAL TIM3->CCR4 
#define PWM5_VAL TIM2->CCR1  
#define PWM6_VAL TIM2->CCR2  
 
void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM2_Cap_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);
void PWM(u16 PWMa,u16 PWMb,u16 PWMc,u16 PWMd);
#endif
