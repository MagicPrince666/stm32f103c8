#ifndef __MFRC_TASK_H_
#define __MFRC_TASK_H_

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"

//////////////////////////////////
//端口定义

#define     MF522_SDA  	 PBout(6)       //SDA
#define     MF522_SCK    PBout(3)       //SCK
#define     MF522_MOSI   PBout(5)       //MOSI
#define     MF522_MISO   PBout(4)       //MISO
#define     MF522_RST    PBout(7)       //RST
//指示灯
#define     LED_GREEN   LED0
//蜂鸣器引脚定义
#define     MF522_IRQ    PAin(15)

/////////////////////////////////////////////////////////////////////
//函数原型
/////////////////////////////////////////////////////////////////////
void InitializeSystem();    
void MFRC_main();                               
                                    
#endif
