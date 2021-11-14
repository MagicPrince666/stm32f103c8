#ifndef __LED_H
#define __LED_H	 

#include "sys.h"   

//LED端口定义
#define LED0 PCout(3)// DS0
#define LED1 PBout(13)// DS1	
#define LED2 PBout(14)// DS0
#define LED3 PBout(15)// DS1	

void LED_Init(void);	//初始化		
 				    
#endif
