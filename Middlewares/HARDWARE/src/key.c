#include "key.h"
#include "delay.h"

 	    
//按键初始化函数 
//PA0.15和PC5 设置成输入
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;     //使能PORTA时钟
	RCC->APB2ENR|=1<<6;     //使能PORTE时钟
	GPIOA->CRL&=0XFFFFFFF0;	//PA0设置成输入，默认下拉	  
	GPIOA->CRL|=0X00000008; 
  
	GPIOE->CRL&=0XFF0FF0FF;	//PE2~4设置成输入	  
	GPIOE->CRL|=0X00800800; 				   
	GPIOE->ODR|=1<<5|1<<2;	   	//PE2~4 上拉
} 
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//返回值：
//0，没有任何按键按下
//KEY0_PRES，KEY0按下
//KEY1_PRES，KEY1按下
//WKUP_PRES，WK_UP按下 
//注意此函数有响应优先级,KEY0>KEY1>WK_UP!!
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY0==0||KEY1==0||WK_UP==1))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY0==0)return KEY0_PRES;
		else if(KEY1==0)return KEY1_PRES;
		else if(WK_UP==1)return WKUP_PRES; 
	}else if(KEY0==1&&KEY1==1&&WK_UP==0)key_up=1; 
	return 0;// 无按键按下
}
//单独扫描KEY0是否被按下
//返回值:0,没按下;1,按下了.
u8 KEY0_Scan(void)
{	 
	static u8 key_up=1;//按键按松开标志 	  
	if(key_up&&KEY2==0)
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY2==0)return 1;
	}else if(KEY2==1)key_up=1; 	     
	return 0;// 无按键按下
}

