#include "sys.h"
#include "uart5.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	   
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板 
//串口3初始化代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2015/3/14
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 	


//串口接收缓存区 	
u8 UART5_RX_BUF[UART5_MAX_RECV_LEN]; 				//接收缓冲,最大UART5_MAX_RECV_LEN个字节.
u8 UART5_TX_BUF[UART5_MAX_SEND_LEN]; 			//发送缓冲,最大UART5_MAX_SEND_LEN字节

//通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过10ms,则认为不是1次连续数据.也就是超过10ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
vu16 UART5_RX_STA=0;   	 
void UART5_IRQHandler(void)
{
	u8 res;	      
	if(UART5->SR&(1<<5))//接收到数据
	{	 
		res=UART5->DR; 			 
		if((UART5_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(UART5_RX_STA<UART5_MAX_RECV_LEN)	//还可以接收数据
			{
				if(UART5_RX_STA==0) 				//使能定时器7的中断 
				{
				}
				UART5_RX_BUF[UART5_RX_STA++]=res;	//记录接收到的值	 
			}else 
			{
				UART5_RX_STA|=1<<15;				//强制标记接收完成
			} 
		}
	}  				 											 
}   
//初始化IO 串口5
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率 
void uart5_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction;

	// PC12 - UART5_TX PD2 - UART5_RX
	RCC->APB2ENR |= 1<<4 | 1<<5;   //使能PORTC D口时钟  
	GPIOC->CRH &= 0XFFF0FFFF;//IO状态设置
	GPIOC->CRH |= 0X000B0000;//IO状态设置 
	GPIOD->CRL &= 0XFFFFF0FF;//IO状态设置
	GPIOD->CRL |= 0X00000800;//IO状态设置 

	RCC->APB1ENR |= 1<<20;  	//使能串口时钟 	 
	RCC->APB1RSTR |= 1<<20;   //复位串口5
	RCC->APB1RSTR &= ~(1<<20);//停止复位
	
	//波特率设置
 	UART5->BRR = mantissa;// 波特率设置	 
	UART5->CR1 |= 0X200C;  	//1位停止,无校验位.
	//使能接收中断 
	UART5->CR1 |= 1<<5;    	//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,1,UART5_IRQn,2);//组2
	UART5_RX_STA=0;		//清零
}

//串口3,printf 函数
//确保一次发送数据不超过UART5_MAX_SEND_LEN字节
void u5_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)UART5_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)UART5_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		while((UART5->SR&0X40)==0);			//循环发送,直到发送完毕   
		UART5->DR=UART5_TX_BUF[j];  
	} 
}

