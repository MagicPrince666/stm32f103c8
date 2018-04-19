#include "timer.h"
#include "led.h"
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
 
//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{
		LED1=!LED1;			    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr;  	//设定计数器自动重装值 
	TIM3->PSC=psc;  	//预分频器设置
	TIM3->DIER|=1<<0;   //允许更新中断				
	TIM3->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(1,3,TIM3_IRQn,2);//抢占1，子优先级3，组2									 
}
//TIM1_CH1 PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u16 arr,u16 psc)
{		 					 
	//此部分需手动修改IO口设置
	RCC->APB2ENR|=1<<11; 	//TIM1时钟使能   
	RCC->APB2ENR |= 1<<2;    //使能PORTA时钟  
	GPIOA->CRH&=0XFFFF0FF0;	//PA8 11清除之前的设置
	GPIOA->CRH|=0X0000B00B;	//复用功能输出 
	
	TIM1->ARR=arr;			//设定计数器自动重装值 
	TIM1->PSC=psc;			//预分频器设置
  
	TIM1->CCMR1|=7<<4;  	//CH1 PWM2模式		 
	TIM1->CCMR1|=1<<3; 		//CH1预装载使能	 
 	TIM1->CCER|=3<<0;   	//OC1 输出使能

	TIM1->CCMR2|=7<<12;  	//CH4 PWM2模式		 
	TIM1->CCMR2|=1<<11; 	//CH4预装载使能	 
 	TIM1->CCER|=3<<12;   	//OC4 输出使能

	TIM1->BDTR|=1<<15;   	//MOE 主输出使能	   

	TIM1->CR1|=1<<7;   	//ARPE使能 
	TIM1->CR1|=0x01;    	//使能定时器1 										  
}  

//TIM2 PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM2_PWM_Init(u16 arr,u16 psc)
{		 					 
	//此部分需手动修改IO口设置
	RCC->APB1ENR|=1<<0; 	//TIM2时钟使能  
	RCC->APB2ENR |= 1<<2;    //使能PORTA时钟  
	GPIOA->CRL&=0XFFFFFF00;	//PA0 1清除之前的设置
	GPIOA->CRL|=0X000000BB;	//复用功能输出 
	
	TIM2->ARR=arr;			//设定计数器自动重装值 
	TIM2->PSC=psc;			//预分频器设置
  
	TIM2->CCMR1|=7<<4;  	//CH1 PWM2模式		 
	TIM2->CCMR1|=1<<3; 		//CH1预装载使能	 
 	TIM2->CCER|=3<<0;   	//OC1 输出使能

	TIM2->CCMR1|=7<<12;  	//CH2 PWM2模式		 
	TIM2->CCMR1|=1<<11; 	//CH2预装载使能	 
 	TIM2->CCER|=3<<4;   	//OC2 输出使能   

	TIM2->CR1|=1<<7;   	//ARPE使能 
	TIM2->CR1|=0x01;    	//使能定时器1 										  
}  
//TIM3 PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{		 					 
	//此部分需手动修改IO口设置
	RCC->APB1ENR|=1<<1; 	//TIM3时钟使能  
	RCC->APB2ENR |= 1<<2|1<<3;    //使能PORTA时钟  
	GPIOA->CRL&=0X00FFFFFF;	//PA0 1清除之前的设置
	GPIOA->CRL|=0XBB000000;	//复用功能输出 
	GPIOB->CRL&=0XFFFFFF00;	//PA0 1清除之前的设置
	GPIOB->CRL|=0X000000BB;	//复用功能输出 
	
	TIM3->ARR=arr;			//设定计数器自动重装值 
	TIM3->PSC=psc;			//预分频器设置
  
	TIM3->CCMR1|=7<<4;  	//CH1 PWM2模式		 
	TIM3->CCMR1|=1<<3; 		//CH1预装载使能	 
 	TIM3->CCER|=3<<0;   	//OC1 输出使能

	TIM3->CCMR1|=7<<12;  	//CH2 PWM2模式		 
	TIM3->CCMR1|=1<<11; 	//CH2预装载使能	 
 	TIM3->CCER|=3<<4;   	//OC2 输出使能   
	
	TIM3->CCMR2|=7<<4;  	//CH3 PWM2模式		 
	TIM3->CCMR2|=1<<3; 		//CH3预装载使能	 
 	TIM3->CCER|=3<<8;   	//OC3 输出使能

	TIM3->CCMR2|=7<<12;  	//CH4 PWM2模式		 
	TIM3->CCMR2|=1<<11; 	//CH4预装载使能	 
 	TIM3->CCER|=3<<12;   	//OC4 输出使能   

	TIM3->CR1|=1<<7;   	//ARPE使能 
	TIM3->CR1|=0x01;    	//使能定时器1 										  
} 

void TIM4_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<2;	
 	TIM4->ARR=arr;  	
	TIM4->PSC=psc;  	
	TIM4->DIER|=1<<0;   				
	TIM4->CR1|=0x01;    
    MY_NVIC_Init(0,3,TIM4_IRQn,2);								 
}

//定时器2通道1输入捕获配置
//arr：自动重装值
//psc：时钟预分频数
void TIM2_Cap_Init(u16 arr,u16 psc)
{		 
	RCC->APB1ENR|=1<<0;   	//TIM2 时钟使能 
	RCC->APB2ENR|=1<<2;    	//使能PORTA时钟  
	 
	GPIOA->CRL&=0XFFFFFFF0;	//PA0 清除之前设置  
	GPIOA->CRL|=0X00000008;	//PA0 输入   
	GPIOA->ODR|=0<<0;		//PA0 下拉
	  
 	TIM2->ARR=arr;  		//设定计数器自动重装值   
	TIM2->PSC=psc;  		//预分频器 

	TIM2->CCMR1|=1<<0;		//CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM2->CCMR1|=1<<4; 		//IC1F=0001 配置输入滤波器 以Fck_int采样，2个事件后有效
 	TIM2->CCMR1|=0<<10; 	//IC2PS=00 	配置输入分频,不分频 

	TIM2->CCER|=0<<1; 		//CC1P=0	上升沿捕获
	TIM2->CCER|=1<<0; 		//CC1E=1 	允许捕获计数器的值到捕获寄存器中

	TIM2->DIER|=1<<1;   	//允许捕获中断				
	TIM2->DIER|=1<<0;   	//允许更新中断	
	TIM2->CR1|=0x01;    	//使能定时器2
	MY_NVIC_Init(2,0,TIM2_IRQn,2);//抢占2，子优先级0，组2	   
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到高电平;1,已经捕获到高电平了.
//[5:0]:捕获高电平后溢出的次数
u8  TIM2CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM2CH1_CAPTURE_VAL;	//输入捕获值
//定时器2中断服务程序	 
void TIM2_IRQHandler(void)
{ 		    
	u16 tsr;
	tsr=TIM2->SR;
 	if((TIM2CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(tsr&0X01)//溢出
		{	    
			if(TIM2CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM2CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM2CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM2CH1_CAPTURE_VAL=0XFFFF;
				}else TIM2CH1_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x02)//捕获1发生捕获事件
		{	
			if(TIM2CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			    TIM2CH1_CAPTURE_VAL=TIM2->CCR1;	//获取当前的捕获值.
	 			TIM2->CCER&=~(1<<1);			//CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{ 
				TIM2CH1_CAPTURE_VAL=0;
				TIM2CH1_CAPTURE_STA=0X40;		//标记捕获到了上升沿
				TIM2->CNT=0;					//计数器清空
				TIM2->CCER|=1<<1; 				//CC1P=1 设置为下降沿捕获 
			}		    
		}			     	    					   
 	}
	TIM2->SR=0;//清除中断标志位 	    
}

void PWM(u16 PWMa,u16 PWMb,u16 PWMc,u16 PWMd)
{  
	PWM1_VAL=PWMa;
	PWM2_VAL=PWMb;
	PWM3_VAL=PWMc;
	PWM4_VAL=PWMd;
}