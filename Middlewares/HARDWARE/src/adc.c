#include "adc.h"
#include "delay.h"					   
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK MiniSTM32开发板
//ADC 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/18
//版本：V1.2
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//********************************************************************************
//V1.1 20140309
//在Adc_Init函数增加了内部温度测量的初始化参数。							
//V1.2 20140318
//新增Get_Temp函数，用于获取内部温度值。							
////////////////////////////////////////////////////////////////////////////////// 
	   
//初始化ADC
//这里我们仅以规则通道为例
//我们默认仅开启通道1，并开启内部温度传感器了。
void  Adc_Init(void)
{    
	//先初始化IO口
 	RCC->APB2ENR|=1<<2;    //使能PORTA口时钟 
	GPIOB->CRL&=0XFF00FFFF;//PA4 PA5 anolog输入
	//通道10/11设置			 
	RCC->APB2ENR|=1<<9;    //ADC1时钟使能	  
	RCC->APB2RSTR|=1<<9;   //ADC1复位
	RCC->APB2RSTR&=~(1<<9);//复位结束	    
	RCC->CFGR&=~(3<<14);   //分频因子清零	
	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
	//否则将导致ADC准确度下降! 
	RCC->CFGR|=2<<14;      	 
	ADC1->CR1&=0XF0FFFF;   //工作模式清零
	ADC1->CR1|=0<<16;      //独立工作模式  
	ADC1->CR1&=~(1<<8);    //非扫描模式	  
	ADC1->CR2&=~(1<<1);    //单次转换模式
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //软件控制转换  
	ADC1->CR2|=1<<20;      //使用用外部触发(SWSTART)!!!	必须使用一个事件来触发
	ADC1->CR2&=~(1<<11);   //右对齐	 
	ADC1->CR2|=1<<23;      //使能温度传感器

	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1|=0<<20;     //1个转换在规则序列中 也就是只转换规则序列1 	

#ifdef ADC_CH3
	ADC1->SMPR2&=~(7<<3*3);  //通道3采样时间清空	  
 	ADC1->SMPR2|=7<<3*3;     //通道3  239.5周期,提高采样时间可以提高精确度	 
#endif

#ifdef ADC_CH4
	ADC1->SMPR2&=~(7<<3*4);  //通道4采样时间清空	  
 	ADC1->SMPR2|=7<<3*4;     //通道4  239.5周期,提高采样时间可以提高精确度	 
#endif

#ifdef ADC_CH5
	ADC1->SMPR2&=~(7<<3*5);  //通道5采样时间清空	  
 	ADC1->SMPR2|=7<<3*5;     //通道5  239.5周期,提高采样时间可以提高精确度	 
#endif

#ifdef ADC_CH6
	ADC1->SMPR2&=~(7<<3*6);  //通道5采样时间清空	  
 	ADC1->SMPR2|=7<<3*6;     //通道5  239.5周期,提高采样时间可以提高精确度	 
#endif

#ifdef ADC_CH7
	ADC1->SMPR2&=~(7<<3*7);  //通道7采样时间清空	  
 	ADC1->SMPR2|=7<<3*7;     //通道7  239.5周期,提高采样时间可以提高精确度	 
#endif	
		   
#ifdef ADC_CH8
	ADC1->SMPR2&=~(7<<3*8);  //通道8采样时间清空	  
 	ADC1->SMPR2|=7<<3*8;     //通道8  239.5周期,提高采样时间可以提高精确度	 
#endif
#ifdef ADC_CH9
	ADC1->SMPR2&=~(7<<3*9);  //通道9采样时间清空	  
 	ADC1->SMPR2|=7<<3*9;     //通道9  239.5周期,提高采样时间可以提高精确度	 
#endif

 	ADC1->SMPR1&=~(7<<18);  //清除通道16原来的设置	 
	ADC1->SMPR1|=7<<18;     //通道16  239.5周期,提高采样时间可以提高精确度	 

	ADC1->CR2|=1<<0;	   //开启AD转换器	 
	ADC1->CR2|=1<<3;       //使能复位校准  
	while(ADC1->CR2&1<<3); //等待校准结束 			 
    //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
	ADC1->CR2|=1<<2;        //开启AD校准	   
	while(ADC1->CR2&1<<2);  //等待校准结束
	//该位由软件设置以开始校准，并在校准结束时由硬件清除  
}

//获得ADC值
//ch:通道值 0~16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	//设置转换序列	  		 
	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //启动规则转换通道 
	while(!(ADC1->SR&1<<1));//等待转换结束	 	   
	return ADC1->DR;		//返回adc值	
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 
//得到ADC采样内部温度传感器的温度值
//返回值3位温度值 XXX*0.1C	 
int Get_Temp(void)
{				 
	u16 temp_val=0;
	u8 t;
	float temperate;   
	for(t=0;t<20;t++)//读20次,取平均值
	{
		temp_val+=Get_Adc(ADC_CH_TEMP);
		delay_ms(1);
	}
	temp_val/=20;
	temperate=(float)temp_val*(3.3/4096);//得到温度传感器的电压值
	temperate=(1.43-temperate)/0.0043+25;//计算出当前温度值	 
	temperate*=10;//扩大十倍,使用小数点后一位
	return (int)temperate;	 

}
	 
	 