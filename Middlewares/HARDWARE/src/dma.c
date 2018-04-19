#include "dma.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>

/*
功能：初始化uart dma接收
DMA_CHx DMA通道
cpar 外设地址
cmar 存储器地址
cndtr 数据长度
*/
void MYDMA_uart_rx_conf(DMA_Channel_TypeDef* DMA_CHx,uint32_t cpar,uint32_t cmar,uint16_t cndtr)
{
  RCC->AHBENR |= 1<<0;//开启DMA控制器
  DMA_CHx->CPAR=cpar; 	 	//DMA1 外设地址 
	DMA_CHx->CMAR=(uint32_t)cmar; 	//DMA1,存储器地址
	DMA_CHx->CCR=0X00000000;	//复位
	DMA_CHx->CCR|=0<<4;  		//0从外设读 1从存储器读
	DMA_CHx->CCR|=1<<5;  		//0普通模式 1循环模式
	DMA_CHx->CCR|=0<<6; 		//外设地址非增量模式
	DMA_CHx->CCR|=1<<7; 	 	//存储器增量模式
	DMA_CHx->CCR|=0<<8; 	 	//外设数据宽度为8位 8-16-32
	DMA_CHx->CCR|=0<<10; 		//存储器数据宽度8位
	DMA_CHx->CCR|=1<<12; 		//中等优先级 13:12 --> 00:low 01:medium 10:hight 11:very high
	DMA_CHx->CCR|=0<<14; 		//非存储器到存储器模式
	DMA_CHx->CCR|=1<<3;          //传输错误中断 Transfer error interrupt enable
	//DMA_CHx->CCR|=1<<2;          //Half transfer interrupt enable
	DMA_CHx->CCR|=1<<1;          //传输完成中断 Transfer complete interrupt enable

  DMA_CHx->CCR&=~(1<<0);       //关闭DMA传输 
  DMA_CHx->CNDTR=cndtr;    	//DMA1,传输数据量
	DMA_CHx->CCR|=1<<0;          //开启DMA传输
}
/*
DMA 发送配置函数
DMA_CHx DMA通道
cpar 外设地址
cmar 存储器地址
cndtr 数据长度
*/
void MYDMA_uart_tx_conf(DMA_Channel_TypeDef* DMA_CHx,uint32_t cpar,uint32_t cmar,uint16_t cndtr)
{
  RCC->AHBENR |= 1<<0;//开启DMA控制器
  DMA_CHx->CPAR=cpar; 	 	//DMA1 外设地址 
	DMA_CHx->CMAR=(uint32_t)cmar; 	//DMA1,存储器地址
	DMA_CHx->CCR=0X00000000;	//复位
	DMA_CHx->CCR|=1<<4;  		//0从外设读 1从存储器读
	DMA_CHx->CCR|=0<<5;  		//0普通模式 1循环模式
	DMA_CHx->CCR|=0<<6; 		//外设地址非增量模式
	DMA_CHx->CCR|=1<<7; 	 	//存储器增量模式
	DMA_CHx->CCR|=0<<8; 	 	//外设数据宽度为8位 8-16-32
	DMA_CHx->CCR|=0<<10; 		//存储器数据宽度8位
	DMA_CHx->CCR|=1<<12; 		//中等优先级
	DMA_CHx->CCR|=0<<14; 		//非存储器到存储器模式
	//DMA_CHx->CCR|=1<<3;          //传输错误中断 Transfererrorinterruptenable
	//DMA_CHx->CCR|=1<<2;          //Half transfer interrupt enable
	//DMA_CHx->CCR|=1<<1;          //传输完成中断 Transfer complete interrupt enable
  //DMA_CHx->CCR|=1<<0;          //开启DMA传输
}

/*
DMA传输一次数据
DMA_CHx DMA通道
cpar 外设地址
cmar 存储器地址
cndtr 数据长度
*/
void DMA_uart_tx(DMA_Channel_TypeDef* DMA_CHx,uint32_t cpar,uint32_t cmar,uint16_t cndtr)
{
  DMA_CHx->CPAR = cpar; 	 	      //DMA 外设地址 
	DMA_CHx->CMAR = cmar;           //DMA1,存储器地址
  DMA_CHx->CCR &= ~(1<<0);        //必须关闭DMA传输 才能写入DMA_CHx->CNDTR 否则为只读计数器
	DMA_CHx->CNDTR = cndtr;    	    //DMA1,传输数据量
	DMA_CHx->CCR |= 1<<0;           //开启DMA传输
}

//开启一次DMA传输
void Start_UART_DMA_Transmit(DMA_Channel_TypeDef *DMA_CHx,uint32_t len)
{
	DMA_CHx->CCR &= ~(1<<0);       //关闭DMA传输 
	DMA_CHx->CNDTR = len; //DMA1,传输数据量 
	DMA_CHx->CCR |= 1<<0;          //开启DMA传输
}	  








