#ifndef __USART2_H
#define __USART2_H 
#include "sys.h"
#include "stdio.h"	 

#define USART2_MAX_RECV_LEN		64					//最大接收缓存字节数
#define USART2_MAX_SEND_LEN		64					//最大发送缓存字节数
#define USART2_RX_EN 			1					//0,不接收;1,接收.

extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 		//接收缓冲,最大USART3_MAX_RECV_LEN字节
extern u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		//发送缓冲,最大USART3_MAX_SEND_LEN字节
extern vu16 USART2_RX_STA;   						//接收数据状态

void usart2_init(u32 pclk1,u32 bound);
void u2_printf(char* fmt,...);


#endif	   
