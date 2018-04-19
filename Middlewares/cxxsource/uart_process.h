#ifndef __UART_PROCESS_H
#define __UART_PROCESS_H

#include "sys.h"

#define BAUD_RATE 115200

//配置双缓冲大小 对于接收大量数据的串口 该缓冲不能太小 至少要在下一包接收完成之前处理完上一包的数据
#define Recv1_LEN 128

/* DMA接收通道 */
#define UART1_RXDMA_CHN     DMA1_Channel5
#define UART2_RXDMA_CHN     DMA1_Channel6

/* DMA发送通道 */
#define UART1_TXDMA_CHN     DMA1_Channel4
#define UART2_TXDMA_CHN     DMA1_Channel7

/* DMA接收计数 */
#define UART1_DMA_CNT   (Recv1_LEN - DMA1_Channel5->CNDTR)
#define UART2_DMA_CNT   (Recv1_LEN - DMA1_Channel6->CNDTR)

/* DMA 通道状态 */
#define DMACHN5_STA 0x000F0000
#define DMACHN6_STA 0x00F00000

/* DMA 传输错误标志*/
#define DMACHN5_ERR (1<<19)
#define DMACHN6_ERR (1<<23)

/* DMA 传输完成标志*/
#define DMACHN5_COM (1<<17)
#define DMACHN6_COM (1<<21)

/* DMA 中断标志*/
#define DMACHN5_IRQ (1<<16)
#define DMACHN6_IRQ (1<<20)

#define UART_RX1    //外接uart1


#ifdef __cplusplus
extern "C" {
#endif

void Uart_RecvBuffTask(void const * argument);

#ifdef __cplusplus
}
#endif

#endif
