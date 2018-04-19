#ifndef __DMA_H
#define __DMA_H

#include "stm32f1xx.h"
#include "cmsis_os.h"
#include "sys.h"


void MYDMA_uart_rx(DMA_Channel_TypeDef* DMA_CHx,uint32_t cpar,uint32_t cmar,uint16_t cndtr);
void MYDMA_uart_rx_conf(DMA_Channel_TypeDef* DMA_CHx,uint32_t cpar,uint32_t cmar,uint16_t cndtr);

void DMA_uart_tx(DMA_Channel_TypeDef* DMA_CHx,uint32_t cpar,uint32_t cmar,uint16_t cndtr);
void MYDMA_uart_tx_conf(DMA_Channel_TypeDef* DMA_CHx,uint32_t cpar,uint32_t cmar,uint16_t cndtr);

void Start_UART_DMA_Transmit(DMA_Channel_TypeDef* DMA_CHx,uint32_t len);

#endif