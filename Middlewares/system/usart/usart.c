#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////
// 如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h" //FreeRTOS使用
#endif

// gnu下用此函数重定向printf
int _write(int fd, char *pBuffer, int size)
{
    for (int i = 0; i < size; i++) {
        while ((USART1->SR & 0X40) == 0)
            ;                        // 等待上一次串口数据发送完成
        USART1->DR = (u8)pBuffer[i]; // 写DR,串口1将发送数据
    }
    return size;
}

// end
//////////////////////////////////////////////////////////////////

#if EN_USART1_RX // 如果使能了接收
// 串口1中断服务程序
// 注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.
// 接收状态
// bit15，	接收完成标志
// bit14，	接收到0x0d
// bit13~0，	接收到的有效字节数目
u16 USART_RX_STA = 0; // 接收状态标记

void USART1_IRQHandler(void)
{
    u8 res;

    if (USART1->SR & (1 << 5)) // 接收到数据
    {
        res = USART1->DR;
        if ((USART_RX_STA & 0x8000) == 0) // 接收未完成
        {
            if (USART_RX_STA & 0x4000) // 接收到了0x0d
            {
                if (res != 0x0a)
                    USART_RX_STA = 0; // 接收错误,重新开始
                else
                    USART_RX_STA |= 0x8000; // 接收完成了
            } else {                         // 还没收到0X0D
                if (res == 0x0d)
                    USART_RX_STA |= 0x4000;
                else {
                    USART_RX_BUF[USART_RX_STA & 0X3FFF] = res;
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0; // 接收数据错误,重新开始接收
                }
            }
        }
    }
}
#endif
// 初始化IO 串口1
// pclk2:PCLK2时钟频率(Mhz)
// bound:波特率
void uart_init(u32 pclk2, u32 bound)
{
    float temp;
    u16 mantissa;
    u16 fraction;
    temp     = (float)(pclk2 * 1000000) / (bound * 16); // 得到USARTDIV
    mantissa = temp;                                    // 得到整数部分
    fraction = (temp - mantissa) * 16;                  // 得到小数部分
    mantissa <<= 4;
    mantissa += fraction;
#if 1
    // PA9 - TX PA10 - RX
    RCC->APB2ENR |= 1 << 2;   // 使能PORTA口时钟
    GPIOA->CRH &= 0XFFFFF00F; // IO状态设置
    GPIOA->CRH |= 0X000008B0; // IO状态设置
#else
    // PC12 - UART5_TX PD2 - UART5_RX
    RCC->APB2ENR |= 1 << 4 | 1 << 5; // 使能PORTC D口时钟
    GPIOC->CRH &= 0XFFF0FFFF;        // IO状态设置
    GPIOC->CRH |= 0X000B0000;        // IO状态设置
    GPIOD->CRL &= 0XFFFFF0FF;        // IO状态设置
    GPIOD->CRL |= 0X00000800;        // IO状态设置
#endif
    RCC->APB2ENR |= 1 << 14;     // 使能串口时钟
    RCC->APB2RSTR |= 1 << 14;    // 复位串口1
    RCC->APB2RSTR &= ~(1 << 14); // 停止复位
    // 波特率设置
    USART1->BRR = mantissa; // 波特率设置
    USART1->CR1 |= 0X200C;  // 1位停止,无校验位.

    USART1->CR3 |= 1 << 6; // 使能串口1DMA接收

#if EN_USART1_RX // 如果使能了接收
    // 使能接收中断
    USART1->CR1 |= 1 << 5;              // 接收缓冲区非空中断使能
    MY_NVIC_Init(3, 3, USART1_IRQn, 2); // 组2，最低优先级
#endif
}
