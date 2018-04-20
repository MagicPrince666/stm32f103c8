#include "ringbuffer.h"
#include "uart_process.h"
#include "line.h"

extern "C"
{
#include "dma.h"
#include "led.h"
#include "timer.h"
#include "24l01.h"
#include "mpu6050.h"
#include "stmflash.h"
}

/*
osThreadId Uart_RecvBuffTaskHandle;
osThreadDef(Uart_RecvBuffTask, Uart_RecvBuffTask, osPriorityNormal, 0, 400);
Uart_RecvBuffTaskHandle = osThreadCreate(osThread(Uart_RecvBuffTask),NULL);
*/


struct 
{
  uint8_t recvbuf[Recv1_LEN + 1];
  uint8_t flag;
}RECV1={{0},0};

extern u8 RxBuf[20];
uint8_t readbuf[Recv1_LEN + 1];

uint32_t index1 = 0;

void DMA1_Channel5_IRQHandler(void)
{
  index1 = DMA1->ISR & DMACHN5_STA;//获取中断状态
  //index &=0xF00;//只关心通道5的状态

  if(index1 & DMACHN5_ERR)//传输出错
  {
    printf("uart1 transfer error\n");
  }

  if(index1 & DMACHN5_COM)//接收到一个完整的数据包
  {
    DMA1->IFCR |= DMACHN5_IRQ;//清除通道5所有中断标志

    if(RECV1.flag)//交换缓存
    {   
      RECV1.flag = 0;
      //ring.write(rbuf,RECV1.recvbuf,Recv1_LEN);

      Start_UART_DMA_Transmit(UART1_RXDMA_CHN,Recv1_LEN);

      printf("%s\n",RECV1.recvbuf);  //处理第二缓存数据    
    }
  }
}

extern cycle_buffer* buffer;
/* uart数据接收任务 */
void Uart_RecvBuffTask(void const * argument)
{
    int recv_cnt = 0;//记录接收记数
    int i = 0;

    
    MYDMA_uart_rx_conf(UART1_RXDMA_CHN,(uint32_t)&USART1->DR,(uint32_t)(RECV1.recvbuf),Recv1_LEN);

    RingBuffer ring;


    TIM1_PWM_Init(1000,71);
    TIM2_PWM_Init(1000,71);

    PWM1_VAL = 0;
    PWM2_VAL = 0;
    PWM3_VAL = 0;
    PWM4_VAL = 0;

    int res = 0;
    NRF24L01_Init();    	
	while(NRF24L01_Check())		
	{
 		res++;
		if(res>5)return ;
	}	
	NRF24L01_RX_Mode(); 
    RxBuf[1] = 128;
	RxBuf[2] = 128;
	RxBuf[3] = 128;
	RxBuf[4] = 0;

    //MPU_Init();
    //IAPRead();
    
    LED0 = 0;
    LED1 = 1;

    Line line;
    //line.setLength(6.2); 
    //printf("Length of line : %f\n",line.getLength());
    for(;;)
    {  
        recv_cnt = UART1_DMA_CNT;  //接收计数
        if(recv_cnt)//如果有收到数据 recv_cnt不为0
        {
            recv_cnt = UART1_DMA_CNT;  //重新读取接收计数
            /* 使用第一缓存区 */
            if(RECV1.flag == 0)
            {       
                RECV1.flag = 1;                                     //切换至第二缓存标志
                RECV1.recvbuf[recv_cnt] = 0;
                ring.write(buffer,RECV1.recvbuf,recv_cnt);
                Start_UART_DMA_Transmit(UART1_RXDMA_CHN,Recv1_LEN); //开始接收
                printf("uart1:%s\n",RECV1.recvbuf);
            }
        }

       if(NRF24L01_RxPacket(RxBuf)==0)//
		{
			if(RxBuf[5] == 1)	//
			{
				RxBuf[5] = 0;
				if(RxBuf[4] < 20)
				{
					IAP_Gyro();
					PWM(40,40,40,40);
					osDelay(10); //
					PWM(0,0,0,0);	
				}
			}

			if(RxBuf[6] == 1)	//
			{
				RxBuf[6] = 0;
				if(RxBuf[4] < 20)
				{
					STMFLASH_ErasePage(STM32_FLASH_BASE); //
					PWM(40,40,40,40); 
					osDelay(10); //
					PWM(0,0,0,0);	
				}
			}
	    }	

        i++;
        if(i >= 30)
        {
            i = 0;
            LED0 = ~LED0;
            LED1 = ~LED1;
            
            //unsigned char readbuf[64];
            if(ring.read(buffer,readbuf,DEFAULT_BUF_SIZE))
            {
                readbuf[Recv1_LEN] = 0;
                printf("%s",readbuf);
            }
            
        }
        //PWM4_VAL = i*30;
        osDelay(10);//任务延时 会影响系统调度
    }
}