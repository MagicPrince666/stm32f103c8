/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
//#include "usb_device.h"
#include "usb.h"
#include "uart_process.h"
#include "led.h"
#include "usart.h"
#include "usart2.h"
#include "stmflash.h"
#include "adc.h"
#include "MFRC_Task.h"
#include "pstwo.h"


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel6;
osThreadId defaultTaskHandle;
osThreadId Uart_RecvBuffTaskHandle;

/* Private function prototypes -----------------------------------------------*/

void StartDefaultTask(void const * argument);

int main(void)
{
  Stm32_Clock_Init(9);
  JTAG_Set(SWD_ENABLE);
  LED_Init();
  // uart_init(72,115200);
  PS2_Init();
  uart5_init(32,115200);
  printf("STM32 F103RET6\n");

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
//  osThreadDef(Uart_RecvBuffTask, Uart_RecvBuffTask, osPriorityNormal, 0, 256);
//  Uart_RecvBuffTaskHandle = osThreadCreate(osThread(Uart_RecvBuffTask),NULL);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);


  /* Start scheduler */
  osKernelStart();
  
  while (1)
  {
  }

}

u8 Buf[22];
void USB_SendData(u8 *str)
{
	u8 len = 0;
	while(*(str+len))
		len++;
	SetEpTxBuf(Buf,22,EP2);
	SetEpTxBuf(str,len,EP2);
	SetEP2TxCount(22);
	SetEpTxStatus(EP2,EP_TX_VALID);
}

//#define MESSAGE_ADDR 0x08007800
#define MESSAGE_ADDR 0x08007C00
/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  u8 i = 0;
/*
  char read[31] = {0};
  STMFLASH_Read((uint16_t)MESSAGE_ADDR,(uint8_t *)read,31);//从eeprom读取MCU信息

  if(read[30] != 0x55)//还未写入信息
  {
    sprintf(read,"STM32 version 0.1\n");
    read[30] = 0x55;
    STMFLASH_Write((uint16_t)MESSAGE_ADDR,(uint8_t *)read,31);
    printf("write message\n");
    STMFLASH_Read((uint16_t)MESSAGE_ADDR,(uint8_t *)read,31);
  }
  read[30] = 0;
  printf("%s",read);
*/
  //Adc_Init();
  //MX_USB_DEVICE_Init();
  //USB_Config();

  uint8_t key = 0;
  uint8_t lx,ly,rx,ry;
  PS2_SetInit();
  printf("STM32 F103RET6 PS2_SetInit\n");

  while (1) {
    key = PS2_DataKey();

    if(key != 0) {
        if(key > 0) {
            printf("key = ", key );
        }

        if(key == 12) {
            PS2_Vibration(0xFF,0x00);
        } else if(key == 11) {
            PS2_Vibration(0x00,0xFF);
        } else {
            PS2_Vibration(0x00,0x00);
        }
    }

    lx = PS2_AnologData(PSS_LX);
    ly = PS2_AnologData(PSS_LY);
    rx = PS2_AnologData(PSS_RX);
    ry = PS2_AnologData(PSS_RY);

    if(1) {
        printf("x.left = ", lx );
        printf("x.right = ", ly );
        printf("y.left = ", rx );
        printf("y.right = ", ry );
    }
    LED0 = !LED0;
    osDelay(500);//任务延时 会影响系统调度
  }

#if 0
  float power=5.1;
  u8 str[64];
  u16 len = 0;

  for(;;)
  {
    //LED0 = !LED0;
    //LED1 = !LED1;
    Clr_CTR_Rx(EP1);
    GetEpRxBuf(str,EP1);
    if(str[0])
    {
      //len = GetEP1TxCount();
      //str[len] = 0;
      printf("%s\n",str);
      //str[0] = 0;
      memset(str,0,sizeof(str));
      SetEpRxStatus(EP1, EP_RX_VALID);
    }

  //  LED0 = !LED0;
  //  LED1 = !LED1;
    LED2 = !LED2;
    LED3 = !LED3;
    osDelay(500);
    
    i++;
    // if(i > 0xf)
    // {
    //   GPIOB->ODR &= 0x0000FFFF;
    //   GPIOB->ODR |= i<<12 & 0x00FF;
    //   i = 0;
    // }
    //printf("rest uart transmit %d\r\n",i);
    if(i%5 == 0)
    {
      // power=(Get_Adc(8)/1242.0)*2;
      // if(power<3.8)
      // {
      //   LED2=!LED2;
      //   LED3=!LED3;
      // }
      //USB_SendData("STM32 send buff\n");
      //printf("send buf to host\n");			
      //printf("x: %.2f  y: %.2f  z: %.2f\n",Angle_ax,Angle_ay,Angle_az);
      //printf("task1 get adc:%d\n",Get_Adc(ADC_CH9));
    }
    
  }
#endif
}

void _Error_Handler(char * file, int line)
{
  while(1) 
  {
    printf("file %s on line %d\n", file, line);//error message
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
