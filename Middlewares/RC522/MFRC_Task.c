#include <stdio.h>
#include <string.h>
#include "MFRC_Task.h"
#include "MFRC522.h"

const unsigned char data1[16] = {0x12,0x34,0x56,0x78,0xED,0xCB,0xA9,0x87,0x12,0x34,0x56,0x78,0x01,0xFE,0x01,0xFE};
//M1卡的某一块写为如下格式，则该块为钱包，可接收扣款和充值命令
//4字节金额（低字节在前）＋4字节金额取反＋4字节金额＋1字节块地址＋1字节块地址取反＋1字节块地址＋1字节块地址取反 
const unsigned char data2[4]  = {0,0,0,0x01};
const unsigned char DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 


unsigned char g_ucTempbuf[20];                        
void delay1(unsigned int z)
{
	osDelay(z);
}  


void MFRC_main()
{    
    unsigned char status,i;
	unsigned int temp;
    InitializeSystem( );
    PcdReset();
    PcdAntennaOff(); 
    PcdAntennaOn(); 
    while ( 1 )
    {   
        status = PcdRequest(PICC_REQALL, g_ucTempbuf);//寻卡
        if (status != MI_OK)
        {    
            InitializeSystem( );
            PcdReset();
            PcdAntennaOff(); 
            PcdAntennaOn(); 
			continue;
        }
			     
        printf("card type:");
        for(i=0;i<2;i++)
        {
            temp=g_ucTempbuf[i];
            printf("%X",temp);        
        }
        printf("\n");
			
        status = PcdAnticoll(g_ucTempbuf);//防冲撞
        if (status != MI_OK)
        {    continue;    }

         
        ////////以下为超级终端打印出的内容////////////////////////
    
        printf("card ID:");	//超级终端显示,
        for(i=0;i<4;i++)
        {
            temp=g_ucTempbuf[i];
            printf("%X",temp); 
        }
        printf("\n");
        ///////////////////////////////////////////////////////////

        status = PcdSelect(g_ucTempbuf);//选定卡片
        if (status != MI_OK)
        {    continue;    }
         
        status = PcdAuthState(PICC_AUTHENT1A, 1, DefaultKey, g_ucTempbuf);//验证卡片密码
        if (status != MI_OK)
        {    continue;    }
         
        status = PcdWrite(1, data1);//写块
        if (status != MI_OK)
        {    continue;    }
        
        while(1)
		{
            status = PcdRequest(PICC_REQALL, g_ucTempbuf);//寻卡
            if (status != MI_OK)
            {   
                InitializeSystem();
                PcdReset();
                PcdAntennaOff(); 
                PcdAntennaOn(); 
                continue;
            }

            status = PcdAnticoll(g_ucTempbuf);//防冲撞
            if (status != MI_OK)
            {    continue;    }

            status = PcdSelect(g_ucTempbuf);//选定卡片
            if (status != MI_OK)
            {    continue;    }

            status = PcdAuthState(PICC_AUTHENT1A, 1, DefaultKey, g_ucTempbuf);//验证卡片密码
            if (status != MI_OK)
            {    continue;    }

            status = PcdValue(PICC_DECREMENT,1,data2);//扣款
            if (status != MI_OK)
            {    continue;    }
            
            status = PcdBakValue(1, 2);//块备份
            if (status != MI_OK)
            {    continue;    }
            
            status = PcdRead(2, g_ucTempbuf);//读块
            if (status != MI_OK)
            {    continue;    }

            printf("read block:");	//超级终端显示,
            for(i=0;i<16;i++)
            {
                temp=g_ucTempbuf[i];
                printf("%X",temp);
            }

            printf("\n");
            LED_GREEN = 0;
            printf("beeeeeeee\n");//刷卡声
            delay1(100);
            LED_GREEN = 1;
            delay1(100);
            LED_GREEN = 0;
            delay1(200);
            LED_GREEN = 1;				 		         
            PcdHalt();
		}
        delay1(200);
    }
}


/////////////////////////////////////////////////////////////////////
//系统初始化
/////////////////////////////////////////////////////////////////////

void InitializeSystem()
{
    RCC->APB2ENR |= 1<<3;    //使能PORTB时钟

	GPIOB->CRL &= 0X00000FFF;
	GPIOB->CRL |= 0X33333000;
    GPIOB->ODR |= 0x1f<<3;

    MF522_SDA = 1;
    MF522_SCK = 1;
    MF522_MOSI = 1;
    //gpio_set_direction(MF522_MISO, GPIO_MODE_INPUT);
    MF522_RST = 1;
    LED_GREEN = 0;
    //MF522_IRQ = 1;
}
