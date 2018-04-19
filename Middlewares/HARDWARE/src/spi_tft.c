#include "spi_tft.h"
#include "spi.h"
#include "LCD.h"
#include "delay.h"

//写寄存器函数
//regval:寄存器值
void SPI_LCD_WR_REG(u8 regval)
{ 
	LCD_RS = 0;
	SPI1_ReadWriteByte(regval);	 
}
//写LCD数据
//data:要写入的值
void SPI_LCD_WR_DATA(u8 data)
{										    	   
	LCD_RS = 1;	 
    SPI1_ReadWriteByte(data);
}

void WriteComm(u16 CMD)
{			
	LCD_RS = 0;
	SPI1_ReadWriteByte((u8)CMD>>8);
    SPI1_ReadWriteByte((u8)CMD);
}

void WriteData(u16 tem_data)
{			
	LCD_RS = 1;
	SPI1_ReadWriteByte((u8)tem_data>>8);
    SPI1_ReadWriteByte((u8)tem_data);
}

/**********************************************
Lcd初始化函数
Initial condition  (DB0-15,RS,CSB,WRD,RDB,RESETB="L") 
***********************************************/
void Lcd_Initialize(void)
{
    unsigned int i = 0;
    RCC->APB2ENR|=1<<2|1<<4;    //使能PORTC时钟	   	 
	GPIOA->CRL&=0XFFF0FFFF; 
	GPIOA->CRL|=0X00030000;//PC.6 7 推挽输出   	 
    GPIOA->ODR|=1<<4;      //PC.6 7 输出高   	 
	GPIOC->CRL&=0XFF00FFFF; 
	GPIOC->CRL|=0X00330000;//PC.6 7 推挽输出   	 
    GPIOC->ODR|=3<<4;      //PC.6 7 输出高
    
	LCD_CS = 1;
    LCD_BL = 0;
    SPI1_Init();
    SPI1_SetSpeed(SPI_SPEED_2);

    SPI_LCD_WR_REG(0x01);
    delay_ms(100);
    LCD_CS = 0;
    SPI_LCD_WR_REG(0x01);
    delay_ms(100);
    SPI_LCD_WR_REG(0xB0);//{setc, [107], W, 0x000B0}
    SPI_LCD_WR_DATA(0x00);

    SPI_LCD_WR_REG(0xB3);
    SPI_LCD_WR_DATA(0x02);
    SPI_LCD_WR_DATA(0x00);
    SPI_LCD_WR_DATA(0x00);
    SPI_LCD_WR_DATA(0x00);

    SPI_LCD_WR_REG(0xB4);
    SPI_LCD_WR_DATA(0x00);

    SPI_LCD_WR_REG(0xC0);
    SPI_LCD_WR_DATA(0x13);//
    SPI_LCD_WR_DATA( 0x3B);//480
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x03);//02
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x01);
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x43);

    SPI_LCD_WR_REG( 0xC1);
    SPI_LCD_WR_DATA( 0x08);
    SPI_LCD_WR_DATA( 0x12);//CLOCK
    SPI_LCD_WR_DATA( 0x08);
    SPI_LCD_WR_DATA( 0x08);

    SPI_LCD_WR_REG( 0xC4);
    SPI_LCD_WR_DATA( 0x11);
    SPI_LCD_WR_DATA( 0x07);
    SPI_LCD_WR_DATA( 0x03);
    SPI_LCD_WR_DATA( 0x03);

    SPI_LCD_WR_REG( 0xC8);//GAMMA
    SPI_LCD_WR_DATA( 0x04);
    SPI_LCD_WR_DATA( 0x09);
    SPI_LCD_WR_DATA( 0x16);
    SPI_LCD_WR_DATA( 0x5A);
    SPI_LCD_WR_DATA( 0x02);
    SPI_LCD_WR_DATA( 0x0A);
    SPI_LCD_WR_DATA( 0x16);
    SPI_LCD_WR_DATA( 0x05);
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x32);
    SPI_LCD_WR_DATA( 0x05);
    SPI_LCD_WR_DATA( 0x16);
    SPI_LCD_WR_DATA( 0x0A);
    SPI_LCD_WR_DATA( 0x53);//43/55
    SPI_LCD_WR_DATA( 0x08);
    SPI_LCD_WR_DATA( 0x16);
    SPI_LCD_WR_DATA( 0x09);
    SPI_LCD_WR_DATA( 0x04);
    SPI_LCD_WR_DATA( 0x32);
    SPI_LCD_WR_DATA( 0x00);

    SPI_LCD_WR_REG( 0x2A);
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x01);
    SPI_LCD_WR_DATA( 0x3F);//320

    SPI_LCD_WR_REG( 0x2B);
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x01);
    SPI_LCD_WR_DATA( 0xDF);//480

    SPI_LCD_WR_REG( 0x35);
    SPI_LCD_WR_DATA( 0x00);

    SPI_LCD_WR_REG( 0x3A);
    SPI_LCD_WR_DATA( 0x55);

    SPI_LCD_WR_REG( 0x44);
    SPI_LCD_WR_DATA( 0x00);
    SPI_LCD_WR_DATA( 0x01);

    SPI_LCD_WR_REG( 0x2C);
    SPI_LCD_WR_REG( 0x11);

    delay_ms(150);

    SPI_LCD_WR_REG( 0xD0);
    SPI_LCD_WR_DATA( 0x07);
    SPI_LCD_WR_DATA( 0x07);
    SPI_LCD_WR_DATA( 0x1E);
    SPI_LCD_WR_DATA( 0x07);
    SPI_LCD_WR_DATA( 0x03);

    SPI_LCD_WR_REG( 0xD1);
    SPI_LCD_WR_DATA( 0x03);
    SPI_LCD_WR_DATA( 0x52);//VCM
    SPI_LCD_WR_DATA( 0x10);//VDV

    SPI_LCD_WR_REG( 0xD2);
    SPI_LCD_WR_DATA( 0x03);
    SPI_LCD_WR_DATA( 0x24);

    SPI_LCD_WR_REG(0xB0);//{setc, [107], W, 0x000B0}
    SPI_LCD_WR_DATA(0x00);//{setp, [104], W, 0x00000}

    SPI_LCD_WR_REG(0x29);
    delay_ms(10);
    SPI_LCD_WR_REG(0x36);
    SPI_LCD_WR_DATA(0x28);

    delay_us(1000);
    
    /////////////////////////////////////////////////////

    SPI_LCD_WR_REG(0x3A);//
    SPI_LCD_WR_DATA(0x55);//
    LCD_BL = 1;

    //WriteComm(0x36);//竖屏
    //WriteData(0x48);//
    BlockWrite(0,319,0,479);
    // 	while(!(SPI1->SR&0x0002));
    for(i=0;i<320*480;i++)
    {
        WriteData(WHITE);
    }
    test_color();
}

void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend) 
{
	//HX8357-C
	
    WriteComm(0x2A);
    WriteData(Xstart);
    WriteData(Xend);
  
    WriteComm(0x2B);
    WriteData(Ystart);
    WriteData(Yend);
	 
	 WriteComm(0x2C);
}

void test_color(void)
{
    unsigned short i,j;
    BlockWrite(0,319,0,479);
    for(i=0;i<480;i++)
    {	for(j=0;j<320;j++)
        {
            if(j<40)
            {
                WriteData(BLACK); 
            }
            else if(j<80)
            {
                WriteData(BLUE);
            }
            else if(j<120)
            {
                WriteData(BRED);
            }
            else if(j<160)
            {
                WriteData(GRED);
            }
            else if(j<200)
            {
                WriteData(RED);
            }
            else if(j<240)
            {
                WriteData(GREEN);
            }
            else if(j<280)
            {
                WriteData(YELLOW);
            }
            else if(j<320)
            {
                WriteData(BROWN);
            }
        }
    }
}

