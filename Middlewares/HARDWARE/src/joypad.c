#include  "joypad.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//游戏手柄驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//初始化手柄接口.	 
void JOYPAD_Init(void)
{
 	RCC->APB2ENR|=1<<4;//先使能外设PORTC时钟    	
	GPIOC->CRL&=0XFFF00000;
	GPIOC->CRL|=0X00083338;     
	GPIOC->ODR|=0x1F<<0;  
}

//读取手柄按键值.
//FC手柄数据输出格式:
//每给一个脉冲,输出一位数据,输出顺序:
//A->B->SELECT->START->UP->DOWN->LEFT->RIGHT.
//总共8位,对于有C按钮的手柄,按下C其实就等于A+B同时按下.
//按下是0,松开是1.
//返回值:
//[0]:右
//[1]:左
//[2]:下
//[3]:上
//[4]:Start
//[5]:Select
//[6]:B
//[7]:A
u8 JOYPAD_Read(void)
{
	u8 temp=0;
	u8 t;
	JOYPAD_LAT=1;					//锁存当前状态
 	JOYPAD_LAT=0;
	for(t=0;t<8;t++)
	{
		temp<<=1;	 
		if(JOYPAD_DAT)temp|=0x01;	//LOAD之后，就得到第一个数据
		JOYPAD_CLK=1;			   	//每给一次脉冲，收到一个数据
 		JOYPAD_CLK=0;	
	}
	return temp;
}

u8 PS2_Wireless_JOYPAD_WriteRead(u8 cmd)
{
	u8 i=0;
	u8 value=0;
	u8 CMD=cmd;
	for(i=0;i<8;i++)
	{
		PS2_JOYPAD_CLOCK=1;
		if(CMD&0x01)PS2_JOYPAD_CMND=1;
		else PS2_JOYPAD_CMND=0;
		CMD>>=1;
		delay_us(10);
		PS2_JOYPAD_CLOCK=0;
		delay_us(15);
		value>>=1;
		if(PS2_JOYPAD_DATA)value|=0x80; //0000 0000
	}
	PS2_JOYPAD_CLOCK=1;
	return value;
}

//检测应答函数
//有应答信号返回0，无应答信号返回1
u8 PS2_Wireless_JOYPAD_ACK(void)
{
	u8 i=0;
	PS2_JOYPAD_CLOCK=1;
	delay_us(5);
	PS2_JOYPAD_CLOCK=0;
	while(i<100)
	{
		if(0==PS2_JOYPAD_ACK)break;
		i++;
	}
	PS2_JOYPAD_CLOCK=1;
	return i<100?0:1;
}

//读取数据函数
//成功返回0，失败返回1
u16 PS2_Wireless_JOYPAD_DATA(void)
{
	u8 PS2_JOYPAD_ID=0;
	u8 table[3]={0};
	//LEFT DOWN RGHT UP STRT X X SLCT       正方形 叉 圆形三角形 R1 L1 R2 L2
	u16 PS2_VALUE=0XFFFF;

	PS2_JOYPAD_ATT=0;//片选使能
	delay_us(10);

	PS2_Wireless_JOYPAD_WriteRead(PS2_JOYPAD_CMND_START);
	//if(1==PS2_Wireless_JOYPAD_ACK())return 1;
	//手柄ID号，我的是数字手柄返回0X41
	PS2_JOYPAD_ID=PS2_Wireless_JOYPAD_WriteRead(PS2_JOYPAD_CMND_DEMAND);
	printf("%d\n",PS2_JOYPAD_ID);
//	if(1==PS2_Wireless_JOYPAD_ACK())return 2;	
	table[0]=PS2_Wireless_JOYPAD_WriteRead(PS2_JOYPAD_CMND_NOP);//0x5a
//	if(1==PS2_Wireless_JOYPAD_ACK())return 3;
	table[1]=PS2_Wireless_JOYPAD_WriteRead(PS2_JOYPAD_CMND_NOP);//data1
//	if(1==PS2_Wireless_JOYPAD_ACK())return 4;	
	table[2]=PS2_Wireless_JOYPAD_WriteRead(PS2_JOYPAD_CMND_NOP);//data2

	PS2_VALUE=(table[1]<<8)|table[2];

	delay_us(10);
	PS2_JOYPAD_ATT=1;

	return PS2_VALUE;
}


u8 t=0;
u8  HAND;                       
u8  keybuf0;  //手柄按键编码存储单元
u8  keybuf1;
u8  RES[6]; 

u16 key_scan(void)//键扫描
{
    t=0;
    PS2_JOYPAD_ATT=0;         //主机读手柄先拉低ATT
    HAND=PS2_JOYPAD_CMND_START;     //主机发送开始命令
    psinout();     //0xff
    delay_us(10);
	
    HAND=PS2_JOYPAD_CMND_DEMAND;     //主机发送请求数据命令
    psinout();     //0x41：手柄返回请求应答信号
    delay_us(5);
    psout();       //0x5A
    delay_us(5);
    psout(); //keybuf0(空0xff)  //手柄返回按键编码第一字节
    delay_us(5);
    psout(); //keybuf1(空0xff)  //手柄返回按键编码第二字节
    keybuf0=RES[3];              
    keybuf1=RES[4];
	
    delay_us(10);
    PS2_JOYPAD_ATT=1;
	
	  return RES[3]<<8|RES[4];
}

void psout(void)//主机接收子程序            
{
    int j,k;
    unsigned char duf=0;
    j=1;
    for(k=0;k<=7;k++)       //逐位发送 
    {
       PS2_JOYPAD_CLOCK = 1;
       delay_us(10);
       PS2_JOYPAD_CLOCK = 0;
       delay_us(15);
       if(PS2_JOYPAD_DATA == 1)
         duf=duf+j;
       j=j*2;
       PS2_JOYPAD_CLOCK = 1;
     }
    RES[t++]=duf;
}

void psinout(void)//手柄发送子程序
{
    u8 buf,duf=0;
    u8 i,j=1;
    buf=HAND;
    for(i=0;i<=7;i++)     //逐位接收     
    {
     PS2_JOYPAD_CLOCK=1;
     delay_us(10);
     if(buf&0x01)
        PS2_JOYPAD_CMND = 1;
     else
        PS2_JOYPAD_CMND = 0;
     buf=buf>>1;
     PS2_JOYPAD_CLOCK = 0;
     delay_us(15);
     if(PS2_JOYPAD_DATA == 1)
        duf=duf+j;
     j=j*2;
     PS2_JOYPAD_CLOCK = 1;
    }
    RES[t++]=duf;
}

