#include "line.h"
#include <stdio.h>

extern "C"
{
#include "sys.h"
#include "dma.h"
#include "led.h"
#include "timer.h"
#include "24l01.h"
#include "mpu6050.h"
#include "imu.h"
#include <math.h>
}
//******************************************************************************************************

u8	TxBuf[20]={0};
u8	RxBuf[20]={0};  

//******************************************************************************************************

u8	YM=0;              //ÓÍÃÅ±ä»¯ËÙ¶È¿ØÖÆ£¬²»ÕâÑù×öµÄ»°¿ìËÙ±ä»¯ÓÍÃÅÊ±ËÄÖá»áÊ§ËÙ·­×ª²¢GG

int	 speed0=0,speed1=0,speed2=0,speed3=0;           //µç»úËÙ¶È²ÎÊý
int	 PWM0=0,PWM1=0,PWM2=0,PWM3=0;	//¼ÓÔØÖÁPWMÄ£¿éµÄ²ÎÊý
int	 g_x=0,g_y=0,g_z=0;			//ÍÓÂÝÒÇ½ÃÕý²ÎÊý
float a_x,a_y;						//½Ç¶È½ÃÕý²ÎÊý

//*****************½Ç¶È²ÎÊý*************************************************

short aacx,aacy,aacz;		//¼ÓËÙ¶È´«¸ÐÆ÷Ô­Ê¼Êý¾Ý
short gyrox,gyroy,gyroz;	//ÍÓÂÝÒÇÔ­Ê¼Êý¾Ý

float  Angle_ax=0,Angle_ay=0,Angle_az=0;	//ÓÉ¼ÓËÙ¶È¼ÆËãµÄ¼ÓËÙ¶È(»¡¶ÈÖÆ)
float  Angle_gy=0,Angle_gx=0,Angle_gz=0;	//ÓÉ½ÇËÙ¶È¼ÆËãµÄ½ÇËÙÂÊ(½Ç¶ÈÖÆ)
//float  AngleAx=0,AngleAy=0;				//Èý½Çº¯Êý½âËã³öµÄÅ·À­½Ç
float  Angle=0,Angley=0;					//ËÄÔªÊý½âËã³öµÄÅ·À­½Ç
float  Anglezlate=0;				//ZÖáÏà¹Ø
float  Ax=0,Ay=0;					//¼ÓÈëÒ£¿ØÆ÷¿ØÖÆÁ¿ºóµÄ½Ç¶È

u8  lastR0=0,ZT=0; //ÉÏÒ»´ÎRxBuf[0]Êý¾Ý(RxBuf[0]Êý¾ÝÔÚ²»¶Ï±ä¶¯µÄ)   ×´Ì¬±êÊ¶


//****************×ËÌ¬´¦ÀíºÍPID*********************************************

float PID_Output;	//PID×îÖÕÊä³öÁ¿

float Last_Angle_gx=0;	//Íâ»·PIÊä³öÁ¿  ÉÏÒ»´ÎÍÓÂÝÒÇÊý¾Ý
float ERRORX_Out=0;	//Íâ»·P  Íâ»·I  Íâ»·Îó²î»ý·Ö
float ERRORX_In=0;	//ÄÚ»·P  ÄÚ»·I   ÄÚ»·D  ÄÚ»·Îó²î»ý·Ö

float Last_Angle_gy=0;
float ERRORY_Out=0;
float ERRORY_In=0;

float ERRORZ_Out=0;

#define	Out_XP	35.0f	//Íâ»·P
#define	Out_XI	0.01f	//Íâ»·I

#define	In_XP	0.3f	//ÄÚ»·P
#define	In_XI	0.01f	//ÄÚ»·I
#define	In_XD	9.0f	//ÄÚ»·D

#define	In_YP	In_XP
#define	In_YI	In_XI
#define	In_YD	In_XD

#define	Out_YP	Out_XP
#define	Out_YI	Out_XI

#define	ZP	5.0f
#define	ZD	4.0f	//×ÔÐý¿ØÖÆµÄP D

#define	ERR_MAX	500

u8	tp[16];

Line::Line(void)
{
    printf("Object is being created\n");
}

Line::~Line(void)
{
    printf("Line::~Line\n");
}

void Line::setLength( double len )
{
    YM = RxBuf[4];	//ÓÍÃÅ

    if(RxBuf[0] == lastR0)	//Èç¹ûRxBuf[0]µÄÊý¾ÝÃ»ÓÐÊÕµ½ ¼´Ê§Áª
    {
        if(++ZT >= 128)
        {
            ZT = 120;	//×´Ì¬±êÊ¶´óÓÚ128¼´1ÃëÃ»ÓÐÊÕµ½Êý¾Ý£¬Ê§¿Ø±£»¤
            RxBuf[1] = 128;
            RxBuf[2] = 128; //´¥·¢Ê§¿Ø±£»¤ ÓÍÃÅÎª1°ëÉÙÒ»µã£¬»ºÂýÏÂ½µ£¬¸©Ñöºá¹ö·½Ïò¶æ¹éÖÐ
            RxBuf[3] = 128;
            if(RxBuf[4] != 0)	RxBuf[4]--;
        }
    }
    else	ZT = 0;
    lastR0 = RxBuf[0];

//			MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,16,tp);
//			Read_MPU6050(tp);
    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//µÃµ½¼ÓËÙ¶È´«¸ÐÆ÷Êý¾Ý
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//µÃµ½ÍÓÂÝÒÇÊý¾Ý
    
    Angle_ax = (float)aacx / 8192.0;	//¼ÓËÙ¶È´¦Àí	½á¹ûµ¥Î»ÊÇ +- g
    Angle_ay = (float)aacy / 8192.0;	//×ª»»¹ØÏµ	8192 LSB/g, 1g¶ÔÓ¦¶ÁÊý8192
    Angle_az = (float)aacz / 8192.0;	//¼ÓËÙ¶ÈÁ¿³Ì +-4g/S
    Last_Angle_gx = Angle_gx;		//´¢´æÉÏÒ»´Î½ÇËÙ¶ÈÊý¾Ý
    Last_Angle_gy = Angle_gy;
    Angle_gx = (float)gyrox / 65.5;	//ÍÓÂÝÒÇ´¦Àí	½á¹ûµ¥Î»ÊÇ +-¶È
    Angle_gy = (float)gyroy / 65.5;	//ÍÓÂÝÒÇÁ¿³Ì +-500¶È/S, 1¶È/Ãë ¶ÔÓ¦¶ÁÊý 65.536
    Angle_gz = (float)gyroz / 65.5;	//×ª»»¹ØÏµ65.5 LSB/¶È


//*********************************** ËÄÔªÊý½âËã ***********************************
    IMUupdate(Angle_gx*0.0174533f, Angle_gy*0.0174533f, Angle_gz*0.0174533f, Angle_ax,Angle_ay,Angle_az);
    //0.174533ÎªPI/180 Ä¿µÄÊÇ½«½Ç¶È×ª»¡¶È	ËÄÔªÊý¼ÆËã³ö ¸©Ñö(XÖá): Angle, ºá¹ö(YÖá): Angley

    
//******************** Èý½Çº¯ÊýÖ±½Ó½âËãÒÔ¹©±È½ÏËÄÔªÊý½âËã¾«×¼¶È ********************
//			AngleAx = atan(Angle_ax/sqrt(Angle_ay*Angle_ay+Angle_az*Angle_az)) * 57.2957795f; //ºóÃæµÄÊý×ÖÊÇ180/PI Ä¿µÄÊÇ»¡¶È×ª½Ç¶È
//			AngleAy = atan(Angle_ay/sqrt(Angle_ax*Angle_ax+Angle_az*Angle_az)) * 57.2957795f;
    
//************** XÖáÖ¸Ïò ***********************************************************
    Ax = Angle - (((float)RxBuf[1] - 128) / 7) - a_x;	//½Ç¶È¿ØÖÆÁ¿¼ÓÔØÖÁ½Ç¶È,  Angle: µ±Ç°½Ç¶È, RxBuf[1]: ¸ø¶¨½Ç¶È, a_x: 0µã½Ç¶È
    
    if(YM > 20)	ERRORX_Out += Ax;	//Íâ»·»ý·Ö(ÓÍÃÅÐ¡ÓÚÄ³¸öÖµÊ±²»»ý·Ö)
    else		ERRORX_Out = 0;		//ÓÍÃÅÐ¡ÓÚ¶¨ÖµÊ±Çå³ý»ý·ÖÖµ

            if(ERRORX_Out >  ERR_MAX)	ERRORX_Out =  ERR_MAX;	//»ý·ÖÏÞ·ù
    else if(ERRORX_Out < -ERR_MAX)	ERRORX_Out = -ERR_MAX;	//»ý·ÖÏÞ·ù
    
    PID_Output = Ax*Out_XP + ERRORX_Out*Out_XI;	//Íâ»·PI
    
    if(YM > 20)	ERRORX_In += (Angle_gy - PID_Output);	//ÄÚ»·»ý·Ö(ÓÍÃÅÐ¡ÓÚÄ³¸öÖµÊ±²»»ý·Ö)
    else		ERRORX_In  = 0; //ÓÍÃÅÐ¡ÓÚ¶¨ÖµÊ±Çå³ý»ý·ÖÖµ

            if(ERRORX_In >  ERR_MAX)	ERRORX_In =  ERR_MAX;
    else if(ERRORX_In < -ERR_MAX)	ERRORX_In = -ERR_MAX;	//»ý·ÖÏÞ·ù
    
    PID_Output = (Angle_gy + PID_Output)*In_XP + ERRORX_In*In_XI + (Angle_gy - Last_Angle_gy)*In_XD;	//ÄÚ»·PID

    if(PID_Output >  1000)	PID_Output =  1000;  //Êä³öÁ¿ÏÞ·ù
    if(PID_Output < -1000)	PID_Output = -1000;
    
    speed0 = 0 - PID_Output, speed2 = 0 + PID_Output;

//**************YÖáÖ¸Ïò**************************************************
    Ay  = Angley + ((float)RxBuf[2] - 128) / 7 - a_y;			//½Ç¶È¿ØÖÆÁ¿¼ÓÔØÖÁ½Ç¶È
    
    if(YM > 20)		ERRORY_Out += Ay;				//Íâ»·»ý·Ö(ÓÍÃÅÐ¡ÓÚÄ³¸öÖµÊ±²»»ý·Ö)
    else			ERRORY_Out = 0;					//ÓÍÃÅÐ¡ÓÚ¶¨ÖµÊ±Çå³ý»ý·ÖÖµ
            if(ERRORY_Out >  ERR_MAX)	ERRORY_Out =  ERR_MAX;
    else if(ERRORY_Out < -ERR_MAX)	ERRORY_Out = -ERR_MAX;	//»ý·ÖÏÞ·ù
    
    PID_Output = Ay*Out_YP + ERRORY_Out*Out_YI;	//Íâ»·PI
    
    if(YM > 20)		ERRORY_In += (Angle_gx - PID_Output);	//ÄÚ»·»ý·Ö(ÓÍÃÅÐ¡ÓÚÄ³¸öÖµÊ±²»»ý·Ö)
    else			ERRORY_In = 0;							//ÓÍÃÅÐ¡ÓÚ¶¨ÖµÊ±Çå³ý»ý·ÖÖµ
            if(ERRORY_In >  ERR_MAX)	ERRORY_In =  ERR_MAX;
    else if(ERRORY_In < -ERR_MAX)	ERRORY_In = -ERR_MAX;	//»ý·ÖÏÞ·ù
    
    PID_Output = (Angle_gx + PID_Output)*In_YP + ERRORY_In*In_YI + (Angle_gx - Last_Angle_gx)*In_YD;	//ÄÚ»·PID
    
    if(PID_Output >  1000)	PID_Output =  1000;  //Êä³öÁ¿ÏÞ·ù
    if(PID_Output < -1000)	PID_Output = -1000;
    
    speed3 = 0 + PID_Output, speed1 = 0 - PID_Output;		//¼ÓÔØµ½ËÙ¶È²ÎÊý

//************** ZÖáÖ¸Ïò(ZÖáËæ±ãÀ²£¬×ÔÐý¿ØÖÆÃ»±ØÒªÉÏ´®¼¶PID) *****************************	
    Angle_gz = Angle_gz - ((float)RxBuf[3] - 128) * 1.0f;	//²Ù×÷Á¿

    if(YM > 20)		ERRORZ_Out += Angle_gz;
    else			ERRORZ_Out  = 0; 
    if(ERRORZ_Out >  500)	ERRORZ_Out =  500;
    else if(ERRORZ_Out < -500)	ERRORZ_Out = -500;	//»ý·ÖÏÞ·ù
    PID_Output = (Angle_gz)*ZP + ERRORZ_Out * 0.2f + (Angle_gz - Anglezlate) * ZD;

    Anglezlate = Angle_gz;
    speed0 = speed0 + PID_Output, speed2 = speed2 + PID_Output;
    speed1 = speed1 - PID_Output, speed3 = speed3 - PID_Output;

//**************½«ËÙ¶È²ÎÊý¼ÓÔØÖÁPWMÄ£¿é*************************************************	
    if(YM < 10)		PWM0 = 0,	PWM1 = 0,	PWM2 = 0,	PWM3 = 0;
    else
    {
        PWM0 = (int)YM*4 + speed0;
                if(PWM0 > 1000)	PWM0 = 1000;	//ËÙ¶È²ÎÊý¿ØÖÆ£¬·ÀÖ¹³¬¹ýPWM²ÎÊý·¶Î§0-1000
        else if(PWM0 < 0)		PWM0 = 0;

        PWM1 = (int)YM*4 + speed1;
                if(PWM1 > 1000)	PWM1 = 1000;
        else if(PWM1 < 0)		PWM1 = 0;

        PWM2 = (int)YM*4 + speed2;
                if(PWM2 > 1000)	PWM2 = 1000;
        else if(PWM2 < 0)		PWM2 = 0;

        PWM3 = (int)YM*4 + speed3;
                if(PWM3 > 1000)	PWM3 = 1000;
        else if(PWM3 < 0)		PWM3 = 0;
    }			
    
    TIM2->CCR2 = PWM0;//ÓÒÏÂ½Ç
    TIM2->CCR1 = PWM1;//×óÏÂ½Ç		
    TIM1->CCR4 = PWM2;//×óÉÏ½Ç
    TIM1->CCR1 = PWM3;//ÓÒÉÏ½Ç
}
 
double Line::getLength( void )
{
    return length;
}