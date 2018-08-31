#include "usb.h"
#include "stdio.h"
#include "string.h"
#include "sys.h"
#include "led.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

extern u32 time;
u16 SaveRxStatus,SaveTxStatus;
u8 ControlStatus;
u8 EP0_RxBuf[64];
u8 EP_Buf[64];
u8 Flag,Rec;
u8 Max_Lun[2] = {0,0};

u8 Device_Descriptor[18] =
{
	0x12,	/* 设备描述符长度 */
	0x01, 	/* 描述符类型(0x01:设备描述符) */
	0x00,	/*bcdUSB */
	0x02,
	0x00,	/* 设备类 */
	0x00,	/* 子设备类 */
	0x00,	/* 设备协议 */
	0x40,	/* 端点0包的最大长度(字节) */
	0x01,	/* 厂商ID(0x0483) */
	0x3a,
	0x01,	/* 产品ID(0x5750) */
	0x3a,
	0x00,	/*bcdDevice rel. 2.00*/
	0x02,
	0x01,	/* 描述厂商的字符串索引 */
	0x02,	/* 描述产品信息的字符串索引 */
	0x03,	/* 描述设备序列号信息的字符串索引 */
	0x01	/* 可能配置的数目(配置描述符数目) */
}; /* 设备描述符 */

u8 Config_Descriptor[41] =
{
	0x09,	/* 配置描述符长度 */
	0x02,	/* 描述符类型(0x02:配置描述符) */
	41,		/* 总的描述符长度 */
	0x00,
	0x01,	/* 接口数目 */
	0x01,	/* bConfigurationValue: Configuration value */
	0x00,	/* iConfiguration: Index of string descriptor describing the configuration*/
	0xC0,	/* bmAttributes: Bus powered (Bus powered: 7th bit, Self Powered: 6th bit, Remote wakeup: 5th bit, reserved: 4..0 bits )*/
	0x32,	/* 最大使用电流大小(2mA为单位) */
	/************** 接口描述符****************/
	0x09,	/* 接口描述符长度 */
	0x04,	/* 描述符类型(0x04:接口描述符) */
	0x00,	/* 接口号(从0开始) */
	0x00,	/* 可选设置的索引值 */
	0x02,	/* 使用端点数目(0表示只是用端点0) */
	0x03,	/* 接口类(0x03:HID类) */
	0x00,	/* 子接口类 : 1 = BOOT, 0 = NO BOOT */
	0x00,	/* 接口协议 : 0 = 无, 1 = 键盘, 2 = 鼠标 */
	0x00,	/* 描述接口的字符串索引(0表示不使用) */
	/******************** HID描述符 ********************/
	0x09,	/* HID描述符大小 */
	0x21,	/* 描述符类型(0x21: HID) */
	0x10,	/* bcdHID: HID Class Spec release number */
	0x01,	
	0x00,	/* bCountryCode: Hardware target country */
	0x01,	/* bNumDescriptors: Number of HID class descriptors to follow */
	0x22,	/* bDescriptorType */
	33,		/* 报告描述符总长度 */
	0x00,
	/******************** 端点2描述符 ******************/
	0x07,	/* 描述符长度 */
	0x05,	/* 描述符类型(0x05:端点) */
	0x82,	/* 指明端点号和方向(0~3位为端点号,第7位= 1为IN端点,第7位= 0为OUT端点，其他位保留)*/               
	0x03,	/* 端点类型(0x00->控制,0x01->同步,0x02->批量,0x03->中断) */
	0x40,	/* 发送或者接收的最大包大小 */
	0x00,
	0xff,	/* 中断端点主机查询间隔(单位为1ms)，同步端点此字节为0x01,批量、控制端点不使用此字节 */
	/******************** 端点1描述符 ******************/
	0x07,	/* 描述符长度 */
	0x05,	/* 描述符类型(0x05:端点) */
	0x01,	/* 指明端点号和方向(0~3位为端点号,第7位= 1为IN端点,第7位= 0为OUT端点，其他位保留) */
	0x03,	/* 端点类型(0x00->控制,0x01->同步,0x02->批量,0x03->中断) */
	0x40,	/* 发送或者接收的最大包大小 */
	0x00,
	0xff,	/* 中断端点主机查询间隔(单位为1ms)，同步端点此字节为0x01,批量、控制端点不使用此字节 */
}; /* CustomHID_ConfigDescriptor */

u8 HID_ReportDescriptor[33] = /*报告描述符*/
{ 
//#ifdef 0 
0x05, 0x8c, /* USAGE_PAGE (ST Page) */ 
0x09, 0x01, /* USAGE (Demo Kit) */ 
0xa1, 0x01, /* COLLECTION (Application) */ 
/* 6 */ 

// The Input report 
0x09,0x03, // USAGE ID - Vendor defined 
0x15,0x00, // LOGICAL_MINIMUM (0) 
0x26,0x00, 0xFF, // LOGICAL_MAXIMUM (255) 
0x75,0x08, // REPORT_SIZE (8) 
0x95,0x16, // REPORT_COUNT (20) 
0x81,0x02, // INPUT (Data,Var,Abs) 
//19
// The Output report 
0x09,0x04, // USAGE ID - Vendor defined 
0x15,0x00, // LOGICAL_MINIMUM (0) 
0x26,0x00,0xFF, // LOGICAL_MAXIMUM (255) 
0x75,0x08, // REPORT_SIZE (8) 
0x95,0x16, // REPORT_COUNT (20) 
0x91,0x02, // OUTPUT (Data,Var,Abs) 
//32
0xc0 /* END_COLLECTION */ 
//#endif 
}; /* CustomHID_ReportDescriptor */ 
u8 Interface_Descriptor[9] = {0x09,0x04,0x00,0x00,0x02,0x08,0x06,0x50,0x01};
u8 Ep1_Descriptor[7] = {0x07,0x05,0x81,0x02,0x40,0x00,0x00};
u8 Ep2_Descriptor[7] = {0x07,0x05,0x02,0x02,0x40,0x00,0x00};
u8 StringLangID[4] = {0x04,0x03,0x09,0x04};/*语言ID*/
u8 StringVendor[38] = {0x26,0x03,'X',0,'A',0,'G',0,'i',0,'c',0,'r',0,'o',0,'e',0,'l',0,'e',0,'c',0,'t',0,'r',0,'o',0,'n',0,'i',0,'c',0,'s',0};/*厂商描述符*/
u8 StringProduct[18] = {0x12,0x03,'U', 0, 'S', 0, 'B', 0,' ',0, '3', 0, 'a', 0, '0', 0, '1',0};/*产品描述符*/
u8 StringSerial[16] = {0x10,0x03,'X', 0, 'A', 0, 'G', 0, '3', 0, 'a', 0, '0', 0, '1', 0}; /*序列号描述符*/
u8 StringInterface[16] ={0x10, 0x03, 'X', 0, 'A', 0, 'G', 0, ' ', 0, 'h', 0, 'i', 0, 'd', 0}; /*接口描述符*/
void RESET_CallBack(void) /*复位中断处理*/
{
	Flag = 0;
	Rec = 0;
	SetDADDR(DADDR_EF);
	ControlStatus = 0;
	SetBTABLE(BASEADDR_BTABLE);//设置缓冲区描述表偏移量(这里设为了0)
	SetDeviceAddr(0); /*设置设备地址设为0*/

	SetEpTxStatus(EP0,EP_TX_NAK);
	SetEpRxStatus(EP0,EP_RX_VALID);
	SetEpRxStatus(EP1,EP_RX_VALID);
	SetEpTxStatus(EP2,EP_TX_NAK);

	SetEpType(EP0,EP_CONTROL);   /*设置端点0为控制类型*/
	SetEpType(EP1,EP_INTERRUPT); /*设置端点1为中断类型*/
	SetEpType(EP2,EP_INTERRUPT); /*设置端点2为中断类型*/

	SetEpAddr(EP0,0);      /*设置端点0端点号*/
	SetEpAddr(EP1,1);      /*设置端点1端点号*/
	SetEpAddr(EP2,2);      /*设置端点2端点号*/

	/*缓冲区描述表*/
	*((u16 *)(PMAAddr)) =  0x0080;  /*端点0发送缓冲区偏移*/
	*((u16 *)(PMAAddr+2*2)) =	 0x0000;
	*((u16 *)(PMAAddr+4*2)) = 0x0040;/*端点0接收缓冲区偏移*/
	*((u16 *)(PMAAddr+6*2)) =	 0x8400;
	*((u16 *)(PMAAddr+16)) =  0x0e0;/*端点1发送缓冲区偏移*/
	*((u16 *)(PMAAddr+16+2*2)) =	 0x0000;
	*((u16 *)(PMAAddr+16+4*2)) = 0x00c0;/*端点1接收缓冲区偏移*/
	*((u16 *)(PMAAddr+16+6*2)) =	 0x8000;
	*((u16 *)(PMAAddr+32)) =  0x0120;/*端点2发送缓冲区偏移*/
	*((u16 *)(PMAAddr+32+2*2)) =	 0x0000;
	*((u16 *)(PMAAddr+32+4*2)) = 0x0100;/*端点2接收缓冲区偏移*/
	*((u16 *)(PMAAddr+32+6*2)) =	 0x8000;
}
void SetEpTxBuf(u8 *str,u16 num,u8 ep) /* 复制数据到端点缓冲区 ，STM32读取缓冲区数据时需要将缓冲区描述表里的偏移量乘以2*/
{
	u32 at,eps;
	u16 i;
	if(ep == 0)
	{
		at = EP0_TXADDR;
		eps	= EP0_PACKETSIZE;
	}
	else if(ep == 1)
	{
		at = EP1_TXADDR;
		eps	= EP1_PACKETSIZE;
	}
	else if(ep == 2)
	{
		at = EP2_TXADDR;
		eps	= EP2_PACKETSIZE;
	}
	else
		return;
	for(i=0;i<(num+1)/2 && i<eps/2;i++,str+=2)
		*((u16 *)(at+i*4)) = *((u16 *)str);
}

void GetEpRxBuf(u8 *str,u8 ep)  /* 从端点x缓冲区复制数据 */
{
	u32 arc,ar,eps;
	u16 i;
	if(ep == 0)
	{
		arc = EP0_RXCOUNT;
		ar = EP0_RXADDR;
		eps	= EP0_PACKETSIZE;
	}
	else if(ep == 1)
	{
		arc = EP1_RXCOUNT;
		ar = EP1_RXADDR;
		eps	= EP1_PACKETSIZE;
	}
	else if(ep == 2)
	{
		arc = EP2_RXCOUNT;
		ar = EP2_RXADDR;
		eps	= EP2_PACKETSIZE;
	}
	else
		return;
	for(i=0;i<((*(u16 *)arc & 0x3ff) + 1)/2 && i<eps/2;i++,str+=2)
		*((u16 *)str) = *((u16 *)(ar+i*4));
	*((u8 *)str) = 0x00;
}

void DOVR_CallBack(void)
{
	printf("DOVR\r\n");
}

void ERR_CallBack(void) /*USB错误中断*/
{
	printf("ERR\r\n");
}

void WKUP_CallBack(void)
{
	printf("WKUP\r\n");
}

void SUSP_CallBack(void)
{
	printf("SUSP\r\n");
}

void SOF_CallBack(void)
{
	printf("SOF\r\n");
}

void ESOF_CallBack(void)
{
	printf("ESOF\r\n");
}

void CTR_CallBack(void) /*完成一次正确传输中断*/
{
	SETUP_DATA *data;
	u16 EpNum,EpReg;
	EpNum = (*(u16 *)ISTR & 0x000f);
	if(EpNum == 0) /*如果是端点0完成一次正确的传输*/
	{
		SaveRxStatus = GetEPRxStatus(EP0);
		SaveTxStatus = GetEPTxStatus(EP0);
		SetEpTxStatus(EP0,EP_TX_NAK);
		SetEpRxStatus(EP0,EP_RX_NAK);
		if(*(u16 *)ISTR & ISTR_DIR)
		{
			if((*(u16 *)EP0REG) & EP_SETUP)
			{
				Clr_CTR_Rx(EP0);
				GetEpRxBuf(EP0_RxBuf,EP0);
				data = 	(SETUP_DATA *)&EP0_RxBuf[0];
				if(data->bRequest == GET_DESCRIPTOR )
				{
					if((data->bmRequestType & 0x7f) == (STANDARD_REQUEST | DEVICE_RECIPIENT))
					{
						if(data->wValue.b.LSB == DEVICE_DESCRIPTOR)
						{
							SetEpTxBuf(Device_Descriptor,18,EP0);
							SetEP0TxCount(18);
						}
						else if(data->wValue.b.LSB == STRING_DESCRIPTOR)
						{
							if(data->wValue.b.HSB == 0)
							{
								SetEpTxBuf(StringLangID,4,EP0);
								SetEP0TxCount(4);
							}
							else if(data->wValue.b.HSB == 1)
							{
								SetEpTxBuf(StringVendor,38,EP0);
								SetEP0TxCount(38);
							}
							else if(data->wValue.b.HSB == 2)
							{
								SetEpTxBuf(StringProduct,18,EP0);
								SetEP0TxCount(18);
							}
							else if(data->wValue.b.HSB == 3)
							{
								SetEpTxBuf(StringSerial,16,EP0);
								SetEP0TxCount(16);
							}
							else if(data->wValue.b.HSB == 4)
							{
								SetEpTxBuf(StringInterface,16,EP0);
								SetEP0TxCount(16);
							}
						}
						else if(data->wValue.b.LSB == CONFIG_DESCRIPTOR)
						{
							SetEpTxBuf(Config_Descriptor,41,EP0);
							SetEP0TxCount(41);
						}
						else
						{
								printf("UnSurpport_DESCRIPTOR\r\n");
						}
					}
					else if((data->bmRequestType & 0x7f) == (STANDARD_REQUEST | INTERFACE_RECIPIENT))
					{
						if (data->wValue.b.LSB == REPORT_DESCRIPTOR)
						{
							SetEpTxBuf(HID_ReportDescriptor,33,EP0);
							SetEP0TxCount(33);
						}
						else if (data->wValue.b.LSB == HID_DESCRIPTOR_TYPE)
						{
							SetEpTxBuf(Config_Descriptor + 0x12,9,EP0);
							SetEP0TxCount(9);
						}
					}
					if(GetEP0TxCount() > data->wLength.w)
						SetEP0TxCount(data->wLength.w);
					ControlStatus = WAIT_DATA_IN;
					SaveTxStatus = EP_TX_VALID;
					SaveRxStatus = EP_RX_VALID;
				}
				else if(data->bRequest == SET_ADDRESS)
				{
					ControlStatus = WAIT_STATUS_IN;
					ControlStatus |= WAIT_SET_ADDRESS;
					SetEP0TxCount(0);
					SaveTxStatus = EP_TX_VALID;
				}
				else if(data->bRequest == GET_MAX_LUN)
				{
					SetEpTxBuf(Max_Lun,1,EP0);
					SetEP0TxCount(1);
					ControlStatus = WAIT_DATA_IN;
					SaveTxStatus = EP_TX_VALID;
					SaveRxStatus = EP_RX_VALID;
				}
				else if(data->bRequest == GET_CONFIGURATION)
				{
					printf("GET_CONFIGURATION\r\n");
				}
				else if(data->bRequest == SET_CONFIGURATION)
				{
					ControlStatus = WAIT_STATUS_IN;
					SetEP0TxCount(0);
					SaveTxStatus = EP_TX_VALID;
				}
				else if(data->bRequest == CLEAR_FEATURE)
				{
					ControlStatus = WAIT_STATUS_IN;
					SetEP0TxCount(0);
					SaveTxStatus = EP_TX_VALID;
					if((data->bmRequestType & 0x1f) == DEVICE_RECIPIENT)
					{
						printf("CLEAR_FEATURE(DEVICE)\r\n");
					}
					else if((data->bmRequestType & 0x1f) == INTERFACE_RECIPIENT)
					{
						printf("CLEAR_FEATURE(INTERFACE)\r\n");
					}
					else if((data->bmRequestType & 0x1f) == ENDPOINT_RECIPIENT)
					{
						printf("CLEAR_FEATURE(ENDPOINT)\r\n");
					}
					else if((data->bmRequestType & 0x1f) == OTHER_RECIPIENT)
					{
						printf("CLEAR_FEATURE(OTHER)\r\n");
					}
					else
					{
						printf("CLEAR_FEATURE(UnSurpport)\r\n");
					}
				}
				else if(data->bRequest == SET_FEATURE)
				{
					ControlStatus = WAIT_STATUS_IN;
					SetEP0TxCount(0);
					SaveTxStatus = EP_TX_VALID;
				}
				else if((data->bmRequestType & 0x7f) == (CLASS_REQUEST | INTERFACE_RECIPIENT))
				{
					if(data->bRequest == GET_REPORT)
					{
						printf("GET_REPORT\r\n");			
					}
					else if(data->bRequest == GET_IDLE)
					{
						printf("GET_IDLE\r\n");
					}
					else if(data->bRequest == GET_PROTOCOL)
					{
						printf("GET_PROTOCOL\r\n");
					}
					else if(data->bRequest == SET_REPORT)
					{
						printf("SET_REPORT\r\n");
					}
					else if(data->bRequest == SET_IDLE)
					{
						printf("SET_IDLE\r\n");
					}
					else if(data->bRequest == SET_PROTOCOL)
					{
						printf("SET_PROTOCOL\r\n");
					}
					else
					{
						printf("SET_CONFIGURATION\r\n");
					}
				}
				else
				{
					printf("UnSupport\r\n");
				}
			}
			else if((*(u16 *)EP0REG) & EP_CTR_RX) /*OUT传输完成*/
			{
				Clr_CTR_Rx(EP0);
				if(ControlStatus & WAIT_STATUS_OUT)
				{
					SaveRxStatus = EP_RX_STALL;
					SaveTxStatus = EP_TX_STALL;
					ControlStatus = STALLED;					
				}
				else if(ControlStatus & WAIT_DATA_OUT)
				{
					SaveTxStatus = EP_TX_VALID;
					SaveRxStatus = EP_RX_STALL;
					ControlStatus = WAIT_STATUS_IN;
				}
			}
		}
		else
		{
			if((*(u16 *)EP0REG) & EP_CTR_TX)/*IN传输完成*/
			{
				Clr_CTR_Tx(EP0);
				if(ControlStatus & WAIT_STATUS_IN)
				{
					if(ControlStatus & WAIT_SET_ADDRESS)
						SetDeviceAddr(EP0_RxBuf[2]);
					SaveRxStatus = EP_RX_STALL;
					SaveTxStatus = EP_TX_STALL;
					ControlStatus = STALLED;					
				}
				else if(ControlStatus & WAIT_DATA_IN)
				{
					ControlStatus = WAIT_STATUS_OUT;
					SaveRxStatus = EP_RX_VALID;
					SaveTxStatus = EP_TX_STALL;
				}
			}
		}
		SetEpTxStatus(EP0,SaveTxStatus);
		SetEpRxStatus(EP0,SaveRxStatus);
		
		return;
	}
	else
	{
		EpReg = GetEpReg(EpNum);/*得到端点号*/
		if(EpReg & EP_CTR_RX)
		{
			Clr_CTR_Rx(EpNum);
			GetEpRxBuf(EP_Buf,EpNum);
			if(strcmp((char *)EP_Buf,"End") == 0)
				Flag = 0;
			else if(strcmp((char *)EP_Buf,"Start") == 0)
				Flag = 1;
			else if(strcmp((char *)EP_Buf,"Turn On LED1") == 0)
				LED0=0;
			else if(strcmp((char *)EP_Buf,"Turn On LED2") == 0)
				LED1=0;
			else if(strcmp((char *)EP_Buf,"Turn Off LED1") == 0)
				LED0=1;
			else if(strcmp((char *)EP_Buf,"Turn Off LED2") == 0)
				LED1=1;
			SetEpRxStatus(EpNum,EP_RX_VALID);
		}
		if(EpReg & EP_CTR_TX)
		{
			Rec = 1;
			Clr_CTR_Tx(EpNum);
		}
	}
}
void USB_Config(void)
{
	// NVIC_InitTypeDef NVIC_InitStructure;  

	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
	// RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);	
		

	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
				  			
	// NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	    //USB低优先级中断请求
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//抢占优先级 1
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//子优先级为1
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);
				  
	// NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;			//USB高优先级中断请求
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//抢占优先级 1
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//子优先级为0
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure);	 

	RCC->APB1ENR |= (1 << 23);            /* enable clock for USB */
	MY_NVIC_Init(1,1,USB_LP_CAN1_RX0_IRQn,1);//组2，最低优先级 
	MY_NVIC_Init(1,0,USB_HP_CAN1_TX_IRQn,1);//组2，最低优先级 
	//USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
  	//USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID);
  	//USBD_Start(&hUsbDeviceFS);

	SetCNTR(0xF400);
}
void SetEpTxStatus(u8 EpNum,u16 Status)
{
	u16 t,p;
	t = (*(u16 *)(EP0REG + EpNum));
	p = t;
	if((t & 0x20) != (Status & 0x20))
		p |= 0x20;
	else
		p &= 0xffdf;
	if((t & 0x10) != (Status & 0x10))
		p |= 0x10;
	else
		p &= 0xffef;
	p &= 0x8fbf;
	(*(u16 *)(EP0REG + EpNum)) = p;
}
void SetEpRxStatus(u8 EpNum,u16 Status)
{
	u16 t,p;
	t = (*(u16 *)(EP0REG + EpNum));
	p = t;
	if((t & 0x2000) != (Status & 0x2000))
		p |= 0x2000;
	else
		p &= 0xdfff ;
	if((t & 0x1000) != (Status & 0x1000))
		p |= 0x1000;
	else
		p &= 0xefff;
	p &= 0xbf8f;
	(*(u16 *)(EP0REG + EpNum)) = p;
}
