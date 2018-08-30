#include "sys.h"

#define MAX_TIMES 0x1
#define RegBase (0x40005C00L)
#define PMAAddr (0x40006000L)

#define USB_DEVICE_CLASS_RESERVED           0x00
#define USB_DEVICE_CLASS_AUDIO              0x01
#define USB_DEVICE_CLASS_COMMUNICATIONS     0x02
#define USB_DEVICE_CLASS_HUMAN_INTERFACE    0x03
#define USB_DEVICE_CLASS_MONITOR            0x04
#define USB_DEVICE_CLASS_PHYSICAL_INTERFACE 0x05
#define USB_DEVICE_CLASS_POWER              0x06
#define USB_DEVICE_CLASS_PRINTER            0x07
#define USB_DEVICE_CLASS_STORAGE            0x08
#define USB_DEVICE_CLASS_HUB                0x09
#define USB_DEVICE_CLASS_VENDOR_SPECIFIC    0xFF

#define EP0REG  ((volatile unsigned *)(RegBase))		 /*	端点0寄存器地址 */
#define EP1REG  ((volatile unsigned *)(RegBase + 0x04))	 /*	端点1寄存器地址 */
#define EP2REG  ((volatile unsigned *)(RegBase + 0x08))	 /*	端点2寄存器地址 */
#define EP3REG  ((volatile unsigned *)(RegBase + 0x0C))	 /*	端点3寄存器地址 */
#define EP4REG  ((volatile unsigned *)(RegBase + 0x10))	 /*	端点4寄存器地址 */
#define EP5REG  ((volatile unsigned *)(RegBase + 0x14))	 /*	端点5寄存器地址 */
#define EP6REG  ((volatile unsigned *)(RegBase + 0x18))	 /*	端点6寄存器地址 */
#define EP7REG  ((volatile unsigned *)(RegBase + 0x1C))	 /*	端点7寄存器地址 */
#define CNTR    ((volatile unsigned *)(RegBase + 0x40))	 /* 控制寄存器地址 */
#define ISTR    ((volatile unsigned *)(RegBase + 0x44))	 /* 中断状态寄存器地址 */
#define FNR     ((volatile unsigned *)(RegBase + 0x48))	 /* 帧编号寄存器地址 */
#define DADDR   ((volatile unsigned *)(RegBase + 0x4C))	 /*	设备地址寄存器地址 */
#define BTABLE  ((volatile unsigned *)(RegBase + 0x50))	 /*	分组缓冲区描述表寄存器地址 */

#define EP0   ((u8)0)
#define EP1   ((u8)1)
#define EP2   ((u8)2)
#define EP3   ((u8)3)
#define EP4   ((u8)4)
#define EP5   ((u8)5)
#define EP6   ((u8)6)
#define EP7   ((u8)7)

#define ISTR_CTR    (0x8000)
#define ISTR_DOVR   (0x4000)
#define ISTR_ERR    (0x2000)
#define ISTR_WKUP   (0x1000)
#define ISTR_SUSP   (0x0800)
#define ISTR_RESET  (0x0400)
#define ISTR_SOF    (0x0200)
#define ISTR_ESOF   (0x0100)
#define ISTR_DIR    (0x0010)
#define ISTR_EP_ID  (0x000F)

#define CLR_CTR    (~ISTR_CTR)   /* clear Correct TRansfer bit */
#define CLR_DOVR   (~ISTR_DOVR)  /* clear DMA OVeR/underrun bit*/
#define CLR_ERR    (~ISTR_ERR)   /* clear ERRor bit */
#define CLR_WKUP   (~ISTR_WKUP)  /* clear WaKe UP bit     */
#define CLR_SUSP   (~ISTR_SUSP)  /* clear SUSPend bit     */
#define CLR_RESET  (~ISTR_RESET) /* clear RESET bit      */
#define CLR_SOF    (~ISTR_SOF)   /* clear Start Of Frame bit   */
#define CLR_ESOF   (~ISTR_ESOF)  /* clear Expected Start Of Frame bit */

#define CNTR_CTRM   (0x8000) /* Correct TRansfer Mask */
#define CNTR_DOVRM  (0x4000) /* DMA OVeR/underrun Mask */
#define CNTR_ERRM   (0x2000) /* ERRor Mask */
#define CNTR_WKUPM  (0x1000) /* WaKe UP Mask */
#define CNTR_SUSPM  (0x0800) /* SUSPend Mask */
#define CNTR_RESETM (0x0400) /* RESET Mask   */
#define CNTR_SOFM   (0x0200) /* Start Of Frame Mask */
#define CNTR_ESOFM  (0x0100) /* Expected Start Of Frame Mask */
#define CNTR_RESUME (0x0010) /* RESUME request */
#define CNTR_FSUSP  (0x0008) /* Force SUSPend */
#define CNTR_LPMODE (0x0004) /* Low-power MODE */
#define CNTR_PDWN   (0x0002) /* Power DoWN */
#define CNTR_FRES   (0x0001) /* Force USB RESet */

#define FNR_RXDP (0x8000) /* D+状态位 */
#define FNR_RXDM (0x4000) /* D-状态位 */
#define FNR_LCK  (0x2000) /* 锁定位 */
#define FNR_LSOF (0x1800) /* 帧首丢失标志 */
#define FNR_FN  (0x07FF) /* 帧编号寄存器 */

#define DADDR_EF (0x80)
#define DADDR_ADD (0x7F)

#define EP_CTR_RX      (0x8000) /* EndPoint Correct TRansfer RX */
#define EP_DTOG_RX     (0x4000) /* EndPoint Data TOGGLE RX */
#define EPRX_STAT      (0x3000) /* EndPoint RX STATus bit field */
#define EP_SETUP       (0x0800) /* EndPoint SETUP */
#define EP_T_FIELD     (0x0600) /* EndPoint TYPE */
#define EP_KIND        (0x0100) /* EndPoint KIND */
#define EP_CTR_TX      (0x0080) /* EndPoint Correct TRansfer TX */
#define EP_DTOG_TX     (0x0040) /* EndPoint Data TOGGLE TX */
#define EPTX_STAT      (0x0030) /* EndPoint TX STATus bit field */
#define EPADDR   (0x000F) /* EndPoint ADDRess FIELD */

#define EPADDR_FIELD   (0x000F) /* EndPoint ADDRess FIELD */
#define EPREG_MASK     (EP_CTR_RX|EP_SETUP|EP_T_FIELD|EP_KIND|EP_CTR_TX|EPADDR_FIELD)

/* 端点类型 */
#define EP_TYPE_MASK   (0x0600) /* EndPoint TYPE Mask */
#define EP_BULK        (0x0000) /* 批量端点 */
#define EP_CONTROL     (0x0200) /* 控制端点 */
#define EP_ISOCHRONOUS (0x0400) /* 同步端点 */
#define EP_INTERRUPT   (0x0600) /* 中断端点 */
#define EP_T_MASK      (~EP_T_FIELD & EPREG_MASK)

#define EPKIND_MASK    (~EP_KIND & EPREG_MASK)

/* 接收状态码 */
#define EP_TX_DIS      (0x0000) /* EndPoint TX DISabled */
#define EP_TX_STALL    (0x0010) /* EndPoint TX STALLed */
#define EP_TX_NAK      (0x0020) /* EndPoint TX NAKed */
#define EP_TX_VALID    (0x0030) /* EndPoint TX VALID */
#define EPTX_DTOG1     (0x0010) /* EndPoint TX Data TOGgle bit1 */
#define EPTX_DTOG2     (0x0020) /* EndPoint TX Data TOGgle bit2 */
#define EPTX_DTOGMASK  (EPTX_STAT|EPREG_MASK)

/* 发送状态码 */
#define EP_RX_DIS      (0x0000) /* EndPoint RX DISabled */
#define EP_RX_STALL    (0x1000) /* EndPoint RX STALLed */
#define EP_RX_NAK      (0x2000) /* EndPoint RX NAKed */
#define EP_RX_VALID    (0x3000) /* EndPoint RX VALID */
#define EPRX_DTOG1     (0x1000) /* EndPoint RX Data TOGgle bit1 */
#define EPRX_DTOG2     (0x2000) /* EndPoint RX Data TOGgle bit1 */
#define EPRX_DTOGMASK  (EPRX_STAT|EPREG_MASK)

struct Device_Desciption_
{
	u16 a;
};	
#define BASEADDR_BTABLE        0x0000

// ENP0
#define EP0_PACKETSIZE      0x0040
#define EP0_RXADDR          (0x40006080)
#define EP0_TXADDR          (0x40006100)
#define EP0_TXCOUNT		(0x40006004)
#define EP0_RXCOUNT		(0x4000600C)

#define EP1_PACKETSIZE      0x0020
#define EP1_RXADDR          (0x40006180)
#define EP1_TXADDR          (0x400061c0)
#define EP1_TXCADDR		(0x40006014)
#define EP1_RXCOUNT		(0x4000601C)

#define EP2_PACKETSIZE      0x0020
#define EP2_RXADDR          (0x40006200)
#define EP2_TXADDR          (0x40006240)
#define EP2_TXCOUNT		(0x40006024)
#define EP2_RXCOUNT		(0x4000602C)

#define SetCNTR(wRegValue)  (*(u16 *)CNTR   = (u16)wRegValue)
#define SetISTR(wRegValue)  (*(u16 *)ISTR   = (u16)wRegValue)
#define SetDADDR(wRegValue) (*(u16 *)DADDR  = (u16)wRegValue)
#define SetBTABLE(wRegValue) (*(u16 *)BTABLE = (u16)(wRegValue & 0xFFF8))
#define GetEpReg(bEpNum) (*(u16 *)(EP0REG + bEpNum))
#define GetCNTR()   (*(u16 *)CNTR)
#define GetISTR()   (*(u16 *)ISTR)
#define GetFNR()    (*(u16 *)FNR)
#define GetDADDR()  (*(u16 *)DADDR)
#define GetBTABLE() (*(u16 *)BTABLE)

#define SetEP0TxCount(cou)	*((u16 *)EP0_TXCOUNT) = ((u16)cou & 0x03FF)
#define GetEP0TxCount()	(*((u16 *)EP0_TXCOUNT) & 0x03FF)
#define SetEP1TxCount(cou)	*((u16 *)EP1_TXCOUNT) = ((u16)cou & 0x03FF)
#define GetEP1TxCount()	(*((u16 *)EP1_TXCOUNT) & 0x03FF)
#define SetEP2TxCount(cou)	*((u16 *)EP2_TXCOUNT) = ((u16)cou & 0x03FF)
#define GetEP2TxCount()	(*((u16 *)EP2_TXCOUNT) & 0x03FF)

#define GetEP0RxCount()	(*((u16 *)EP0_RXCOUNT)) & 0x03FF 

#define Clr_CTR_Rx(bEpNum) (*(u16 *)(EP0REG + bEpNum)&= 0xf8f)
#define Clr_CTR_Tx(bEpNum) (*(u16 *)(EP0REG + bEpNum) &= 0x8f0f)
#define GetEPRxStatus(bEpNum) (*((u16 *)(EP0REG + bEpNum)) & EPRX_STAT) //读取端点x寄存器只需要EP0REG+bEpNum就可以了，不需要将bEpNum * 4
#define GetEPTxStatus(bEpNum) (*((u16 *)(EP0REG + bEpNum)) & EPTX_STAT)
#define SetEpType(bEpNum,Type) (*((u16 *)(EP0REG + bEpNum)) = (GetEpReg(bEpNum) & 0x8f8f | Type))
#define SetDeviceAddr(addr) (*(u8 *)DADDR = addr | 0x80)
#define SetEpAddr(bEpNum,Addr) (*((u16 *)(EP0REG + bEpNum)) = (GetEpReg(bEpNum) & 0x8f80 | Addr))

#define GET_STATUS 0
#define CLEAR_FEATURE 1
#define RESERVED1 2
#define SET_FEATURE 3
#define RESERVED2 4
#define SET_ADDRESS 5
#define GET_DESCRIPTOR 6
#define SET_DESCRIPTOR 7
#define GET_CONFIGURATION 8
#define SET_CONFIGURATION 9
#define GET_INTERFACE 10
#define SET_INTERFACE 11
#define SYNCH_FRAME 12  /* Total number of Standard request */

#define GET_MAX_LUN 0xfe

#define DEVICE_DESCRIPTOR 1
#define CONFIG_DESCRIPTOR 2
#define STRING_DESCRIPTOR 3
#define INTERFACE_DESCRIPTOR 4
#define ENDPOINT_DESCRIPTOR	5

#define STALLED 0x00
#define WAIT_STATUS_IN 0x01
#define WAIT_STATUS_OUT 0x02
#define WAIT_DATA_IN 0x04
#define WAIT_DATA_OUT 0x08
#define WAIT_SET_ADDRESS 0x10
#define RESET 0x20

#define STANDARD_REQUEST  0x00  /* Standard request */
#define CLASS_REQUEST     0x20  /* Class request */
#define VENDOR_REQUEST    0x40  /* Vendor request */

#define DEVICE_RECIPIENT  0x00     /* Recipient device */
#define INTERFACE_RECIPIENT  0x01  /* Recipient interface */
#define ENDPOINT_RECIPIENT  0x02   /* Recipient endpoint */
#define OTHER_RECIPIENT 0x03

#define REPORT_DESCRIPTOR	0x22
#define HID_DESCRIPTOR_TYPE	0x21

#define GET_REPORT	1
#define GET_IDLE	2
#define GET_PROTOCOL	3
#define SET_REPORT	9
#define SET_IDLE	10
#define SET_PROTOCOL	11

typedef union _2BYTE_
{
	u16 w;
	struct
	{
		u8 HSB;
		u8 LSB;
	}b;
}_2BYTE;

typedef struct _SETUP_DATA
{
	u8 bmRequestType;
	u8 bRequest;
	_2BYTE wValue;
	_2BYTE wIndex;
	_2BYTE wLength;
}SETUP_DATA;

void RESET_CallBack(void);
void DOVR_CallBack(void);
void ERR_CallBack(void);
void WKUP_CallBack(void);
void SUSP_CallBack(void);
void SOF_CallBack(void);
void ESOF_CallBack(void);
void CTR_CallBack(void);
void USB_Config(void);
void SetEpTxBuf(u8 *str,u16 num,u8 ep);
void GetEpRxBuf(u8 *str,u8 ep);
void SetEpTxStatus(u8 EpNum,u16 Status);
void SetEpRxStatus(u8 EpNum,u16 Status);
