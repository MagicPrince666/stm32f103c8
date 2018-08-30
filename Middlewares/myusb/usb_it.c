#include "usb.h"

void USB_HP_CAN1_TX_IRQHandler(void)
{
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	u16 wIstr = GetISTR(),wInterrupt_Mask = GetCNTR();

  	if(wIstr & ISTR_CTR & wInterrupt_Mask)
  	{
			*(u16 *)ISTR &= ((u16)CLR_CTR);
    	CTR_CallBack();
  	}
  	if(wIstr & ISTR_RESET & wInterrupt_Mask)
  	{
    	*(u16 *)ISTR &= ((u16)CLR_RESET); 
    	RESET_CallBack();
  	}
	else if(wIstr & ISTR_DOVR & wInterrupt_Mask)
  	{
    	*(u16 *)ISTR &= ((u16)CLR_DOVR);
    	DOVR_CallBack();
  	}
	else if(wIstr & ISTR_ERR & wInterrupt_Mask)
	{
		*(u16 *)ISTR &= ((u16)CLR_ERR);
    	ERR_CallBack();
  	}
	else if(wIstr & ISTR_WKUP & wInterrupt_Mask)
	{
		*(u16 *)ISTR &= ((u16)CLR_WKUP);
    	WKUP_CallBack();
	}
	else if(wIstr & ISTR_SUSP & wInterrupt_Mask)//USB挂起
	{
   		SUSP_CallBack();
    	*(u16 *)ISTR &= ((u16)CLR_SUSP);
  	}
  	else if(wIstr & ISTR_SOF & wInterrupt_Mask)
  	{
    	*(u16 *)ISTR &= ((u16)CLR_SOF);
    	SOF_CallBack();
  	}
  	else if(wIstr & ISTR_ESOF & wInterrupt_Mask)
  	{
    	*(u16 *)ISTR &= ((u16)CLR_ESOF); //帧期望标志
    	ESOF_CallBack();
  	}
}
