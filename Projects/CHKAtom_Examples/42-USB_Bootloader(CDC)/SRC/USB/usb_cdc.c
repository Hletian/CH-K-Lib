#include "usb_cdc.h"
#include "usb.h"
#include "sys.h"
CDC_Line_Coding LineCoding;  //ʵ����LINE_CODEING �ṹ��

//CDC�ӿ� �����ж�
u8 USB_CDC_InterfaceReq_Handler(void)
{
	switch(Setup_Pkt->bRequest)
	{        
		case GET_LINE_CODING:
			USB_EP_IN_Transfer(EP0,(u8*)&LineCoding,7);
		break;
		case SET_LINE_CODING:
			if(BIT_CHK(EP0,gu8USB_Toogle_flags))//
			{
				BIT_CLR(EP0,gu8USB_Toogle_flags);
				EP_OUT_Transfer(EP0,(u8*)&LineCoding);
				USB_EP_IN_Transfer(EP0,0,0);       
			}
		break;
		case SET_CONTROL_LINE_STATE:
			USB_EP_IN_Transfer(EP0,0,0);//
			break;
		case LOADER_MODE:
          //  Buffer_Init(CDC_OUT_Data,CDC_BUFFER_SIZE);
         //   FLAG_SET(LOADER,gu8USB_Flags);
         //   CDC_OUT_Data[0]=0xFF;
		break;
       default:break;
    }
}

//USB CDC��������
void USB_CDCComTx(uint8_t *tx_buf,uint8_t len)
{
   USB_EP_IN_Transfer(EP2,tx_buf,len);
}

/*
*  LPLD_USB_VirtualCom_Rx
*    ��USB CDC��ģʽ����ɴ���ģʽ���ú����Ǵ��ڽ��պ���
*    ����
*      |__*rx_buf ָ���û����ݴ洢�������ڴ�����յ�������
*    ����ֵ���������ݵĳ��ȣ����ֽ�Ϊ��λ
*/
u8 USB_CDCComRx(u8 *rx_buf)
{
  u8 len;
  u8 temp=0;
  u8 *pu8EPBuffer;
  if(BIT_CHK(EP_OUT,gu8USB_Flags))// ��������ݵ���
  {
		len = USB_EP_OUT_SizeCheck(EP_OUT); 
		temp = len;
		pu8EPBuffer = CDC_OUTPointer; //ָ���Ӧ�� ������
		//��ʼ����
		while(len--) *rx_buf++=*pu8EPBuffer++;  
	
    usbEP_Reset(EP_OUT);
    usbSIE_CONTROL(EP_OUT);
    BIT_CLR(EP_OUT,gu8USB_Flags);
		}
  return temp;
}


static u32 LWordSwap(u32 u32DataSwap)
{
    u32 u32Temp;
    u32Temp= (u32DataSwap & 0xFF000000) >> 24;
    u32Temp+=(u32DataSwap & 0xFF0000)   >> 8;
    u32Temp+=(u32DataSwap & 0xFF00)     << 8;
    u32Temp+=(u32DataSwap & 0xFF)       << 24;
    return(u32Temp);    
}
void USB_CDC_Init(void)
{
  /*��ʼ��CDC��ͨѶ�Ĳ���*/
  LineCoding.DTERate=LWordSwap(115200);
  LineCoding.CharFormat=0;
  LineCoding.ParityType=0;
  LineCoding.Databits=0x08;
  /*��ʼ��CDC�����ݻ�����*/
 // Buffer_Init(CDC_OUT_Data,CDC_BUFFER_SIZE);
}
