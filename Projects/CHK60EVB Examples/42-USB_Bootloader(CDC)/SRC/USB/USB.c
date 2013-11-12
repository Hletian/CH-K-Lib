#include "USB.h"
#include "USB_Desc.h"
#include "usb_hid.h"
#include "usb_cdc.h"
//BDT��������������ֻҪ����512�ֽڣ��������ڻ�������ַ�ĵ�9λ����Ϊ0����ȻUSBģ���޷�ʹ�ã�
//��������1024�����ҳ���ַ��9λΪ0�Ļ�������ַ��Ϊ�׵�ַ��
__align(512) tBDT tBDTtable[16];													//�ڲ�SRAM�ڴ��

//�����˵�����ݻ�����
u8 gu8EP0_OUT_ODD_Buffer[EP0_SIZE];
u8 gu8EP0_OUT_EVEN_Buffer[EP0_SIZE];
u8 gu8EP0_IN_ODD_Buffer[EP0_SIZE];
u8 gu8EP0_IN_EVEN_Buffer[EP0_SIZE];
u8 gu8EP1_OUT_ODD_Buffer[EP1_SIZE];
u8 gu8EP1_OUT_EVEN_Buffer[EP1_SIZE];
u8 gu8EP1_IN_ODD_Buffer[EP1_SIZE];
u8 gu8EP1_IN_EVEN_Buffer[EP1_SIZE];
u8 gu8EP2_OUT_ODD_Buffer[EP2_SIZE];
u8 gu8EP2_OUT_EVEN_Buffer[EP2_SIZE];
u8 gu8EP2_IN_ODD_Buffer[EP2_SIZE];
u8 gu8EP2_IN_EVEN_Buffer[EP2_SIZE];
u8 gu8EP3_OUT_ODD_Buffer[EP3_SIZE];
u8 gu8EP3_OUT_EVEN_Buffer[EP3_SIZE];
u8 gu8EP3_IN_ODD_Buffer[EP3_SIZE];
u8 gu8EP3_IN_EVEN_Buffer[EP3_SIZE];
//ָ������������ĵ�ַָ��
u8 *BufferPointer[]=
{
    gu8EP0_OUT_ODD_Buffer,
    gu8EP0_OUT_EVEN_Buffer,
    gu8EP0_IN_ODD_Buffer,
    gu8EP0_IN_EVEN_Buffer,
    gu8EP1_OUT_ODD_Buffer,
    gu8EP1_OUT_EVEN_Buffer,
    gu8EP1_IN_ODD_Buffer,
    gu8EP1_IN_EVEN_Buffer,
    gu8EP2_OUT_ODD_Buffer,
    gu8EP2_OUT_EVEN_Buffer,
    gu8EP2_IN_ODD_Buffer,
    gu8EP2_IN_EVEN_Buffer,
    gu8EP3_OUT_ODD_Buffer,
    gu8EP3_OUT_EVEN_Buffer,
    gu8EP3_IN_ODD_Buffer,
    gu8EP3_IN_EVEN_Buffer
};
//ÿ����������С
const u8 cEP_Size[]=
{
    EP0_SIZE,    
    EP0_SIZE,    
    EP0_SIZE,    
    EP0_SIZE,    
    EP1_SIZE,    
    EP1_SIZE,    
    EP1_SIZE,    
    EP1_SIZE,    
    EP2_SIZE,    
    EP2_SIZE,    
    EP2_SIZE,    
    EP2_SIZE,    
    EP3_SIZE,
    EP3_SIZE,
    EP3_SIZE,
    EP3_SIZE
};
//StringDesc������ָ��
const u8* String_Table[4]=
{
    String_Descriptor0,
    String_Descriptor1,
    String_Descriptor2,
    String_Descriptor3
};
//SETUP������ֻ�ܸ�DATA0
//�˵�IN(2)״̬
u8 vEP2State = kUDATA1;
//�˵�OUT(3)״̬
u8 vEP3State = kUDATA0;

//����˵�OUT������
u8 RecDataBuff[EP3_SIZE];
//���ݳ���
u32 RecDataLength;

//USBģ���ڲ�ȫ�ֱ���
u8 gu8USBClearFlags;      //���ڷ��ͺͽ��� ����Ҫ��ע
u8 *pu8IN_DataPointer;    //���ڷ��ͺͽ��� ����Ҫ��ע
u8 gu8IN_Counter;         //���ڷ��ͺͽ��� ����Ҫ��ע
u8 gu8USB_Toogle_flags;   //���ڷ��ͺͽ��� ����Ҫ��ע
u8 gu8USB_State;          //����USB�˵�0��״̬
tUSB_Setup *Setup_Pkt;    //ָ��˵�0OUT�����׵�ַ  
u8 gu8USB_Flags;        //USB������ɱ�־λ


/*
* USB_DeviceEnumed
*   �жϵ�ǰCDC�豸�Ƿ�����ö�� ������ʽ
*/
u8 USB_IsDeviceEnumed(void)
{
	if(gu8USB_State == uENUMERATED) return 1;
	return 0;
}
/*
* EP_IN_Transfer
*   �˵�IN���ͣ��ڲ�ͬUSBģʽ��IN�ĺ��岻ͬ
*     ��USB����ģʽ�£�USB host��  IN��ʾ  USB Host���� USB Device�����ݰ�
*     ��USB�豸ģʽ�£�USB Device��IN��ʾ  USB Host�� USB Device�������ݰ�
*   ��ǰ��USB Deviceģʽ�£��ú�����������Device�豸�������ݰ�
*/
void USB_EP_IN_Transfer(u8 u8EP,u8 *pu8DataPointer,u8 u8DataSize)
{
  u8 *pu8EPBuffer;
  u8 u8EPSize;     //�˵�����ݳ���
  u16 u16Lenght=0;    
  u8 u8EndPointFlag;    
  /*������ǰ��������λ��*/
  u8EndPointFlag=u8EP;
  if(u8EP)   //������Ƕ˵�0,
    u8EP=(u8)(u8EP<<2);
  u8EP+=2; //EPֵ��2
  pu8EPBuffer=BufferPointer[u8EP];   //����EP��BUFFER��ַ��pu8EPBuffer
  
  if(BIT_CHK(fIN,gu8USBClearFlags)) //���gu8USBClearFlags = 1
  {
      pu8IN_DataPointer = pu8DataPointer; //���û����ݴ洢�����׵�ַ��pu8IN_DataPointer
      gu8IN_Counter = u8DataSize;         //���û����ݳ��ȸ�gu8IN_Counter

      u16Lenght=(Setup_Pkt->wLength_h<<8)+Setup_Pkt->wLength_l ;//��setup���ݵĳ��ȸ�u16Lenght
      if((u16Lenght < u8DataSize) && (u8EP==2)) //��������͵����ݳ��ȴ���setup���趨�ĳ��� ͬʱ �˵�ֵ == 2
      {
          gu8IN_Counter=Setup_Pkt->wLength_l; //ֻ����setup�еĵ�8λ����
      }
  }
  /*��鷢�ͳ���*/
  if(gu8IN_Counter > cEP_Size[u8EP]) //����������ݰ��ĳ��� ����32�ֽ�ʱ
  {
      u8EPSize = cEP_Size[u8EP];     //����ʱ�˵�ĳ��������ڶ˵��Ĭ�ϳ���
      gu8IN_Counter-=cEP_Size[u8EP]; //�����ݰ��ĳ��ȼ���EP_Size
      BIT_CLR(fIN,gu8USBClearFlags);//��gu8USBClearFlags����
  }
  else
  { 
      u8EPSize = gu8IN_Counter;      //���С��
      gu8IN_Counter=0;            
      BIT_SET(fIN,gu8USBClearFlags);//��gu8USBClearFlags��һ
  }
  /*���û����������ֵ���Ƶ�EP ��������׼������*/
  tBDTtable[u8EP].Cnt=(u8EPSize);    //������Ҫ���͵����ݳ���
  while(u8EPSize--)
       *pu8EPBuffer++=*pu8IN_DataPointer++; //���û������ݸ�ֵ��EP�洢��                                                        
  if(BIT_CHK(u8EndPointFlag,gu8USB_Toogle_flags)) //�����Ӧ�˵��gu8USB_Toogle_flags == 1
  {
      tBDTtable[u8EP].Stat._byte= kUDATA0;         
      BIT_CLR(u8EndPointFlag,gu8USB_Toogle_flags);//������Ӧ�˵��gu8USB_Toogle_flags
  } 
  else
  {
      tBDTtable[u8EP].Stat._byte= kUDATA1;          
      BIT_SET(u8EndPointFlag,gu8USB_Toogle_flags);//��λgu8USB_Toogle_flags
  }
}

/*
* EP_OUT_Transfer
*   �˵�OUT���ͣ��ڲ�ͬUSBģʽ��OUT�ĺ��岻ͬ
*     ��USB����ģʽ�£�USB host��  OUT��ʾ  USB Host�� USB Device�������ݰ�
*     ��USB�豸ģʽ�£�USB Device��OUT��ʾ  USB Host�� USB Device�������ݰ�
*   ��ǰ��USB Deviceģʽ�£��ú�����������Device�豸�������ݰ�
*   ����ֵ�����ؽ��յ������ݵĳ���
*/
u8 EP_OUT_Transfer(u8 u8EP,u8 *pu8DataPointer)
{
    u8 *pu8EPBuffer;
    u8 u8EPSize; 
    /* ������������λ�� */
    u8EP++;
    /* ������ʵ�EP���壬EP0�ǿ��Ƶ㣬EP1Ϊ�����жϵ㣬EP2������BULK�� */
    pu8EPBuffer=BufferPointer[u8EP];
    
    u8EPSize=tBDTtable[u8EP].Cnt;
    u8EP=u8EPSize;
    /* ��EP��������ݸ��Ƹ��û��� */
    while(u8EPSize--)
      *pu8DataPointer++=*pu8EPBuffer++;
    return(u8EP);
}

/*
* USB_EP_OUT_SizeCheck
*   �˵��������ݳ��ȼ��
*
*/
u16 USB_EP_OUT_SizeCheck(u8 u8EP)
{
  u8 u8EPSize; 
   /* ��ȡ�������ĳ��� */
  u8EPSize = tBDTtable[u8EP<<2].Cnt;
  return(u8EPSize & 0x03FF);
}
//=========================================================================
//������: hw_usb_set_interface
//��  ��: USBģ��������� 
//��  ��: ��
//��  ��: ���յ������ݳ���
//=========================================================================
void USB_EnableInterface(void)
{
  /* �˵�Ĵ������� */
  USB0->ENDPOINT[1].ENDPT= EP1_VALUE | USB_ENDPT_EPHSHK_MASK; //���ö˵�1Ϊ�ж϶˵�                       
  USB0->ENDPOINT[2].ENDPT= EP2_VALUE | USB_ENDPT_EPHSHK_MASK; //���ö˵�2Ϊ���� BULK�˵�                     
  USB0->ENDPOINT[3].ENDPT= EP3_VALUE | USB_ENDPT_EPHSHK_MASK; //���ö˵�3Ϊ��� BULK�˵�                         
  USB0->ENDPOINT[4].ENDPT= EP4_VALUE | USB_ENDPT_EPHSHK_MASK; //��ֹ�˵�                        
  USB0->ENDPOINT[5].ENDPT= EP5_VALUE | USB_ENDPT_EPHSHK_MASK; //��ֹ�˵�                         
  USB0->ENDPOINT[6].ENDPT= EP6_VALUE | USB_ENDPT_EPHSHK_MASK; //��ֹ�˵�    	                   
  
    // ���� 1 BDT ����
    // �ѿ���Ȩ����MCU 
    tBDTtable[bEP1IN_ODD].Stat._byte= kUDATA1;                //����DATA1���ݰ�
    tBDTtable[bEP1IN_ODD].Cnt = 0x00;                         //���������
    tBDTtable[bEP1IN_ODD].Addr =(u32)gu8EP1_IN_ODD_Buffer;    //��ַָ���Ӧ�Ļ�����

    // ���� 2 BDT ����
    // �ѿ���Ȩ����MCU 
    tBDTtable[bEP2IN_ODD].Stat._byte= kUDATA1;
    tBDTtable[bEP2IN_ODD].Cnt = 0x00;
    tBDTtable[bEP2IN_ODD].Addr =(u32  )gu8EP2_IN_ODD_Buffer;            

    // ���� 3 BDT ����
    // �ѿ���Ȩ����MCU 
    tBDTtable[bEP3OUT_ODD].Stat._byte= kUDATA0;
    tBDTtable[bEP3OUT_ODD].Cnt = 0xFF;
    tBDTtable[bEP3OUT_ODD].Addr =(u32)gu8EP3_OUT_ODD_Buffer;            
}

//������������
void USB_GetDescHandler(void)
{
	switch((Setup_Pkt->wValue_h) & 0xFF)
	{
		case DEVICE_DESCRIPTOR:
			#if (DEBUG_PRINT == 1)
			printf("�豸������\r\n");
			#endif
			USB_EP_IN_Transfer(EP0,(u8*)Device_Descriptor,sizeof(Device_Descriptor));//�����豸������
			break;
		case CONFIGURATION_DESCRIPTOR:
			#if (DEBUG_PRINT == 1)
			printf("����������\r\n");
			#endif
			USB_EP_IN_Transfer(EP0,(u8*)Configuration_Descriptor,sizeof(Configuration_Descriptor)); //��������������
			break;
		case STRING_DESCRIPTOR:
			#if (DEBUG_PRINT == 1)
			printf("�ַ���������-%x ",Setup_Pkt->wValue_l);
			#endif
			switch(Setup_Pkt->wValue_l)  //����wValue�ĵ��ֽڣ�����ֵ��ɢת
			{
				case 0:  //��ȡ����ID
					#if (DEBUG_PRINT == 1)
					printf("-����ID\r\n");
					#endif
					USB_EP_IN_Transfer(EP0,(u8*)String_Table[Setup_Pkt->wValue_l],sizeof(String_Table[Setup_Pkt->wValue_l]));
					break;
				case 1:  //�����ַ���������ֵΪ1����������Ϊ�����ַ���
					#if (DEBUG_PRINT == 1)
					printf("-�����ַ���\r\n");
					#endif
					USB_EP_IN_Transfer(EP0,(u8*)String_Table[Setup_Pkt->wValue_l],sizeof(String_Table[Setup_Pkt->wValue_l]));
					break;
				case 2:  //��Ʒ�ַ���������ֵΪ2����������Ϊ��Ʒ�ַ���
					#if (DEBUG_PRINT == 1)
					printf("-��Ʒ�ַ���\r\n");
					#endif
					USB_EP_IN_Transfer(EP0,(u8*)String_Table[Setup_Pkt->wValue_l],sizeof(String_Table[Setup_Pkt->wValue_l]));
					break;
				case 3:  //��Ʒ���кŵ�����ֵΪ3����������Ϊ���к�
					#if (DEBUG_PRINT == 1)
					printf("-���к�\r\n");
					#endif
					USB_EP_IN_Transfer(EP0,(u8*)String_Table[Setup_Pkt->wValue_l],sizeof(String_Table[Setup_Pkt->wValue_l]));
					break;
				default: 
					#if (DEBUG_PRINT == 1)
					printf("-λ�õ�����ֵ\r\n");
					#endif
					break; 
			}
			break;
				case REPORT_DESCRIPTOR:
					#if (DEBUG_PRINT == 1)
					printf("-����������\r\n");
					#endif
				  #if (USB_DEVICE_CLASS == USB_DEVICE_CLASS_HID)
					USB_EP_IN_Transfer(EP0,(u8*)Report_Descriptor,sizeof(Report_Descriptor));
					#endif
					break;
		default:
			#if (DEBUG_PRINT == 1)
			printf("����������,����������:0x%x\r\n",Setup_Pkt->wValue_h);
			#endif
			break;
	}
}
/***********************************************************************************************
 ���ܣ���USB_Handler �ж�ɢת����������ɢת USB_EP0_OUT_Handler
 �βΣ�0
 ���أ�0
 ��⣺��SETUP������ʹ���
		(1)ֻ�н��յ�SETUP���ŵ��øú���
		(2)SETUP����8�ֽ�����
			bmRequestType:1
			bRequest:1
			wValue.H:1 :������������
			wValue.L:1 :������������
			wIndex:2
			wLength:2
************************************************************************************************/
void USB_EP0_OUT_Handler(void)
{
	u8 u8State;
	u8 *p =(u8*)Setup_Pkt;
	u8 i;
	// ��DATA0 ��ʼ���䡣
	//BIT_CLR(0,gu8USB_Toogle_flags); //���ȡ��Ҳ����ѽ
	//��ӡ���յ�������
	#if (DEBUG_PRINT == 1)
	for(i=0;i<8;i++,p++) printf("0x%x ",*p); 	printf("\r\n");
	#endif
	if((Setup_Pkt->bmRequestType & 0x80) == 0x80)
	{
		//����bmRequestType��D6-5λɢת��D6-5λ��ʾ���������
		//0Ϊ��׼����1Ϊ������2Ϊ��������
	  switch((Setup_Pkt->bmRequestType>>5) & 0x03)
		{
			case 0:
				#if (DEBUG_PRINT == 1)
				printf("USB��׼��������-");
				#endif
				//USBЭ�鶨���˼�����׼��������ʵ����Щ��׼�������󼴿�
				//����Ĵ�����bRequest�У��Բ�ͬ������������ɢת
				switch(Setup_Pkt->bRequest)
				{
					case mGET_CONFIG:
						#if (DEBUG_PRINT == 1)
						printf("��ȡ����\r\n");
						#endif
						break;	
					case mGET_DESC:
						#if (DEBUG_PRINT == 1)
						printf("��ȡ������-");
						#endif
						USB_GetDescHandler(); //ִ�л�ȡ������
						break;
					case mGET_INTF:
						#if (DEBUG_PRINT == 1)
						printf("��ȡ�ӿ�\r\n");
						#endif
						break;
					case mGET_STATUS:
						#if (DEBUG_PRINT == 1)
						printf("��ȡ״̬\r\n");
						#endif
						break;
					case mSYNC_FRAME:
						#if (DEBUG_PRINT == 1)
						printf("ͬ��֡\r\n");
						#endif
						break;
						default:
						#if (DEBUG_PRINT == 1)
						printf("����δ����ı�׼��������\r\n");
						#endif
						break;
				}
				break;
			case 1:
				#if (DEBUG_PRINT == 1)
				printf("USB����������-");
				#endif
					switch(Setup_Pkt->bRequest)
					{
						case GET_LINE_CODING: //GET_LINE_CODING����
							#if (DEBUG_PRINT == 1)
							printf("GET_LINE_CODING\r\n");
							#endif
							USB_CDC_InterfaceReq_Handler();
						break;
						case SERIAL_STATE: //��ȡSERIAL_STATE����
							#if (DEBUG_PRINT == 1)
							printf("SERIAL_STATE\r\n");
							#endif
							#if (USB_DEVICE_CLASS == USB_DEVICE_CLASS_CDC)
							USB_CDC_InterfaceReq_Handler();
						  #endif
						break;
						default:
							#if (DEBUG_PRINT == 1)
							printf("δ֪ ����:%d\r\n",Setup_Pkt->bRequest);
							#endif
						break;
					}
				break;
			case 2:
				#if (DEBUG_PRINT == 1)
				printf("USB������������\r\n");
				#endif
				break;
			default:
				#if (DEBUG_PRINT == 1)
				printf("����δ�������������\r\n");
				#endif
				break;
		}	
	}
	else
	{
		//����bmRequestType��D6-5λɢת,D6-5λ��ʾ���������
		//0Ϊ��׼����1Ϊ������ 2Ϊ��������
		switch((Setup_Pkt->bmRequestType>>5) & 0x03)
		{
			case 0:
				#if (DEBUG_PRINT == 1)
				printf("USB ��׼�������-");
				#endif
				switch(Setup_Pkt->bRequest)
				{
					case mCLR_FEATURE:
						#if (DEBUG_PRINT == 1)
						printf("�������\r\n");
						#endif
						break;
					case mSET_ADDRESS:
						#if (DEBUG_PRINT == 1)
						printf("���õ�ַ ��ַ:%x\r\n",Setup_Pkt->wValue_l);
						#endif
						gu8USB_State=uADDRESS;
						USB_EP_IN_Transfer(EP0,0,0); //Ϊʲô������
						break;
					case mSET_CONFIG:
						#if (DEBUG_PRINT == 1)
						printf("��������\r\n");
						#endif
            if(Setup_Pkt->wValue_h+Setup_Pkt->wValue_l) 
            {
                //ʹ��1 ��2 ��3 �˵� 
							USB_EnableInterface();
							USB_EP_IN_Transfer(EP0,0,0);
							gu8USB_State=uENUMERATED;
            }
						break;
					case  mSET_DESC:
						#if (DEBUG_PRINT == 1)
						printf("����������\r\n");
						#endif
						break;
					case mSET_INTF:
						#if (DEBUG_PRINT == 1)
						printf("���ýӿ�\r\n");
						#endif
						break;
					default:
						#if (DEBUG_PRINT == 1)
						printf("����δ����ı�׼�������\r\n");
						#endif
					break;
				}
				break;
				case 1:
					#if (DEBUG_PRINT == 1)
					printf("USB���������-");
					#endif
					switch(Setup_Pkt->bRequest)
					{
						case 0x0A:
							USB_EP_IN_Transfer(EP0,0,0); //�ȴ�������0���ݰ�����
							printf("���ÿ���\r\n");
							break;
						case SET_CONTROL_LINE_STATE:
							#if (DEBUG_PRINT == 1)
							printf("SET_CONTROL_LINE_STATE\r\n");
							#endif
							USB_CDC_InterfaceReq_Handler();
							break;
						default:
							USB_CDC_InterfaceReq_Handler();
							#if (DEBUG_PRINT == 1)
							printf("δ֪���������:%d\r\n",Setup_Pkt->bRequest);
							#endif
							break;
					}
					break;
				case 2:
					#if (DEBUG_PRINT == 1)
					printf("USB�����������r\n");
					#endif
					break;
				default:
					#if (DEBUG_PRINT == 1)
					printf("����δ������������\r\n");
					#endif
				break;
		}
	}
	tBDTtable[bEP0OUT_ODD].Stat._byte= kUDATA0;
	BIT_CLR(USB_CTL_TXSUSPENDTOKENBUSY_SHIFT,USB0->CTL); // Ϊ0 ʱ: SIE ������������
}
/***********************************************************************************************
 ���ܣ�USB_Init ��ʼ��USBģ��   ���뱣֤USBģ������Ƶ��Ϊ96M
 �βΣ�0
 ���أ�0 �ɹ�  ���� �������
 ��⣺
************************************************************************************************/
u8 USB_Init(void)
{
	Setup_Pkt=(tUSB_Setup*)BufferPointer[bEP0OUT_ODD];                       //��Setup���ݽṹ��ָ���Ӧ�Ļ�����
  gu8USB_State=uPOWER;                                                     //���״̬Ϊ�����׶�
	BIT_SET(FMC_PFAPR_M4AP_MASK,FMC->PFAPR);       //׼��д��FLASH
	BIT_SET(SIM_SOPT1_USBREGEN_SHIFT,SIM->SOPT1);  //����USB��Դ��ѹ��
	BIT_CLR(SIM_SOPT1_USBSTBY_SHIFT,SIM->SOPT1);   //USB��ѹ��ֹͣ����
  MPU->CESR=0;																	 //ȡ��MPU����
  BIT_SET(SIM_SOPT2_USBSRC_SHIFT,SIM->SOPT2);    // ʹ��PLL/FLLΪʱ��Դ
	GetCPUInfo();                                  //���CPU��������
	if((CPUInfo.CoreClock < 95000000) || (CPUInfo.CoreClock > 97000000)) //��Ƶ������
	{
		return 1;
	}
//	if(PUInfo.ClkOpt != 0) BIT_SET(SIM_SOPT2_PLLFLLSEL_SHIFT,SIM->SOPT2);  // ʹ��PLL���
//	else                 		BIT_CLR(SIM_SOPT2_PLLFLLSEL_SHIFT,SIM->SOPT2);  // ʹ��FLL���
	BIT_CLR(SIM_SOPT2_PLLFLLSEL_SHIFT,SIM->SOPT2);  // ʹ��FLL���
  SIM->CLKDIV2|=USB_FARCTIONAL_VALUE;              //USB��Ƶ��������  //��Ƶ96M ��Ƶ��50M
  SIM->SCGC4|=(SIM_SCGC4_USBOTG_MASK);             //USBģ��ʱ����ʹ��
	NVIC_EnableIRQ(USB0_IRQn); //ʹ��USBģ��IRQ�ж�
	//USBģ��Ĵ�������
	
	USB0->USBTRC0|=USB_USBTRC0_USBRESET_MASK;        //��λUSBģ��
	while(BIT_CHK(USB_USBTRC0_USBRESET_SHIFT,USB0->USBTRC0)){}; //�ȴ���λ���

	//����BDT��ַ�Ĵ���
	//( ��9 λ��Ĭ��512 �ֽڵ�ƫ��) 512 = 16 * 4 * 8 ��
	//8 λ��ʾ: 4 ���ֽڵĿ���״̬��4 ���ֽڵĻ�������ַ ��
	USB0->BDTPAGE1=(u8)((u32)tBDTtable>>8);
	USB0->BDTPAGE2=(u8)((u32)tBDTtable>>16);
	USB0->BDTPAGE3=(u8)((u32)tBDTtable>>24);
	BIT_SET(USB_ISTAT_USBRST_MASK,USB0->ISTAT);    //���USBģ�鸴λ��־
	BIT_SET(USB_INTEN_USBRSTEN_SHIFT,USB0->INTEN); //ʹ��USBģ�鸴λ�ж�
	USB0->USBCTRL=0x40;                             //D-  D+ ����
	USB0->USBTRC0|=0x40;                            //ǿ�����õ�6λΪ1  ���Ǿ��ᣬDS�Ͼ���ôд��
	USB0->CTL|=0x01;                                //USBģ��ʹ��
	USB0->CONTROL|=USB_CONTROL_DPPULLUPNONOTG_MASK; //DPP����ʹ��
	#if (USB_DEVICE_CLASS == USB_DEVICE_CLASS_CDC)
		USB_CDC_Init();
	#endif
}
//�Ͽ�USB
void USB_DisConnect()
{
	#if (DEBUG_PRINT == 1)
	printf("USB�Ͽ�\r\n");
	#endif
	USB0->CONTROL&=~USB_CONTROL_DPPULLUPNONOTG_MASK; //DPP����ȡ��
}
//����USB
void USB_Connect()
{
	#if (DEBUG_PRINT == 1)
	printf("USB����\r\n");
	#endif
	USB0->CONTROL|=USB_CONTROL_DPPULLUPNONOTG_MASK; //DPP����ʹ��
}
/***********************************************************************************************
 ���ܣ�USB ��λ�ж�
 �βΣ�0
 ���أ�0
 ��⣺��ֹ����EP(�˵�Ĵ���)��ͬʱ��EP0��ʼΪ���ƶ˵�
       ����EP0�����롢�����ż������г�ʼ����
			 ����USB�����жϡ�USB��һ֡�����жϡ�USB�����жϡ�USB��λ�ж�
************************************************************************************************/
void USB_ResetHandler(void)
{
	//���־
	gu8USBClearFlags=0xFF;
	gu8USB_Toogle_flags=0xFE;
	//��ֹ���ж˵� 0�˵����
	USB0->ENDPOINT[1].ENDPT=0x00;
	USB0->ENDPOINT[2].ENDPT=0x00;
	USB0->ENDPOINT[3].ENDPT=0x00;
	USB0->ENDPOINT[4].ENDPT=0x00;
	USB0->ENDPOINT[5].ENDPT=0x00;
	USB0->ENDPOINT[6].ENDPT=0x00;
  /*�˵�0 BDT �����˵�����*/
  tBDTtable[bEP0OUT_ODD].Cnt = EP0_SIZE;   // EP0 OUT BDT ����
  tBDTtable[bEP0OUT_ODD].Addr =(u32)gu8EP0_OUT_ODD_Buffer;
  tBDTtable[bEP0OUT_ODD].Stat._byte = kUDATA1;//USB-FS(Ӳ��USBģ��) ��ר��Ȩ���� BD
                                              //ʹ��USB-FSȥ�������ݷ�תͬ��
                                              //����DATA1�����ͻ��߽���        
   
  tBDTtable[bEP0OUT_EVEN].Cnt = EP0_SIZE; // EP0 OUT BDT ����
  tBDTtable[bEP0OUT_EVEN].Addr =(u32)gu8EP0_OUT_EVEN_Buffer;
  tBDTtable[bEP0OUT_EVEN].Stat._byte = kUDATA1;//USB-FS(Ӳ��USBģ��) ��ר��Ȩ���� BD
                                               //ʹ��USB-FSȥ�������ݷ�תͬ��
                                               //����DATA1�����ͻ��߽���       
   
  tBDTtable[bEP0IN_ODD].Cnt = EP0_SIZE;   // EP0 IN BDT ����     
  tBDTtable[bEP0IN_ODD].Addr =(u32)gu8EP0_IN_ODD_Buffer;      
  tBDTtable[bEP0IN_ODD].Stat._byte = kUDATA0;//USB-FS(Ӳ��USBģ��) ��ר��Ȩ���� BD
                                             //ʹ��USB-FSȥ�������ݷ�תͬ��
                                             //����DATA0�����ͻ��߽��� 
   
  tBDTtable[bEP0IN_EVEN].Cnt = EP0_SIZE;  // EP0 IN BDT ����            
  tBDTtable[bEP0IN_EVEN].Addr =(u32)gu8EP0_IN_EVEN_Buffer;      
  tBDTtable[bEP0IN_EVEN].Stat._byte = kUDATA0;//USB-FS(Ӳ��USBģ��) ��ר��Ȩ���� BD
                                              //ʹ��USB-FSȥ�������ݷ�תͬ��
                                              //����DATA0�����ͻ��߽���          
	USB0->ENDPOINT[0].ENDPT=0x0D; // ʹ�� EP0 �������� ����ʱ��
	USB0->ERRSTAT=0xFF;           // ������еĴ���
	USB0->ISTAT=0xFF;             // ������е��жϱ�־
	USB0->ADDR=0x00;  					  // USBö��ʱ��Ĭ���豸��ַ0
	USB0->ERREN=0xFF;             // ʹ�����еĴ����ж�
	// USBģ���ж�ʹ��
	BIT_SET(USB_INTEN_TOKDNEEN_SHIFT,USB0->INTEN);
	BIT_SET(USB_INTEN_SOFTOKEN_SHIFT,USB0->INTEN);
	BIT_SET(USB_INTEN_ERROREN_SHIFT,USB0->INTEN); 
	BIT_SET(USB_INTEN_USBRSTEN_SHIFT,USB0->INTEN);    
}

void USB_EP0_IN_Handler(void)
{
    if(gu8USB_State==uADDRESS)
    {
        USB0->ADDR = Setup_Pkt->wValue_l; //д���ַ
        gu8USB_State=uREADY;              //��ΪReady״̬
        BIT_SET(fIN,gu8USBClearFlags);   //
				#if (DEBUG_PRINT == 1)
				printf("�µ�ַ��%d\r\n",USB0->ADDR);
				#endif
    }
		USB_EP_IN_Transfer(EP0,0,0); 
}
void USB_EP2_IN_Handler(void)
{
	if(USB_DEVICE_CLASS == USB_DEVICE_CLASS_HID) //�����HID�豸
	{
		USB_HID_SendReport();
	}
// tBDTtable[bEP2IN_ODD].Stat._byte = kUDATA0;
//	BIT_CLR(USB_CTL_TXSUSPENDTOKENBUSY_SHIFT,USB0->CTL);
}
void USB_EP2_OUT_Handler(void)
{
	
}
void USB_EP3_IN_Handler(void)
{
	
}
extern u8 RecFlag;
void USB_EP3_OUT_Handler(void)
{
	if(USB_DEVICE_CLASS == USB_DEVICE_CLASS_HID) //�����HID�豸
	{
		RecFlag = 1; 
	}
	if(USB_DEVICE_CLASS == USB_DEVICE_CLASS_CDC) //CDC�豸
	{
		
	}
//	tBDTtable[EP3<<2].Stat._byte= kSIE;
//  tBDTtable[bEP3OUT_ODD].Cnt = EP3_SIZE;
}
/***********************************************************************************************
 ���ܣ���USB0_IRQHandler Ӳ���ж�ɢת����������ɢת
 �βΣ�0
 ���أ�0
 ��⣺��USB0_IRQHandler Ӳ���ж�ɢת��������������ɢת
************************************************************************************************/
void USB_Handler(void)
{
	u8 u8EndPoint;
	u8 u8IN;
  u8IN = USB0->STAT & 0x08;    //��õ�ǰ�Ĵ���״̬��1���ͣ�0����
  u8EndPoint = USB0->STAT >> 4;//��õ�ǰ�������ƵĶ˵��ַ
	if(u8EndPoint == 0) //�˵�0
	{
		if(u8IN) //IN
		{
			#if (DEBUG_PRINT == 1)
			printf("�˵�0�����жϴ���\r\n");
			#endif
			USB_EP0_IN_Handler(); //�˵�0���Ͱ�
		}
		else    //OUT
		{
			#if (DEBUG_PRINT == 1)
			printf("�˵�0����жϴ���-");
			#endif
			BIT_SET(u8EndPoint,gu8USB_Flags);  //���� gu8USB_Flags = 1   
	//		usbMCU_CONTROL(u8EndPoint);   
			USB_EP0_OUT_Handler();
		}
	}
	if(u8EndPoint == 1) //�˵�1
	{
		if(u8IN) //IN
		{
			#if (DEBUG_PRINT == 1)
			printf("�˵�1�����жϴ���\r\n");
			#endif
			//USB_EP1_IN_Handler();
		}
		else //OUT
		{
			#if (DEBUG_PRINT == 1)
			printf("�˵�1����жϴ���\r\n");
			#endif
			//USB_EP1_OUTHandler();	
			BIT_SET(u8EndPoint,gu8USB_Flags);  //���� gu8USB_Flags = 1   
	//		usbMCU_CONTROL(u8EndPoint);    
		}
	}
	if(u8EndPoint == 2)//�˵�2
	{
		if(u8IN) //IN
		{
			#if (DEBUG_PRINT == 1)
			printf("�˵�2�����жϴ���\r\n");
			#endif
			USB_EP2_IN_Handler();
		}
		else //OUT
		{
			#if (DEBUG_PRINT == 1)
			printf("�˵�2����жϴ���\r\n");
			#endif
			USB_EP2_OUT_Handler();
			BIT_SET(u8EndPoint,gu8USB_Flags);  //���� gu8USB_Flags = 1   
		}	
	}
	if(u8EndPoint == 3)//�˵�3
	{
		if(u8IN) //IN
		{
			#if (DEBUG_PRINT == 1)
			printf("�˵�3�����жϴ���\r\n");
			#endif
			USB_EP3_IN_Handler();
		}
		else //OUT
		{
			#if (DEBUG_PRINT == 1)
			printf("�˵�3����жϴ���\r\n");
			#endif
			USB_EP3_OUT_Handler();
			BIT_SET(u8EndPoint,gu8USB_Flags);  //���� gu8USB_Flags = 1    
		}	
		
	}
}

/***********************************************************************************************
 ���ܣ�Ӳ��USB0 �ж�
 �βΣ�0
 ���أ�0
 ��⣺��ʼUSBɢת����  
************************************************************************************************/
void USB0_IRQHandler(void)
{
	u8 err;
	//���USB ģ���Ƿ��������Ч�ĸ�λ��
	if(BIT_CHK(USB_ISTAT_USBRST_SHIFT,USB0->ISTAT)) 
	{
		#if (DEBUG_PRINT == 1)
		printf("USB���߸�λ\r\n");
		#endif
		USB_ResetHandler();
	}
	//�յ�SOF��
	if(BIT_CHK(USB_ISTAT_SOFTOK_SHIFT,USB0->ISTAT)) 
	{
		#if (DEBUG_PRINT == 1)
		//printf("�յ�SOF��\r\n");
		#endif
		USB0->ISTAT = USB_ISTAT_SOFTOK_MASK;   
	}
	//�յ�STALL��
	if(BIT_CHK(USB_ISTAT_STALL_SHIFT,USB0->ISTAT))
	{
		#if (DEBUG_PRINT == 1)
		printf("�յ�STALL��\r\n");
		#endif
		if(BIT_CHK(USB_ENDPT_EPSTALL_SHIFT,USB0->ENDPOINT[0].ENDPT))
		BIT_CLR(USB_ENDPT_EPSTALL_SHIFT,USB0->ENDPOINT[0].ENDPT);
    BIT_SET(USB_ISTAT_STALL_SHIFT,USB0->ISTAT);
	}
	//��������ж�
	if(BIT_CHK(USB_ISTAT_TOKDNE_SHIFT,USB0->ISTAT)) 
	{
		BIT_SET(USB_CTL_ODDRST_SHIFT,USB0->CTL);//ָ��BDT EVEN ��
		USB_Handler(); //����USB Handler
		//�����������ж�
		BIT_SET(USB_ISTAT_TOKDNE_SHIFT,USB0->ISTAT);
	}
	//SLEEP 
	if(BIT_CHK(USB_ISTAT_SLEEP_SHIFT,USB0->ISTAT)) 
	{
		#if (DEBUG_PRINT == 1)
		printf("SLEEP�ж�\r\n");
		#endif
		BIT_SET(USB_ISTAT_SLEEP_SHIFT,USB0->ISTAT);        
	}
	// ����
	if(BIT_CHK(USB_ISTAT_ERROR_SHIFT,USB0->ISTAT))
	{
		err = USB0->ERRSTAT;
		#if (DEBUG_PRINT == 1)
		printf("���� �������:%d\r\n",err);
		#endif
		BIT_SET(USB_ISTAT_ERROR_SHIFT,USB0->ISTAT);
		USB0->ERRSTAT=0xFF; //��������жϴ���
	}
}



