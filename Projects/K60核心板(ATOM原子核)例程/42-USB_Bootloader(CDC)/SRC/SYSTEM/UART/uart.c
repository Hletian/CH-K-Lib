/***********************************************************************************************
 CHKD�ڲ����԰汾V0.1
 2012.12.3
************************************************************************************************/
#include "uart.h"
#include "sys.h"
#include <stdio.h>

/***********************************************************************************************
 ���ܣ�UART3���ڽ����뷢���жϷ�����               
 �βΣ�uartch: ���ں�           
       ch:    ���յ����ֽ�   
 ���أ�0ʧ��  1 �ɹ�
 ��⣺�����ڲ���һ�����ջ��߷����ź�ʱ��������һ���ж��źţ�����˺����������жϴ����жϱ���Զ���� 
************************************************************************************************/
void UART3_RX_TX_IRQHandler(void)
{ 
   u8 ch;
   if(UART_Re1(UART3,&ch))
   {
   		//�жϴ���
	    //UART_Send1(UART3,ch);	  //���� ���ؽ��յ�������
   }
}
/***********************************************************************************************
 ���ܣ���ʼ������
 �βΣ�uartch:���ں�         
       baud:�����ʣ���9600��115200��      
 ���أ�0
 ��⣺��k10оƬ�ϣ�һ����5��uart���ֱ�ΪUART0,UART1,UART2,UART3,UART4  
       �˺����Զ�������Ƶ�����Բ���Ҫ��дƵ��
************************************************************************************************/
void UART_Init(UART_Type *uartch,u32 baud)
{	    
  //���д������ʼ���
  u16 sbr;
	u8 brfa; 
	u32 clock;
	GetCPUInfo();  //����ϵͳʱ��
  clock=CPUInfo.BusClock;
	if((u32)uartch==UART0_BASE||(u32)uartch==UART1_BASE) clock=CPUInfo.CoreClock;; //UART0 UART1ʹ��CoreClock
	sbr=(u16)((clock)/(baud*16));
	brfa=((clock*2)/baud-(sbr*32));
	//��uart��ص����ŵ�ʱ��ʹ��
	switch((u32)uartch)
    {case UART0_BASE:
	                  SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;  //��uart0��ص����ŵ�ʱ��ʹ�ܣ��μ��ֲ�267ҳ
					  //����uart0��Ź��� �μ��ֲ�239ҳ
					  PORTD->PCR[6]&=~PORT_PCR_MUX_MASK;
					  PORTD->PCR[6]=PORT_PCR_MUX(0x2);   //��PTD6��ʹ��UART0_TXD����
	          PORTD->PCR[7]&=~PORT_PCR_MUX_MASK;
					  PORTD->PCR[7]|=PORT_PCR_MUX(0x2);  //��PTD7��ʹ��UART0_RXD
					  SIM->SCGC4|=SIM_SCGC4_UART0_MASK;	 //ʹ��uart0ʱ��ģ�飬�μ��ֲ�265ҳ
				//	  NVIC_EnableIRQ(UART0_RX_TX_IRQn);  //���������жϣ��μ��ֲ�1221ҳ,��������core_cm4.h
					  break; 
					  		
	  case UART1_BASE:
	                  SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;  //��uart1��ص����ŵ�ʱ��ʹ�ܣ��μ��ֲ�267ҳ
					  //����uart1��Ź��� �μ��ֲ�239ҳ
					  PORTC->PCR[4]&=~PORT_PCR_MUX_MASK;
					  PORTC->PCR[4]=PORT_PCR_MUX(0x3);   //��PTC4��ʹ��UART1_TXD����
	          PORTC->PCR[3]&=~PORT_PCR_MUX_MASK;
					  PORTC->PCR[3]|=PORT_PCR_MUX(0x3);  //��PTC3��ʹ��UART1_RXD
					  SIM->SCGC4|=SIM_SCGC4_UART1_MASK;	 //ʹ��uart1ʱ��ģ�飬�μ��ֲ�265ҳ
					//  NVIC_EnableIRQ(UART1_RX_TX_IRQn);  //���������жϣ��μ��ֲ�1221ҳ,��������core_cm4.h
					  break;
	  case UART2_BASE:
	                  SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;  //��uart2��ص����ŵ�ʱ��ʹ�ܣ��μ��ֲ�267ҳ
					  //����uart2��Ź��� �μ��ֲ�239ҳ
					  PORTD->PCR[3]&=~PORT_PCR_MUX_MASK;
					  PORTD->PCR[3]=PORT_PCR_MUX(0x3);   //��PTD3��ʹ��UART2_TXD����
	          PORTD->PCR[2]&=~PORT_PCR_MUX_MASK;
					  PORTD->PCR[2]|=PORT_PCR_MUX(0x3);  //��PTD2��ʹ��UART2_RXD
					  SIM->SCGC4|=SIM_SCGC4_UART2_MASK;	 //ʹ��uart2ʱ��ģ�飬�μ��ֲ�265ҳ
					//  NVIC_EnableIRQ(UART2_RX_TX_IRQn);  //���������жϣ��μ��ֲ�1221ҳ,��������core_cm4.h
					  break; 
	  case UART3_BASE:
	                  SIM->SCGC5|=SIM_SCGC5_PORTB_MASK;  //��uart3��ص����ŵ�ʱ��ʹ�ܣ��μ��ֲ�267ҳ
					  //����uart3��Ź��� �μ��ֲ�239ҳ
					  PORTB->PCR[11]&=~PORT_PCR_MUX_MASK;
					  PORTB->PCR[11]=PORT_PCR_MUX(0x3);   //��PTB11��ʹ��UART3_TXD����
	          PORTB->PCR[10]&=~PORT_PCR_MUX_MASK;
					  PORTB->PCR[10]|=PORT_PCR_MUX(0x3);  //��PTD10��ʹ��UART3_RXD����
					  SIM->SCGC4|=SIM_SCGC4_UART3_MASK;	 //ʹ��uart3ʱ��ģ�飬�μ��ֲ�265ҳ
					 // NVIC_EnableIRQ(UART3_RX_TX_IRQn);  //���������жϣ��μ��ֲ�1221ҳ,��������core_cm4.h
					  break; 
	  case UART4_BASE:
	                  SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;  //��uart4��ص����ŵ�ʱ��ʹ�ܣ��μ��ֲ�267ҳ
					  //����uart4��Ź��� �μ��ֲ�239ҳ
					  PORTC->PCR[15]&=~PORT_PCR_MUX_MASK;
					  PORTC->PCR[15]=PORT_PCR_MUX(0x3);   //��PTE24��ʹ��UART4_TXD����
	          PORTC->PCR[14]&=~PORT_PCR_MUX_MASK;
					  PORTC->PCR[14]|=PORT_PCR_MUX(0x3);  //��PTE25��ʹ��UART4_RXD����
					  SIM->SCGC1|=SIM_SCGC1_UART4_MASK;	 //ʹ��uart4ʱ��ģ�飬�μ��ֲ�262ҳ
					  //NVIC_EnableIRQ(UART4_RX_TX_IRQn);  //���������жϣ��μ��ֲ�1221ҳ,��������core_cm4.h
					  break;
	  default : break;
	}
//����uart���ƼĴ�����ʵ�ֻ����İ�λ���书��
  uartch->C2&=~(UART_C2_RE_MASK|UART_C2_TE_MASK);	 //��ֹ���ͽ���,�μ��ֲ�1221ҳ
	uartch->C1&=~UART_C1_M_MASK;   //��������λ��Ϊ8λ���μ������ֲ�1220ҳ
	uartch->C1&=~(UART_C1_PE_MASK);//����Ϊ����żУ��λ���μ������ֲ�1220ҳ
	uartch->S2&=~UART_S2_MSBF_MASK;//����Ϊ���λ���ȴ���
//���ô���Ƶ��
	uartch->BDH|=((sbr>>8)&UART_BDH_SBR_MASK);//���ø�5λ�����ݣ��μ��ֲ�1218ҳ
	uartch->BDL=(sbr&UART_BDL_SBR_MASK);//���õ�8λ����
	uartch->C4|=brfa&(UART_BDL_SBR_MASK>>3);//����С��λ���μ��ֲ�1232ҳ
//ʹ�ܽ������뷢����
	uartch->C2|=(UART_C2_RE_MASK|UART_C2_TE_MASK);	 //�������ݷ��ͽ���,�μ��ֲ�1221ҳ
//�����ж�ʹ��
	//uartch->C2|=UART_C2_RIE_MASK; //���������жϣ��μ��ֲ�1221ҳ
}
//�ض���fputc���� Ϊ��ʹ��printf����
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout; 
int fputc(int ch,FILE *f)
{
	UART_Send1(DEBUG_UART_PORT,ch);
	return ch;
}
/***********************************************************************************************
 ���ܣ����ڷ���1���ֽ�
 �βΣ�uartch: ���ں�         
       ch    : ���͵�����
 ���أ�0
 ��⣺���ô��ڷ���1���ַ���
************************************************************************************************/
void UART_Send1(UART_Type *uartch, u8 ch)
{
	//�ȴ����ͻ������գ��μ��ֲ�1223ҳ
	while(!(uartch->S1 & UART_S1_TDRE_MASK));
	//��������,�μ��ֲ�1230ҳ
	uartch->D=(u8)ch;
 }
/***********************************************************************************************
 ���ܣ����ڷ���N���ֽ�
 �βΣ�uartch  : ���ں�         
       buff    : ����ָ��
			 len     : ���ݳ���
 ���أ�0
 ��⣺��������������һ���ļ�
************************************************************************************************/
void UART_SendN (UART_Type *uartch,u8 *buff,u32 len)
{
	u32 i;
	for(i=0;i<len;i++)
	{
		UART_Send1(uartch,buff[i]);
	}
}
/***********************************************************************************************
 ���ܣ����ڽ���1���ֽ� 
 �βΣ�uartch: ���ں�           
       ch:    ���յ����ֽ�   
 ���أ�0ʧ��  1 �ɹ�
 ��⣺�˴����ò�ѯ�ķ�ʽ�������ݽ��գ��в������أ�����ʹ���жϽ��յķ�ʽ     
************************************************************************************************/
uint8_t UART_Re1(UART_Type *UARTx, uint8_t *ch)
{
    if((UARTx->S1 & UART_S1_RDRF_MASK)!= 0)//�жϽ��ջ������Ƿ���,�μ��ֲ�1223ҳ
    {
        *ch =(uint8_t)(UARTx->D);	//��������,�μ��ֲ�1230ҳ
        return 1; 			//���ܳɹ�
    }
    return 0;			//�����ʱ������ʧ��
}

///////////////////////////////////////////////
//������: UART3_RX_TX_IRQHandler                                                         
//��  ��: UART3���ڽ����뷢���жϷ�����                                                 
//��  ��: ��                                                 
//��  ��: ��                                                    
//˵  ��: �����ڲ���һ�����ջ��߷����ź�ʱ��������һ���ж��źţ�����˺����������жϴ���
//         �жϱ���Զ����                                                                   
//////////////////////////////////////////////
void UART4_RX_TX_IRQHandler(void)
{ 
   u8 ch;
   if(UART_Re1(UART4,&ch))
   {
   		//�жϴ���
	    //UART_Send1(UART3,ch);	  //���� ���ؽ��յ�������
   }
}

