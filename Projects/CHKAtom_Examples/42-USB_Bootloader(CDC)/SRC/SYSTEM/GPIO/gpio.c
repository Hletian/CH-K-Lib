/***********************************************************************************************
 CHKD�ڲ����԰汾V0.1
 2012.12.3
************************************************************************************************/
 #include "gpio.h"
/***********************************************************************************************
 ���ܣ���ʼ������GPIO����
 �βΣ�port:   PTA PTB  PTC PTD PTE
       index:  ��Ҫ���Ƶ����ź� 0-31
			 dir:    ����  0 ���� 1 ���
			 data:   �����ʱ�� 0�͵�ƽ 1�ߵ�ƽ  ������ʱ�� 0 �������� 1 ��������
 ���أ�0
 ��⣺GPIO��ʼ��
************************************************************************************************/
void GPIO_Init(GPIO_Type *port,int index,int dir,int data)
{  
	 PORT_Type *p;
	 switch((u32)port)
	 {
	  case PTA_BASE:p=PORTA;SIM->SCGC5|=SIM_SCGC5_PORTA_MASK;break; //����PORTA��ʹ��ʱ�ӣ�������ǰ���ȿ���ʹ��ʱ�Ӳμ�k10�ֲ�268ҳ��
	  case PTB_BASE:p=PORTB;SIM->SCGC5|=SIM_SCGC5_PORTB_MASK;break;	//����PORTB��ʹ��ʱ��
	  case PTC_BASE:p=PORTC;SIM->SCGC5|=SIM_SCGC5_PORTC_MASK;break;	//����PORTC��ʹ��ʱ��
	  case PTD_BASE:p=PORTD;SIM->SCGC5|=SIM_SCGC5_PORTD_MASK;break;	//����PORTD��ʹ��ʱ��
	  case PTE_BASE:p=PORTE;SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;break;	//����PORTE��ʹ��ʱ��
	  default : break;
	 } 
     p->PCR[index]&=~(PORT_PCR_MUX_MASK);    
     p->PCR[index]|=PORT_PCR_MUX(1);   //����PORT�ڵ�MUXλΪGPIOģʽ���μ�k10�ֲ�238ҳ
	 if(dir==1)  //����Ϊ�����
	 {					
	  port->PDDR|=(1<<index);	//����PORTn�ڵĵ�index����Ϊ������μ�k10�ֲ�1484ҳ
	  p->PCR[index]&=~(PORT_PCR_PE_MASK); //��Ϊ�����ʱ�رո����ŵ����������蹦�ܣ��μ�k10�ֲ�240ҳ
	  if(data==1) //�������ֵ���μ�k10�ֲ�1482 ҳ
	    port->PDOR|=(1<<index); //�˿����ֵΪ1
	  else
	    port->PDOR&=~(1<<index);//�˿����ֵΪ0
	 }
	 else//����Ϊ�����
	 {
	  port->PDDR&=~(1<<index);//����PORTn�ڵĵ�index����Ϊ���룬�μ�k10�ֲ�1484ҳ
	  if(data==1) //�������裬�μ�k10�ֲ�240ҳ
		p->PCR[index]|=PORT_PCR_PS_MASK;
	  else	//�������裬�μ�k10�ֲ�240ҳ
	  	p->PCR[index]&=~(PORT_PCR_PS_MASK);  
	    p->PCR[index]|=PORT_PCR_PE_MASK; 
	 }
}
/***********************************************************************************************
 ���ܣ�����GPIO���ŵĵ�ƽ
 �βΣ�port:   PTA  PTB  PTC  PTD  PTE
       index:  ��Ҫ���Ƶ����ź� 0-31
			 data:   0�͵�ƽ 1�ߵ�ƽ
 ���أ�0
 ��⣺����GPIO���ŵĵ�ƽ
************************************************************************************************/
void GPIO_Ctrl(GPIO_Type *port,int index,int data)
{	 
	if(data==1)//�����������Ϊ1���ο�k10�ֲ�1482ҳ
	{ 	 
		port->PSOR|=1<<index;
	}
	else //�����������Ϊ0���ο�k10�ֲ�1483ҳ
	{
		port->PCOR|=1<<index;
	}
}
/***********************************************************************************************
 ���ܣ���תGPIO���ŵĵ�ƽ
 �βΣ�port:   PTA  PTB  PTC  PTD  PTE
       index:  ��Ҫ���Ƶ����ź� 0-31
 ���أ�0
 ��⣺��תGPIO���ŵĵ�ƽ
************************************************************************************************/
void GPIO_Toggle(GPIO_Type *port,int index)
{
	 port->PTOR|=1<<index;	//�����ŵĵ�ƽ���з�ת���ο�k10�ֲ�1483ҳ
}
/***********************************************************************************************
 ���ܣ���ȡ���ŵĵ�ƽ״̬
 �βΣ�port:   PTA  PTB  PTC  PTD  PTE
       index:  ��Ҫ���Ƶ����ź� 0-31
 ���أ�GPIO���ŵĵ�ƽ
 ��⣺��ȡ���ŵĵ�ƽ״̬
************************************************************************************************/
u8 GPIO_Get(GPIO_Type *port,int index)
{	 
	 return((port->PDIR>>index)&1);//������ŵĵ�ƽ״̬������������״̬��ʹ�ã��ο�k10�ֲ�1484ҳ
}
