#include "sys.h"
#include "uart.h"
#include "gpio.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "delay.h"
#include "dma.h"
#include "enet.h"
#include "flash.h"
#include "ftm.h"
#include "i2c.h"
#include "lptm.h"
#include "pdb.h"
#include "pit.h"
#include "rtc.h"
#include "sd.h"
#include "spi.h"
#include "tsi.h"
#include "wdog.h"

//LED Devices
#include "led.h"
#include "led_chk60evb.h"
//KBI Devices
#include "kbi.h"
#include "kbi_chk60evb.h"
//SPI Flash Devices;
#include "spiflash.h"
//LCD Devices
#include "lcd_chk60evb.h"  
#include "ads7843.h"

//CHGUI 
#include "chgui.h"         
#include "chgui_char.h"    
#include "chgui_bmp.h"     
#include "chgui_touch.h"

//MiniShell
#include "minishell.h"

//uIP
#include "uip.h"
#include "uip_arp.h"
#include "timer.h"
#include "tapdev.h"
#include "uip_demo.h"

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��

/*
* ����֪ʶ:
* 1- ENET ��̫��ģ�� Internet��������� ����ĵ�λΪ��̫֡
* ��ʵ�� ��ֲTCP/IPЭ��ջ uIP ʵ�� TCP Server TCP Client �� web����
*  web: PC IP����Ϊ 192.168.0.2  ����������� 192.168.0.201
*  PC Server:  PC IP����Ϊ 192.168.0.2  ������������� IP����Ϊ 192.168.0.2   ѡ�� TCP Server �˿�1400 
*  PC Client:  PC IP����Ϊ 192.168.0.2  ������������� IP����Ϊ 192.168.0.201 ѡ�� TCP Client �˿�1200
*/

const uint8_t LocalMacAdress[6]={0x00,0xCF,0x52,0x35,0x00,0x01};	//MAC��ַ
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])	

static void uip_polling(void);

//��ʼ��uIP
void uIPInit(void)
{
	uip_ipaddr_t ipaddr;
		tapdev_init();
		uip_init();				//uIP��ʼ��	  
		uip_arp_init();
		uip_ipaddr(ipaddr, 192,168,0,201);	//���ñ�������IP��ַ
		uip_sethostaddr(ipaddr);					    
		uip_ipaddr(ipaddr, 192,168,0,1); 	//��������IP��ַ(��ʵ������·������IP��ַ)
		uip_setdraddr(ipaddr);						 
		uip_ipaddr(ipaddr, 255,255,255,0);	//������������
		uip_setnetmask(ipaddr);

		uip_listen(HTONS(1200));			//����1200�˿�,����TCP Server
		uip_listen(HTONS(80));				//����80�˿�,����Web Server
		tcp_client_reconnect(); //�˿�Ϊ1400 ��������1400�˿�  ���� TCP client 

}

//�ַ���������
uint8_t strBuffer[30];

int main(void)
{
	  uint8_t res;
    uint32_t len;
    uint32_t i;
    ENET_InitTypeDef ENET_InitStruct1;
	  PIT_InitTypeDef PIT_InitStruct1;
	  ADC_InitTypeDef ADC_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
	  KBI_Init(KBI_PinLookup_CHK60EVB, kNumOfKEY);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
    //��ʼ����̫��
	  ENET_InitStruct1.pMacAddress = (uint8_t*)LocalMacAdress;
	  ENET_Init(&ENET_InitStruct1);
	  if(ENET_MiiLinkState() == TRUE)
		{
        UART_printf("ENET Phy Connected succ.\r\n");
		}
		else
		{
			  UART_printf("ENET Phy Connected failed.\r\n");
		}
		//��ʼ��ADC
    ADC_InitStruct1.ADCxMap = ADC0_TEMP_SENSOR_DIFF;     
    ADC_InitStruct1.ADC_Precision = ADC_PRECISION_12BIT; 
    ADC_InitStruct1.ADC_TriggerSelect = ADC_TRIGGER_SW;
    ADC_Init(&ADC_InitStruct1);
		//��ʼ��PIT ����uIP��ʱ
		PIT_InitStruct1.PITx = PIT2;
		PIT_InitStruct1.PIT_Interval = 10;
		PIT_Init(&PIT_InitStruct1);
		PIT_ITConfig(PIT2, PIT_IT_TIF, ENABLE);
		NVIC_EnableIRQ(PIT2_IRQn);
		//��ʼ��uIP
    uIPInit();
    //����һ����ʱ�� ΪuIP�ṩʱ��
    while(1) 
    {
        if(KBI_GetKeyState(kKEY1) == kKBI_SINGLE)
        {
            res = app_tcp_server_senddata("TCP Server Send Message", 24); //����TCP����
            break;
        }
        if(KBI_GetKeyState(kKEY1) == kKBI_LONG)
        {
            res = app_tcp_client_senddata("TCP client Send Message", 23);	//����TCP����
            break;
        }
        len = app_tcp_server_recdata(strBuffer); //�����ݴ�PC��(Client) ������
        if(len > 0) //�����ݴ�PC��������
        {
            UART_printf("\r\nData from client(PC):\r\n");
            for(i=0;i<len;i++)
            {
                UART_printf("%c",strBuffer[i]);
            }
        }
				len = app_tcp_client_recdata(strBuffer); //�����ݴ�PC��(server) ������
				if(len > 0) //�����ݴ�PC��������
				{
						UART_printf("\r\nData from server(PC):\r\n");	
						for(i=0;i<len;i++)
						{
								UART_printf("%c",strBuffer[i]);
						}
				}
				uip_polling();	//����uip�¼���������뵽�û������ѭ������  
    }
}

//���ʱ��
void get_time(uint8_t* msg)
{
	sprintf((char*)msg,"%s %s", __DATE__,  __TIME__);
}
//����¶�
void get_temperature(uint8_t *msg)
{
    float temp;
    float vtemp;
    uint32_t ADC_Value = 0;
    ADC_Value = ADC_GetConversionValue(ADC0_TEMP_SENSOR_DIFF) & 0x0FFF; 
    vtemp=((float)ADC_Value/4096)*3.3;
    if(vtemp>=.7012)
    {
        temp=(vtemp-.7012)/.001646;
    }
    else
    {
        temp=(vtemp-.7012)/.001769;
    }
    temp=25-temp;
    sprintf((char *)msg,"%0.3f C\r\n",temp);	
}
//uip�¼�������
//���뽫�ú��������û���ѭ��,ѭ������.
void uip_polling(void)
{
	uint8_t i;
	static struct timer periodic_timer, arp_timer;
	static uint8_t timer_ok=0;	 
	if(timer_ok==0)//����ʼ��һ��
	{
		timer_ok = 1;
		timer_set(&periodic_timer,CLOCK_SECOND/2);  //����1��0.5��Ķ�ʱ�� 
		timer_set(&arp_timer,CLOCK_SECOND*10);	   	//����1��10��Ķ�ʱ�� 
	}				 
	uip_len=tapdev_read();	//�������豸��ȡһ��IP��,�õ����ݳ���.uip_len��uip.c�ж���
	if(uip_len>0) 			//������
	{   
		 //printf("���յ�һ֡len:%d\r\n",uip_len);
		//����IP���ݰ�(ֻ��У��ͨ����IP���Żᱻ����) 
		if(BUF->type == htons(UIP_ETHTYPE_IP))//�Ƿ���IP��? 
		{
			uip_arp_ipin();	//ȥ����̫��ͷ�ṹ������ARP��
			uip_input();   	//IP������
			//������ĺ���ִ�к������Ҫ�������ݣ���ȫ�ֱ��� uip_len > 0
			//��Ҫ���͵�������uip_buf, ������uip_len  (����2��ȫ�ֱ���)		
			if(uip_len>0)//��Ҫ��Ӧ����
			{
				uip_arp_out();//����̫��ͷ�ṹ������������ʱ����Ҫ����ARP����
				tapdev_send();//�������ݵ���̫��
			}
		}else if (BUF->type==htons(UIP_ETHTYPE_ARP))//����arp����,�Ƿ���ARP�����?
		{
			uip_arp_arpin();
 			//������ĺ���ִ�к������Ҫ�������ݣ���ȫ�ֱ���uip_len>0
			//��Ҫ���͵�������uip_buf, ������uip_len(����2��ȫ�ֱ���)
 			if(uip_len>0)tapdev_send();//��Ҫ��������,��ͨ��tapdev_send����	 
		}
	}else if(timer_expired(&periodic_timer))	//0.5�붨ʱ����ʱ
	{
		timer_reset(&periodic_timer);		//��λ0.5�붨ʱ�� 
		//��������ÿ��TCP����, UIP_CONNSȱʡ��40��  
		for(i=0;i<UIP_CONNS;i++)
		{
			uip_periodic(i);	//����TCPͨ���¼�  
	 		//������ĺ���ִ�к������Ҫ�������ݣ���ȫ�ֱ���uip_len>0
			//��Ҫ���͵�������uip_buf, ������uip_len (����2��ȫ�ֱ���)
	 		if(uip_len>0)
			{
				uip_arp_out();//����̫��ͷ�ṹ������������ʱ����Ҫ����ARP����
				tapdev_send();//�������ݵ���̫��
			}
		}
#if UIP_UDP	//UIP_UDP 
		//��������ÿ��UDP����, UIP_UDP_CONNSȱʡ��10��
		for(i=0;i<UIP_UDP_CONNS;i++)
		{
			uip_udp_periodic(i);	//����UDPͨ���¼�
	 		//������ĺ���ִ�к������Ҫ�������ݣ���ȫ�ֱ���uip_len>0
			//��Ҫ���͵�������uip_buf, ������uip_len (����2��ȫ�ֱ���)
			if(uip_len > 0)
			{
				uip_arp_out();//����̫��ͷ�ṹ������������ʱ����Ҫ����ARP����
				tapdev_send();//�������ݵ���̫��
			}
		}
#endif 
		//ÿ��10�����1��ARP��ʱ������ ���ڶ���ARP����,ARP��10�����һ�Σ��ɵ���Ŀ�ᱻ����
		if(timer_expired(&arp_timer))
		{
			timer_reset(&arp_timer);
			uip_arp_timer();
		}
	}
}


