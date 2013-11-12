#include "tcp_client_demo.h"
#include "uip.h"
#include "string.h"

//��ʵ������Ҫ���ڴ涨��
uint8_t tcp_client_rec_buf[64];
uint8_t tcp_client_send_buf[64];   	//�������ݻ���	  
uint8_t tcp_client_sta;				//�ͻ���״̬
uint8_t tcp_client_rec_len;               //���յ������ݳ���
//��������
void tcp_client_reconnect(void);
void tcp_client_aborted(void);
void tcp_client_timedout(void);
void tcp_client_closed(void);
void tcp_client_connected(void);
void tcp_client_acked(void);
void tcp_client_senddata(void);
//����һ��TCP �ͻ���Ӧ�ûص�������
//�ú���ͨ��UIP_APPCALL(tcp_demo_appcall)����,ʵ��Web Client�Ĺ���.
//��uip�¼�����ʱ��UIP_APPCALL�����ᱻ����,���������˿�(1400),ȷ���Ƿ�ִ�иú�����
//���� : ��һ��TCP���ӱ�����ʱ�����µ����ݵ�������Ѿ���Ӧ��������Ҫ�ط����¼�
void tcp_client_demo_appcall(void)
{
 	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	if(uip_aborted())
	{
		tcp_client_aborted();		//������ֹ	 
	}
	if(uip_timedout())
	{
		tcp_client_timedout();	//���ӳ�ʱ   
	}
	if(uip_closed())
	{
		tcp_client_closed();		//���ӹر�	
	}
 	if(uip_connected())
	{
		tcp_client_connected();	//���ӳɹ�
	}    
	if(uip_acked())
	{
		tcp_client_acked();			//���͵����ݳɹ��ʹ�  
	}
 	//���յ�һ���µ�TCP���ݰ� 
	if (uip_newdata())
	{
		if((tcp_client_sta&(1<<6))==0)//��δ�յ�����
		{
			if(uip_len>199)
			{		   
				((uint8_t*)uip_appdata)[199]=0;
			}		    
			if(uip_len>64) tcp_client_rec_len = 64; else tcp_client_rec_len = uip_len;
	    memcpy(tcp_client_rec_buf,uip_appdata,tcp_client_rec_len);			   	  		  
			tcp_client_sta|=1<<6;//��ʾ�յ��ͻ�������
		}				  
	}else if(tcp_client_sta&(1<<5))//��������Ҫ����
	{
		s->textptr=tcp_client_send_buf;
		s->textlen=strlen((const char*)tcp_client_send_buf);
		tcp_client_sta&=~(1<<5);//������
	}  
	//����Ҫ�ط��������ݵ�����ݰ��ʹ���ӽ���ʱ��֪ͨuip�������� 
	if(uip_rexmit()||uip_newdata()||uip_acked()||uip_connected()||uip_poll())
	{
		tcp_client_senddata();
	}			
}

//�������Ǽٶ�Server�˵�IP��ַΪ:192.168.0.1
//���IP�������Server�˵�IP�޸�.
//������������
void tcp_client_reconnect()
{
	uip_ipaddr_t ipaddr;
	uip_ipaddr(&ipaddr,192,168,0,2);	//����PC�� IPΪ192.168.0.2
	uip_connect(&ipaddr,htons(1400)); 	//�˿�Ϊ1400
}
//��ֹ����				    
void tcp_client_aborted(void)
{
	tcp_client_sta&=~(1<<7);	//��־û������
	tcp_client_reconnect();		//������������
	uip_log("tcp_client aborted!\r\n");//��ӡlog
}
//���ӳ�ʱ
void tcp_client_timedout(void)
{
	tcp_client_sta&=~(1<<7);	//��־û������	   
	uip_log("tcp_client timeout!\r\n");//��ӡlog
}
//���ӹر�
void tcp_client_closed(void)
{
	tcp_client_sta&=~(1<<7);	//��־û������
	tcp_client_reconnect();		//������������
	uip_log("tcp_client closed!\r\n");//��ӡlog
}	 
//���ӽ���
void tcp_client_connected(void)
{ 
	struct tcp_demo_appstate *s=(struct tcp_demo_appstate *)&uip_conn->appstate;
 	tcp_client_sta|=1<<7;		//��־���ӳɹ�
  	uip_log("tcp_client connected!\r\n");//��ӡlog
	s->state=STATE_CMD; 		//ָ��״̬
	s->textlen=0;
	s->textptr="�ѳɹ����ӳ���K60 ������ \r\n";//��Ӧ��Ϣ
	s->textlen=strlen((char *)s->textptr);	  
}
//���͵����ݳɹ��ʹ�
void tcp_client_acked(void)
{											    
	struct tcp_demo_appstate *s=(struct tcp_demo_appstate *)&uip_conn->appstate;
	s->textlen=0;//��������
	uip_log("tcp_client acked!\r\n");//��ʾ�ɹ�����		 
}
//�������ݸ������
void tcp_client_senddata(void)
{
	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	//s->textptr:���͵����ݰ�������ָ��
	//s->textlen:���ݰ��Ĵ�С����λ�ֽڣ�		   
	if(s->textlen>0)uip_send(s->textptr, s->textlen);//����TCP���ݰ�	 
}
//�û��ӿ� TCP client ��������
uint8_t app_tcp_client_senddata(uint8_t* data,uint16_t len)
{
	if(len > sizeof(tcp_client_send_buf)) //̫��
	{
		return 2;
	}
	if(tcp_client_sta&(1<<7))	//���ӻ�����
	{
	  memcpy(tcp_client_send_buf,data,len);
		tcp_client_sta|=1<<5;//�����������Ҫ����
		return 0;//���ͳɹ� 
	}
	else return 1;//����ʧ��
}
//�û��ӿ� TCP ������ ��������
uint16_t app_tcp_client_recdata(uint8_t* data)
{
	if(tcp_client_sta&(1<<6))	//�յ�������
	{
		memcpy(data,tcp_client_rec_buf,tcp_client_rec_len);
		tcp_client_sta&=~(1<<6);		//��������Ѿ�������	
		return tcp_client_rec_len;
	}
	return 0;
}
