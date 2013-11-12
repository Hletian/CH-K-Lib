#include "tcp_server_demo.h"
#include "uip.h"
#include "string.h"
void tcp_server_demo_appcall(void);


//ȫ�ֱ���
uint8_t tcp_server_send_buf[64];   	//�������ݻ���	 
uint8_t tcp_server_rec_buf[64];   	//�������ݻ���
uint8_t tcp_server_sta;				//�����״̬
uint8_t rec_len;
//��������

//Ӧ�ó������
void tcp_server_demo_appcall(void)
{

 	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	//������ֹ	
	if(uip_aborted())
	{
		tcp_server_aborted();		
	}   
	//���ӳ�ʱ  
	if(uip_timedout())
	{
		tcp_server_timedout();	
	}
	//���ӹر�	   	 
	if(uip_closed())
	{
		tcp_server_closed();		
	}
	//���ӳɹ�	 
  if(uip_connected())
	{
		tcp_server_connected();
	} 
	//���͵����ݳɹ��ʹ� 
	if(uip_acked())
	{
		tcp_server_acked();	
	}
	//���յ�һ���µ�TCP���ݰ� 
	if(uip_newdata())//�յ��ͻ��˷�����������
	{
		if((tcp_server_sta&(1<<6))==0)//��δ�յ�����
		{
			if(uip_len>64) rec_len = 64; else rec_len = uip_len;
	    memcpy(tcp_server_rec_buf,uip_appdata,uip_len);				   	  		  
			tcp_server_sta|=1<<6;//��ʾ�յ��ͻ�������
		}
	}
	else if(tcp_server_sta&(1<<5))//��������Ҫ����
	{
		s->textptr=tcp_server_send_buf;
		s->textlen=strlen((const char*)tcp_server_send_buf);
		tcp_server_sta&=~(1<<5);//������
	}   
	//����Ҫ�ط��������ݵ�����ݰ��ʹ���ӽ���ʱ��֪ͨuip�������� 
	if(uip_rexmit()||uip_newdata()||uip_acked()||uip_connected()||uip_poll())
	{
		tcp_server_senddata(); //��������
	}	
}


//��ֹ����				    
void tcp_server_aborted(void)
{
	tcp_server_sta&=~(1<<7);	//��־û������
	uip_log("tcp_server aborted!\r\n");//��ӡlog
}
//���ӳ�ʱ
void tcp_server_timedout(void)
{
	tcp_server_sta&=~(1<<7);	//��־û������
	uip_log("tcp_server timeout!\r\n");//��ӡlog
}
//���ӹر�
void tcp_server_closed(void)
{
	tcp_server_sta&=~(1<<7);	//��־û������
	uip_log("tcp_server closed!\r\n");//��ӡlog
}

//���ӽ���
void tcp_server_connected(void)
{								  
	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	//uip_conn�ṹ����һ��"appstate"�ֶ�ָ��Ӧ�ó����Զ���Ľṹ�塣
	//����һ��sָ�룬��Ϊ�˱���ʹ�á�
 	//����Ҫ�ٵ���Ϊÿ��uip_conn�����ڴ棬����Ѿ���uip�з�����ˡ�
	//��uip.c �� ����ش������£�
	//		struct uip_conn *uip_conn;
	//		struct uip_conn uip_conns[UIP_CONNS]; //UIP_CONNSȱʡ=10
	//������1�����ӵ����飬֧��ͬʱ�����������ӡ�
	//uip_conn��һ��ȫ�ֵ�ָ�룬ָ��ǰ��tcp��udp���ӡ�
	tcp_server_sta|=1<<7;		//��־���ӳɹ�
  	uip_log("tcp_server connected!\r\n");//��ӡlog
	s->state=STATE_CMD; 		//ָ��״̬
	s->textlen=0;
	s->textptr="�ѳɹ����ӵ�����K60������!\r\n";
	s->textlen=strlen((char *)s->textptr);
} 

//���͵����ݳɹ��ʹ�
void tcp_server_acked(void)
{						    	 
	struct tcp_demo_appstate *s=(struct tcp_demo_appstate *)&uip_conn->appstate;
	s->textlen=0;//��������
	uip_log("tcp_server acked!\r\n");//��ʾ�ɹ�����		 
}
//�������ݸ��ͻ���
void tcp_server_senddata(void)
{
	struct tcp_demo_appstate *s = (struct tcp_demo_appstate *)&uip_conn->appstate;
	//s->textptr : ���͵����ݰ�������ָ��
	//s->textlen �����ݰ��Ĵ�С����λ�ֽڣ�		 
	if(s->textlen>0)uip_send(s->textptr, s->textlen);//����TCP���ݰ�	 
}
//�û��ӿ� TCP server ��������
uint8_t app_tcp_server_senddata(uint8_t* data,uint16_t len)
{
	if(len > sizeof(tcp_server_send_buf)) //̫��
	{
		return 2;
	}
	if(tcp_server_sta&(1<<7))	//���ӻ�����
	{
	  memcpy(tcp_server_send_buf,data,len);
		tcp_server_sta|=1<<5;//�����������Ҫ����
		return 0;//���ͳɹ� 
	}
	else return 1;//����ʧ��
}
//�û��ӿ� TCP ������ ��������
uint16_t app_tcp_server_recdata(uint8_t* data)
{
	if(tcp_server_sta&(1<<6))	//�յ�������
	{
		memcpy(data,tcp_server_rec_buf,rec_len);
		tcp_server_sta&=~(1<<6);		//��������Ѿ�������	
		return rec_len;
	}
	return 0;
}

