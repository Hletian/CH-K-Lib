#ifndef __UIP_DEMO_H_
#define	__UIP_DEMO_H_

#include "uipopt.h"
#include "sys.h"


//ͨ�ų���״̬��(�û������Լ�����)  
enum
{
	STATE_CMD		= 0,	//�������״̬ 
	STATE_TX_TEST	= 1,	//�����������ݰ�״̬(�ٶȲ���)  
	STATE_RX_TEST	= 2		//�����������ݰ�״̬(�ٶȲ���)  
};	 

//���� uip_tcp_appstate_t �������ͣ��û��������Ӧ�ó�����Ҫ�õ�
//��Ա��������Ҫ���Ľṹ�����͵����֣���Ϊ����������ᱻuip���á�
//uip.h �ж���� 	struct uip_conn  �ṹ���������� uip_tcp_appstate_t		  
struct tcp_demo_appstate
{
	uint8_t state;
	uint8_t *textptr;
	uint16_t textlen;
};	 

typedef struct tcp_demo_appstate uip_tcp_appstate_t;

#ifndef UIP_APPCALL
#define UIP_APPCALL uip_demo //����ص�����Ϊ tcp_demo_appcall 
#endif
//������ʵ�ֵĺ����ӿ�
//TCP������
void tcp_server_demo_appcall(void);  
void tcp_server_aborted(void);
void tcp_server_timedout(void);
void tcp_server_closed(void);
void tcp_server_connected(void);
void tcp_server_acked(void);
void tcp_server_senddata(void);
//TCP ������ �û��ӿ�
uint8_t app_tcp_server_senddata(uint8_t* data,uint16_t len);
uint16_t app_tcp_server_recdata(uint8_t* data);

//TCP�ͻ���
void tcp_client_demo_appcall(void);  
void tcp_client_aborted(void);
void tcp_client_timedout(void);
void tcp_client_closed(void);
void tcp_client_connected(void);
void tcp_client_acked(void);
void tcp_client_senddata(void);
void tcp_client_reconnect(void);
//TCP�ͻ����û�����ӿ�
uint8_t app_tcp_client_senddata(uint8_t* data,uint16_t len);
uint16_t app_tcp_client_recdata(uint8_t* data);
//������ʵ�ֵĽӿڳ���
void uip_demo(void);
#endif

