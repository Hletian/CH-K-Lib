#include "uip_demo.h"
#include "uip.h"
#include "httpd.h"

void uip_demo(void)
{
	switch(uip_conn->lport)//���ؼ����˿�80��1200 
	{
		case HTONS(80):
			httpd_appcall();  //HTTP web����
			break;
		case HTONS(1200):
	    tcp_server_demo_appcall();  //Զ��1200�˿ڴ��� TCP server
			break;
		default:break;					  
		    
	}		    
	switch(uip_conn->rport)	//Զ������1400�˿�
	{
	    case HTONS(1400):
				tcp_client_demo_appcall(); // 1400�˿� TCP client
	      break;
	    default:break;
	}   
}


