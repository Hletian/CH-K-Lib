#include "isr.h"



void UART4_RX_TX_IRQHandler(void)
{
	//�жϷ��ʹ������
	UART_SendDataIntProcess(UART4);
	//����ɹ����յ�������

}

