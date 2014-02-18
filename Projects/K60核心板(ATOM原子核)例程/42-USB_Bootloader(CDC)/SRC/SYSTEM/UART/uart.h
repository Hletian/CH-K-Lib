/***********************************************************************************************
 CHKD�ڲ����԰汾V0.1
 2012.12.3
************************************************************************************************/
#ifndef __UART_H__
#define __UART_H__
#include "sys.h"

//����printf��ӡ��
#define DEBUG_UART_PORT  UART4       //ָ���Ǹ�������Ϊprintf�����
#define DEBUG_UART_BAUD  (115200)    //printf ������

//������ʵ�ֵĽӿں����б�
void UART_Init(UART_Type *uartch,u32 baud);
void UART_Send1(UART_Type *uartch, u8 ch);                                                                 
void UART_SendN (UART_Type *uartch,u8 *buff,u32 len);
u8 UART_Re1 (UART_Type *uartch,u8 *ch);
void UART1_RX_TX_IRQHandler(void);
#endif
