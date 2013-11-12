#include "isr.h"


void UART4_RX_TX_IRQHandler(void)
{
    uint8_t ch;
    //�жϷ��ʹ������
    UART_SendDataIntProcess(UART4);
    //����ɹ����յ�������
    if(UART_ReceiveData(UART4, &ch))
    {
        //�ѽ��յ�����Ϣ���ͻ�ȥ echo
        UART_SendData(UART4,ch);
    }
}

void PORTB_IRQHandler(void)
{
    if(GPIO_GetITStates(PTB, GPIO_Pin_9) == SET)
    {
        LED_Toggle(kLED1);
        GPIO_ClearITPendingBit(PTB, GPIO_Pin_9);
    }

}

