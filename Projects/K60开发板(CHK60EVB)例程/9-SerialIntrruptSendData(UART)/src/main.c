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


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- ��ʵ������жϷ�ʽ���ʹ�������
* 2- ��ν�����ж� ���� ������ÿ�������һ���ֽں�����жϷ������ ֪ͨCPU CPU���ݵ�ǰ״̬ ����Buffer��һ���ַ�����
* 3- ���ַ�ʽҲ�����첽���� ���� ������ʽ���� ��UART_SendDataInt �����ú� ��ʼ���ʹ������� �������ǵȴ�������ɺ��ִ����
* 4- �жϷ��������ѯ���ͼ���Ľ�Լ��CPU��Դ
*/

uint8_t gTestBuffer[] = "UART Interrupt Send\r\n";

int main(void)
{
    //���崮�ڳ�ʼ���ṹ
    UART_InitTypeDef UART_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    //��ʼ��LED
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
	  //KBI ��ʼ��
	  KBI_Init(KBI_PinLookup_CHK60EVB, kNumOfKEY);
	
    UART_InitStruct1.UARTxMAP = UART4_RX_C14_TX_C15; //UART4ģ�� ӳ������:PTC14 PTC15
    UART_InitStruct1.UART_BaudRate = 115200;         //������ 115200
    UART_Init(&UART_InitStruct1);                    //��ʼ������
    //���������жϹ���
    UART_ITConfig(UART4,UART_IT_TDRE,ENABLE);
    //��ͨNVIC�϶�Ӧ�����ж���
    NVIC_EnableIRQ(UART4_RX_TX_IRQn);
    //��ʼ�жϷ���
    UART_SendDataInt(UART4,gTestBuffer,sizeof(gTestBuffer));
    //�ȴ��жϷ������
    while(UART_TxIntStruct1.IsComplete == FALSE); 
    while(1)
		{
        LED_Toggle(kLED1);
        DelayMs(500);
		}
}

