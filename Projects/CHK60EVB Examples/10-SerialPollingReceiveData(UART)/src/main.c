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
* 1- ��ʵ�������ѯ��ʽ���մ�������
* 2- ����whileѭ���� ����ɨ�� ������û�н��յ����� һ�����յ�������echo��ȥ
* 3- ���ַ�ʽ ��Ҫ CPU���ϵĽ���ɨ�� ������Դ�˷� �������while������������ �� ���ܵ��´���������Ӧ����ʱ
*/


int main(void)
{
    uint8_t ch;
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

    while(1)
		{
        //���ճɹ�
        if(UART_ReceiveData(UART4, &ch) == TRUE)
				{
				    //echo
            UART_SendData(UART4, ch);
				}
		}
}

