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
#include "led_chkatom.h"
//KBI Devices
#include "kbi.h"
#include "kbi_chkatom.h"


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- ���ʵ�����в�ʹ��UartDebugPort ֻ��ʹ��UART�б�׼����UART_Init��ʼ������ ������ѯ��������
     ���Ǵ��ڵ���ѯ���ͱ�׼�÷�
* 2- ��Ȼ �����Լ�����UART�����з���һ���ַ��ĺ���UART_SendData��дһ�����Ͷ���ֽڵĺ��� �籾����UART_SendBytes
*/

//���Ͷ���ַ���
static void UART_SendBytes(UART_Type* UARTx, uint8_t* pBuffer, uint32_t size)
{
    while(size--)
    {
        UART_SendData(UARTx, *(pBuffer++));
		}
}

int main(void)
{
    //���崮�ڳ�ʼ���ṹ
    UART_InitTypeDef UART_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    //��ʼ��LED
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    //KBI ��ʼ��
   KBI_Init(KBI_PinLookup_CHKATOM, kNumOfKEY);
    
    UART_InitStruct1.UARTxMAP = UART4_RX_C14_TX_C15; //UART4ģ�� ӳ������:PTC14 PTC15
    UART_InitStruct1.UART_BaudRate = 115200;         //������ 115200
    UART_Init(&UART_InitStruct1);                    //��ʼ������
	
    UART_SendData(UART4, 'H');
    UART_SendData(UART4, 'e');
    UART_SendData(UART4, 'l');
    UART_SendData(UART4, 'l');
    UART_SendData(UART4, 'o');
    //һ�η��Ͷ���ַ���
    UART_SendBytes(UART4, "12345678", 8);
    while(1)
    {
			
    }
}

