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
* 1- ���Դ����Ǳ��̼����е�һ����ɫ �䱾����ʹ�ô�����ѯ����
* 2- ��ѯ����(Polling) Ҳ��ͬ������ ������ʽ���ʹ��� ÿ������һ���ֽں�CPU��ȴ����跢����ɺ��ٷ��ͺ�����ֽ�
*    ���ַ�ʽ������˷�CPU��Դ ���������Ҳ����ѧ��һ�ַ��ͷ�ʽ ��ȻЧ�ʵ��µ���Ӧ�����
* 3- UART_printfʹ����ѯ����ģʽ ʹ�ô��������Ϣ �ǳ����ߵ��Ե���Ҫ�ֶ�
* 4- DisplayCPUInfoʵ��uart�����еĺ��� ��������UART_printf���оƬ��Ϣ ǰ���� UART_DebugPortInit�����ȱ�����
*/


int main(void)
{
    uint8_t i = 0;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    //��ʼ��LED
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    //KBI ��ʼ��
    KBI_Init(KBI_PinLookup_CHKATOM, kNumOfKEY);
    //��ʼ��UART���Դ��� ʹ�� UART4 ռ��PC14 PC15 ���� ������115200
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    //��ӡHelloWorld�ַ�
    UART_printf("HelloWorld\r\n");
    //��ӡCPU��Ϣ
    DisplayCPUInfo();
    while(1)
    {
        UART_printf("i:%d\r\n", i++);
        DelayMs(500);
    }
}

