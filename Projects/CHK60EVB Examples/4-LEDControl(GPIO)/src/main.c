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

//LED Components
#include "led.h"
#include "led_chk60evb.h"


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- ��װʽ���������˼�� LED��С�� ����Ҫ�����װ�ɹ���
*   1: led.c led.h �����ܹ� ����Ҫ�û��޸Ĵ���
*   2: led_<boradname>.c  led_<boradname>.h �Ͱ����йص�������Ϣ ����Ӳ����������
*   3: kLED1,kLED2����led_chk60evb.h�ж����ö���� ǰ���k ֻ��Ϊ�˱�ʾ������ö�ٳ���
*   4: LED_PinLookup_CHK60EVB[] ��һ�������� led_chk60evb.c�е� �ṹ������ ��Ű��ӵ�LED������Ϣ CHK60EVB �ǳ���K60������ļ�д
*   5: kNumOfLED �Ǵ���� led_chk60evb.h�е�ö�ٳ��� ��ʾ����LED������ ��CHK60�������� ������2
*   6: LED_Init ��һ���������� LED_PinLookup_CHK60EVB ����ṹ��ָ�� �ڶ����������� �ṹ���Ա���� Ҳ���ǰ�����LED����Ŀ
*   7: LED��������API�����μ� led.c �� led.h
*/


int main(void)
{
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    //��ʼ��LED
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
	  //��LED1����Ϊ�ߵ�ƽ
    LED_Set(kLED1);
    while(1)
    {   
			  //2LED��˸
			  LED_Toggle(kLED1);
        LED_Toggle(kLED2);
        DelayMs(500);
    }
}

