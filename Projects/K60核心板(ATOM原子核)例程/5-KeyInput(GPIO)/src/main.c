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
* 1- ��װʽ���������˼�� KBI: KeyBorad Input ���������� 
*   1: kbi.c kbi.h �ܹ� ����Ҫ�޸�
*   2: kbi<boradname>.c  led_<boradname>.h �Ͱ����йص�������Ϣ ����Ӳ����������
*   3: KBI_Init ��һ���������� KBI_PinLookup_CHK60EVB ����ṹ��ָ�� �ڶ����������� �ṹ���Ա���� Ҳ���ǰ�����KEY����Ŀ
*   4: KBI��������API�����μ� kbi.c �� kbi.h
*   5: KBI_Scan() �ǰ�������ɨ�躯�� ɨ�����ڱ����� KBI_SCAN_PERIOD_IN_US ��λΪUS��
*/

//���ʹ�ó���K60�����壬�뽫kbi.h�� #define	KBI_STATE_DOWN ����Ϊ Bit_SET
//���ʹ�ó���K60���İ壬�뽫kbi.h�� #define    KBI_STATE_DOWN ����Ϊ Bit_Reset

int main(void)
{
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    //��ʼ��LED
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    //KBI ��ʼ��
    KBI_Init(KBI_PinLookup_CHKATOM, kNumOfKEY);
    while(1)
    {   
			  //ɨ�谴��
        KBI_Scan();
        if(KBI_GetKeyState(kKEY1) == kKBI_SINGLE)
        {
            LED_Toggle(kLED1);
        }
        if(KBI_GetKeyState(kKEY1) == kKBI_LONG)
        {
            LED_Toggle(kLED2);
        }
				//ɨ������ʱ
        DelayMs(KBI_SCAN_PERIOD_IN_US/1000);
    }
}

