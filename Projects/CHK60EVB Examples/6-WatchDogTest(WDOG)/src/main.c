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
* 1- ���Ź� - ��ֹоƬ���������ܷ�
* 2- ��ʵ�� - �ϵ������10�� ������Կ����ĸ�λЧ�� ���1S��û�а��� �� û��ι�� оƬ��λ�������� ����а������� ι���ɹ� �򲻸�λ
*/


int main(void)
{
    uint8_t i;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    //��ʼ��LED
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
	  //KBI ��ʼ��
	  KBI_Init(KBI_PinLookup_CHK60EVB, kNumOfKEY);
    //���� ���츴λЧ��
    for(i = 0; i < 10; i++)
    {
        LED_Toggle(kLED1);
        DelayMs(50);
    }
	  //��ʼ�����Ź� ���1000MS ��û��ι���� ��λ
	  WDOG_Init(1000);
    while(1)
    {   
        KBI_Scan();
        if((KBI_GetKeyState(kKEY1) == kKBI_SINGLE) || (KBI_GetKeyState(kKEY2) == kKBI_SINGLE))
        {  
					  //����������� ι��:
            WDOG_Feed();
        }
				//ɨ������ʱ
        DelayMs(KBI_SCAN_PERIOD_IN_US/1000);
    }
}

