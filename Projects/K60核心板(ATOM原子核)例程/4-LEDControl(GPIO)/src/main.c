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
#include "led_chkatom.h"


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//user:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- ��װʽ���������˼�� LED��С�� ����Ҫ�����װ�ɹ���
*   1: led.c led.h �����ܹ� ����Ҫ�û��޸Ĵ���
*   2: led_<boradname>.c  led_<boradname>.h �Ͱ����йص�������Ϣ ����Ӳ����������
*   3: kLED1,kLED2����led_chkatom.h�ж����ö���� ǰ���k ֻ��Ϊ�˱�ʾ������ö�ٳ���
*   4: LED_PinLookup_chkatom[] ��һ�������� led_chkatom.c�е� �ṹ������ ��Ű��ӵ�LED������Ϣ chkatom �ǳ���K60���İ�ļ�д
*   5: kNumOfLED �Ǵ���� led_chkatom.h�е�ö�ٳ��� ��ʾ����LED������ ��CHK60�������� ������2
*   6: LED_Init ��һ���������� LED_PinLookup_chkatom ����ṹ��ָ�� �ڶ����������� �ṹ���Ա���� Ҳ���ǰ�����LED����Ŀ
*   7: LED��������API�����μ� led.c �� led.h
*/


int main(void)
{
     uint8_t i;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    //��ʼ��LED
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    while(1)
    {   
        for(i = 0; i < kNumOfLED; i++)
        {
            LED_Toggle(i);
            DelayMs(100);
        }
    }
}





