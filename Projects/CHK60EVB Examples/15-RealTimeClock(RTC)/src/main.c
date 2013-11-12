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
*/


int main(void)
{
    uint8_t last_sec;
    RTC_CalanderTypeDef RTC_Calander1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    UART_printf("RTC TEST\r\n");
    RTC_Init();
	
	  //��������ʱ��
    RTC_Calander1.Hour = 10;
    RTC_Calander1.Minute = 57;
    RTC_Calander1.Second = 58;
    RTC_Calander1.Month = 10;
    RTC_Calander1.Date = 10;
    RTC_Calander1.Year = 2013;
    //RTC_SetCalander(&RTC_Calander1);
    NVIC_EnableIRQ(RTC_IRQn);
    while(1) 
		{
        RTC_GetCalander(&RTC_Calander1); //��ȡʱ��
        if(last_sec != RTC_Calander1.Second)
				{
            UART_printf("%d-%d-%d %d:%d:%d\r\n", RTC_Calander1.Year, RTC_Calander1.Month, RTC_Calander1.Date, RTC_Calander1.Hour, RTC_Calander1.Minute, RTC_Calander1.Second);
            last_sec = RTC_Calander1.Second;
				}	
		}
 }

