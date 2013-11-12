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
* 
*/



int main(void)
{
    uint32_t ADC_Value = 0;
    ADC_InitTypeDef ADC_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
    //��ʼ��ADC
    ADC_InitStruct1.ADCxMap = ADC0_SE13_PB3;              //PC0 ������ΪADC0 14ͨ��
    ADC_InitStruct1.ADC_Precision = ADC_PRECISION_10BIT;  //10λ����
    ADC_InitStruct1.ADC_TriggerSelect = ADC_TRIGGER_SW;   //�������(A ͨ����ʹ����/Ӳ������ B ͨ��ֻ��ʹ��Ӳ������)
    ADC_Init(&ADC_InitStruct1);
    while(1) 
    {
        ADC_Value = ADC_GetConversionValue(ADC0_SE13_PB3); //���ADת��ֵ
        UART_printf("ADC0_SE13_PB3:%d\r\n",ADC_Value);
        DelayMs(300);
    }
}

